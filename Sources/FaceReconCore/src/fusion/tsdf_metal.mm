// License: Apache 2.0. See LICENSE file in root directory.
//
// Metal backend of the TSDF fusion. The volume lives in shared-storage buffers (unified memory), so marching cubes
// and the CPU path read and write the same bytes without copies. The shader is compiled at runtime from source:
// the Command Line Tools have no offline `metal` compiler. Math mode is "safe" so the GPU follows the same IEEE
// arithmetic as the CPU fallback (results agree to rounding).

#include "fusion_internal.h"
#include "../common/fr_internal.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <mutex>

#if ! __has_feature( objc_arc )
#error "tsdf_metal.mm relies on ARC (SwiftPM compiles Objective-C++ with -fobjc-arc)"
#endif

namespace
{
const char * const kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

// Must match fr::IntegrationConstants (fusion_internal.h).
struct Constants
{
    float4x4 camera_from_world;
    float fx, fy, cx, cy;
    float origin_x, origin_y, origin_z, voxel;
    float truncation, inv_truncation, max_weight, depth_jump;
    int nx, ny, nz, width;
    int height, integrate_color, has_rgb, pad0;
};

// One thread per voxel; the same arithmetic as fr::integrate_voxel (fr_tsdf.cpp).
kernel void integrate_tsdf( device half2 * tw [[buffer( 0 )]],
                            device uchar4 * colors [[buffer( 1 )]],
                            device const float * depth [[buffer( 2 )]],
                            device const float * weight [[buffer( 3 )]],
                            device const uchar * rgb [[buffer( 4 )]],
                            constant Constants & c [[buffer( 5 )]],
                            uint3 gid [[thread_position_in_grid]] )
{
    if( gid.x >= uint( c.nx ) || gid.y >= uint( c.ny ) || gid.z >= uint( c.nz ) )
        return;
    const float px = c.origin_x + c.voxel * float( gid.x );
    const float py = c.origin_y + c.voxel * float( gid.y );
    const float pz = c.origin_z + c.voxel * float( gid.z );
    const float4x4 m = c.camera_from_world;
    const float x = m[0][0] * px + m[1][0] * py + m[2][0] * pz + m[3][0];
    const float y = m[0][1] * px + m[1][1] * py + m[2][1] * pz + m[3][1];
    const float z = m[0][2] * px + m[1][2] * py + m[2][2] * pz + m[3][2];
    if( z <= 0.0f )
        return;
    const float u = c.fx * x / z + c.cx;
    const float v = c.fy * y / z + c.cy;
    const float fu = floor( u + 0.5f ), fv = floor( v + 0.5f );
    if( !( fu >= 0.0f && fv >= 0.0f && fu < float( c.width ) && fv < float( c.height ) ) )
        return;
    const int i = int( fv ) * c.width + int( fu );
    float d = depth[i];
    float w = weight[i];
    if( d <= 0.0f || w <= 0.0f )
        return;
    const float bu = floor( u ), bv = floor( v );
    if( bu >= 0.0f && bv >= 0.0f && bu + 1.0f < float( c.width ) && bv + 1.0f < float( c.height ) )
    {
        const int j = int( bv ) * c.width + int( bu );
        const float d00 = depth[j], d10 = depth[j + 1], d01 = depth[j + c.width], d11 = depth[j + c.width + 1];
        const float w00 = weight[j], w10 = weight[j + 1], w01 = weight[j + c.width], w11 = weight[j + c.width + 1];
        const float lo = min( min( d00, d10 ), min( d01, d11 ) );
        const float hi = max( max( d00, d10 ), max( d01, d11 ) );
        const float wlo = min( min( w00, w10 ), min( w01, w11 ) );
        if( lo > 0.0f && wlo > 0.0f && hi - lo <= c.depth_jump )
        {
            const float ax = u - bu, ay = v - bv;
            d = ( d00 * ( 1.0f - ax ) + d10 * ax ) * ( 1.0f - ay ) + ( d01 * ( 1.0f - ax ) + d11 * ax ) * ay;
            w = ( w00 * ( 1.0f - ax ) + w10 * ax ) * ( 1.0f - ay ) + ( w01 * ( 1.0f - ax ) + w11 * ax ) * ay;
        }
    }
    const float sdf = d - z;
    if( sdf < -c.truncation )
        return;
    const float tsdf = min( 1.0f, sdf * c.inv_truncation );
    const uint idx = gid.x + uint( c.nx ) * ( gid.y + uint( c.ny ) * gid.z );
    const half2 cell = tw[idx];
    const float wold = float( cell.y );
    const float dold = float( cell.x );
    const float wnew = wold + w;
    tw[idx] = half2( half( ( dold * wold + tsdf * w ) / wnew ), half( min( wnew, c.max_weight ) ) );
    if( c.integrate_color != 0 && c.has_rgb != 0 && fabs( sdf ) < c.truncation )
    {
        const uchar4 cc = colors[idx];
        const float n = float( cc.w );
        const float r = ( float( cc.x ) * n + float( rgb[3 * i] ) ) / ( n + 1.0f ) + 0.5f;
        const float g = ( float( cc.y ) * n + float( rgb[3 * i + 1] ) ) / ( n + 1.0f ) + 0.5f;
        const float b = ( float( cc.z ) * n + float( rgb[3 * i + 2] ) ) / ( n + 1.0f ) + 0.5f;
        colors[idx] = uchar4( uchar( r ), uchar( g ), uchar( b ), uchar( min( n + 1.0f, 255.0f ) ) );
    }
}
)METAL";

// Device and pipeline are created once per process and shared (both are thread-safe Metal objects).
struct SharedPipeline
{
    id< MTLDevice > device = nil;
    id< MTLComputePipelineState > integrate = nil;
    std::string error;
};

const SharedPipeline & shared_pipeline()
{
    static SharedPipeline shared;
    static std::once_flag once;
    std::call_once( once, [] {
        @autoreleasepool
        {
            shared.device = MTLCreateSystemDefaultDevice();
            if( ! shared.device )
            {
                shared.error = "no Metal device";
                return;
            }
            MTLCompileOptions * options = [[MTLCompileOptions alloc] init];
            options.mathMode = MTLMathModeSafe;
            NSError * err = nil;
            id< MTLLibrary > library = [shared.device newLibraryWithSource:[NSString stringWithUTF8String:kShaderSource]
                                                                   options:options
                                                                     error:&err];
            if( ! library )
            {
                shared.error = std::string( "Metal shader compilation failed: " )
                             + ( err ? err.localizedDescription.UTF8String : "unknown error" );
                shared.device = nil;
                return;
            }
            id< MTLFunction > function = [library newFunctionWithName:@"integrate_tsdf"];
            if( function )
                shared.integrate = [shared.device newComputePipelineStateWithFunction:function error:&err];
            if( ! shared.integrate )
            {
                shared.error = std::string( "Metal pipeline creation failed: " )
                             + ( err ? err.localizedDescription.UTF8String : "missing kernel" );
                shared.device = nil;
            }
        }
    } );
    return shared;
}
}  // namespace

namespace fr
{
namespace metal
{
struct Volume
{
    id< MTLCommandQueue > queue = nil;
    id< MTLBuffer > tsdf_weight = nil;
    id< MTLBuffer > colors = nil;
    id< MTLBuffer > dummy = nil;   // bound where an optional buffer is absent
    size_t voxels = 0;
};

void VolumeDeleter::operator()( Volume * volume ) const
{
    delete volume;
}

VolumePtr create_volume( size_t voxel_count, bool with_color, std::string & error )
{
    const SharedPipeline & shared = shared_pipeline();
    if( ! shared.device )
    {
        error = shared.error;
        return nullptr;
    }
    @autoreleasepool
    {
        const size_t bytes = voxel_count * 2 * sizeof( half_t );
        if( bytes > shared.device.maxBufferLength )
        {
            error = "volume exceeds the Metal buffer size limit";
            return nullptr;
        }
        VolumePtr volume( new Volume() );
        volume->voxels = voxel_count;
        volume->queue = [shared.device newCommandQueue];
        volume->tsdf_weight = [shared.device newBufferWithLength:bytes options:MTLResourceStorageModeShared];
        if( with_color )
            volume->colors = [shared.device newBufferWithLength:voxel_count * 4 options:MTLResourceStorageModeShared];
        volume->dummy = [shared.device newBufferWithLength:16 options:MTLResourceStorageModeShared];
        if( ! volume->queue || ! volume->tsdf_weight || ( with_color && ! volume->colors ) || ! volume->dummy )
        {
            error = "cannot allocate the Metal volume";
            return nullptr;
        }
        // Clear on the GPU: faster than touching every page from the CPU.
        id< MTLCommandBuffer > cb = [volume->queue commandBuffer];
        id< MTLBlitCommandEncoder > blit = [cb blitCommandEncoder];
        [blit fillBuffer:volume->tsdf_weight range:NSMakeRange( 0, bytes ) value:0];
        if( volume->colors )
            [blit fillBuffer:volume->colors range:NSMakeRange( 0, voxel_count * 4 ) value:0];
        [blit endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        if( cb.status != MTLCommandBufferStatusCompleted )
        {
            error = "Metal volume initialization failed";
            return nullptr;
        }
        return volume;
    }
}

half_t * tsdf_weight( Volume * volume )
{
    return static_cast< half_t * >( volume->tsdf_weight.contents );
}

uint8_t * colors( Volume * volume )
{
    return volume->colors ? static_cast< uint8_t * >( volume->colors.contents ) : nullptr;
}

fr_status integrate( Volume * volume, const IntegrationConstants & c, const PreparedFrame & frame )
{
    const SharedPipeline & shared = shared_pipeline();
    @autoreleasepool
    {
        const size_t pixels = size_t( frame.width ) * size_t( frame.height );
        id< MTLBuffer > depth = [shared.device newBufferWithBytes:frame.depth.data() length:pixels * sizeof( float )
                                                          options:MTLResourceStorageModeShared];
        id< MTLBuffer > weight = [shared.device newBufferWithBytes:frame.weight.data() length:pixels * sizeof( float )
                                                           options:MTLResourceStorageModeShared];
        id< MTLBuffer > rgb = frame.rgb ? [shared.device newBufferWithBytes:frame.rgb length:pixels * 3
                                                                    options:MTLResourceStorageModeShared]
                                        : volume->dummy;
        if( ! depth || ! weight || ! rgb )
            return fail( FR_ERR_OUT_OF_MEMORY, "fr_tsdf_integrate: cannot allocate Metal frame buffers" );
        id< MTLCommandBuffer > cb = [volume->queue commandBuffer];
        id< MTLComputeCommandEncoder > encoder = [cb computeCommandEncoder];
        [encoder setComputePipelineState:shared.integrate];
        [encoder setBuffer:volume->tsdf_weight offset:0 atIndex:0];
        [encoder setBuffer:( volume->colors ? volume->colors : volume->dummy ) offset:0 atIndex:1];
        [encoder setBuffer:depth offset:0 atIndex:2];
        [encoder setBuffer:weight offset:0 atIndex:3];
        [encoder setBuffer:rgb offset:0 atIndex:4];
        [encoder setBytes:&c length:sizeof( c ) atIndex:5];
        const NSUInteger width = 32, height = std::max< NSUInteger >( 1, std::min< NSUInteger >( 8, shared.integrate.maxTotalThreadsPerThreadgroup / 32 ) );
        [encoder dispatchThreads:MTLSizeMake( NSUInteger( c.nx ), NSUInteger( c.ny ), NSUInteger( c.nz ) )
            threadsPerThreadgroup:MTLSizeMake( width, height, 1 )];
        [encoder endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        if( cb.status != MTLCommandBufferStatusCompleted )
        {
            const char * reason = cb.error ? cb.error.localizedDescription.UTF8String : "unknown error";
            return fail( FR_ERR_FAILED, std::string( "fr_tsdf_integrate: Metal command failed: " ) + reason );
        }
    }
    return FR_OK;
}
}  // namespace metal
}  // namespace fr
