// License: Apache 2.0. See LICENSE file in root directory.
//
// TSDF volume: parameter validation and size checks, per-frame preparation (depth in meters, mask, depth range,
// noise-aware observation weights), CPU integration and the public C API. The Metal path (tsdf_metal.mm) runs the
// same per-voxel arithmetic on the GPU over the same shared-memory volume, and marching cubes reads it in place.

#include "fusion_internal.h"
#include "../registration/recon_cloud.h"
#include "../common/fr_internal.h"

#include <sys/sysctl.h>

#include <cstdlib>
#include <cstring>
#include <sstream>

struct fr_tsdf
{
    fr_tsdf_params params {};
    fr::VolumeGeometry geometry;
    fr::metal::VolumePtr gpu;
    fr::half_t * tsdf_weight = nullptr;   // owned by gpu, or by cpu_storage
    uint8_t * colors = nullptr;
    std::unique_ptr< void, void ( * )( void * ) > cpu_storage { nullptr, std::free };
    std::unique_ptr< void, void ( * )( void * ) > cpu_colors { nullptr, std::free };
};

namespace fr
{
namespace
{
// Observation weights are normalized so a frontal view at the typical scan distance (0.6 m) weighs ~1.
constexpr float kReferenceDepth = 0.6f;
constexpr size_t kMaxVoxels = size_t( 1500 ) * 1000 * 1000;   // keeps 32-bit voxel indices in the shader
constexpr int kNormalRadiusPx = 3;                             // depth noise makes 1-pixel normals useless

size_t memory_budget_bytes()
{
    uint64_t physical = 0;
    size_t size = sizeof( physical );
    if( sysctlbyname( "hw.memsize", &physical, &size, nullptr, 0 ) != 0 || physical == 0 )
        physical = uint64_t( 8 ) << 30;
    return size_t( std::min< uint64_t >( physical / 2, uint64_t( 32 ) << 30 ) );
}

std::string format_bytes( double bytes )
{
    std::ostringstream s;
    s.precision( 3 );
    s << bytes / double( 1u << 30 ) << " GiB";
    return s.str();
}

bool validate_tsdf_params( const fr_tsdf_params & p, VolumeGeometry & g, std::string & error )
{
    if( ! ( p.voxel_size_m > 0 ) || ! std::isfinite( p.voxel_size_m ) )
        error = "voxel_size_m must be positive";
    else if( ! ( p.truncation_m > p.voxel_size_m ) || ! std::isfinite( p.truncation_m ) )
        error = "truncation_m must be larger than the voxel size";
    else if( ! ( p.min_depth_m >= 0 ) || ! ( p.max_depth_m > p.min_depth_m ) || ! std::isfinite( p.max_depth_m ) )
        error = "invalid depth range";
    else if( ! ( p.max_weight > 0 ) || ! std::isfinite( p.max_weight ) )
        error = "max_weight must be positive";
    else if( p.weight_mode != 0 && p.weight_mode != 1 )
        error = "weight_mode must be 0 or 1";
    else if( ! all_finite( p.bounds_min, 3 ) || ! all_finite( p.bounds_max, 3 ) )
        error = "bounds are not finite";
    else
    {
        int32_t dims[3];
        for( int k = 0; k < 3; ++k )
        {
            const double extent = double( p.bounds_max[k] ) - double( p.bounds_min[k] );
            if( ! ( extent > 0 ) )
            {
                error = "bounds_max must be larger than bounds_min on every axis";
                return false;
            }
            const double n = std::ceil( extent / double( p.voxel_size_m ) ) + 1;
            if( n > 16384 )
            {
                error = "volume has " + std::to_string( int64_t( n ) ) + " voxels along an axis (limit 16384); increase "
                        "voxel_size_m or shrink the bounds";
                return false;
            }
            dims[k] = int32_t( std::max( 2.0, n ) );
        }
        g.nx = dims[0];
        g.ny = dims[1];
        g.nz = dims[2];
        g.voxel = p.voxel_size_m;
        for( int k = 0; k < 3; ++k )
            g.origin[k] = p.bounds_min[k];
        const size_t voxels = g.voxel_count();
        const double bytes = double( voxels ) * ( p.integrate_color ? 8.0 : 4.0 );
        const size_t budget = memory_budget_bytes();
        if( voxels > kMaxVoxels || bytes > double( budget ) )
        {
            error = "TSDF volume of " + std::to_string( g.nx ) + "x" + std::to_string( g.ny ) + "x" + std::to_string( g.nz )
                  + " voxels (" + format_bytes( bytes ) + ") exceeds the limit of " + std::to_string( kMaxVoxels / 1000000 )
                  + "M voxels / " + format_bytes( double( budget ) ) + "; increase voxel_size_m or shrink the bounds";
            return false;
        }
        return true;
    }
    return false;
}

inline void integrate_voxel( half_t * tw, uint8_t * colors, size_t idx, float px, float py, float pz,
                             const IntegrationConstants & c, const PreparedFrame & f )
{
    const float * m = c.camera_from_world;
    const float x = m[0] * px + m[4] * py + m[8] * pz + m[12];
    const float y = m[1] * px + m[5] * py + m[9] * pz + m[13];
    const float z = m[2] * px + m[6] * py + m[10] * pz + m[14];
    if( z <= 0.f )
        return;
    const float u = c.fx * x / z + c.cx;
    const float v = c.fy * y / z + c.cy;
    const float fu = std::floor( u + 0.5f ), fv = std::floor( v + 0.5f );
    if( ! ( fu >= 0.f && fv >= 0.f && fu < float( c.width ) && fv < float( c.height ) ) )
        return;
    const int iu = int( fu ), iv = int( fv );
    const int i = iv * c.width + iu;
    float d = f.depth[size_t( i )];
    float w = f.weight[size_t( i )];
    if( d <= 0.f || w <= 0.f )
        return;
    // Bilinear depth when the four neighbors lie on the same surface: voxels are smaller than the pixel footprint.
    const float bu = std::floor( u ), bv = std::floor( v );
    if( bu >= 0.f && bv >= 0.f && bu + 1.f < float( c.width ) && bv + 1.f < float( c.height ) )
    {
        const int j = int( bv ) * c.width + int( bu );
        const float d00 = f.depth[size_t( j )], d10 = f.depth[size_t( j + 1 )];
        const float d01 = f.depth[size_t( j + c.width )], d11 = f.depth[size_t( j + c.width + 1 )];
        const float w00 = f.weight[size_t( j )], w10 = f.weight[size_t( j + 1 )];
        const float w01 = f.weight[size_t( j + c.width )], w11 = f.weight[size_t( j + c.width + 1 )];
        const float lo = std::min( std::min( d00, d10 ), std::min( d01, d11 ) );
        const float hi = std::max( std::max( d00, d10 ), std::max( d01, d11 ) );
        const float wlo = std::min( std::min( w00, w10 ), std::min( w01, w11 ) );
        if( lo > 0.f && wlo > 0.f && hi - lo <= c.depth_jump )
        {
            const float ax = u - bu, ay = v - bv;
            d = ( d00 * ( 1.f - ax ) + d10 * ax ) * ( 1.f - ay ) + ( d01 * ( 1.f - ax ) + d11 * ax ) * ay;
            w = ( w00 * ( 1.f - ax ) + w10 * ax ) * ( 1.f - ay ) + ( w01 * ( 1.f - ax ) + w11 * ax ) * ay;
        }
    }
    const float sdf = d - z;
    if( sdf < -c.truncation )
        return;
    const float tsdf = std::min( 1.f, sdf * c.inv_truncation );
    half_t * cell = tw + 2 * idx;
    const float wold = float( cell[1] );
    const float dold = float( cell[0] );
    const float wnew = wold + w;
    cell[0] = half_t( ( dold * wold + tsdf * w ) / wnew );
    cell[1] = half_t( std::min( wnew, c.max_weight ) );
    if( c.integrate_color && c.has_rgb && colors && std::abs( sdf ) < c.truncation )
    {
        uint8_t * cc = colors + 4 * idx;
        const float n = float( cc[3] );
        const uint8_t * rgb = f.rgb + 3 * size_t( i );
        for( int k = 0; k < 3; ++k )
            cc[k] = uint8_t( ( float( cc[k] ) * n + float( rgb[k] ) ) / ( n + 1.f ) + 0.5f );
        cc[3] = uint8_t( std::min( n + 1.f, 255.f ) );
    }
}
}  // namespace

PreparedFrame prepare_frame( const fr_rgbd_frame & frame, const fr_tsdf_params & p )
{
    PreparedFrame f;
    const int w = frame.intrinsics.width, h = frame.intrinsics.height;
    f.width = w;
    f.height = h;
    f.rgb = frame.rgb;
    const size_t n = size_t( w ) * size_t( h );
    f.depth.assign( n, 0.f );
    f.weight.assign( n, 0.f );
    for( size_t i = 0; i < n; ++i )
    {
        const uint16_t raw = frame.depth[i];
        if( raw == 0 || ( frame.mask && frame.mask[i] < 128 ) )
            continue;
        const float d = float( raw ) * frame.depth_scale;
        if( d >= p.min_depth_m && d <= p.max_depth_m )
            f.depth[i] = d;
    }
    const float frame_weight = frame.weight > 0 && std::isfinite( frame.weight ) ? std::min( frame.weight, 1.f ) : 1.f;
    const float fx = frame.intrinsics.fx, fy = frame.intrinsics.fy, cx = frame.intrinsics.cx, cy = frame.intrinsics.cy;
    const float jump = std::max( 0.01f, 2.f * p.truncation_m );
    parallel_chunks( size_t( h ), chunk_count( size_t( h ), 8 ), [&]( size_t y0, size_t y1, size_t ) {
        for( size_t yy = y0; yy < y1; ++yy )
        {
            const int y = int( yy );
            for( int x = 0; x < w; ++x )
            {
                const size_t i = size_t( y ) * size_t( w ) + size_t( x );
                const float d = f.depth[i];
                if( d <= 0.f )
                    continue;
                if( p.weight_mode == 0 )
                {
                    f.weight[i] = frame_weight;
                    continue;
                }
                auto point = [&]( int px, int py ) -> Eigen::Vector3f {
                    const float z = f.depth[size_t( py ) * size_t( w ) + size_t( px )];
                    return { ( float( px ) - cx ) / fx * z, ( float( py ) - cy ) / fy * z, z };
                };
                auto usable = [&]( int px, int py ) {
                    if( px < 0 || py < 0 || px >= w || py >= h )
                        return false;
                    const float z = f.depth[size_t( py ) * size_t( w ) + size_t( px )];
                    return z > 0.f && std::abs( z - d ) < jump;
                };
                // Tangents from the farthest usable neighbors within the radius on each side (one-sided at edges).
                auto tangent = [&]( int dx, int dy, Eigen::Vector3f & t ) {
                    int a = 0, b = 0;
                    for( int r = kNormalRadiusPx; r >= 1 && ! a; --r )
                        if( usable( x + r * dx, y + r * dy ) )
                            a = r;
                    for( int r = kNormalRadiusPx; r >= 1 && ! b; --r )
                        if( usable( x - r * dx, y - r * dy ) )
                            b = r;
                    if( ! a && ! b )
                        return false;
                    t = point( x + a * dx, y + a * dy ) - point( x - b * dx, y - b * dy );
                    return true;
                };
                Eigen::Vector3f tu, tv;
                if( ! tangent( 1, 0, tu ) || ! tangent( 0, 1, tv ) )
                    continue;   // isolated pixel: most likely noise
                const Eigen::Vector3f nrm = tu.cross( tv );
                const Eigen::Vector3f ray( ( float( x ) - cx ) / fx, ( float( y ) - cy ) / fy, 1.f );
                const float denom = nrm.norm() * ray.norm();
                if( ! ( denom > 0 ) )
                    continue;
                const float cos_view = std::max( 0.02f, std::abs( nrm.dot( ray ) ) / denom );
                f.weight[i] = frame_weight * cos_view * ( kReferenceDepth * kReferenceDepth ) / ( d * d );
            }
        }
    } );
    return f;
}

IntegrationConstants make_constants( const VolumeGeometry & g, const fr_tsdf_params & p, const PreparedFrame & frame,
                                     const fr_rgbd_frame & raw, const Eigen::Matrix4d & world_from_camera )
{
    IntegrationConstants c {};
    store_matrix4( rigid_inverse( world_from_camera ), c.camera_from_world );
    c.fx = raw.intrinsics.fx;
    c.fy = raw.intrinsics.fy;
    c.cx = raw.intrinsics.cx;
    c.cy = raw.intrinsics.cy;
    c.origin_x = g.origin[0];
    c.origin_y = g.origin[1];
    c.origin_z = g.origin[2];
    c.voxel = g.voxel;
    c.truncation = p.truncation_m;
    c.inv_truncation = 1.f / p.truncation_m;
    c.max_weight = p.max_weight;
    c.depth_jump = std::max( 0.5f * p.truncation_m, 2.f * g.voxel );
    c.nx = g.nx;
    c.ny = g.ny;
    c.nz = g.nz;
    c.width = frame.width;
    c.height = frame.height;
    c.integrate_color = p.integrate_color ? 1 : 0;
    c.has_rgb = frame.rgb ? 1 : 0;
    return c;
}

void integrate_cpu( half_t * tw, uint8_t * colors, const IntegrationConstants & c, const PreparedFrame & f )
{
    const size_t slices = size_t( c.nz );
    parallel_for( slices, [&]( size_t zi ) {
        const float pz = c.origin_z + c.voxel * float( zi );
        for( int y = 0; y < c.ny; ++y )
        {
            const float py = c.origin_y + c.voxel * float( y );
            const size_t row = size_t( c.nx ) * ( size_t( y ) + size_t( c.ny ) * zi );
            for( int x = 0; x < c.nx; ++x )
                integrate_voxel( tw, colors, row + size_t( x ), c.origin_x + c.voxel * float( x ), py, pz, c, f );
        }
    } );
}
}  // namespace fr

extern "C" {

void fr_tsdf_params_default( fr_tsdf_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->voxel_size_m = 0.00075f;
    p->truncation_m = 0.004f;
    p->min_depth_m = 0.15f;
    p->max_depth_m = 1.5f;
    p->max_weight = 64.f;
    p->use_gpu = 1;
    p->integrate_color = 1;
    p->weight_mode = 1;
}

fr_status fr_tsdf_create( const fr_tsdf_params * params, fr_tsdf ** out )
{
    return fr::guarded( "fr_tsdf_create", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_create: out is NULL" );
        *out = nullptr;
        if( ! params )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_create: params is NULL (bounds are required)" );
        auto tsdf = std::make_unique< fr_tsdf >();
        tsdf->params = *params;
        std::string error;
        if( ! fr::validate_tsdf_params( tsdf->params, tsdf->geometry, error ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_create: " + error );
        const size_t voxels = tsdf->geometry.voxel_count();
        const bool color = tsdf->params.integrate_color != 0;
        if( tsdf->params.use_gpu )
        {
            tsdf->gpu = fr::metal::create_volume( voxels, color, error );
            if( tsdf->gpu )
            {
                tsdf->tsdf_weight = fr::metal::tsdf_weight( tsdf->gpu.get() );
                tsdf->colors = fr::metal::colors( tsdf->gpu.get() );
            }
        }
        if( ! tsdf->gpu )
        {
            // calloc of large blocks maps zero pages lazily: allocation is cheap until the voxels are touched.
            tsdf->cpu_storage.reset( std::calloc( voxels, 2 * sizeof( fr::half_t ) ) );
            if( ! tsdf->cpu_storage )
                return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_tsdf_create: cannot allocate the volume" );
            tsdf->tsdf_weight = static_cast< fr::half_t * >( tsdf->cpu_storage.get() );
            if( color )
            {
                tsdf->cpu_colors.reset( std::calloc( voxels, 4 ) );
                if( ! tsdf->cpu_colors )
                    return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_tsdf_create: cannot allocate the color volume" );
                tsdf->colors = static_cast< uint8_t * >( tsdf->cpu_colors.get() );
            }
        }
        *out = tsdf.release();
        return FR_OK;
    } );
}

fr_status fr_tsdf_integrate( fr_tsdf * tsdf, const fr_rgbd_frame * frame, const float world_from_camera[16] )
{
    return fr::guarded( "fr_tsdf_integrate", [&]() -> fr_status {
        if( ! tsdf || ! frame || ! world_from_camera )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_integrate: NULL argument" );
        std::string error;
        if( ! fr::validate_frame( *frame, error ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_integrate: " + error );
        if( ! fr::all_finite( world_from_camera, 16 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_integrate: pose is not finite" );
        Eigen::Matrix4d pose = fr::load_matrix4( world_from_camera );
        pose.block< 3, 3 >( 0, 0 ) = fr::orthonormalize( pose.block< 3, 3 >( 0, 0 ) );
        pose.row( 3 ) << 0, 0, 0, 1;
        const fr::PreparedFrame prepared = fr::prepare_frame( *frame, tsdf->params );
        const fr::IntegrationConstants c = fr::make_constants( tsdf->geometry, tsdf->params, prepared, *frame, pose );
        if( tsdf->gpu )
            return fr::metal::integrate( tsdf->gpu.get(), c, prepared );
        fr::integrate_cpu( tsdf->tsdf_weight, tsdf->colors, c, prepared );
        return FR_OK;
    } );
}

fr_status fr_tsdf_extract_mesh( fr_tsdf * tsdf, float min_weight, fr_mesh ** out )
{
    return fr::guarded( "fr_tsdf_extract_mesh", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_extract_mesh: out is NULL" );
        *out = nullptr;
        if( ! tsdf )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_extract_mesh: tsdf is NULL" );
        if( ! std::isfinite( min_weight ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_tsdf_extract_mesh: min_weight is not finite" );
        // Weight 0 means "never observed"; such voxels never take part, whatever min_weight says.
        const float threshold = std::max( min_weight, 1e-6f );
        return fr::marching_cubes( tsdf->geometry, tsdf->tsdf_weight, tsdf->params.integrate_color ? tsdf->colors : nullptr,
                                   threshold, out );
    } );
}

int32_t fr_tsdf_uses_gpu( const fr_tsdf * tsdf )
{
    return tsdf && tsdf->gpu ? 1 : 0;
}

void fr_tsdf_free( fr_tsdf * tsdf )
{
    delete tsdf;
}

fr_status fr_fuse_frames( const fr_rgbd_frame * frames, const float * world_from_camera, uint32_t count,
                          const fr_tsdf_params * params, float min_weight, fr_mesh ** out, fr_progress_cb progress,
                          void * user )
{
    return fr::guarded( "fr_fuse_frames", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_fuse_frames: out is NULL" );
        *out = nullptr;
        if( ! frames || ! world_from_camera || count == 0 )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_fuse_frames: no frames" );
        fr_tsdf * raw = nullptr;
        fr_status status = fr_tsdf_create( params, &raw );
        if( status != FR_OK )
            return status;
        std::unique_ptr< fr_tsdf, void ( * )( fr_tsdf * ) > tsdf( raw, fr_tsdf_free );
        for( uint32_t i = 0; i < count; ++i )
        {
            if( ! fr::report( progress, user, 0.9f * float( i ) / float( count ), "integrating frames" ) )
                return fr::fail( FR_ERR_CANCELLED, "fr_fuse_frames: cancelled" );
            status = fr_tsdf_integrate( tsdf.get(), &frames[i], world_from_camera + 16 * size_t( i ) );
            if( status != FR_OK )
                return status;
        }
        if( ! fr::report( progress, user, 0.9f, "extracting mesh" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_fuse_frames: cancelled" );
        status = fr_tsdf_extract_mesh( tsdf.get(), min_weight, out );
        if( status != FR_OK )
            return status;
        fr::report( progress, user, 1.f, "fusion finished" );
        return FR_OK;
    } );
}

}  // extern "C"
