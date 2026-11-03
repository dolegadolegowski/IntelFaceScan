// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal interface of the volumetric fusion: the dense TSDF volume (shared by the Metal and the CPU paths), the
// per-frame integration constants (byte-compatible with the Metal shader struct) and marching cubes.

#pragma once

#include "fr_fusion.h"
#include "../registration/recon_math.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fr
{
// Voxel (x, y, z) is at index x + nx * (y + ny * z); its center is origin + voxel * (x, y, z).
// Per voxel: half tsdf in [-1, 1] (signed distance / truncation, positive in front of the surface) and half
// weight, interleaved; optionally RGBA8 color where A counts the color observations (saturating at 255).
struct VolumeGeometry
{
    int32_t nx = 0, ny = 0, nz = 0;
    float origin[3] = { 0, 0, 0 };
    float voxel = 0;
    size_t voxel_count() const { return size_t( nx ) * size_t( ny ) * size_t( nz ); }
};

using half_t = _Float16;   // IEEE binary16 on arm64, same layout and rounding as Metal's half

// Per-frame constants of the integration kernel. Plain scalars only, so the C++ and Metal layouts agree (the
// matrix comes first; Metal's float4x4 is 16-byte aligned, and the size is padded to a multiple of 16).
struct alignas( 16 ) IntegrationConstants
{
    float camera_from_world[16];   // column-major
    float fx, fy, cx, cy;
    float origin_x, origin_y, origin_z, voxel;
    float truncation, inv_truncation, max_weight, depth_jump;
    int32_t nx, ny, nz, width;
    int32_t height, integrate_color, has_rgb, pad0;
};
static_assert( sizeof( IntegrationConstants ) == 144, "IntegrationConstants must match the Metal struct" );

// Per-pixel input shared by both paths: depth in meters (0 = unusable: no data, masked out or out of range) and
// the observation weight (frame weight * cos(view angle) * (z_ref / z)^2, or the constant frame weight).
struct PreparedFrame
{
    int32_t width = 0, height = 0;
    std::vector< float > depth;
    std::vector< float > weight;
    const uint8_t * rgb = nullptr;   // borrowed from the fr_rgbd_frame for the duration of the call
};

PreparedFrame prepare_frame( const fr_rgbd_frame & frame, const fr_tsdf_params & params );
IntegrationConstants make_constants( const VolumeGeometry & geometry, const fr_tsdf_params & params,
                                     const PreparedFrame & frame, const fr_rgbd_frame & raw,
                                     const Eigen::Matrix4d & world_from_camera );

// CPU integration (multi-threaded over z slices); the same arithmetic as the Metal kernel.
void integrate_cpu( half_t * tsdf_weight, uint8_t * colors, const IntegrationConstants & c, const PreparedFrame & frame );

// Marching cubes over voxels with weight >= min_weight; vertices are shared (one per crossed voxel edge), normals
// come from the TSDF gradient, colors are interpolated along the edge. Triangles are counter-clockwise seen from
// the positive (outside) side.
fr_status marching_cubes( const VolumeGeometry & geometry, const half_t * tsdf_weight, const uint8_t * colors,
                          float min_weight, fr_mesh ** out );

namespace metal
{
struct Volume;   // GPU volume (defined in tsdf_metal.mm)

struct VolumeDeleter
{
    void operator()( Volume * volume ) const;
};
using VolumePtr = std::unique_ptr< Volume, VolumeDeleter >;

// Allocates the volume in shared-storage Metal buffers (zero tsdf and weight). Returns nullptr and sets error when
// no Metal device is available or the shaders/buffers cannot be created.
VolumePtr create_volume( size_t voxel_count, bool with_color, std::string & error );
half_t * tsdf_weight( Volume * volume );
uint8_t * colors( Volume * volume );   // nullptr without color
// Runs the integration kernel and waits for it (the CPU can read the volume afterwards).
fr_status integrate( Volume * volume, const IntegrationConstants & constants, const PreparedFrame & frame );
}  // namespace metal
}  // namespace fr
