// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: mesh/texture/export module). Defaults are the documented starting values; every other
// function returns FR_ERR_NOT_IMPLEMENTED until replaced by the real implementation (delete this file then).

#include "fr_mesh.h"
#include "fr_texture.h"
#include "fr_export.h"
#include "../common/fr_internal.h"

#include <cstring>

extern "C" {

void fr_smooth_params_default( fr_smooth_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->taubin_iterations = 5;
    p->taubin_lambda = 0.5f;
    p->taubin_mu = -0.53f;
    p->bilateral_iterations = 2;
    p->bilateral_sigma_spatial_m = 0.0015f;
    p->bilateral_sigma_normal = 0.3f;
}

void fr_uv_params_default( fr_uv_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->texture_size = 4096;
    p->padding_px = 4;
}

void fr_texture_params_default( fr_texture_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->texture_size = 4096;
    p->view_angle_power = 4.f;
    p->min_cos_angle = 0.2f;
    p->blend_views = 3;
    p->occlusion_epsilon_m = 0.004f;
    p->dilate_px = 8;
    p->exposure_compensation = 1;
}

void fr_export_options_default( fr_export_options * o )
{
    std::memset( o, 0, sizeof( *o ) );
    o->scale = 1000.f;
    for( int i = 0; i < 4; ++i )
        o->transform[i * 5] = 1.f;
    o->write_normals = 1;
    o->write_colors = 1;
    o->write_uvs = 1;
}

fr_status fr_mesh_compute_normals( fr_mesh * ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_compute_normals" ); }
fr_status fr_mesh_clean( fr_mesh *, float ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_clean" ); }
fr_status fr_mesh_remove_small_components( fr_mesh *, float, int32_t, uint32_t * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_remove_small_components" ); }
fr_status fr_mesh_smooth( fr_mesh *, const fr_smooth_params * ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_smooth" ); }
fr_status fr_mesh_fill_holes( fr_mesh *, uint32_t, uint32_t * ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_fill_holes" ); }
fr_status fr_mesh_decimate( fr_mesh *, uint32_t, float ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_decimate" ); }
fr_status fr_mesh_crop_plane( fr_mesh *, const float[3], const float[3] ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_crop_plane" ); }
fr_status fr_mesh_transform( fr_mesh *, const float[16] ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_transform" ); }
fr_status fr_mesh_bounds( const fr_mesh *, float[3], float[3] ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_bounds" ); }

fr_status fr_mesh_uv_unwrap( fr_mesh *, const fr_uv_params *, fr_progress_cb, void * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_uv_unwrap" ); }
fr_status fr_bake_texture( const fr_mesh *, const fr_rgbd_frame *, const float *, const float *, uint32_t,
                           const fr_texture_params *, fr_texture **, fr_progress_cb, void * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_bake_texture" ); }
fr_status fr_colorize_vertices( fr_mesh *, const fr_rgbd_frame *, const float *, const float *, uint32_t,
                                const fr_texture_params * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_colorize_vertices" ); }

fr_status fr_export_obj( const fr_mesh *, const fr_texture *, const char *, const fr_export_options * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_export_obj" ); }
fr_status fr_export_ply( const fr_mesh *, const char *, const fr_export_options * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_export_ply" ); }
fr_status fr_import_obj( const char *, fr_mesh ** ) { return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_import_obj" ); }
fr_status fr_write_png( const char *, const uint8_t *, int32_t, int32_t, int32_t )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_write_png" ); }

}  // extern "C"
