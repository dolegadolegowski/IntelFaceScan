// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: reconstruction module). Defaults are the documented starting values; every other function
// returns FR_ERR_NOT_IMPLEMENTED until replaced by the real implementation (delete this file then).

#include "fr_fusion.h"
#include "../common/fr_internal.h"

#include <cstring>

extern "C" {

void fr_tsdf_params_default( fr_tsdf_params * p )
{
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

void fr_poisson_params_default( fr_poisson_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->depth = 10;
    p->point_weight = 4.f;
    p->samples_per_node = 1.5f;
    p->density_trim_quantile = 0.02f;
    p->threads = 0;
}

fr_status fr_tsdf_create( const fr_tsdf_params *, fr_tsdf ** out )
{ if( out ) *out = nullptr; return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_tsdf_create" ); }
fr_status fr_tsdf_integrate( fr_tsdf *, const fr_rgbd_frame *, const float[16] )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_tsdf_integrate" ); }
fr_status fr_tsdf_extract_mesh( fr_tsdf *, float, fr_mesh ** )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_tsdf_extract_mesh" ); }
int32_t fr_tsdf_uses_gpu( const fr_tsdf * ) { return 0; }
void fr_tsdf_free( fr_tsdf * ) {}

fr_status fr_fuse_frames( const fr_rgbd_frame *, const float *, uint32_t, const fr_tsdf_params *, float, fr_mesh **,
                          fr_progress_cb, void * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_fuse_frames" ); }

fr_status fr_poisson_reconstruct( const fr_cloud *, const fr_poisson_params *, fr_mesh **, fr_progress_cb, void * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_poisson_reconstruct" ); }

}  // extern "C"
