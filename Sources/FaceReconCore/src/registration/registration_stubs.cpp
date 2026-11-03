// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: reconstruction module). Defaults are the documented starting values; every other function
// returns FR_ERR_NOT_IMPLEMENTED until replaced by the real implementation (delete this file then).

#include "fr_registration.h"
#include "../common/fr_internal.h"

#include <cstring>

extern "C" {

void fr_cloud_params_default( fr_cloud_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->min_depth_m = 0.15f;
    p->max_depth_m = 1.5f;
    p->stride = 1;
    p->normal_radius_px = 3;
    p->max_view_angle_deg = 75.f;
    p->mask_erode_px = 2;
}

void fr_icp_params_default( fr_icp_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->levels = 3;
    p->max_iterations = 30;
    p->voxel_size_m = 0.001f;
    p->max_correspondence_m = 0.004f;
    p->colored = 1;
    p->color_lambda = 0.968f;
    p->robust_kernel = 2;
    p->robust_scale_m = 0.003f;
    p->convergence_rotation_deg = 0.01f;
    p->convergence_translation_m = 1e-5f;
}

void fr_pose_graph_params_default( fr_pose_graph_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    p->max_iterations = 100;
    p->edge_prune_threshold = 0.25f;
    p->loop_closure_preference = 0.1f;
    p->reference_node = 0;
}

void fr_registration_params_default( fr_registration_params * p )
{
    std::memset( p, 0, sizeof( *p ) );
    fr_cloud_params_default( &p->cloud );
    fr_icp_params_default( &p->icp );
    fr_pose_graph_params_default( &p->graph );
    p->global_optimization = 1;
    p->loop_closure_max_angle_deg = 70.f;
    p->min_fitness = 0.3f;
    p->max_rmse_m = 0.003f;
}

fr_status fr_cloud_from_rgbd( const fr_rgbd_frame *, const fr_cloud_params *, fr_cloud ** )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_from_rgbd" ); }
fr_status fr_cloud_create( const float *, const float *, const uint8_t *, uint32_t, fr_cloud ** )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_create" ); }
fr_status fr_cloud_voxel_downsample( const fr_cloud *, float, fr_cloud ** )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_voxel_downsample" ); }
fr_status fr_cloud_transform( fr_cloud *, const float[16] )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_transform" ); }
fr_status fr_cloud_append( fr_cloud *, const fr_cloud * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_append" ); }
uint32_t fr_cloud_size( const fr_cloud * ) { return 0; }
const float * fr_cloud_points( const fr_cloud * ) { return nullptr; }
const float * fr_cloud_normals( const fr_cloud * ) { return nullptr; }
const uint8_t * fr_cloud_colors( const fr_cloud * ) { return nullptr; }
fr_status fr_cloud_bounds( const fr_cloud *, float[3], float[3] )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_cloud_bounds" ); }
void fr_cloud_free( fr_cloud * ) {}

fr_status fr_icp( const fr_cloud *, const fr_cloud *, const float[16], const fr_icp_params *, fr_icp_result * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_icp" ); }

fr_status fr_pose_graph_create( fr_pose_graph ** out )
{ if( out ) *out = nullptr; return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_pose_graph_create" ); }
uint32_t fr_pose_graph_add_node( fr_pose_graph *, const float[16] ) { return 0; }
fr_status fr_pose_graph_add_edge( fr_pose_graph *, uint32_t, uint32_t, const float[16], const double[36], int32_t )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_pose_graph_add_edge" ); }
fr_status fr_pose_graph_optimize( fr_pose_graph *, const fr_pose_graph_params *, fr_pose_graph_report * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_pose_graph_optimize" ); }
uint32_t fr_pose_graph_node_count( const fr_pose_graph * ) { return 0; }
fr_status fr_pose_graph_get_node( const fr_pose_graph *, uint32_t, float[16] )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_pose_graph_get_node" ); }
void fr_pose_graph_free( fr_pose_graph * ) {}

fr_status fr_register_frames( const fr_rgbd_frame *, uint32_t, const fr_registration_params *, float *,
                              fr_registration_report *, fr_progress_cb, void * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_register_frames" ); }

}  // extern "C"
