// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests module). Returns FR_ERR_NOT_IMPLEMENTED until replaced (delete this file then).

#include "fr_eval.h"
#include "../common/fr_internal.h"

extern "C" {

fr_status fr_mesh_distance( const fr_mesh *, const fr_mesh *, uint32_t, fr_distance_stats * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_mesh_distance" ); }
fr_status fr_fit_plane( const float *, uint32_t, float, fr_plane_fit * )
{ return fr::fail( FR_ERR_NOT_IMPLEMENTED, "fr_fit_plane" ); }
void fr_pose_error( const float[16], const float[16], float * rotation_deg, float * translation_m )
{
    if( rotation_deg ) *rotation_deg = -1.f;
    if( translation_m ) *translation_m = -1.f;
}

}  // extern "C"
