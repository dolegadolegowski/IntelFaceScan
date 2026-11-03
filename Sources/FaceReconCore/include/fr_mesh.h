// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - mesh post-processing: cleanup, detail-preserving smoothing, hole filling, decimation, cropping.
// Owner: mesh/texture/export module (docs/ARCHITECTURE.md). See fr_types.h for conventions.
// All functions modify the mesh in place and keep attribute arrays consistent (normals are recomputed where
// geometry changes; colors/uvs are carried along or interpolated).

#ifndef FR_MESH_H
#define FR_MESH_H

#include "fr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Area-weighted vertex normals (adds the normals array if missing). */
fr_status fr_mesh_compute_normals(fr_mesh *mesh);

/* Welds duplicate vertices (within epsilon_m), removes degenerate/duplicate triangles and unreferenced vertices. */
fr_status fr_mesh_clean(fr_mesh *mesh, float epsilon_m);

/* Removes connected components whose triangle count is below min_relative_size * (largest component).
   keep_largest_only = 1 keeps just the largest component. */
fr_status fr_mesh_remove_small_components(fr_mesh *mesh, float min_relative_size, int32_t keep_largest_only,
                                          uint32_t *removed_triangles /* nullable */);

typedef struct fr_smooth_params {
    int32_t taubin_iterations;     /* Taubin lambda/mu smoothing (no shrinkage), 0 = off */
    float taubin_lambda;           /* e.g. 0.5 */
    float taubin_mu;               /* e.g. -0.53 */
    int32_t bilateral_iterations;  /* bilateral normal filtering + vertex update (feature preserving), 0 = off */
    float bilateral_sigma_spatial_m;  /* e.g. 1.5 * mean edge length */
    float bilateral_sigma_normal;     /* e.g. 0.3 */
} fr_smooth_params;

void fr_smooth_params_default(fr_smooth_params *params);
fr_status fr_mesh_smooth(fr_mesh *mesh, const fr_smooth_params *params);

/* Fills boundary loops with at most max_hole_edges edges (minimal-area triangulation + fairing). Leaves the
   largest boundary (e.g. the neck cut) open. */
fr_status fr_mesh_fill_holes(fr_mesh *mesh, uint32_t max_hole_edges, uint32_t *filled_holes /* nullable */);

/* Quadric decimation to target_triangles (or until max_error_m is reached); preserves borders and attributes. */
fr_status fr_mesh_decimate(fr_mesh *mesh, uint32_t target_triangles, float max_error_m);

/* Removes the part of the mesh on the negative side of the plane (point, normal); keeps where normal points. */
fr_status fr_mesh_crop_plane(fr_mesh *mesh, const float point[3], const float normal[3]);

fr_status fr_mesh_transform(fr_mesh *mesh, const float transform[16]);
fr_status fr_mesh_bounds(const fr_mesh *mesh, float min[3], float max[3]);

#ifdef __cplusplus
}
#endif

#endif /* FR_MESH_H */
