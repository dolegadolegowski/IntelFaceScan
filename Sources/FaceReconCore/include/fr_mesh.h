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

/* Welds duplicate vertices (within epsilon_m; with UVs only vertices with the same UV), removes degenerate and
   duplicate triangles, then the smallest triangles on every edge that is still used by more than two triangles or
   twice in one direction (welding collapses short edges without regard to the topology: marching-cubes vertices
   near a voxel corner lie within a micrometer of each other), and unreferenced vertices. The result has no
   non-manifold edges; the removed triangles leave small holes. */
fr_status fr_mesh_clean(fr_mesh *mesh, float epsilon_m);

/* Removes connected components whose triangle count is below min_relative_size * (largest component).
   keep_largest_only = 1 keeps just the largest component. Triangles are connected through shared edges: a piece
   that touches the rest only at a vertex (a flap along a ragged border) is a component of its own. Vertices split
   on UV seams (same position) count as one. */
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

typedef struct fr_fill_params {
    float max_hole_perimeter_m;  /* holes whose boundary loop is at most this long are filled, e.g. 0.25; 0 = any.
                                    Ignored with close_largest (a watertight result fills every loop). */
    uint32_t max_hole_edges;     /* safety cap on the boundary edges of a filled hole, e.g. 4096 */
    int32_t close_largest;       /* non-zero: also close the longest boundary loop (on a head scan the open back of the
                                    head joined with the neck cut) into a watertight surface; 0 leaves it open */
    float closing_edge_m;        /* edge length inside that closing surface (graded up from the loop's own edges);
                                    0 = automatic: 4 x the mean edge length, within 1-5 mm */
} fr_fill_params;

typedef struct fr_fill_report {
    uint32_t filled_holes;       /* loops filled, not counting the closed longest loop */
    uint32_t rejected_holes;     /* loops within the limits whose patch was discarded (folded, or not manifold) */
    uint32_t open_loops;         /* boundary loops left open (0: watertight) */
    uint32_t closed_largest;     /* 1 when the longest loop was closed */
    uint32_t first_new_vertex;   /* vertices from this index on were created by the filling: synthesized, not
                                    measured surface. The others keep their order (trimming may drop some). */
    uint32_t folded_triangles;   /* filling triangles (and measured ones at a filled border) still folded against the
                                    surrounding surface; only with close_largest, which fills a hole anyway */
    uint32_t trimmed_triangles;  /* fringe triangles removed along the longest loop before closing it */
    float filled_area_m2;        /* area of all filling triangles */
    float closing_area_m2;       /* the part of it that closes the longest loop */
} fr_fill_report;

void fr_fill_params_default(fr_fill_params *params);

/* Fills boundary loops within the limits: minimal-area / maximal-dihedral triangulation (Liepa), refined to the
   density of the surrounding mesh and faired (thin plate: the patch continues the surface's curvature across the
   border). A patch that folds against the surrounding surface (a triangle whose normal points against the summed
   normals of its corners) is flipped and faired again, smoothed locally, then replaced by the unrefined
   triangulation, and otherwise discarded (without close_largest). The longest loop stays open unless close_largest
   is set. Closing it (for a head scan: a watertight bust) first trims the fusion's ragged fringe along it, then
   builds a graded triangulation in the stereographic plane of the directions around the head center (a sphere
   fitted to the measured band along the loop) and fairs the distance from that center as a thin plate, so the
   closing continues the head's roundness. Where the loop runs along a planar cut (the chin crop) the closing stays
   above the cut's plane and becomes a flat cap there. Without a usable center the closing falls back to a refined
   fan faired in Cartesian coordinates. */
fr_status fr_mesh_fill_holes_ex(fr_mesh *mesh, const fr_fill_params *params, fr_fill_report *report /* nullable */);

/* Fills boundary loops with at most max_hole_edges edges (no length limit) and leaves the longest one open. */
fr_status fr_mesh_fill_holes(fr_mesh *mesh, uint32_t max_hole_edges, uint32_t *filled_holes /* nullable */);

/* Quadric decimation to target_triangles (or until max_error_m is reached); preserves borders and attributes.
   Non-manifold edges the simplifier leaves are repaired (their extra triangles removed, the small holes filled). */
fr_status fr_mesh_decimate(fr_mesh *mesh, uint32_t target_triangles, float max_error_m);

/* Removes the part of the mesh on the negative side of the plane (point, normal); keeps where normal points. */
fr_status fr_mesh_crop_plane(fr_mesh *mesh, const float point[3], const float normal[3]);

fr_status fr_mesh_transform(fr_mesh *mesh, const float transform[16]);
fr_status fr_mesh_bounds(const fr_mesh *mesh, float min[3], float max[3]);

#ifdef __cplusplus
}
#endif

#endif /* FR_MESH_H */
