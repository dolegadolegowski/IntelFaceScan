// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - volumetric fusion (TSDF on the GPU with Metal, CPU fallback), marching cubes, and screened
// Poisson surface reconstruction for comparison.
// Owner: reconstruction module (docs/ARCHITECTURE.md). See fr_types.h for conventions.

#ifndef FR_FUSION_H
#define FR_FUSION_H

#include "fr_types.h"
#include "fr_registration.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fr_tsdf fr_tsdf;

typedef struct fr_tsdf_params {
    float voxel_size_m;          /* target 0.0005 - 0.001; choose >= ~1/2 of the depth noise at working distance */
    float truncation_m;          /* signed distance truncation, typically 4-6 voxels and >= 3x depth noise */
    float min_depth_m, max_depth_m;
    float bounds_min[3];         /* world-space AABB of the volume (meters), e.g. registered head cloud bounds */
    float bounds_max[3];         /*   plus a margin; required */
    float max_weight;            /* cap of the running weight (allows slow adaptation), e.g. 64 */
    int32_t use_gpu;             /* 1 = Metal compute (CPU fallback when no device), 0 = CPU */
    int32_t integrate_color;     /* 1 = also fuse RGB per voxel (vertex colors of the extracted mesh) */
    int32_t weight_mode;         /* 0 = constant, 1 = noise-aware: cos(view angle) / z^2 */
} fr_tsdf_params;

void fr_tsdf_params_default(fr_tsdf_params *params);
fr_status fr_tsdf_create(const fr_tsdf_params *params, fr_tsdf **out);
/* Integrates one frame (its mask is honored) at the given pose. */
fr_status fr_tsdf_integrate(fr_tsdf *tsdf, const fr_rgbd_frame *frame, const float world_from_camera[16]);
/* Marching cubes over voxels with weight >= min_weight. Output has normals (and colors when integrated). */
fr_status fr_tsdf_extract_mesh(fr_tsdf *tsdf, float min_weight, fr_mesh **out);
/* 1 if this volume runs on the GPU. */
int32_t fr_tsdf_uses_gpu(const fr_tsdf *tsdf);
void fr_tsdf_free(fr_tsdf *tsdf);

/* Convenience: integrates all frames at their poses and extracts the mesh. */
fr_status fr_fuse_frames(const fr_rgbd_frame *frames, const float *world_from_camera, uint32_t count,
                         const fr_tsdf_params *params, float min_weight, fr_mesh **out,
                         fr_progress_cb progress, void *user);

typedef struct fr_poisson_params {
    int32_t depth;               /* octree depth, e.g. 10-11 for sub-millimeter detail on a head */
    float point_weight;          /* screening weight, e.g. 4 */
    float samples_per_node;      /* e.g. 1.5 */
    float density_trim_quantile; /* remove the lowest-density fraction of vertices (hallucinated surface), e.g. 0.02 */
    int32_t threads;             /* 0 = all cores */
} fr_poisson_params;

void fr_poisson_params_default(fr_poisson_params *params);
/* Screened Poisson reconstruction of an oriented cloud (normals required); colors are interpolated when present. */
fr_status fr_poisson_reconstruct(const fr_cloud *cloud, const fr_poisson_params *params, fr_mesh **out,
                                 fr_progress_cb progress, void *user);

#ifdef __cplusplus
}
#endif

#endif /* FR_FUSION_H */
