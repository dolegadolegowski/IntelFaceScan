// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - accuracy evaluation helpers (tests, synthetic ground truth, depth noise measurements).
// Owner: tests module (docs/ARCHITECTURE.md). See fr_types.h for conventions.

#ifndef FR_EVAL_H
#define FR_EVAL_H

#include "fr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fr_distance_stats {
    double mean_m;       /* mean unsigned point-to-surface distance */
    double rms_m;
    double median_m;
    double p95_m;
    double max_m;        /* Hausdorff (one-sided) */
    uint32_t samples;
} fr_distance_stats;

/* One-sided distances from points sampled on 'from' (samples points, area-uniform) to the surface of 'to'. */
fr_status fr_mesh_distance(const fr_mesh *from, const fr_mesh *to, uint32_t samples, fr_distance_stats *out);

typedef struct fr_plane_fit {
    float normal[3];
    float d;             /* plane: dot(normal, x) + d = 0 */
    double rms_m;        /* RMS of point-to-plane residuals (spatial noise) */
    uint32_t inliers;
} fr_plane_fit;

/* Least-squares plane through points (RANSAC-robust when ransac_threshold_m > 0). */
fr_status fr_fit_plane(const float *points, uint32_t count, float ransac_threshold_m, fr_plane_fit *out);

/* Rigid transform error between two poses: rotation angle (degrees) and translation distance (meters). */
void fr_pose_error(const float a[16], const float b[16], float *rotation_deg, float *translation_m);

#ifdef __cplusplus
}
#endif

#endif /* FR_EVAL_H */
