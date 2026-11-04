// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - UV unwrapping (xatlas), texture baking from the best RGB views with blending, vertex colors.
// Owner: mesh/texture/export module (docs/ARCHITECTURE.md). See fr_types.h for conventions.

#ifndef FR_TEXTURE_H
#define FR_TEXTURE_H

#include "fr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fr_uv_params {
    int32_t texture_size;        /* atlas resolution, e.g. 4096 */
    int32_t padding_px;          /* gutter between charts, e.g. 4 */
    float max_chart_area;        /* xatlas chart options (0 = default) */
    float max_cost;              /* xatlas chart cost (0 = default) */
} fr_uv_params;

void fr_uv_params_default(fr_uv_params *params);
/* Computes a UV atlas; vertices on seams are split (vertex_count grows), positions/normals/colors are copied. */
fr_status fr_mesh_uv_unwrap(fr_mesh *mesh, const fr_uv_params *params, fr_progress_cb progress, void *user);

typedef struct fr_texture_params {
    int32_t texture_size;        /* must match the atlas, e.g. 4096 */
    float view_angle_power;      /* weight = cos(angle between normal and view ray)^power, e.g. 4 */
    float min_cos_angle;         /* views more grazing than this are ignored, e.g. 0.2 */
    int32_t blend_views;         /* blend the N best views per texel (1 = best view only), e.g. 3 */
    float occlusion_epsilon_m;   /* depth test tolerance against each frame's depth map, e.g. 0.004 */
    int32_t dilate_px;           /* fill the atlas gutter by dilation, e.g. 8 */
    int32_t exposure_compensation; /* 1 = per-frame gain normalization before blending */
    /* Nullable, one byte per mesh vertex: non-zero marks surface that was synthesized, not measured (hole filling,
       the closed back of a head; fr_fill_report.first_new_vertex). The frames are not projected onto it (its
       position is a guess); it takes a smooth interpolation of the surrounding measured colors instead. */
    const uint8_t *synthesized;
} fr_texture_params;

void fr_texture_params_default(fr_texture_params *params);

/* Bakes a texture for a mesh with UVs from the frames at their poses. frame_weights (nullable) scale each frame's
   contribution (sharpness / quality). The result is texture_size x texture_size RGBA, top-left origin. */
fr_status fr_bake_texture(const fr_mesh *mesh, const fr_rgbd_frame *frames, const float *world_from_camera,
                          const float *frame_weights, uint32_t count, const fr_texture_params *params,
                          fr_texture **out, fr_progress_cb progress, void *user);

/* Sets per-vertex colors from the frames (same view selection and blending as the texture). */
fr_status fr_colorize_vertices(fr_mesh *mesh, const fr_rgbd_frame *frames, const float *world_from_camera,
                               const float *frame_weights, uint32_t count, const fr_texture_params *params);

#ifdef __cplusplus
}
#endif

#endif /* FR_TEXTURE_H */
