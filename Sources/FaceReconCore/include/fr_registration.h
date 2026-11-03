// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - point clouds, rigid registration (point-to-plane / colored ICP) and pose graph optimization.
// Owner: reconstruction module (docs/ARCHITECTURE.md). See fr_types.h for conventions.

#ifndef FR_REGISTRATION_H
#define FR_REGISTRATION_H

#include "fr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Point clouds ------------------------------------------------------------------------------------------ */

typedef struct fr_cloud fr_cloud;   /* points + normals + colors (+ intensity gradients for colored ICP) */

typedef struct fr_cloud_params {
    float min_depth_m, max_depth_m;   /* depth range kept (after the mask) */
    int32_t stride;                   /* pixel subsampling, 1 = every pixel */
    int32_t normal_radius_px;         /* neighborhood (in pixels) for normals estimated on the depth image */
    float max_view_angle_deg;         /* drop grazing samples: angle between normal and viewing ray */
    int32_t mask_erode_px;            /* erode the mask/depth boundary (removes flying pixels at silhouettes) */
} fr_cloud_params;

void fr_cloud_params_default(fr_cloud_params *params);

/* Back-projects a frame into a cloud in its camera frame (masked, range-limited, with normals and colors). */
fr_status fr_cloud_from_rgbd(const fr_rgbd_frame *frame, const fr_cloud_params *params, fr_cloud **out);
/* Creates a cloud from arrays (normals and colors may be NULL; normals are then estimated with k-NN). */
fr_status fr_cloud_create(const float *points, const float *normals, const uint8_t *colors, uint32_t count,
                          fr_cloud **out);
fr_status fr_cloud_voxel_downsample(const fr_cloud *cloud, float voxel_size_m, fr_cloud **out);
fr_status fr_cloud_transform(fr_cloud *cloud, const float transform[16]);
/* Appends 'other' (already in the same frame) to 'cloud'. */
fr_status fr_cloud_append(fr_cloud *cloud, const fr_cloud *other);
uint32_t fr_cloud_size(const fr_cloud *cloud);
const float *fr_cloud_points(const fr_cloud *cloud);    /* 3 * size, valid until the cloud is modified/freed */
const float *fr_cloud_normals(const fr_cloud *cloud);   /* 3 * size or NULL */
const uint8_t *fr_cloud_colors(const fr_cloud *cloud);  /* 3 * size or NULL */
/* Axis-aligned bounds of the points. */
fr_status fr_cloud_bounds(const fr_cloud *cloud, float min[3], float max[3]);
void fr_cloud_free(fr_cloud *cloud);

/* ---- ICP ----------------------------------------------------------------------------------------------------- */

typedef struct fr_icp_params {
    int32_t levels;                       /* coarse-to-fine pyramid levels (voxel size doubles per level) */
    int32_t max_iterations;               /* per level */
    float voxel_size_m;                   /* finest level downsampling; 0 = no downsampling */
    float max_correspondence_m;           /* finest level correspondence distance (scaled with the level) */
    int32_t colored;                      /* 1 = colored ICP (Park et al. 2017), 0 = point-to-plane */
    float color_lambda;                   /* colored ICP: weight of the geometric term, (0, 1], e.g. 0.968 */
    int32_t robust_kernel;                /* 0 none, 1 Huber, 2 Tukey */
    float robust_scale_m;                 /* kernel scale */
    float convergence_rotation_deg;       /* stop when the update is below both thresholds */
    float convergence_translation_m;
} fr_icp_params;

typedef struct fr_icp_result {
    float target_from_source[16];
    float fitness;                        /* inlier correspondences / source points, [0, 1] */
    float inlier_rmse_m;
    uint32_t correspondences;
    int32_t iterations;
    int32_t converged;
    double information[36];               /* 6x6 information matrix (column-major, [rx ry rz tx ty tz]) */
} fr_icp_result;

void fr_icp_params_default(fr_icp_params *params);

/* Aligns source onto target starting from init (target_from_source). Point-to-plane needs target normals. */
fr_status fr_icp(const fr_cloud *source, const fr_cloud *target, const float init_target_from_source[16],
                 const fr_icp_params *params, fr_icp_result *out);

/* ---- Pose graph ----------------------------------------------------------------------------------------------- */

typedef struct fr_pose_graph fr_pose_graph;

typedef struct fr_pose_graph_params {
    int32_t max_iterations;               /* Levenberg-Marquardt iterations */
    float edge_prune_threshold;           /* line-process weight below which uncertain edges are dropped */
    float loop_closure_preference;        /* prior weight of uncertain (loop-closure) edges */
    int32_t reference_node;               /* node kept fixed (gauge), usually 0 = first frontal frame */
} fr_pose_graph_params;

typedef struct fr_pose_graph_report {
    int32_t iterations;
    float initial_error;
    float final_error;
    uint32_t pruned_edges;
} fr_pose_graph_report;

void fr_pose_graph_params_default(fr_pose_graph_params *params);
fr_status fr_pose_graph_create(fr_pose_graph **out);
/* Returns the new node index. */
uint32_t fr_pose_graph_add_node(fr_pose_graph *graph, const float world_from_node[16]);
/* Edge measurement: target_from_source with its 6x6 information matrix. uncertain = 1 for loop closures (robust,
   may be pruned), 0 for odometry edges between consecutive frames. */
fr_status fr_pose_graph_add_edge(fr_pose_graph *graph, uint32_t source, uint32_t target,
                                 const float target_from_source[16], const double information[36],
                                 int32_t uncertain);
fr_status fr_pose_graph_optimize(fr_pose_graph *graph, const fr_pose_graph_params *params,
                                 fr_pose_graph_report *report /* nullable */);
uint32_t fr_pose_graph_node_count(const fr_pose_graph *graph);
fr_status fr_pose_graph_get_node(const fr_pose_graph *graph, uint32_t node, float world_from_node[16]);
void fr_pose_graph_free(fr_pose_graph *graph);

/* ---- Whole-scan registration ------------------------------------------------------------------------------------ */

typedef struct fr_registration_params {
    fr_cloud_params cloud;
    fr_icp_params icp;
    fr_pose_graph_params graph;
    int32_t global_optimization;          /* 1 = pose graph with loop closures after sequential registration */
    float loop_closure_max_angle_deg;     /* try loop-closure edges between frames whose poses differ less */
    float min_fitness;                    /* edges with lower ICP fitness are rejected */
    float max_rmse_m;                     /* edges with higher inlier RMSE are rejected */
} fr_registration_params;

typedef struct fr_registration_report {
    uint32_t frames;
    uint32_t odometry_edges;
    uint32_t loop_closures;
    uint32_t rejected_edges;
    float mean_rmse_m;
    float max_rmse_m;
    float mean_fitness;
    fr_pose_graph_report graph;
} fr_registration_report;

void fr_registration_params_default(fr_registration_params *params);

/* Registers all frames into the world (head) frame: per-frame clouds, sequential ICP initialized from each
   frame's initial pose, loop closures between frames with overlapping views (e.g. left<->front<->right),
   global pose graph optimization. Frame 0 defines the world frame. Writes count poses (16 floats each,
   world_from_camera) into out_world_from_camera. */
fr_status fr_register_frames(const fr_rgbd_frame *frames, uint32_t count, const fr_registration_params *params,
                             float *out_world_from_camera, fr_registration_report *report /* nullable */,
                             fr_progress_cb progress, void *user);

#ifdef __cplusplus
}
#endif

#endif /* FR_REGISTRATION_H */
