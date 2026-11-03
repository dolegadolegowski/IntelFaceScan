// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - common types of the C API. The library is C++ / Objective-C++ (Metal) inside; its public
// interface is plain C so that Swift imports it without C++ interop.
//
// Conventions (all headers):
//   * Units are meters. Export converts to millimeters (fr_export_options.scale = 1000).
//   * Matrices are 4x4 float[16] in column-major order (same memory layout as simd_float4x4 and Eigen::Matrix4f).
//   * Camera frame = RealSense/OpenCV convention: +X right, +Y down, +Z forward (into the scene).
//   * A frame pose is world_from_camera: it maps points from that frame's camera coordinates to the world (head)
//     frame. The head is the rigid object; the world frame is fixed to the head and equals the camera frame of the
//     first frontal keyframe. (The camera is static and the head moves, which is equivalent to a camera moving
//     around a static head.)
//   * Images are row-major, top-left origin, tightly packed (stride = width * channels).
//   * Depth images are uint16 with a per-frame depth_scale (meters per unit); 0 = no data. Depth given to the
//     library is aligned to the color image, so color and depth share fr_rgbd_frame.intrinsics.
//   * Objects returned through an out pointer are owned by the caller and released with the matching _free.
//   * Functions are thread-safe for distinct objects; one object must not be used from two threads at once.

#ifndef FR_TYPES_H
#define FR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum fr_status {
    FR_OK = 0,
    FR_ERR_INVALID_ARGUMENT = 1,
    FR_ERR_NOT_IMPLEMENTED = 2,
    FR_ERR_NO_GPU = 3,
    FR_ERR_OUT_OF_MEMORY = 4,
    FR_ERR_CANCELLED = 5,
    FR_ERR_IO = 6,
    FR_ERR_NOT_ENOUGH_DATA = 7,   /* e.g. too few points / frames / correspondences */
    FR_ERR_FAILED = 8
} fr_status;

/* Static description of a status code. */
const char *fr_status_string(fr_status status);

/* Human-readable detail of the last error on the calling thread ("" if none). Valid until the next call that
   fails on the same thread. */
const char *fr_last_error(void);

/* Progress callback: fraction in [0, 1], stage is a short static description. Return non-zero to cancel; the
   running function then returns FR_ERR_CANCELLED. May be called from worker threads. */
typedef int (*fr_progress_cb)(void *user, float fraction, const char *stage);

/* Pinhole camera. Distortion coefficients are informational: RealSense D4xx color/depth streams have zero or
   negligible distortion and the library treats the camera as an ideal pinhole. */
typedef struct fr_intrinsics {
    int32_t width, height;
    float fx, fy;   /* focal lengths in pixels */
    float cx, cy;   /* principal point in pixels */
    float coeffs[5];
    int32_t model;  /* 0 none, 1 brown-conrady, 2 inverse brown-conrady (informational) */
} fr_intrinsics;

/* One RGB-D frame. The library never keeps these pointers beyond the call they are passed to. */
typedef struct fr_rgbd_frame {
    const uint16_t *depth;        /* width*height, aligned to color */
    float depth_scale;            /* meters per depth unit (e.g. 0.0001) */
    const uint8_t *rgb;           /* width*height*3, RGB8; may be NULL for geometry-only processing */
    const uint8_t *mask;          /* width*height, 255 = head, 0 = background; NULL = everything valid */
    fr_intrinsics intrinsics;     /* of color and of the aligned depth */
    double timestamp_ms;
    float initial_world_from_camera[16];  /* initial pose guess (e.g. from Vision head pose) */
    int32_t has_initial_pose;             /* 0 = identity / unknown */
    float weight;                 /* quality weight in (0, 1] (sharpness, frontalness); 0 means 1 */
} fr_rgbd_frame;

/* Attribute flags of fr_mesh */
#define FR_MESH_NORMALS 0x1u
#define FR_MESH_COLORS  0x2u
#define FR_MESH_UVS     0x4u

/* Indexed triangle mesh, allocated by the library (fr_mesh_create) and released with fr_mesh_free.
   Arrays are NULL when the attribute is absent. Triangles are counter-clockwise seen from outside. UVs are
   per vertex (fr_mesh_uv_unwrap splits vertices on seams) with (0,0) at the bottom-left of the texture, as in OBJ. */
typedef struct fr_mesh {
    uint32_t vertex_count;
    uint32_t triangle_count;
    float *positions;    /* 3 * vertex_count */
    float *normals;      /* 3 * vertex_count, unit length */
    uint8_t *colors;     /* 3 * vertex_count, RGB8 */
    float *uvs;          /* 2 * vertex_count */
    uint32_t *indices;   /* 3 * triangle_count */
} fr_mesh;

/* RGBA8 texture image, top-left origin, allocated by the library and released with fr_texture_free. */
typedef struct fr_texture {
    int32_t width, height;
    uint8_t *rgba;       /* 4 * width * height */
} fr_texture;

/* Allocates a mesh with uninitialized arrays for the given attributes (FR_MESH_* flags). */
fr_status fr_mesh_create(uint32_t vertex_count, uint32_t triangle_count, uint32_t attributes, fr_mesh **out);
/* Deep copy. */
fr_status fr_mesh_clone(const fr_mesh *mesh, fr_mesh **out);
/* Adds (uninitialized) or removes attribute arrays. */
fr_status fr_mesh_set_attributes(fr_mesh *mesh, uint32_t attributes);
void fr_mesh_free(fr_mesh *mesh);

fr_status fr_texture_create(int32_t width, int32_t height, fr_texture **out);
void fr_texture_free(fr_texture *texture);

#ifdef __cplusplus
}
#endif

#endif /* FR_TYPES_H */
