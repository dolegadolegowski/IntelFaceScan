// License: Apache 2.0. See LICENSE file in root directory.
//
// FaceReconCore - OBJ (+MTL +PNG) and PLY export, OBJ import (tests, ground truth), PNG writing.
// Owner: mesh/texture/export module (docs/ARCHITECTURE.md). See fr_types.h for conventions.

#ifndef FR_EXPORT_H
#define FR_EXPORT_H

#include "fr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fr_export_options {
    float scale;                 /* applied after transform; 1000 = meters -> millimeters */
    float transform[16];         /* world -> export frame (rigid, det = +1), applied first */
    int32_t has_transform;       /* 0 = identity */
    int32_t write_normals;       /* vn */
    int32_t write_colors;        /* OBJ: "v x y z r g b" (0..1, as read by MeshLab/Blender); PLY: uchar rgb */
    int32_t write_uvs;           /* vt + texture (when a texture is given) */
    const char *material_name;   /* e.g. "head" (NULL = "material0") */
    const char *comment;         /* header comment, e.g. app version and units (nullable) */
} fr_export_options;

void fr_export_options_default(fr_export_options *options);

/* Writes <path> (.obj), and when texture != NULL also <stem>.mtl and <stem>.png next to it (mtllib/map_Kd use
   relative file names). */
fr_status fr_export_obj(const fr_mesh *mesh, const fr_texture *texture /* nullable */, const char *obj_path,
                        const fr_export_options *options);
/* Binary little-endian PLY with optional normals, colors and texture coordinates. */
fr_status fr_export_ply(const fr_mesh *mesh, const char *ply_path, const fr_export_options *options);
/* Reads v/vt/vn/f (triangulating polygons, optional vertex colors) - for tests and ground-truth meshes. */
fr_status fr_import_obj(const char *obj_path, fr_mesh **out);
/* 8-bit PNG with 1, 3 or 4 channels, top-left origin. */
fr_status fr_write_png(const char *path, const uint8_t *pixels, int32_t width, int32_t height, int32_t channels);

#ifdef __cplusplus
}
#endif

#endif /* FR_EXPORT_H */
