// License: Apache 2.0. See LICENSE file in root directory.
//
// View selection shared by texture baking and vertex coloring: per-frame visibility (the mesh rendered into each
// frame plus the frame's own depth and mask), view weights with feathering, exposure compensation and the
// fallback colors for surface no frame sees.

#pragma once

#include "fr_texture.h"
#include "../mesh/mesh_internal.h"

#include <atomic>

namespace fr
{
namespace texture
{

using mesh::vec3;

// Most views blended per surface point.
constexpr int max_blend_views = 16;

struct frame_view
{
    const fr_rgbd_frame * frame = nullptr;
    mesh::affine camera_from_world;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    float weight = 1;                // frame quality weight
    float gain[3] = { 1, 1, 1 };     // exposure compensation, applied to the sampled color
    std::vector< uint16_t > zbuffer; // mesh depth in zbuffer_unit meters, 0 = no surface
    std::vector< uint8_t > feather;  // 0..255, falls off near mask/silhouette/image borders and depth jumps
};

constexpr float zbuffer_unit = 0.0001f;

struct sample
{
    float weight;
    float rgb[3];  // 0..255, gain applied
    uint32_t view;
};

struct view_set
{
    std::vector< frame_view > views;
    float min_cos = 0.2f, power = 4, epsilon = 0.004f;
    int blend = 3;

    // Collects the visible views of surface point p with unit normal n (world frame); returns the count, at most
    // max_out, ordered by decreasing weight.
    int evaluate( const vec3 & p, const vec3 & n, sample * out, int max_out ) const;
    // Weighted blend of the best `blend` views; false when no view sees the point.
    bool blend_color( const vec3 & p, const vec3 & n, float rgb[3], float * total_weight = nullptr ) const;
};

struct cancel_state
{
    fr_progress_cb progress = nullptr;
    void * user = nullptr;
    std::mutex mutex;
    std::atomic< bool > cancelled{ false };
    // Thread-safe progress report; returns false once cancelled.
    bool report( float fraction, const char * stage );
};

// Validates the frames and builds one frame_view per frame with color (zbuffer + feather from the mesh).
fr_status prepare_views( const fr_mesh & mesh, const fr_rgbd_frame * frames, const float * world_from_camera,
                         const float * frame_weights, uint32_t count, const fr_texture_params & params, view_set & out,
                         const char * where );

// Least-squares per-frame, per-channel gains from the colors of shared surface points (log domain, robust).
void estimate_gains( const float * positions, const float * normals, uint32_t vertex_count, view_set & views );

// Blended vertex colors (0..255); vertices no frame sees get colors diffused from their seen neighbors.
// Returns false when no vertex is seen at all (then every vertex is mid gray).
bool vertex_colors( const fr_mesh & mesh, const float * normals, const view_set & views, std::vector< float > & rgb );

// Validates fr_texture_params.
fr_status check_params( const fr_texture_params * params, const char * where, bool need_texture_size );

}  // namespace texture
}  // namespace fr
