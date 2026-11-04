// License: Apache 2.0. See LICENSE file in root directory.
//
// View selection shared by texture baking and vertex coloring: per-frame visibility (the mesh rendered into each
// frame plus the frame's own depth and mask), view weights with feathering, color compensation (a global camera
// gain per frame plus a smooth local correction where views overlap) and the fallback colors for surface no frame
// sees.

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
    float gain[3] = { 1, 1, 1 };     // global color compensation (camera exposure / white balance) of the sampled color
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

// Smooth per-view log color correction, trilinear over the nodes of a coarse grid (estimate_gains): fitted where
// views overlap, fading to zero away from the overlaps.
struct correction_field
{
    vec3 origin;
    float cell = 0;
    int nodes[3] = { 0, 0, 0 };  // grid nodes per axis
    uint32_t view_count = 0;
    std::vector< int32_t > slot;  // per grid node: its index among the fitted nodes, -1 = none
    std::vector< float > value;   // per fitted node and view: log correction r, g, b and 1 where fitted (else 0)

    bool empty() const { return value.empty(); }
    // The 8 nodes around p (slot, -1 = none) and their trilinear weights; false outside the grid.
    bool corners( const vec3 & p, int32_t slots[8], float weights[8] ) const;
    // Log correction of `view` at the point with these corners, renormalized over the nodes fitted for the view (a
    // point at the edge of the region the view was sampled in can have corners outside it); 0 where none is.
    void at( const int32_t slots[8], const float weights[8], uint32_t view, float out[3] ) const;
};

struct view_set
{
    std::vector< frame_view > views;
    correction_field field;
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

// Color compensation (docs/RECONSTRUCTION.md, "Texturing"): a global per-frame camera gain (from the static
// background when the camera stood still, else one brightness gain per frame from the shared surface points) plus
// a smooth per-view correction field that makes overlapping views agree and fades out away from the overlaps.
// synthesized (nullable, per vertex): surface that was not measured, left out.
void estimate_gains( const float * positions, const float * normals, uint32_t vertex_count, const uint8_t * synthesized,
                     view_set & views );

// Blended vertex colors (0..255). Vertices no frame sees, and synthesized ones (nullable flags, per vertex; never
// colored from the frames), get a smooth (harmonic) interpolation of the surrounding seen colors, whose border values
// are blurred over a few rings first. Returns false when no vertex is seen at all (then every vertex is mid gray).
bool vertex_colors( const fr_mesh & mesh, const float * normals, const view_set & views, const uint8_t * synthesized,
                    std::vector< float > & rgb );

// Validates fr_texture_params.
fr_status check_params( const fr_texture_params * params, const char * where, bool need_texture_size );

}  // namespace texture
}  // namespace fr
