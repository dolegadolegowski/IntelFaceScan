// License: Apache 2.0. See LICENSE file in root directory.
//
// Depth quality measures of the probe (docs/CAMERA.md "Measurement procedure"), on Z16 images of a flat target:
// plane-fit RMS and subpixel RMS (as the RealSense Depth Quality Tool defines them), temporal noise, fill rate,
// median distance and the nearest valid depth. Plain math, no librealsense calls.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ifs
{

struct DepthView
{
    const uint16_t * data = nullptr;
    int width = 0;
    int height = 0;
    size_t stride_bytes = 0;
    float depth_scale = 0.001f;  // meters per unit
    float fx = 0, fy = 0, ppx = 0, ppy = 0;

    uint16_t at( int x, int y ) const
    {
        auto row = reinterpret_cast< const uint8_t * >( data ) + size_t( y ) * stride_bytes;
        return reinterpret_cast< const uint16_t * >( row )[x];
    }
};

struct Roi
{
    int x = 0, y = 0, width = 0, height = 0;
};

// Centered rectangle covering 'fraction' of the width and of the height.
Roi central_roi( int width, int height, double fraction );

struct PlaneFit
{
    double normal[3] = { 0, 0, 1 };  // unit length, oriented toward the camera (normal[2] < 0)
    double offset = 0;               // normal . p + offset = 0
    double rms_m = 0;                // RMS point-to-plane distance of the fitted points
};

// Least-squares plane through 3D points (meters); nullopt for fewer than 3 points or a degenerate set.
// xyz holds x0, y0, z0, x1, y1, z1, ...
std::optional< PlaneFit > fit_plane( const std::vector< double > & xyz );

struct FrameStats
{
    double fill_rate = 0;        // valid pixels in the ROI / ROI pixels
    double full_fill_rate = 0;   // valid pixels in the whole image / image pixels
    double median_m = 0;         // median valid depth in the ROI
    double plane_rms_mm = 0;     // RMS distance of the ROI points to their best-fit plane
    double subpixel_rms = 0;     // the same error expressed as disparity (pixels), needs baseline_m > 0
    double nearest_m = 0;        // 0.5th percentile of all valid depths (robust minimum)
    size_t roi_valid = 0;
};

FrameStats measure_frame( const DepthView & view, const Roi & roi, double baseline_m );

// Per-pixel standard deviation over consecutive frames in a fixed ROI.
class TemporalNoise
{
public:
    explicit TemporalNoise( const Roi & roi );
    void add( const DepthView & view );
    // Median of the per-pixel std (mm) over pixels valid in at least 'min_valid_fraction' of the frames;
    // nullopt before two frames.
    std::optional< double > median_std_mm( double min_valid_fraction = 0.8 ) const;
    size_t frames() const { return m_frames; }

private:
    Roi m_roi;
    size_t m_frames = 0;
    std::vector< double > m_sum, m_sum_sq;
    std::vector< uint32_t > m_count;
};

// Mean of a list of values (0 for an empty list).
double mean( const std::vector< double > & values );
// q-quantile (0..1) of the values (sorted copy); 0 for an empty list.
double quantile( std::vector< double > values, double q );

}  // namespace ifs
