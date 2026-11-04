// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal point cloud representation, k-d tree index (nanoflann) and the coarse-to-fine pyramid used by ICP and
// the whole-scan registration.

#pragma once

#include "fr_registration.h"
#include "recon_math.h"

#include <nanoflann.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Definition of the opaque C type. Attribute arrays are either empty or have one entry (or triple) per point.
struct fr_cloud
{
    std::vector< float > points;      // 3 per point
    std::vector< float > normals;     // 3 per point; oriented toward the sensor / outward
    std::vector< uint8_t > colors;    // 3 per point, RGB8
    std::vector< float > intensity;   // 1 per point, [0, 1], derived from colors
    std::vector< float > gradients;   // 3 per point: intensity gradient in the tangent plane (1/m), colored ICP

    size_t size() const { return points.size() / 3; }
    bool has_normals() const { return ! normals.empty(); }
    bool has_colors() const { return ! colors.empty(); }
    bool has_intensity() const { return ! intensity.empty(); }
    bool has_gradients() const { return ! gradients.empty(); }

    Eigen::Map< const Eigen::Vector3f > point( size_t i ) const { return Eigen::Map< const Eigen::Vector3f >( &points[3 * i] ); }
    Eigen::Map< const Eigen::Vector3f > normal( size_t i ) const { return Eigen::Map< const Eigen::Vector3f >( &normals[3 * i] ); }
    Eigen::Map< const Eigen::Vector3f > gradient( size_t i ) const { return Eigen::Map< const Eigen::Vector3f >( &gradients[3 * i] ); }
};

namespace fr
{
inline float rgb_to_intensity( const uint8_t * rgb )
{
    return ( 0.299f * rgb[0] + 0.587f * rgb[1] + 0.114f * rgb[2] ) * ( 1.f / 255.f );
}

struct CloudAdaptor
{
    const float * pts = nullptr;
    size_t count = 0;
    size_t kdtree_get_point_count() const { return count; }
    float kdtree_get_pt( size_t i, size_t d ) const { return pts[3 * i + d]; }
    template< class BBox >
    bool kdtree_get_bbox( BBox & ) const { return false; }
};

using KdTree = nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor< float, CloudAdaptor >, CloudAdaptor, 3,
                                                    uint32_t >;

// k-d tree over the points of a cloud. The cloud must outlive the index and stay unmodified. Queries are
// thread-safe.
class CloudIndex
{
public:
    explicit CloudIndex( const fr_cloud & cloud );
    CloudIndex( const CloudIndex & ) = delete;
    CloudIndex & operator=( const CloudIndex & ) = delete;

    // Nearest neighbor within sqrt(max_dist2); false when none.
    bool nearest( const float query[3], float max_dist2, uint32_t & index, float & dist2 ) const;
    // Up to k nearest neighbors within sqrt(max_dist2) (max_dist2 <= 0: unbounded); returns the count.
    size_t knn( const float query[3], size_t k, uint32_t * indices, float * dist2, float max_dist2 ) const;

private:
    CloudAdaptor _adaptor;
    std::unique_ptr< KdTree > _tree;
};

// Normals by PCA over k nearest neighbors, oriented toward 'viewpoint'.
void estimate_normals_knn( fr_cloud & cloud, const CloudIndex & index, size_t k, const Eigen::Vector3f & viewpoint );

// Intensity gradients in the tangent plane by least squares over neighbors within radius (needs normals and
// intensity), as in Park, Zhou, Koltun: "Colored point cloud registration revisited" (ICCV 2017).
void estimate_gradients_knn( fr_cloud & cloud, const CloudIndex & index, float radius, size_t max_neighbors );

// Least-squares tangent-plane intensity gradient from neighbor offsets (relative to the point) and intensity
// differences; returns zero when the neighborhood is degenerate.
Eigen::Vector3f tangent_gradient( const Eigen::Vector3f & normal, const Eigen::Vector3f * offsets,
                                  const float * delta_intensity, size_t count );

// Hash-grid voxel averaging (throws std::invalid_argument for a non-positive voxel size).
fr_cloud voxel_downsample( const fr_cloud & cloud, float voxel );

void transform_cloud( fr_cloud & cloud, const Eigen::Matrix4d & transform );

struct PyramidLevel
{
    fr_cloud cloud;
    std::unique_ptr< CloudIndex > index;   // only for targets
    float voxel = 0;
};

// levels[0] is the finest level.
struct CloudPyramid
{
    std::vector< std::unique_ptr< PyramidLevel > > levels;
};

// Downsampled copies at voxel * 2^l (l = 0 .. levels-1). voxel <= 0 keeps a single level with the raw cloud.
// with_index builds k-d trees (target role); with_gradients makes sure colored targets have gradients.
CloudPyramid build_pyramid( const fr_cloud & cloud, int levels, float voxel, bool with_index, bool with_gradients );

struct IcpOutput
{
    Eigen::Matrix4d target_from_source = Eigen::Matrix4d::Identity();
    float fitness = 0;
    float inlier_rmse = 0;
    uint32_t correspondences = 0;
    int iterations = 0;
    bool converged = false;
    Matrix6d information = Matrix6d::Zero();
};

// Multi-scale ICP over prebuilt pyramids (the target pyramid needs indices). Returns FR_ERR_NOT_ENOUGH_DATA when
// too few correspondences were found, FR_ERR_CANCELLED when *cancel became true.
fr_status icp_pyramids( const CloudPyramid & source, const CloudPyramid & target, const Eigen::Matrix4d & init,
                        const fr_icp_params & params, IcpOutput & out, const std::atomic< bool > * cancel );

bool validate_icp_params( const fr_icp_params & params, std::string & error );
bool validate_frame( const fr_rgbd_frame & frame, std::string & error );
bool validate_cloud_params( const fr_cloud_params & params, std::string & error );

// Back-projection of a frame (implementation of fr_cloud_from_rgbd, throws on invalid input).
fr_cloud cloud_from_rgbd( const fr_rgbd_frame & frame, const fr_cloud_params & params );

// Where a frame's cloud has points: the camera-frame depth of each pixel's point (0 = none), cropped to the points'
// bounding box. Built from the camera-frame cloud of cloud_from_rgbd, so it holds exactly the surface that survived
// mask, erosion and the view-angle limit.
struct ViewMap
{
    float fx = 0, fy = 0, cx = 0, cy = 0;
    int x0 = 0, y0 = 0, width = 0, height = 0;
    std::vector< float > z;
};
ViewMap build_view_map( const fr_cloud & camera_cloud, const fr_intrinsics & intrinsics );

// Projective overlap of a source cloud with a target view under target_from_source. A source point is visible when it
// projects onto target data (within +-max(1, ceil(stride / 2)) pixels: the view map of a strided cloud holds every
// stride-th pixel) and faces the target camera within the view-angle limit; it is expected (the target could have
// measured it) unless it lies more than tau behind the target surface; it is an inlier when it is expected and within
// max_d of a target point; a violation when it lies more than tau in front of the observed target surface, i.e. in
// space the target camera saw through.
struct OverlapStats
{
    size_t visible = 0, expected = 0, inliers = 0, violations = 0;
    double inlier_area = 0;   // m², each source point covering max(voxel, stride * z / fx)^2
    float fitness() const { return expected ? float( double( inliers ) / double( expected ) ) : 0.f; }
    float violation_ratio() const { return visible ? float( double( violations ) / double( visible ) ) : 0.f; }
};
OverlapStats projective_overlap( const fr_cloud & source, const fr_cloud & target, const CloudIndex & target_index,
                                 const ViewMap & target_view, const Eigen::Matrix4d & target_from_source, float max_d, float tau,
                                 float cos_max_view_angle, float source_fx, float voxel, int stride );
}  // namespace fr
