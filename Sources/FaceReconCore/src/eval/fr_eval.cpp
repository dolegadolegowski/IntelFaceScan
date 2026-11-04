// License: Apache 2.0. See LICENSE file in root directory.
//
// Accuracy evaluation: surface-to-surface distances, plane fitting (depth noise measurements) and pose errors.
// Every result is deterministic (fixed seeds, sampling done before the parallel part) so that test thresholds and
// numbers in docs/RECONSTRUCTION.md are reproducible.

#include "fr_eval.h"
#include "../common/fr_internal.h"
#include "triangle_bvh.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace
{
using fr::eval::vec3;

// splitmix64: tiny, fast and fully specified, so samples are identical on every platform and standard library
// (std::uniform_real_distribution is implementation-defined).
struct rng
{
    uint64_t state;
    explicit rng( uint64_t seed ) : state( seed ) {}
    uint64_t next()
    {
        uint64_t z = ( state += 0x9E3779B97F4A7C15ull );
        z = ( z ^ ( z >> 30 ) ) * 0xBF58476D1CE4E5B9ull;
        z = ( z ^ ( z >> 27 ) ) * 0x94D049BB133111EBull;
        return z ^ ( z >> 31 );
    }
    double uniform() { return double( next() >> 11 ) * ( 1.0 / 9007199254740992.0 ); }   // [0, 1)
    uint32_t below( uint32_t n ) { return uint32_t( uniform() * n ) % n; }
};

constexpr uint64_t sampling_seed = 0x5EED0F1FACE5CA9ull;
constexpr uint64_t ransac_seed = 0x9A7E5EED12345678ull;
constexpr uint32_t max_samples = 100000000u;   // 100M samples ~ 2.4 GB of points + distances

fr_status validate_mesh( const fr_mesh * mesh, const char * name )
{
    if( ! mesh )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, std::string( "fr_mesh_distance: " ) + name + " is NULL" );
    if( mesh->triangle_count == 0 || mesh->vertex_count == 0 )
        return fr::fail( FR_ERR_NOT_ENOUGH_DATA, std::string( "fr_mesh_distance: " ) + name + " has no triangles" );
    if( ! mesh->positions || ! mesh->indices )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, std::string( "fr_mesh_distance: " ) + name + " has no arrays" );
    const size_t n = mesh->vertex_count;
    for( size_t i = 0; i < size_t( mesh->triangle_count ) * 3; ++i )
        if( mesh->indices[i] >= n )
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             std::string( "fr_mesh_distance: " ) + name + " has a vertex index out of range" );
    for( size_t i = 0; i < n * 3; ++i )
        if( ! std::isfinite( mesh->positions[i] ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT,
                             std::string( "fr_mesh_distance: " ) + name + " has a non-finite vertex" );
    return FR_OK;
}

vec3 vertex( const fr_mesh * mesh, uint32_t index )
{
    const float * p = mesh->positions + size_t( index ) * 3;
    return { p[0], p[1], p[2] };
}

// Splits [0, count) into contiguous ranges processed by all cores. f must not throw.
template< typename F >
void parallel_ranges( size_t count, F && f )
{
    const size_t hardware = std::max( 1u, std::thread::hardware_concurrency() );
    const size_t workers = std::min( hardware, std::max< size_t >( 1, count / 2048 ) );
    if( workers <= 1 )
    {
        f( size_t( 0 ), count );
        return;
    }
    std::vector< std::thread > threads;
    threads.reserve( workers - 1 );
    const size_t chunk = ( count + workers - 1 ) / workers;
    try
    {
        for( size_t w = 1; w < workers; ++w )
        {
            const size_t begin = std::min( count, w * chunk ), end = std::min( count, begin + chunk );
            threads.emplace_back( [&f, begin, end] { f( begin, end ); } );
        }
    }
    catch( ... )
    {
        // Could not start every worker: finish the work that was handed out, then report the failure.
        for( auto & t : threads )
            t.join();
        throw;
    }
    f( size_t( 0 ), std::min( count, chunk ) );
    for( auto & t : threads )
        t.join();
}

// Linear interpolation between order statistics (sorted must be ascending, non-empty).
double quantile( const std::vector< double > & sorted, double q )
{
    const double position = q * double( sorted.size() - 1 );
    const size_t i = size_t( position );
    if( i + 1 >= sorted.size() )
        return sorted.back();
    const double fraction = position - double( i );
    return sorted[i] + fraction * ( sorted[i + 1] - sorted[i] );
}

struct plane
{
    Eigen::Vector3d normal{ 0, 0, 1 };
    double d = 0;
    double distance( const Eigen::Vector3d & p ) const { return normal.dot( p ) + d; }
};

// Total least squares plane: the normal is the right singular vector of the centered points with the smallest
// singular value (computed from the 3x3 scatter matrix, which has the same singular vectors). Returns false when
// the points do not span a plane (all equal or collinear).
bool least_squares_plane( const std::vector< Eigen::Vector3d > & points, const std::vector< uint32_t > * subset,
                          plane & out )
{
    const size_t n = subset ? subset->size() : points.size();
    if( n < 3 )
        return false;
    auto at = [&]( size_t i ) -> const Eigen::Vector3d & { return points[subset ? ( *subset )[i] : i]; };
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for( size_t i = 0; i < n; ++i )
        centroid += at( i );
    centroid /= double( n );
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for( size_t i = 0; i < n; ++i )
    {
        const Eigen::Vector3d q = at( i ) - centroid;
        scatter += q * q.transpose();
    }
    Eigen::JacobiSVD< Eigen::Matrix3d, Eigen::ComputeFullU > svd( scatter );
    const Eigen::Vector3d sigma = svd.singularValues();   // descending
    if( ! ( sigma( 1 ) > 1e-12 * sigma( 0 ) ) || ! ( sigma( 0 ) > 0 ) )
        return false;
    out.normal = svd.matrixU().col( 2 ).normalized();
    out.d = -out.normal.dot( centroid );
    return out.normal.allFinite() && std::isfinite( out.d );
}

}  // namespace

extern "C" {

fr_status fr_mesh_distance( const fr_mesh * from, const fr_mesh * to, uint32_t samples, fr_distance_stats * out )
{
    return fr::guarded( "fr_mesh_distance", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_distance: out is NULL" );
        std::memset( out, 0, sizeof( *out ) );
        if( samples == 0 || samples > max_samples )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_distance: samples must be in [1, 100000000]" );
        fr_status status = validate_mesh( from, "from" );
        if( status != FR_OK )
            return status;
        status = validate_mesh( to, "to" );
        if( status != FR_OK )
            return status;

        // Area-uniform sampling of 'from': pick a triangle with probability proportional to its area, then a
        // uniform point inside it (square-root parameterization of the barycentric coordinates).
        const size_t triangles = from->triangle_count;
        std::vector< double > cumulative( triangles );
        double total = 0;
        for( size_t t = 0; t < triangles; ++t )
        {
            const uint32_t * f = from->indices + 3 * t;
            const vec3 a = vertex( from, f[0] ), b = vertex( from, f[1] ), c = vertex( from, f[2] );
            total += 0.5 * std::sqrt( fr::eval::length2( fr::eval::cross( b - a, c - a ) ) );
            cumulative[t] = total;
        }
        if( ! ( total > 0 ) )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_mesh_distance: 'from' has zero surface area" );

        std::vector< vec3 > points( samples );
        rng random( sampling_seed );
        for( uint32_t s = 0; s < samples; ++s )
        {
            const double u = random.uniform() * total;
            size_t t = size_t( std::upper_bound( cumulative.begin(), cumulative.end(), u ) - cumulative.begin() );
            t = std::min( t, triangles - 1 );
            // Zero-area triangles share their cumulative value with the previous one and are never selected by
            // upper_bound, except at the very end; walk back to a triangle that has area.
            while( t > 0 && cumulative[t] == cumulative[t - 1] )
                --t;
            const uint32_t * f = from->indices + 3 * t;
            const vec3 a = vertex( from, f[0] ), b = vertex( from, f[1] ), c = vertex( from, f[2] );
            const double r1 = std::sqrt( random.uniform() ), r2 = random.uniform();
            points[s] = a * ( 1 - r1 ) + b * ( r1 * ( 1 - r2 ) ) + c * ( r1 * r2 );
        }

        const fr::eval::triangle_bvh bvh( to->positions, to->indices, to->triangle_count );
        std::vector< double > distances( samples );
        parallel_ranges( samples, [&]( size_t begin, size_t end ) {
            for( size_t i = begin; i < end; ++i )
                distances[i] = std::sqrt( bvh.closest( points[i] ) );
        } );

        double sum = 0, sum2 = 0;
        for( double d : distances )
        {
            sum += d;
            sum2 += d * d;
        }
        std::sort( distances.begin(), distances.end() );
        out->samples = samples;
        out->mean_m = sum / samples;
        out->rms_m = std::sqrt( sum2 / samples );
        out->median_m = quantile( distances, 0.5 );
        out->p95_m = quantile( distances, 0.95 );
        out->max_m = distances.back();
        fr::clear_error();
        return FR_OK;
    } );
}

fr_status fr_fit_plane( const float * points, uint32_t count, float ransac_threshold_m, fr_plane_fit * out )
{
    return fr::guarded( "fr_fit_plane", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_fit_plane: out is NULL" );
        std::memset( out, 0, sizeof( *out ) );
        if( ! points && count )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_fit_plane: points is NULL" );
        if( ! std::isfinite( ransac_threshold_m ) || ransac_threshold_m < 0 )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_fit_plane: ransac_threshold_m must be >= 0" );

        // Invalid depth back-projects to NaN/inf in some callers; those samples carry no information.
        std::vector< Eigen::Vector3d > pts;
        pts.reserve( count );
        for( uint32_t i = 0; i < count; ++i )
        {
            const float * p = points + size_t( i ) * 3;
            if( std::isfinite( p[0] ) && std::isfinite( p[1] ) && std::isfinite( p[2] ) )
                pts.emplace_back( p[0], p[1], p[2] );
        }
        if( pts.size() < 3 )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_fit_plane: fewer than 3 valid points" );

        plane best;
        std::vector< uint32_t > inliers;
        if( ransac_threshold_m > 0 )
        {
            const double threshold = ransac_threshold_m;
            const uint32_t n = uint32_t( pts.size() );
            rng random( ransac_seed );
            size_t best_count = 0;
            double best_cost = std::numeric_limits< double >::infinity();
            bool found = false;
            const int max_iterations = 2000;
            int needed = max_iterations;
            for( int iteration = 0; iteration < needed && iteration < max_iterations; ++iteration )
            {
                const uint32_t i0 = random.below( n ), i1 = random.below( n ), i2 = random.below( n );
                if( i0 == i1 || i1 == i2 || i0 == i2 )
                    continue;
                Eigen::Vector3d normal = ( pts[i1] - pts[i0] ).cross( pts[i2] - pts[i0] );
                const double length = normal.norm();
                if( ! ( length > 1e-12 ) )
                    continue;
                plane candidate;
                candidate.normal = normal / length;
                candidate.d = -candidate.normal.dot( pts[i0] );
                // MSAC score: inlier count first, truncated squared residuals break ties between equal counts.
                size_t inlier_count = 0;
                double cost = 0;
                for( const auto & p : pts )
                {
                    const double r = std::abs( candidate.distance( p ) );
                    if( r <= threshold )
                    {
                        ++inlier_count;
                        cost += r * r;
                    }
                    else
                        cost += threshold * threshold;
                }
                if( inlier_count > best_count || ( inlier_count == best_count && cost < best_cost ) )
                {
                    best_count = inlier_count;
                    best_cost = cost;
                    best = candidate;
                    found = true;
                    // Adaptive iteration count for 99.9% confidence of having drawn one all-inlier sample.
                    const double w = double( inlier_count ) / n;
                    const double p_fail = 1.0 - w * w * w;
                    if( p_fail <= 1e-12 )
                        needed = iteration + 1;
                    else
                        needed = std::min( max_iterations,
                                           int( std::ceil( std::log( 1e-3 ) / std::log( p_fail ) ) ) + 1 );
                }
            }
            if( ! found )
                return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_fit_plane: the points do not span a plane" );
            // Refit on the inliers; the refined plane can gain or lose a few inliers, so repeat until stable.
            for( int round = 0; round < 3; ++round )
            {
                inliers.clear();
                for( uint32_t i = 0; i < n; ++i )
                    if( std::abs( best.distance( pts[i] ) ) <= threshold )
                        inliers.push_back( i );
                plane refined;
                if( ! least_squares_plane( pts, &inliers, refined ) )
                    break;
                best = refined;
            }
            inliers.clear();
            for( uint32_t i = 0; i < n; ++i )
                if( std::abs( best.distance( pts[i] ) ) <= threshold )
                    inliers.push_back( i );
            if( inliers.size() < 3 )
                return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_fit_plane: fewer than 3 inliers" );
        }
        else
        {
            if( ! least_squares_plane( pts, nullptr, best ) )
                return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_fit_plane: the points are collinear or identical" );
            inliers.resize( pts.size() );
            for( uint32_t i = 0; i < inliers.size(); ++i )
                inliers[i] = i;
        }

        // Orient the normal toward the origin (the camera for camera-frame points), so the sign of d is stable.
        if( best.d < 0 || ( best.d == 0 && best.normal.z() > 0 ) )
        {
            best.normal = -best.normal;
            best.d = -best.d;
        }
        double sum2 = 0;
        for( uint32_t i : inliers )
        {
            const double r = best.distance( pts[i] );
            sum2 += r * r;
        }
        out->normal[0] = float( best.normal.x() );
        out->normal[1] = float( best.normal.y() );
        out->normal[2] = float( best.normal.z() );
        out->d = float( best.d );
        out->rms_m = std::sqrt( sum2 / double( inliers.size() ) );
        out->inliers = uint32_t( inliers.size() );
        fr::clear_error();
        return FR_OK;
    } );
}

void fr_pose_error( const float a[16], const float b[16], float * rotation_deg, float * translation_m )
{
    if( rotation_deg )
        *rotation_deg = -1.f;
    if( translation_m )
        *translation_m = -1.f;
    if( ! a || ! b )
        return;
    // Relative rotation Ra^T Rb (column-major: element (row r, column c) is m[4 * c + r]).
    double r[3][3];
    for( int i = 0; i < 3; ++i )
        for( int j = 0; j < 3; ++j )
        {
            double s = 0;
            for( int k = 0; k < 3; ++k )
                s += double( a[4 * i + k] ) * double( b[4 * j + k] );
            r[i][j] = s;
        }
    // atan2 of sin and cos of the angle stays accurate near 0 and 180 degrees, where acos((trace - 1) / 2) does not.
    const double cos_angle = 0.5 * ( r[0][0] + r[1][1] + r[2][2] - 1.0 );
    const double ax = r[2][1] - r[1][2], ay = r[0][2] - r[2][0], az = r[1][0] - r[0][1];
    const double sin_angle = 0.5 * std::sqrt( ax * ax + ay * ay + az * az );
    const double angle = std::atan2( sin_angle, cos_angle ) * 180.0 / 3.14159265358979323846;
    const double dx = double( a[12] ) - b[12], dy = double( a[13] ) - b[13], dz = double( a[14] ) - b[14];
    if( rotation_deg )
        *rotation_deg = std::isfinite( angle ) ? float( angle ) : -1.f;
    if( translation_m )
    {
        const double distance = std::sqrt( dx * dx + dy * dy + dz * dz );
        *translation_m = std::isfinite( distance ) ? float( distance ) : -1.f;
    }
}

}  // extern "C"
