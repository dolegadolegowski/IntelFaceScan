// License: Apache 2.0. See LICENSE file in root directory.
//
// Depth quality measures of the probe (see capture_measure.h).

#include "capture_measure.h"

#include <algorithm>
#include <cmath>

namespace
{
// Eigen decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations; 'v' receives the eigenvectors as
// columns. Converges in a handful of sweeps for 3x3, and has no failure modes for the covariance matrices here.
void jacobi3( double a[3][3], double eigenvalues[3], double v[3][3] )
{
    for( int i = 0; i < 3; ++i )
        for( int j = 0; j < 3; ++j )
            v[i][j] = i == j ? 1.0 : 0.0;
    for( int sweep = 0; sweep < 50; ++sweep )
    {
        double off = std::fabs( a[0][1] ) + std::fabs( a[0][2] ) + std::fabs( a[1][2] );
        if( off < 1e-300 )
            break;
        for( int p = 0; p < 2; ++p )
            for( int q = p + 1; q < 3; ++q )
            {
                if( std::fabs( a[p][q] ) < 1e-300 )
                    continue;
                double theta = ( a[q][q] - a[p][p] ) / ( 2.0 * a[p][q] );
                double t = ( theta >= 0 ? 1.0 : -1.0 ) / ( std::fabs( theta ) + std::sqrt( theta * theta + 1.0 ) );
                double c = 1.0 / std::sqrt( t * t + 1.0 ), s = t * c;
                for( int k = 0; k < 3; ++k )
                {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for( int k = 0; k < 3; ++k )
                {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for( int k = 0; k < 3; ++k )
                {
                    double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    for( int i = 0; i < 3; ++i )
        eigenvalues[i] = a[i][i];
}
}  // namespace

namespace ifs
{

Roi central_roi( int width, int height, double fraction )
{
    fraction = std::clamp( fraction, 0.01, 1.0 );
    Roi r;
    r.width = std::max( 1, int( std::lround( width * fraction ) ) );
    r.height = std::max( 1, int( std::lround( height * fraction ) ) );
    r.x = ( width - r.width ) / 2;
    r.y = ( height - r.height ) / 2;
    return r;
}

std::optional< PlaneFit > fit_plane( const std::vector< double > & xyz )
{
    size_t n = xyz.size() / 3;
    if( n < 3 )
        return std::nullopt;
    double c[3] = { 0, 0, 0 };
    for( size_t i = 0; i < n; ++i )
        for( int k = 0; k < 3; ++k )
            c[k] += xyz[i * 3 + k];
    for( double & v : c )
        v /= double( n );
    double cov[3][3] = {};
    for( size_t i = 0; i < n; ++i )
    {
        double d[3] = { xyz[i * 3] - c[0], xyz[i * 3 + 1] - c[1], xyz[i * 3 + 2] - c[2] };
        for( int r = 0; r < 3; ++r )
            for( int k = 0; k < 3; ++k )
                cov[r][k] += d[r] * d[k];
    }
    double eigenvalues[3], vectors[3][3];
    jacobi3( cov, eigenvalues, vectors );
    int smallest = 0;
    for( int i = 1; i < 3; ++i )
        if( eigenvalues[i] < eigenvalues[smallest] )
            smallest = i;
    int largest = 0;
    for( int i = 1; i < 3; ++i )
        if( eigenvalues[i] > eigenvalues[largest] )
            largest = i;
    // Points on a line (or one point) do not define a plane.
    double middle = eigenvalues[0] + eigenvalues[1] + eigenvalues[2] - eigenvalues[smallest] - eigenvalues[largest];
    if( ! ( eigenvalues[largest] > 0 ) || middle <= eigenvalues[largest] * 1e-12 )
        return std::nullopt;

    PlaneFit plane;
    for( int k = 0; k < 3; ++k )
        plane.normal[k] = vectors[k][smallest];
    double len = std::sqrt( plane.normal[0] * plane.normal[0] + plane.normal[1] * plane.normal[1]
                            + plane.normal[2] * plane.normal[2] );
    if( ! ( len > 0 ) )
        return std::nullopt;
    double sign = plane.normal[2] > 0 ? -1.0 : 1.0;  // facing the camera
    for( double & v : plane.normal )
        v = v * sign / len;
    plane.offset = -( plane.normal[0] * c[0] + plane.normal[1] * c[1] + plane.normal[2] * c[2] );
    double sum_sq = 0;
    for( size_t i = 0; i < n; ++i )
    {
        double d = plane.normal[0] * xyz[i * 3] + plane.normal[1] * xyz[i * 3 + 1] + plane.normal[2] * xyz[i * 3 + 2]
                 + plane.offset;
        sum_sq += d * d;
    }
    plane.rms_m = std::sqrt( sum_sq / double( n ) );
    return plane;
}

FrameStats measure_frame( const DepthView & view, const Roi & roi, double baseline_m )
{
    FrameStats stats;
    if( ! view.data || view.width <= 0 || view.height <= 0 )
        return stats;

    std::vector< double > all;
    all.reserve( size_t( view.width ) * size_t( view.height ) / 2 );
    size_t valid_total = 0;
    for( int y = 0; y < view.height; ++y )
        for( int x = 0; x < view.width; ++x )
            if( uint16_t d = view.at( x, y ) )
            {
                ++valid_total;
                all.push_back( d * double( view.depth_scale ) );
            }
    stats.full_fill_rate = double( valid_total ) / ( double( view.width ) * view.height );
    stats.nearest_m = quantile( std::move( all ), 0.005 );

    int x0 = std::max( roi.x, 0 ), y0 = std::max( roi.y, 0 );
    int x1 = std::min( roi.x + roi.width, view.width ), y1 = std::min( roi.y + roi.height, view.height );
    if( x1 <= x0 || y1 <= y0 )
        return stats;
    std::vector< double > xyz, depths;
    for( int y = y0; y < y1; ++y )
        for( int x = x0; x < x1; ++x )
            if( uint16_t d = view.at( x, y ) )
            {
                double z = d * double( view.depth_scale );
                depths.push_back( z );
                if( view.fx > 0 && view.fy > 0 )
                {
                    xyz.push_back( ( x - view.ppx ) / view.fx * z );
                    xyz.push_back( ( y - view.ppy ) / view.fy * z );
                    xyz.push_back( z );
                }
            }
    stats.roi_valid = depths.size();
    stats.fill_rate = double( depths.size() ) / ( double( x1 - x0 ) * ( y1 - y0 ) );
    stats.median_m = quantile( depths, 0.5 );

    if( auto plane = fit_plane( xyz ) )
    {
        stats.plane_rms_mm = plane->rms_m * 1000.0;
        if( baseline_m > 0 )
        {
            // Disparity error against the plane along each pixel's ray: d = f*B/z (Depth Quality Tool definition).
            double fb = view.fx * baseline_m, sum_sq = 0;
            size_t n = 0;
            for( size_t i = 0; i < xyz.size() / 3; ++i )
            {
                double rx = xyz[i * 3] / xyz[i * 3 + 2], ry = xyz[i * 3 + 1] / xyz[i * 3 + 2];
                double denom = plane->normal[0] * rx + plane->normal[1] * ry + plane->normal[2];
                if( std::fabs( denom ) < 1e-9 )
                    continue;
                double z_plane = -plane->offset / denom;
                if( ! ( z_plane > 0 ) )
                    continue;
                double e = fb / xyz[i * 3 + 2] - fb / z_plane;
                sum_sq += e * e;
                ++n;
            }
            if( n )
                stats.subpixel_rms = std::sqrt( sum_sq / double( n ) );
        }
    }
    return stats;
}

TemporalNoise::TemporalNoise( const Roi & roi )
    : m_roi( roi )
{
    size_t n = size_t( std::max( roi.width, 0 ) ) * size_t( std::max( roi.height, 0 ) );
    m_sum.assign( n, 0.0 );
    m_sum_sq.assign( n, 0.0 );
    m_count.assign( n, 0 );
}

void TemporalNoise::add( const DepthView & view )
{
    if( ! view.data )
        return;
    ++m_frames;
    for( int ry = 0; ry < m_roi.height; ++ry )
    {
        int y = m_roi.y + ry;
        if( y < 0 || y >= view.height )
            continue;
        for( int rx = 0; rx < m_roi.width; ++rx )
        {
            int x = m_roi.x + rx;
            if( x < 0 || x >= view.width )
                continue;
            if( uint16_t d = view.at( x, y ) )
            {
                size_t i = size_t( ry ) * size_t( m_roi.width ) + size_t( rx );
                double z = d * double( view.depth_scale );
                m_sum[i] += z;
                m_sum_sq[i] += z * z;
                ++m_count[i];
            }
        }
    }
}

std::optional< double > TemporalNoise::median_std_mm( double min_valid_fraction ) const
{
    if( m_frames < 2 )
        return std::nullopt;
    auto min_count = uint32_t( std::max( 2.0, std::ceil( double( m_frames ) * min_valid_fraction ) ) );
    std::vector< double > stds;
    for( size_t i = 0; i < m_count.size(); ++i )
    {
        if( m_count[i] < min_count )
            continue;
        double n = m_count[i];
        double mean = m_sum[i] / n;
        double var = std::max( 0.0, ( m_sum_sq[i] - n * mean * mean ) / ( n - 1 ) );
        stds.push_back( std::sqrt( var ) * 1000.0 );
    }
    if( stds.empty() )
        return std::nullopt;
    return quantile( std::move( stds ), 0.5 );
}

double mean( const std::vector< double > & values )
{
    if( values.empty() )
        return 0;
    double sum = 0;
    for( double v : values )
        sum += v;
    return sum / double( values.size() );
}

double quantile( std::vector< double > values, double q )
{
    if( values.empty() )
        return 0;
    q = std::clamp( q, 0.0, 1.0 );
    size_t k = size_t( std::floor( q * double( values.size() - 1 ) + 0.5 ) );
    std::nth_element( values.begin(), values.begin() + std::ptrdiff_t( k ), values.end() );
    return values[k];
}

}  // namespace ifs
