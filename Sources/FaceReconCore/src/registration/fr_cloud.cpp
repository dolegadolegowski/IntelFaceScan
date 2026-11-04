// License: Apache 2.0. See LICENSE file in root directory.
//
// Point clouds: back-projection of RGB-D frames (mask, depth range, silhouette erosion against flying pixels,
// normals and tangent-plane intensity gradients estimated on the depth image), k-d tree index, voxel averaging,
// rigid transforms.

#include "recon_cloud.h"
#include "../common/fr_internal.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace fr
{
namespace
{
// Regions of at most this many pixels without depth are isolated holes inside a surface, not silhouettes. With 8%
// randomly scattered holes, 99.5% of the empty pixels lie in such regions (84% at 20%).
constexpr size_t kMaxHolePixels = 4;
}  // namespace

CloudIndex::CloudIndex( const fr_cloud & cloud )
{
    _adaptor.pts = cloud.points.data();
    _adaptor.count = cloud.size();
    if( _adaptor.count == 0 )
        return;
    // Multi-threaded construction only pays off for large merged clouds.
    const unsigned threads = _adaptor.count > 300000 ? 0u : 1u;
    const nanoflann::KDTreeSingleIndexAdaptorParams params( 16, nanoflann::KDTreeSingleIndexAdaptorFlags::None, threads );
    _tree = std::make_unique< KdTree >( 3, _adaptor, params );
}

bool CloudIndex::nearest( const float query[3], float max_dist2, uint32_t & index, float & dist2 ) const
{
    if( ! _tree )
        return false;
    uint32_t idx = 0;
    float d2 = std::numeric_limits< float >::max();
    if( _tree->rknnSearch( query, 1, &idx, &d2, max_dist2 ) == 0 )
        return false;
    index = idx;
    dist2 = d2;
    return true;
}

size_t CloudIndex::knn( const float query[3], size_t k, uint32_t * indices, float * dist2, float max_dist2 ) const
{
    if( ! _tree || k == 0 )
        return 0;
    if( max_dist2 > 0 )
        return _tree->rknnSearch( query, k, indices, dist2, max_dist2 );
    return _tree->knnSearch( query, k, indices, dist2 );
}

Eigen::Vector3f tangent_gradient( const Eigen::Vector3f & normal, const Eigen::Vector3f * offsets,
                                  const float * delta_intensity, size_t count )
{
    if( count < 3 )
        return Eigen::Vector3f::Zero();
    const Eigen::Vector3f n = normal.normalized();
    // Any unit vector orthogonal to n spans the tangent plane together with n x t1.
    Eigen::Vector3f t1 = std::abs( n.x() ) < 0.9f ? Eigen::Vector3f::UnitX().cross( n ) : Eigen::Vector3f::UnitY().cross( n );
    t1.normalize();
    const Eigen::Vector3f t2 = n.cross( t1 );
    double a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;
    for( size_t k = 0; k < count; ++k )
    {
        const double a = offsets[k].dot( t1 ), b = offsets[k].dot( t2 );
        a00 += a * a;
        a01 += a * b;
        a11 += b * b;
        b0 += a * delta_intensity[k];
        b1 += b * delta_intensity[k];
    }
    const double det = a00 * a11 - a01 * a01;
    const double tr = a00 + a11;
    if( ! ( det > 1e-6 * tr * tr ) || tr <= 0 )
        return Eigen::Vector3f::Zero();
    const double g0 = ( a11 * b0 - a01 * b1 ) / det;
    const double g1 = ( a00 * b1 - a01 * b0 ) / det;
    return float( g0 ) * t1 + float( g1 ) * t2;
}

void estimate_normals_knn( fr_cloud & cloud, const CloudIndex & index, size_t k, const Eigen::Vector3f & viewpoint )
{
    const size_t n = cloud.size();
    cloud.normals.assign( 3 * n, 0.f );
    k = std::clamp< size_t >( k, 3, 64 );
    parallel_chunks( n, chunk_count( n, 2048 ), [&]( size_t begin, size_t end, size_t ) {
        std::vector< uint32_t > idx( k );
        std::vector< float > d2( k );
        for( size_t i = begin; i < end; ++i )
        {
            const size_t found = index.knn( &cloud.points[3 * i], k, idx.data(), d2.data(), 0.f );
            Eigen::Vector3f nrm = Eigen::Vector3f::UnitZ();
            if( found >= 3 )
            {
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                for( size_t j = 0; j < found; ++j )
                    mean += cloud.point( idx[j] ).cast< double >();
                mean /= double( found );
                Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                for( size_t j = 0; j < found; ++j )
                {
                    const Eigen::Vector3d d = cloud.point( idx[j] ).cast< double >() - mean;
                    cov += d * d.transpose();
                }
                Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > es;
                es.computeDirect( cov );
                nrm = es.eigenvectors().col( 0 ).cast< float >();
            }
            if( nrm.dot( viewpoint - cloud.point( i ) ) < 0 )
                nrm = -nrm;
            Eigen::Map< Eigen::Vector3f >( &cloud.normals[3 * i] ) = nrm.normalized();
        }
    } );
}

void estimate_gradients_knn( fr_cloud & cloud, const CloudIndex & index, float radius, size_t max_neighbors )
{
    const size_t n = cloud.size();
    cloud.gradients.assign( 3 * n, 0.f );
    if( ! cloud.has_normals() || ! cloud.has_intensity() )
        return;
    max_neighbors = std::clamp< size_t >( max_neighbors, 4, 64 );
    const float r2 = radius * radius;
    parallel_chunks( n, chunk_count( n, 2048 ), [&]( size_t begin, size_t end, size_t ) {
        std::vector< uint32_t > idx( max_neighbors );
        std::vector< float > d2( max_neighbors );
        std::vector< Eigen::Vector3f > offsets( max_neighbors );
        std::vector< float > di( max_neighbors );
        for( size_t i = begin; i < end; ++i )
        {
            const size_t found = index.knn( &cloud.points[3 * i], max_neighbors, idx.data(), d2.data(), r2 );
            size_t m = 0;
            for( size_t j = 0; j < found; ++j )
            {
                if( idx[j] == i )
                    continue;
                offsets[m] = cloud.point( idx[j] ) - cloud.point( i );
                di[m] = cloud.intensity[idx[j]] - cloud.intensity[i];
                ++m;
            }
            Eigen::Map< Eigen::Vector3f >( &cloud.gradients[3 * i] ) = tangent_gradient( cloud.normal( i ), offsets.data(), di.data(), m );
        }
    } );
}

fr_cloud voxel_downsample( const fr_cloud & cloud, float voxel )
{
    if( ! ( voxel > 0 ) || ! std::isfinite( voxel ) )
        throw std::invalid_argument( "voxel size must be positive" );
    const size_t n = cloud.size();
    const float inv = 1.f / voxel;
    constexpr int64_t kHalfRange = int64_t( 1 ) << 20;   // 21 bits per axis
    constexpr uint64_t kInvalid = std::numeric_limits< uint64_t >::max();
    std::vector< std::pair< uint64_t, uint32_t > > keys( n );
    parallel_chunks( n, chunk_count( n, 16384 ), [&]( size_t begin, size_t end, size_t ) {
        for( size_t i = begin; i < end; ++i )
        {
            const float * p = &cloud.points[3 * i];
            uint64_t key = kInvalid;
            if( std::isfinite( p[0] ) && std::isfinite( p[1] ) && std::isfinite( p[2] ) )
            {
                const double fx = std::floor( double( p[0] ) * inv ), fy = std::floor( double( p[1] ) * inv ),
                             fz = std::floor( double( p[2] ) * inv );
                if( std::abs( fx ) < kHalfRange && std::abs( fy ) < kHalfRange && std::abs( fz ) < kHalfRange )
                    key = ( uint64_t( int64_t( fx ) + kHalfRange ) << 42 ) | ( uint64_t( int64_t( fy ) + kHalfRange ) << 21 )
                        | uint64_t( int64_t( fz ) + kHalfRange );
            }
            keys[i] = { key, uint32_t( i ) };
        }
    } );
    std::sort( keys.begin(), keys.end() );

    std::vector< size_t > run_starts;
    for( size_t i = 0; i < n && keys[i].first != kInvalid; ++i )
        if( i == 0 || keys[i].first != keys[i - 1].first )
            run_starts.push_back( i );
    size_t valid_end = 0;
    while( valid_end < n && keys[valid_end].first != kInvalid )
        ++valid_end;
    run_starts.push_back( valid_end );
    const size_t m = run_starts.size() - 1;

    fr_cloud out;
    out.points.resize( 3 * m );
    if( cloud.has_normals() )
        out.normals.resize( 3 * m );
    if( cloud.has_colors() )
        out.colors.resize( 3 * m );
    if( cloud.has_intensity() )
        out.intensity.resize( m );
    if( cloud.has_gradients() )
        out.gradients.resize( 3 * m );

    parallel_chunks( m, chunk_count( m, 4096 ), [&]( size_t begin, size_t end, size_t ) {
        for( size_t v = begin; v < end; ++v )
        {
            const size_t a = run_starts[v], b = run_starts[v + 1];
            const double count = double( b - a );
            Eigen::Vector3d p = Eigen::Vector3d::Zero(), nrm = Eigen::Vector3d::Zero(), g = Eigen::Vector3d::Zero();
            double c[3] = { 0, 0, 0 }, intensity = 0;
            for( size_t k = a; k < b; ++k )
            {
                const size_t i = keys[k].second;
                p += cloud.point( i ).cast< double >();
                if( cloud.has_normals() )
                    nrm += cloud.normal( i ).cast< double >();
                if( cloud.has_colors() )
                    for( int ch = 0; ch < 3; ++ch )
                        c[ch] += cloud.colors[3 * i + ch];
                if( cloud.has_intensity() )
                    intensity += cloud.intensity[i];
                if( cloud.has_gradients() )
                    g += cloud.gradient( i ).cast< double >();
            }
            Eigen::Map< Eigen::Vector3f >( &out.points[3 * v] ) = ( p / count ).cast< float >();
            if( cloud.has_normals() )
            {
                // Opposite normals (both sides of a thin structure) cancel; keep the first one then.
                const double len = nrm.norm();
                const Eigen::Vector3f n = len > 1e-6 * count ? ( nrm / len ).cast< float >()
                                                             : Eigen::Vector3f( cloud.normal( keys[a].second ) );
                Eigen::Map< Eigen::Vector3f >( &out.normals[3 * v] ) = n;
                if( cloud.has_gradients() )
                {
                    const Eigen::Vector3d nd = n.cast< double >();
                    g /= count;
                    g -= g.dot( nd ) * nd;
                    Eigen::Map< Eigen::Vector3f >( &out.gradients[3 * v] ) = g.cast< float >();
                }
            }
            else if( cloud.has_gradients() )
                Eigen::Map< Eigen::Vector3f >( &out.gradients[3 * v] ) = ( g / count ).cast< float >();
            if( cloud.has_colors() )
                for( int ch = 0; ch < 3; ++ch )
                    out.colors[3 * v + ch] = uint8_t( std::lround( std::clamp( c[ch] / count, 0.0, 255.0 ) ) );
            if( cloud.has_intensity() )
                out.intensity[v] = float( intensity / count );
        }
    } );
    return out;
}

void transform_cloud( fr_cloud & cloud, const Eigen::Matrix4d & transform )
{
    const Eigen::Matrix3f r = transform.block< 3, 3 >( 0, 0 ).cast< float >();
    const Eigen::Vector3f t = transform.block< 3, 1 >( 0, 3 ).cast< float >();
    const size_t n = cloud.size();
    parallel_chunks( n, chunk_count( n, 32768 ), [&]( size_t begin, size_t end, size_t ) {
        for( size_t i = begin; i < end; ++i )
        {
            Eigen::Map< Eigen::Vector3f > p( &cloud.points[3 * i] );
            p = r * p + t;
            if( cloud.has_normals() )
            {
                Eigen::Map< Eigen::Vector3f > nrm( &cloud.normals[3 * i] );
                nrm = r * nrm;
            }
            if( cloud.has_gradients() )
            {
                Eigen::Map< Eigen::Vector3f > g( &cloud.gradients[3 * i] );
                g = r * g;
            }
        }
    } );
}

bool validate_frame( const fr_rgbd_frame & f, std::string & error )
{
    const fr_intrinsics & in = f.intrinsics;
    if( ! f.depth )
        error = "frame depth is NULL";
    else if( in.width <= 0 || in.height <= 0 || int64_t( in.width ) * in.height > 64 * 1024 * 1024 )
        error = "invalid image size " + std::to_string( in.width ) + "x" + std::to_string( in.height );
    else if( ! ( in.fx > 0 ) || ! ( in.fy > 0 ) || ! std::isfinite( in.fx ) || ! std::isfinite( in.fy )
             || ! std::isfinite( in.cx ) || ! std::isfinite( in.cy ) )
        error = "invalid intrinsics";
    else if( ! ( f.depth_scale > 0 ) || ! std::isfinite( f.depth_scale ) )
        error = "invalid depth_scale";
    else if( f.has_initial_pose && ! all_finite( f.initial_world_from_camera, 16 ) )
        error = "initial pose is not finite";
    else
        return true;
    return false;
}

bool validate_cloud_params( const fr_cloud_params & p, std::string & error )
{
    if( ! ( p.min_depth_m >= 0 ) || ! ( p.max_depth_m > p.min_depth_m ) || ! std::isfinite( p.max_depth_m ) )
        error = "invalid depth range";
    else if( p.stride < 1 || p.stride > 64 )
        error = "stride must be in [1, 64]";
    else if( p.normal_radius_px < 1 || p.normal_radius_px > 10 )
        error = "normal_radius_px must be in [1, 10]";
    else if( ! ( p.max_view_angle_deg > 0 ) || ! ( p.max_view_angle_deg <= 90 ) )
        error = "max_view_angle_deg must be in (0, 90]";
    else if( p.mask_erode_px < 0 || p.mask_erode_px > 32 )
        error = "mask_erode_px must be in [0, 32]";
    else
        return true;
    return false;
}

fr_cloud cloud_from_rgbd( const fr_rgbd_frame & f, const fr_cloud_params & p )
{
    const int w = f.intrinsics.width, h = f.intrinsics.height;
    const float fx = f.intrinsics.fx, fy = f.intrinsics.fy, cx = f.intrinsics.cx, cy = f.intrinsics.cy;
    const size_t npix = size_t( w ) * size_t( h );
    const float cos_max = float( std::cos( deg2rad( p.max_view_angle_deg ) ) );
    // Depth may change by about footprint * tan(angle) per pixel on a slanted surface; beyond that (with a 5 mm
    // floor for noise) two neighbors belong to different surfaces.
    const float tan_jump = float( std::tan( deg2rad( std::min( p.max_view_angle_deg, 80.f ) ) ) );
    auto jump_tolerance = [&]( float zc, int pixels ) { return std::max( 0.005f, 2.f * float( pixels ) * zc / fx * tan_jump ); };

    std::vector< float > z( npix, 0.f );
    parallel_chunks( size_t( h ), chunk_count( size_t( h ), 8 ), [&]( size_t y0, size_t y1, size_t ) {
        for( size_t y = y0; y < y1; ++y )
            for( int x = 0; x < w; ++x )
            {
                const size_t i = y * size_t( w ) + size_t( x );
                const uint16_t raw = f.depth[i];
                if( raw == 0 || ( f.mask && f.mask[i] < 128 ) )
                    continue;
                const float d = float( raw ) * f.depth_scale;
                if( d >= p.min_depth_m && d <= p.max_depth_m )
                    z[i] = d;
            }
    } );

    // Pixels without data, at the image border, or next to a depth discontinuity; dilated by mask_erode_px so the
    // flying pixels stereo depth produces along silhouettes are removed.
    std::vector< uint8_t > keep( npix, 0 );
    const int erode = p.mask_erode_px;
    if( erode == 0 )
    {
        for( size_t i = 0; i < npix; ++i )
            keep[i] = z[i] > 0;
    }
    else
    {
        // Isolated holes (stereo matching failures, speckle) are not silhouettes: a pixel without data counts as one
        // only when its 4-connected region of empty pixels is larger than kMaxHolePixels. Otherwise every speckle
        // would erase (2 erode + 1)^2 pixels: with 8% scattered holes the dilation removed 98% of a head cloud and
        // registration failed. Occlusion bands along silhouettes and creases are long, so they still count.
        std::vector< uint8_t > empty( npix, 0 );
        {
            std::vector< uint8_t > visited( npix, 0 );
            std::vector< uint32_t > region, stack;
            for( size_t start = 0; start < npix; ++start )
            {
                if( z[start] > 0 || visited[start] )
                    continue;
                region.clear();
                stack.assign( 1, uint32_t( start ) );
                visited[start] = 1;
                while( ! stack.empty() )
                {
                    const uint32_t i = stack.back();
                    stack.pop_back();
                    region.push_back( i );
                    const int x = int( i % uint32_t( w ) ), y = int( i / uint32_t( w ) );
                    auto visit = [&]( size_t j ) {
                        if( z[j] <= 0 && ! visited[j] )
                        {
                            visited[j] = 1;
                            stack.push_back( uint32_t( j ) );
                        }
                    };
                    if( x > 0 )
                        visit( i - 1 );
                    if( x + 1 < w )
                        visit( i + 1 );
                    if( y > 0 )
                        visit( i - size_t( w ) );
                    if( y + 1 < h )
                        visit( i + size_t( w ) );
                }
                if( region.size() > kMaxHolePixels )
                    for( uint32_t i : region )
                        empty[i] = 1;
            }
        }
        std::vector< uint8_t > bad( npix, 1 ), tmp( npix, 0 );
        parallel_chunks( size_t( h ), chunk_count( size_t( h ), 8 ), [&]( size_t y0, size_t y1, size_t ) {
            for( size_t y = y0; y < y1; ++y )
                for( int x = 0; x < w; ++x )
                {
                    const size_t i = y * size_t( w ) + size_t( x );
                    const float zc = z[i];
                    if( zc <= 0 )
                    {
                        bad[i] = empty[i];
                        continue;
                    }
                    if( x == 0 || y == 0 || x == w - 1 || int( y ) == h - 1 )
                        continue;
                    const float tol = jump_tolerance( zc, 1 );
                    const size_t n4[4] = { i - 1, i + 1, i - size_t( w ), i + size_t( w ) };
                    bool edge = false;
                    for( size_t j : n4 )
                        edge |= z[j] <= 0 ? empty[j] != 0 : std::abs( z[j] - zc ) > tol;
                    bad[i] = edge;
                }
        } );
        // Separable square dilation of 'bad'.
        parallel_chunks( size_t( h ), chunk_count( size_t( h ), 8 ), [&]( size_t y0, size_t y1, size_t ) {
            for( size_t y = y0; y < y1; ++y )
            {
                const uint8_t * row = &bad[y * size_t( w )];
                uint8_t * out = &tmp[y * size_t( w )];
                // Distance to the previous bad pixel (forward sweep) and to the next one (backward sweep).
                int last_bad = -1000000;
                for( int x = 0; x < w; ++x )
                {
                    if( row[x] )
                        last_bad = x;
                    out[x] = x - last_bad <= erode;
                }
                int next_bad = 1000000;
                for( int x = w - 1; x >= 0; --x )
                {
                    if( row[x] )
                        next_bad = x;
                    out[x] |= next_bad - x <= erode;
                }
            }
        } );
        parallel_chunks( size_t( w ), chunk_count( size_t( w ), 8 ), [&]( size_t x0, size_t x1, size_t ) {
            for( size_t x = x0; x < x1; ++x )
            {
                int last_bad = -1000000;
                std::vector< uint8_t > col( size_t( h ), 0 );
                for( int y = 0; y < h; ++y )
                {
                    if( tmp[size_t( y ) * size_t( w ) + x] )
                        last_bad = y;
                    col[size_t( y )] = y - last_bad <= erode;
                }
                int next_bad = 1000000;
                for( int y = h - 1; y >= 0; --y )
                {
                    if( tmp[size_t( y ) * size_t( w ) + x] )
                        next_bad = y;
                    const bool dilated = col[size_t( y )] || next_bad - y <= erode;
                    const size_t i = size_t( y ) * size_t( w ) + x;
                    keep[i] = z[i] > 0 && ! dilated;
                }
            }
        } );
    }

    const int stride = p.stride;
    const int radius = p.normal_radius_px;
    const int step = std::max( 1, radius / 2 );
    const int half = radius / step;
    const bool has_rgb = f.rgb != nullptr;
    const size_t rows = size_t( ( h + stride - 1 ) / stride );
    const size_t chunks = chunk_count( rows, 8 );
    std::vector< fr_cloud > parts( chunks );
    parallel_chunks( rows, chunks, [&]( size_t r0, size_t r1, size_t chunk ) {
        fr_cloud & part = parts[chunk];
        const size_t max_n = size_t( 2 * half + 1 ) * size_t( 2 * half + 1 );
        std::vector< Eigen::Vector3f > offsets( max_n );
        std::vector< float > di( max_n );
        for( size_t r = r0; r < r1; ++r )
        {
            const int y = int( r ) * stride;
            for( int x = 0; x < w; x += stride )
            {
                const size_t i = size_t( y ) * size_t( w ) + size_t( x );
                if( ! keep[i] )
                    continue;
                const float zc = z[i];
                const Eigen::Vector3f pc( ( float( x ) - cx ) / fx * zc, ( float( y ) - cy ) / fy * zc, zc );
                const float ic = has_rgb ? rgb_to_intensity( &f.rgb[3 * i] ) : 0.f;
                size_t m = 0;
                for( int dv = -half; dv <= half; ++dv )
                {
                    const int yn = y + dv * step;
                    if( yn < 0 || yn >= h )
                        continue;
                    for( int du = -half; du <= half; ++du )
                    {
                        const int xn = x + du * step;
                        if( ( du == 0 && dv == 0 ) || xn < 0 || xn >= w )
                            continue;
                        const size_t j = size_t( yn ) * size_t( w ) + size_t( xn );
                        const float zn = z[j];
                        if( zn <= 0 || std::abs( zn - zc ) > jump_tolerance( zc, std::max( std::abs( du ), std::abs( dv ) ) * step ) )
                            continue;
                        offsets[m] = Eigen::Vector3f( ( float( xn ) - cx ) / fx * zn, ( float( yn ) - cy ) / fy * zn, zn ) - pc;
                        di[m] = has_rgb ? rgb_to_intensity( &f.rgb[3 * j] ) - ic : 0.f;
                        ++m;
                    }
                }
                if( m < 5 )
                    continue;
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                for( size_t k = 0; k < m; ++k )
                    mean += offsets[k].cast< double >();
                mean /= double( m + 1 );   // the center itself is at offset 0
                Eigen::Matrix3d cov = mean * mean.transpose();
                for( size_t k = 0; k < m; ++k )
                {
                    const Eigen::Vector3d d = offsets[k].cast< double >() - mean;
                    cov += d * d.transpose();
                }
                Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > es;
                es.computeDirect( cov );
                if( ! ( es.eigenvalues()( 1 ) > 1e-14 ) )
                    continue;   // collinear neighborhood
                Eigen::Vector3f nrm = es.eigenvectors().col( 0 ).cast< float >().normalized();
                if( nrm.dot( pc ) > 0 )
                    nrm = -nrm;
                if( -nrm.dot( pc ) < cos_max * pc.norm() )
                    continue;   // grazing view: noisy depth, often flying pixels
                part.points.insert( part.points.end(), { pc.x(), pc.y(), pc.z() } );
                part.normals.insert( part.normals.end(), { nrm.x(), nrm.y(), nrm.z() } );
                if( has_rgb )
                {
                    part.colors.insert( part.colors.end(), { f.rgb[3 * i], f.rgb[3 * i + 1], f.rgb[3 * i + 2] } );
                    part.intensity.push_back( ic );
                    const Eigen::Vector3f g = tangent_gradient( nrm, offsets.data(), di.data(), m );
                    part.gradients.insert( part.gradients.end(), { g.x(), g.y(), g.z() } );
                }
            }
        }
    } );

    fr_cloud cloud;
    size_t total = 0;
    for( const auto & part : parts )
        total += part.size();
    cloud.points.reserve( 3 * total );
    cloud.normals.reserve( 3 * total );
    if( has_rgb )
    {
        cloud.colors.reserve( 3 * total );
        cloud.intensity.reserve( total );
        cloud.gradients.reserve( 3 * total );
    }
    for( const auto & part : parts )
    {
        cloud.points.insert( cloud.points.end(), part.points.begin(), part.points.end() );
        cloud.normals.insert( cloud.normals.end(), part.normals.begin(), part.normals.end() );
        cloud.colors.insert( cloud.colors.end(), part.colors.begin(), part.colors.end() );
        cloud.intensity.insert( cloud.intensity.end(), part.intensity.begin(), part.intensity.end() );
        cloud.gradients.insert( cloud.gradients.end(), part.gradients.begin(), part.gradients.end() );
    }
    return cloud;
}
}  // namespace fr

extern "C" {

void fr_cloud_params_default( fr_cloud_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->min_depth_m = 0.15f;
    p->max_depth_m = 1.5f;
    p->stride = 1;
    p->normal_radius_px = 3;
    p->max_view_angle_deg = 75.f;
    p->mask_erode_px = 2;
}

fr_status fr_cloud_from_rgbd( const fr_rgbd_frame * frame, const fr_cloud_params * params, fr_cloud ** out )
{
    return fr::guarded( "fr_cloud_from_rgbd", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_from_rgbd: out is NULL" );
        *out = nullptr;
        if( ! frame )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_from_rgbd: frame is NULL" );
        fr_cloud_params p;
        if( params )
            p = *params;
        else
            fr_cloud_params_default( &p );
        std::string error;
        if( ! fr::validate_frame( *frame, error ) || ! fr::validate_cloud_params( p, error ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_from_rgbd: " + error );
        *out = new fr_cloud( fr::cloud_from_rgbd( *frame, p ) );
        return FR_OK;
    } );
}

fr_status fr_cloud_create( const float * points, const float * normals, const uint8_t * colors, uint32_t count,
                           fr_cloud ** out )
{
    return fr::guarded( "fr_cloud_create", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_create: out is NULL" );
        *out = nullptr;
        if( count && ! points )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_create: points is NULL" );
        if( ! fr::all_finite( points, size_t( count ) * 3 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_create: points are not finite" );
        auto cloud = std::make_unique< fr_cloud >();
        cloud->points.assign( points, points + size_t( count ) * 3 );
        if( colors )
        {
            cloud->colors.assign( colors, colors + size_t( count ) * 3 );
            cloud->intensity.resize( count );
            for( size_t i = 0; i < count; ++i )
                cloud->intensity[i] = fr::rgb_to_intensity( &colors[3 * i] );
        }
        if( normals )
        {
            if( ! fr::all_finite( normals, size_t( count ) * 3 ) )
                return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_create: normals are not finite" );
            cloud->normals.assign( normals, normals + size_t( count ) * 3 );
            for( size_t i = 0; i < count; ++i )
            {
                Eigen::Map< Eigen::Vector3f > n( &cloud->normals[3 * i] );
                const float len = n.norm();
                if( len > 0 )
                    n /= len;
            }
        }
        else if( count >= 3 )
        {
            // Without a viewpoint the sensor is assumed at the origin of the cloud's frame (camera frame).
            fr::CloudIndex index( *cloud );
            fr::estimate_normals_knn( *cloud, index, 16, Eigen::Vector3f::Zero() );
        }
        *out = cloud.release();
        return FR_OK;
    } );
}

fr_status fr_cloud_voxel_downsample( const fr_cloud * cloud, float voxel_size_m, fr_cloud ** out )
{
    return fr::guarded( "fr_cloud_voxel_downsample", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_voxel_downsample: out is NULL" );
        *out = nullptr;
        if( ! cloud )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_voxel_downsample: cloud is NULL" );
        if( ! ( voxel_size_m > 0 ) || ! std::isfinite( voxel_size_m ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_voxel_downsample: voxel size must be positive" );
        *out = new fr_cloud( fr::voxel_downsample( *cloud, voxel_size_m ) );
        return FR_OK;
    } );
}

fr_status fr_cloud_transform( fr_cloud * cloud, const float transform[16] )
{
    return fr::guarded( "fr_cloud_transform", [&]() -> fr_status {
        if( ! cloud || ! transform )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_transform: NULL argument" );
        if( ! fr::all_finite( transform, 16 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_transform: transform is not finite" );
        fr::transform_cloud( *cloud, fr::load_matrix4( transform ) );
        return FR_OK;
    } );
}

fr_status fr_cloud_append( fr_cloud * cloud, const fr_cloud * other )
{
    return fr::guarded( "fr_cloud_append", [&]() -> fr_status {
        if( ! cloud || ! other )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_append: NULL argument" );
        if( cloud == other )
        {
            const fr_cloud copy = *other;
            return fr_cloud_append( cloud, &copy );
        }
        if( cloud->size() == 0 )
        {
            *cloud = *other;
            return FR_OK;
        }
        if( other->size() == 0 )
            return FR_OK;
        // An attribute survives only when both clouds have it, so arrays stay one-per-point.
        auto merge = [&]( auto & dst, const auto & src ) {
            if( dst.empty() || src.empty() )
                dst.clear();
            else
                dst.insert( dst.end(), src.begin(), src.end() );
        };
        cloud->points.insert( cloud->points.end(), other->points.begin(), other->points.end() );
        merge( cloud->normals, other->normals );
        merge( cloud->colors, other->colors );
        merge( cloud->intensity, other->intensity );
        merge( cloud->gradients, other->gradients );
        return FR_OK;
    } );
}

uint32_t fr_cloud_size( const fr_cloud * cloud )
{
    return cloud ? uint32_t( std::min< size_t >( cloud->size(), UINT32_MAX ) ) : 0;
}

const float * fr_cloud_points( const fr_cloud * cloud )
{
    return cloud && cloud->size() ? cloud->points.data() : nullptr;
}

const float * fr_cloud_normals( const fr_cloud * cloud )
{
    return cloud && cloud->has_normals() ? cloud->normals.data() : nullptr;
}

const uint8_t * fr_cloud_colors( const fr_cloud * cloud )
{
    return cloud && cloud->has_colors() ? cloud->colors.data() : nullptr;
}

fr_status fr_cloud_bounds( const fr_cloud * cloud, float min[3], float max[3] )
{
    if( ! cloud || ! min || ! max )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_cloud_bounds: NULL argument" );
    if( cloud->size() == 0 )
        return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_cloud_bounds: empty cloud" );
    Eigen::Vector3f lo = cloud->point( 0 ), hi = lo;
    for( size_t i = 1; i < cloud->size(); ++i )
    {
        lo = lo.cwiseMin( cloud->point( i ) );
        hi = hi.cwiseMax( cloud->point( i ) );
    }
    for( int k = 0; k < 3; ++k )
    {
        min[k] = lo[k];
        max[k] = hi[k];
    }
    return FR_OK;
}

void fr_cloud_free( fr_cloud * cloud )
{
    delete cloud;
}

}  // extern "C"
