// License: Apache 2.0. See LICENSE file in root directory.
//
// Multi-scale ICP: point-to-plane (Gauss-Newton on se(3) with Huber/Tukey weights) and colored ICP (Park, Zhou,
// Koltun: "Colored Point Cloud Registration Revisited", ICCV 2017) with nanoflann correspondences.
//
// Both minimize over a left perturbation T <- exp(xi) T, xi = [omega, v], in the target frame:
//   geometric  r_G = (T q - p) . n_p                     J_G = [ q' x n_p ; n_p ]
//   photometric r_C = I_p + d_p . (T q - p) - g I_q       J_C = [ q' x d_p ; d_p ]
// where d_p is the intensity gradient in p's tangent plane and g the exposure gain between the views (median
// intensity ratio of the current correspondences; auto exposure scales intensities). The robust kernel acts on each
// residual separately: on r_G with robust_scale_m, on r_C (Tukey) with a scale from the median absolute deviation
// of the photometric residuals, clamped to [kPhotoKernelMin, kPhotoKernelMax]. Without it the ~2% of
// correspondences on high-contrast texture edges (eyes, brows, lips, hairline), where the linear intensity model
// fails, have residuals 10-50x the color noise and dominate the normal equations: started at the true pose, ICP
// drifted 0.1-0.6 degrees on noise-free synthetic views (docs/RECONSTRUCTION.md).
// The 6x6 information matrix of the result is computed as in Open3D (sum of G^T G with G = [-[p]x | I] over inlier
// correspondences), so information(5,5) = correspondence count.

#include "recon_cloud.h"
#include "../common/fr_internal.h"

#include <Eigen/Cholesky>

#include <cstring>
#include <limits>
#include <stdexcept>

namespace fr
{
namespace
{
// Photometric Tukey scale (intensity units, [0, 1] scale): 4.685 sigma of the residuals, sigma = 1.4826 MAD.
constexpr double kPhotoKernelMin = 0.02;    // ~4.7x the noise of a clean 8-bit color stream
constexpr double kPhotoKernelMax = 0.25;    // wide while misaligned: keeps the capture range on strong textures
constexpr float kMinGainIntensity = 0.02f;  // darker source points do not vote for the exposure gain

struct Accumulator
{
    Matrix6d h = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    size_t count = 0;
};

// Correspondence of one source point in the current iteration.
struct Match
{
    uint32_t target = std::numeric_limits< uint32_t >::max();   // max = none
    float predicted = 0;                                         // I_p + d_p . (T q - p) (colored levels)
};

double median( std::vector< double > & v )
{
    std::nth_element( v.begin(), v.begin() + std::ptrdiff_t( v.size() / 2 ), v.end() );
    return v[v.size() / 2];
}

double robust_weight( int kernel, double r, double c )
{
    if( c <= 0 )
        return 1.0;
    const double a = std::abs( r );
    switch( kernel )
    {
    case 1:   // Huber
        return a <= c ? 1.0 : c / a;
    case 2:   // Tukey biweight
    {
        if( a >= c )
            return 0.0;
        const double t = 1.0 - ( r / c ) * ( r / c );
        return t * t;
    }
    default:
        return 1.0;
    }
}

float mean_spacing( const fr_cloud & cloud, const CloudIndex & index )
{
    const size_t n = cloud.size();
    if( n < 2 )
        return 0.001f;
    const size_t samples = std::min< size_t >( n, 1000 );
    double sum = 0;
    size_t used = 0;
    for( size_t s = 0; s < samples; ++s )
    {
        const size_t i = s * n / samples;
        uint32_t idx[2];
        float d2[2];
        if( index.knn( &cloud.points[3 * i], 2, idx, d2, 0.f ) == 2 )
        {
            sum += std::sqrt( double( d2[1] ) );
            ++used;
        }
    }
    return used ? float( sum / double( used ) ) : 0.001f;
}
}  // namespace

bool validate_icp_params( const fr_icp_params & p, std::string & error )
{
    if( p.levels < 1 || p.levels > 8 )
        error = "levels must be in [1, 8]";
    else if( p.max_iterations < 0 || p.max_iterations > 1000 )
        error = "max_iterations must be in [0, 1000]";
    else if( ! ( p.voxel_size_m >= 0 ) || ! std::isfinite( p.voxel_size_m ) )
        error = "voxel_size_m must be >= 0";
    else if( ! ( p.max_correspondence_m > 0 ) || ! std::isfinite( p.max_correspondence_m ) )
        error = "max_correspondence_m must be positive";
    else if( p.colored && ! ( p.color_lambda > 0 && p.color_lambda <= 1 ) )
        error = "color_lambda must be in (0, 1]";
    else if( p.robust_kernel < 0 || p.robust_kernel > 2 )
        error = "robust_kernel must be 0, 1 or 2";
    else if( p.robust_kernel != 0 && ! ( p.robust_scale_m > 0 ) )
        error = "robust_scale_m must be positive";
    else if( ! ( p.convergence_rotation_deg >= 0 ) || ! ( p.convergence_translation_m >= 0 ) )
        error = "convergence thresholds must be >= 0";
    else
        return true;
    return false;
}

CloudPyramid build_pyramid( const fr_cloud & cloud, int levels, float voxel, bool with_index, bool with_gradients )
{
    CloudPyramid pyramid;
    if( ! ( voxel > 0 ) )
        levels = 1;
    levels = std::clamp( levels, 1, 8 );
    for( int l = 0; l < levels; ++l )
    {
        auto level = std::make_unique< PyramidLevel >();
        level->voxel = voxel > 0 ? voxel * float( 1 << l ) : 0.f;
        // Every level averages the raw points (not the previous level) to keep coarse positions unbiased.
        level->cloud = voxel > 0 ? voxel_downsample( cloud, level->voxel ) : cloud;
        if( with_index || ( with_gradients && ! level->cloud.has_gradients() ) )
            level->index = std::make_unique< CloudIndex >( level->cloud );
        if( with_gradients && ! level->cloud.has_gradients() && level->cloud.has_intensity() && level->cloud.has_normals() )
        {
            const float spacing = level->voxel > 0 ? level->voxel : mean_spacing( level->cloud, *level->index );
            estimate_gradients_knn( level->cloud, *level->index, 2.5f * spacing, 30 );
        }
        if( ! with_index )
            level->index.reset();
        pyramid.levels.push_back( std::move( level ) );
    }
    return pyramid;
}

fr_status icp_pyramids( const CloudPyramid & source, const CloudPyramid & target, const Eigen::Matrix4d & init,
                        const fr_icp_params & p, IcpOutput & out, const std::atomic< bool > * cancel )
{
    out = IcpOutput();
    const int levels = int( std::min( source.levels.size(), target.levels.size() ) );
    if( levels == 0 )
        return fail( FR_ERR_NOT_ENOUGH_DATA, "icp: empty pyramid" );
    if( source.levels[0]->cloud.size() < 6 || target.levels[0]->cloud.size() < 6 )
        return fail( FR_ERR_NOT_ENOUGH_DATA, "icp: too few points" );
    if( ! target.levels[0]->cloud.has_normals() )
        return fail( FR_ERR_INVALID_ARGUMENT, "icp: point-to-plane ICP needs target normals" );

    const double lambda = p.colored ? double( p.color_lambda ) : 1.0;
    const double conv_rot = deg2rad( p.convergence_rotation_deg ), conv_t = p.convergence_translation_m;
    Eigen::Matrix4d t = init;
    int iterations = 0;
    bool converged = false;

    for( int level = levels - 1; level >= 0; --level )
    {
        const fr_cloud & src = source.levels[size_t( level )]->cloud;
        const fr_cloud & dst = target.levels[size_t( level )]->cloud;
        const CloudIndex * index = target.levels[size_t( level )]->index.get();
        if( ! index )
            throw std::logic_error( "icp: target pyramid without index" );
        if( src.size() == 0 || dst.size() == 0 || ! dst.has_normals() )
            continue;
        const double scale = double( 1 << level );
        const float max_d2 = float( p.max_correspondence_m * scale ) * float( p.max_correspondence_m * scale );
        const double kernel = double( p.robust_scale_m ) * scale;
        const bool colored = p.colored && level == 0 && src.has_intensity() && dst.has_intensity() && dst.has_gradients();
        const size_t n = src.size();
        const size_t chunks = chunk_count( n, 2048 );
        std::vector< Match > matches( colored ? n : 0 );
        converged = false;

        for( int it = 0; it < p.max_iterations; ++it )
        {
            if( cancel && cancel->load( std::memory_order_relaxed ) )
                return fail( FR_ERR_CANCELLED, "icp: cancelled" );
            const Eigen::Matrix3f r = t.block< 3, 3 >( 0, 0 ).cast< float >();
            const Eigen::Vector3f tr = t.block< 3, 1 >( 0, 3 ).cast< float >();
            // Colored levels: correspondences first, so the exposure gain and the photometric kernel scale come from
            // this iteration's residuals before any of them is weighted.
            double gain = 1, photo_kernel = kPhotoKernelMax;
            if( colored )
            {
                parallel_chunks( n, chunks, [&]( size_t begin, size_t end, size_t ) {
                    for( size_t i = begin; i < end; ++i )
                    {
                        Match & m = matches[i];
                        m = Match();
                        const Eigen::Vector3f q = r * src.point( i ) + tr;
                        uint32_t j;
                        float d2;
                        if( ! index->nearest( q.data(), max_d2, j, d2 ) )
                            continue;
                        const Eigen::Vector3f delta = q - dst.point( j );
                        if( robust_weight( p.robust_kernel, double( delta.dot( dst.normal( j ) ) ), kernel ) <= 0 )
                            continue;
                        m.target = j;
                        m.predicted = dst.intensity[j] + dst.gradient( j ).dot( delta );
                    }
                } );
                std::vector< double > values;
                values.reserve( n );
                for( size_t i = 0; i < n; ++i )
                    if( matches[i].target != std::numeric_limits< uint32_t >::max() && src.intensity[i] > kMinGainIntensity )
                        values.push_back( double( matches[i].predicted ) / double( src.intensity[i] ) );
                if( values.size() >= 6 )
                {
                    gain = median( values );
                    values.clear();
                    for( size_t i = 0; i < n; ++i )
                        if( matches[i].target != std::numeric_limits< uint32_t >::max() )
                            values.push_back( std::abs( double( matches[i].predicted ) - gain * double( src.intensity[i] ) ) );
                    photo_kernel = std::clamp( 4.685 * 1.4826 * median( values ), kPhotoKernelMin, kPhotoKernelMax );
                }
            }
            std::vector< Accumulator > parts( chunks );
            parallel_chunks( n, chunks, [&]( size_t begin, size_t end, size_t c ) {
                Accumulator & acc = parts[c];
                for( size_t i = begin; i < end; ++i )
                {
                    const Eigen::Vector3f q = r * src.point( i ) + tr;
                    uint32_t j;
                    if( colored )
                    {
                        j = matches[i].target;
                        if( j == std::numeric_limits< uint32_t >::max() )
                            continue;
                    }
                    else
                    {
                        float d2;
                        if( ! index->nearest( q.data(), max_d2, j, d2 ) )
                            continue;
                    }
                    const Eigen::Vector3d qd = q.cast< double >();
                    const Eigen::Vector3d pd = dst.point( j ).cast< double >();
                    const Eigen::Vector3d nd = dst.normal( j ).cast< double >();
                    const double rg = ( qd - pd ).dot( nd );
                    const double w = robust_weight( p.robust_kernel, rg, kernel );
                    if( w <= 0 )
                        continue;
                    Vector6d jg;
                    jg.head< 3 >() = qd.cross( nd );
                    jg.tail< 3 >() = nd;
                    acc.h.noalias() += ( lambda * w ) * jg * jg.transpose();
                    acc.b.noalias() += ( lambda * w * rg ) * jg;
                    if( colored )
                    {
                        const Eigen::Vector3d dp = dst.gradient( j ).cast< double >();
                        const double rc = double( dst.intensity[j] ) + dp.dot( qd - pd ) - gain * double( src.intensity[i] );
                        const double wc = w * robust_weight( 2, rc, photo_kernel );
                        if( wc > 0 )
                        {
                            Vector6d jc;
                            jc.head< 3 >() = qd.cross( dp );
                            jc.tail< 3 >() = dp;
                            acc.h.noalias() += ( ( 1.0 - lambda ) * wc ) * jc * jc.transpose();
                            acc.b.noalias() += ( ( 1.0 - lambda ) * wc * rc ) * jc;
                        }
                    }
                    ++acc.count;
                }
            } );
            Accumulator total;
            for( const auto & a : parts )
            {
                total.h += a.h;
                total.b += a.b;
                total.count += a.count;
            }
            if( total.count < 6 )
            {
                if( level > 0 )
                    break;   // coarse level found nothing usable; the finer level has more points
                return fail( FR_ERR_NOT_ENOUGH_DATA, "icp: too few correspondences (" + std::to_string( total.count ) + ")" );
            }
            // A tiny ridge keeps directions the data does not constrain (e.g. sliding on a plane) at zero update.
            Matrix6d h = total.h;
            h.diagonal().array() += 1e-9 * std::max( 1e-12, h.trace() / 6.0 );
            const Vector6d xi = h.ldlt().solve( -total.b );
            if( ! xi.allFinite() )
                return fail( FR_ERR_FAILED, "icp: singular system" );
            t = se3_exp( xi ) * t;
            ++iterations;
            if( xi.head< 3 >().norm() < conv_rot && xi.tail< 3 >().norm() < conv_t )
            {
                converged = true;
                break;
            }
        }
    }
    t.block< 3, 3 >( 0, 0 ) = orthonormalize( t.block< 3, 3 >( 0, 0 ) );

    // Evaluation at the finest level with the final transform.
    const fr_cloud & src = source.levels[0]->cloud;
    const fr_cloud & dst = target.levels[0]->cloud;
    const CloudIndex & index = *target.levels[0]->index;
    const float max_d2 = p.max_correspondence_m * p.max_correspondence_m;
    const Eigen::Matrix3f r = t.block< 3, 3 >( 0, 0 ).cast< float >();
    const Eigen::Vector3f tr = t.block< 3, 1 >( 0, 3 ).cast< float >();
    struct Eval
    {
        Matrix6d info = Matrix6d::Zero();
        double sse = 0;
        size_t count = 0;
    };
    const size_t n = src.size();
    const size_t chunks = chunk_count( n, 2048 );
    std::vector< Eval > parts( chunks );
    parallel_chunks( n, chunks, [&]( size_t begin, size_t end, size_t c ) {
        Eval & e = parts[c];
        Eigen::Matrix< double, 3, 6 > g;
        g.block< 3, 3 >( 0, 3 ).setIdentity();
        for( size_t i = begin; i < end; ++i )
        {
            const Eigen::Vector3f q = r * src.point( i ) + tr;
            uint32_t j;
            float d2;
            if( ! index.nearest( q.data(), max_d2, j, d2 ) )
                continue;
            const Eigen::Vector3d pd = dst.point( j ).cast< double >();
            const double rg = ( q.cast< double >() - pd ).dot( dst.normal( j ).cast< double >() );
            e.sse += rg * rg;
            ++e.count;
            g.block< 3, 3 >( 0, 0 ) = -skew( pd );
            e.info.noalias() += g.transpose() * g;
        }
    } );
    Eval total;
    for( const auto & e : parts )
    {
        total.info += e.info;
        total.sse += e.sse;
        total.count += e.count;
    }
    out.target_from_source = t;
    out.correspondences = uint32_t( total.count );
    out.fitness = n ? float( double( total.count ) / double( n ) ) : 0.f;
    out.inlier_rmse = total.count ? float( std::sqrt( total.sse / double( total.count ) ) ) : 0.f;
    out.iterations = iterations;
    out.converged = converged || p.max_iterations == 0;
    out.information = total.info;
    return FR_OK;
}
}  // namespace fr

extern "C" {

void fr_icp_params_default( fr_icp_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->levels = 3;
    p->max_iterations = 30;
    p->voxel_size_m = 0.001f;
    p->max_correspondence_m = 0.004f;
    p->colored = 1;
    p->color_lambda = 0.968f;
    p->robust_kernel = 2;
    p->robust_scale_m = 0.003f;
    p->convergence_rotation_deg = 0.01f;
    p->convergence_translation_m = 1e-5f;
}

// max_iterations = 0 only evaluates fitness, RMSE and information at the initial transform.
fr_status fr_icp( const fr_cloud * source, const fr_cloud * target, const float init_target_from_source[16],
                  const fr_icp_params * params, fr_icp_result * out )
{
    return fr::guarded( "fr_icp", [&]() -> fr_status {
        if( ! source || ! target || ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_icp: NULL argument" );
        std::memset( out, 0, sizeof( *out ) );
        fr_icp_params p;
        if( params )
            p = *params;
        else
            fr_icp_params_default( &p );
        std::string error;
        if( ! fr::validate_icp_params( p, error ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_icp: " + error );
        Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
        if( init_target_from_source )
        {
            if( ! fr::all_finite( init_target_from_source, 16 ) )
                return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_icp: initial transform is not finite" );
            init = fr::load_matrix4( init_target_from_source );
        }
        if( ! target->has_normals() )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_icp: point-to-plane ICP needs target normals" );
        const fr::CloudPyramid src = fr::build_pyramid( *source, p.levels, p.voxel_size_m, false, false );
        const fr::CloudPyramid dst = fr::build_pyramid( *target, p.levels, p.voxel_size_m, true, p.colored != 0 );
        fr::IcpOutput result;
        const fr_status status = fr::icp_pyramids( src, dst, init, p, result, nullptr );
        if( status != FR_OK )
            return status;
        fr::store_matrix4( result.target_from_source, out->target_from_source );
        out->fitness = result.fitness;
        out->inlier_rmse_m = result.inlier_rmse;
        out->correspondences = result.correspondences;
        out->iterations = result.iterations;
        out->converged = result.converged ? 1 : 0;
        Eigen::Map< fr::Matrix6d >( out->information ) = result.information;
        return FR_OK;
    } );
}

}  // extern "C"
