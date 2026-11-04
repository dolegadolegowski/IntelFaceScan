// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal helpers of the reconstruction module (registration and fusion): SE(3) algebra in double precision,
// conversions from the C API's column-major float[16], and a parallel-for on Grand Central Dispatch.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dispatch/dispatch.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>

namespace fr
{
using Matrix6d = Eigen::Matrix< double, 6, 6 >;
using Vector6d = Eigen::Matrix< double, 6, 1 >;

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad( double deg ) { return deg * kPi / 180.0; }
inline double rad2deg( double rad ) { return rad * 180.0 / kPi; }

inline Eigen::Matrix4d load_matrix4( const float m[16] )
{
    return Eigen::Map< const Eigen::Matrix4f >( m ).cast< double >();
}

inline void store_matrix4( const Eigen::Matrix4d & m, float out[16] )
{
    Eigen::Map< Eigen::Matrix4f > dst( out );
    dst = m.cast< float >();
}

inline bool all_finite( const float * values, size_t count )
{
    for( size_t i = 0; i < count; ++i )
        if( ! std::isfinite( values[i] ) )
            return false;
    return true;
}

inline Eigen::Matrix3d skew( const Eigen::Vector3d & v )
{
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

inline Eigen::Matrix4d rigid_inverse( const Eigen::Matrix4d & t )
{
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rt = t.block< 3, 3 >( 0, 0 ).transpose();
    inv.block< 3, 3 >( 0, 0 ) = rt;
    inv.block< 3, 1 >( 0, 3 ) = -rt * t.block< 3, 1 >( 0, 3 );
    return inv;
}

// Rotation angle of R in radians, [0, pi]. Robust for slightly non-orthonormal input.
inline double rotation_angle( const Eigen::Matrix3d & r )
{
    const double c = std::clamp( ( r.trace() - 1.0 ) * 0.5, -1.0, 1.0 );
    // acos is ill-conditioned near 0; use the skew part there.
    const Eigen::Vector3d s( r( 2, 1 ) - r( 1, 2 ), r( 0, 2 ) - r( 2, 0 ), r( 1, 0 ) - r( 0, 1 ) );
    return std::atan2( 0.5 * s.norm(), c );
}

// Nearest rotation matrix (polar decomposition), used after accumulating float round-off.
inline Eigen::Matrix3d orthonormalize( const Eigen::Matrix3d & r )
{
    Eigen::JacobiSVD< Eigen::Matrix3d > svd( r, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Matrix3d out = svd.matrixU() * svd.matrixV().transpose();
    if( out.determinant() < 0 )
    {
        Eigen::Matrix3d u = svd.matrixU();
        u.col( 2 ) *= -1;
        out = u * svd.matrixV().transpose();
    }
    return out;
}

// SE(3) exponential of xi = [omega (rotation vector), v]: a left perturbation T <- exp(xi) * T.
inline Eigen::Matrix4d se3_exp( const Vector6d & xi )
{
    const Eigen::Vector3d w = xi.head< 3 >(), v = xi.tail< 3 >();
    const double th2 = w.squaredNorm(), th = std::sqrt( th2 );
    const Eigen::Matrix3d W = skew( w ), W2 = W * W;
    double a, b, c;
    if( th < 1e-4 )
    {
        a = 1.0 - th2 / 6.0;
        b = 0.5 - th2 / 24.0;
        c = 1.0 / 6.0 - th2 / 120.0;
    }
    else
    {
        a = std::sin( th ) / th;
        b = ( 1.0 - std::cos( th ) ) / th2;
        c = ( th - std::sin( th ) ) / ( th2 * th );
    }
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block< 3, 3 >( 0, 0 ) = Eigen::Matrix3d::Identity() + a * W + b * W2;
    t.block< 3, 1 >( 0, 3 ) = ( Eigen::Matrix3d::Identity() + b * W + c * W2 ) * v;
    return t;
}

inline Vector6d se3_log( const Eigen::Matrix4d & t )
{
    const Eigen::Matrix3d r = t.block< 3, 3 >( 0, 0 );
    const Eigen::AngleAxisd aa( Eigen::Quaterniond( r ).normalized() );
    const double th = aa.angle();
    const Eigen::Vector3d w = aa.axis() * th;
    const Eigen::Matrix3d W = skew( w ), W2 = W * W;
    double k;
    if( th < 1e-4 )
        k = 1.0 / 12.0 + th * th / 720.0;
    else
    {
        const double a = std::sin( th ) / th, b = ( 1.0 - std::cos( th ) ) / ( th * th );
        k = ( 1.0 - a / ( 2.0 * b ) ) / ( th * th );
    }
    const Eigen::Matrix3d v_inv = Eigen::Matrix3d::Identity() - 0.5 * W + k * W2;
    Vector6d xi;
    xi.head< 3 >() = w;
    xi.tail< 3 >() = v_inv * t.block< 3, 1 >( 0, 3 );
    return xi;
}

// Adjoint for the [omega, v] ordering: T * exp(xi) * T^-1 = exp(Ad(T) * xi).
inline Matrix6d se3_adjoint( const Eigen::Matrix4d & t )
{
    const Eigen::Matrix3d r = t.block< 3, 3 >( 0, 0 );
    Matrix6d ad = Matrix6d::Zero();
    ad.block< 3, 3 >( 0, 0 ) = r;
    ad.block< 3, 3 >( 3, 3 ) = r;
    ad.block< 3, 3 >( 3, 0 ) = skew( t.block< 3, 1 >( 0, 3 ) ) * r;
    return ad;
}

// Small adjoint ad(xi) for the [omega, v] ordering.
inline Matrix6d se3_ad( const Vector6d & xi )
{
    Matrix6d ad = Matrix6d::Zero();
    const Eigen::Matrix3d W = skew( xi.head< 3 >() );
    ad.block< 3, 3 >( 0, 0 ) = W;
    ad.block< 3, 3 >( 3, 3 ) = W;
    ad.block< 3, 3 >( 3, 0 ) = skew( xi.tail< 3 >() );
    return ad;
}

// Runs f(i) for i in [0, count) on GCD worker threads (nested calls are fine). The first exception thrown by
// an iteration is rethrown on the calling thread after all iterations finished; exceptions must not unwind
// through libdispatch's C frames.
template< typename F >
void parallel_for( size_t count, F && f )
{
    if( count == 0 )
        return;
    if( count == 1 )
    {
        f( size_t( 0 ) );
        return;
    }
    struct Context
    {
        F * f;
        std::exception_ptr error;
        std::once_flag once;
        std::atomic< bool > failed { false };
    } context { &f };
    dispatch_apply_f( count, DISPATCH_APPLY_AUTO, &context, []( void * raw, size_t i ) {
        auto * c = static_cast< Context * >( raw );
        if( c->failed.load( std::memory_order_relaxed ) )
            return;
        try
        {
            ( *c->f )( i );
        }
        catch( ... )
        {
            std::call_once( c->once, [&] { c->error = std::current_exception(); } );
            c->failed.store( true );
        }
    } );
    if( context.error )
        std::rethrow_exception( context.error );
}

// Splits [0, count) into contiguous chunks and runs f(begin, end, chunk_index) in parallel; returns the number of
// chunks (callers size per-chunk accumulators with chunk_count()).
inline size_t chunk_count( size_t count, size_t min_chunk = 1024 )
{
    if( count == 0 )
        return 0;
    const size_t workers = std::max( 1u, std::thread::hardware_concurrency() );
    const size_t chunks = std::min( count / std::max< size_t >( 1, min_chunk ) + 1, workers * 4 );
    return std::max< size_t >( 1, std::min( chunks, count ) );
}

template< typename F >
void parallel_chunks( size_t count, size_t chunks, F && f )
{
    if( chunks == 0 )
        return;
    parallel_for( chunks, [&]( size_t c ) {
        const size_t begin = count * c / chunks, end = count * ( c + 1 ) / chunks;
        if( begin < end )
            f( begin, end, c );
    } );
}
}  // namespace fr
