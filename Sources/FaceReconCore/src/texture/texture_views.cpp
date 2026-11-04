// License: Apache 2.0. See LICENSE file in root directory.
//
// Per-frame visibility and view weights, exposure compensation and vertex colors (see texture_views.h).

#include "texture_views.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <string>

namespace fr
{
namespace texture
{

using namespace fr::mesh;

namespace
{

// Pixel centers are at integer coordinates (OpenCV/librealsense convention): u = fx * x / z + cx.
void render_zbuffer( const fr_mesh & m, frame_view & v )
{
    const int w = v.width, h = v.height;
    v.zbuffer.assign( size_t( w ) * h, 0 );
    const uint32_t n = m.vertex_count;
    std::vector< vec3 > screen( n );  // x, y in pixels, z in meters (0 = behind the camera)
    for( uint32_t i = 0; i < n; ++i )
    {
        const vec3 pc = v.camera_from_world.apply( vec3::load( m.positions + size_t( i ) * 3 ) );
        if( ! ( pc.z > 0.01f ) || ! finite( pc ) )
        {
            screen[i] = vec3( 0, 0, 0 );
            continue;
        }
        screen[i] = vec3( v.fx * pc.x / pc.z + v.cx, v.fy * pc.y / pc.z + v.cy, pc.z );
    }
    uint16_t * zb = v.zbuffer.data();
    for( uint32_t f = 0; f < m.triangle_count; ++f )
    {
        const uint32_t * tri = m.indices + size_t( f ) * 3;
        const vec3 a = screen[tri[0]], b = screen[tri[1]], c = screen[tri[2]];
        if( a.z <= 0 || b.z <= 0 || c.z <= 0 || ! finite( a ) || ! finite( b ) || ! finite( c ) )
            continue;
        const float area = ( b.x - a.x ) * ( c.y - a.y ) - ( b.y - a.y ) * ( c.x - a.x );
        if( ! ( std::fabs( area ) > 1e-12f ) )
            continue;
        // Clamped before the conversion: vertices just in front of the camera project far outside the image.
        auto pixel = [&]( float v, int n ) { return std::clamp( v, -2.f, float( n ) + 2.f ); };
        const int x0 = std::max( 0, int( std::ceil( pixel( std::min( { a.x, b.x, c.x } ), w ) ) ) );
        const int x1 = std::min( w - 1, int( std::floor( pixel( std::max( { a.x, b.x, c.x } ), w ) ) ) );
        const int y0 = std::max( 0, int( std::ceil( pixel( std::min( { a.y, b.y, c.y } ), h ) ) ) );
        const int y1 = std::min( h - 1, int( std::floor( pixel( std::max( { a.y, b.y, c.y } ), h ) ) ) );
        if( x0 > x1 || y0 > y1 )
            continue;
        const float inv_area = 1.f / area;
        const float iza = 1.f / a.z, izb = 1.f / b.z, izc = 1.f / c.z;
        for( int y = y0; y <= y1; ++y )
            for( int x = x0; x <= x1; ++x )
            {
                const float px = float( x ), py = float( y );
                const float l0 = ( ( c.x - b.x ) * ( py - b.y ) - ( c.y - b.y ) * ( px - b.x ) ) * inv_area;
                const float l1 = ( ( a.x - c.x ) * ( py - c.y ) - ( a.y - c.y ) * ( px - c.x ) ) * inv_area;
                const float l2 = 1.f - l0 - l1;
                if( l0 < 0 || l1 < 0 || l2 < 0 )
                    continue;
                // 1/z is linear in screen space.
                const float z = 1.f / ( l0 * iza + l1 * izb + l2 * izc );
                const float q = z / zbuffer_unit;
                const uint16_t zq = uint16_t( std::clamp( q + 0.5f, 1.f, 65535.f ) );
                uint16_t & dst = zb[size_t( y ) * w + x];
                if( dst == 0 || zq < dst )
                    dst = zq;
            }
    }
}

// Weight ramp from 0 at invalid pixels (outside the mask, no mesh surface, depth discontinuities, image border) to 1
// at feather_px pixels away from them: colors near silhouettes are the most likely to be misregistered or mixed.
void compute_feather( frame_view & v )
{
    const int w = v.width, h = v.height;
    const size_t count = size_t( w ) * h;
    std::vector< uint8_t > valid( count, 0 );
    const uint8_t * mask = v.frame->mask;
    for( size_t i = 0; i < count; ++i )
        valid[i] = v.zbuffer[i] != 0 && ( ! mask || mask[i] >= 128 );
    auto jump = [&]( size_t i, size_t j ) {
        const float zi = v.zbuffer[i] * zbuffer_unit, zj = v.zbuffer[j] * zbuffer_unit;
        return std::fabs( zi - zj ) > std::max( 0.01f, 0.03f * std::min( zi, zj ) );
    };
    for( int y = 0; y < h; ++y )
        for( int x = 0; x < w; ++x )
        {
            const size_t i = size_t( y ) * w + x;
            if( ! v.zbuffer[i] )
                continue;
            if( x + 1 < w && v.zbuffer[i + 1] && jump( i, i + 1 ) )
                valid[i] = valid[i + 1] = 0;
            if( y + 1 < h && v.zbuffer[i + w] && jump( i, i + w ) )
                valid[i] = valid[i + w] = 0;
        }

    // Two-pass 3-4 chamfer distance; pixels outside the image count as invalid.
    constexpr uint16_t inf = 60000;
    std::vector< uint16_t > d( count );
    for( size_t i = 0; i < count; ++i )
        d[i] = valid[i] ? inf : 0;
    auto at = [&]( int x, int y ) -> uint16_t { return x < 0 || y < 0 || x >= w || y >= h ? 0 : d[size_t( y ) * w + x]; };
    for( int y = 0; y < h; ++y )
        for( int x = 0; x < w; ++x )
        {
            uint16_t & c = d[size_t( y ) * w + x];
            if( ! c )
                continue;
            c = std::min< uint16_t >( { c, uint16_t( at( x - 1, y ) + 3 ), uint16_t( at( x, y - 1 ) + 3 ),
                                        uint16_t( at( x - 1, y - 1 ) + 4 ), uint16_t( at( x + 1, y - 1 ) + 4 ) } );
        }
    for( int y = h - 1; y >= 0; --y )
        for( int x = w - 1; x >= 0; --x )
        {
            uint16_t & c = d[size_t( y ) * w + x];
            if( ! c )
                continue;
            c = std::min< uint16_t >( { c, uint16_t( at( x + 1, y ) + 3 ), uint16_t( at( x, y + 1 ) + 3 ),
                                        uint16_t( at( x + 1, y + 1 ) + 4 ), uint16_t( at( x - 1, y + 1 ) + 4 ) } );
        }
    const float feather_px = std::max( 4.f, 0.02f * float( w ) );
    v.feather.resize( count );
    for( size_t i = 0; i < count; ++i )
    {
        const float t = std::min( 1.f, float( d[i] ) / 3.f / feather_px );
        v.feather[i] = uint8_t( std::lround( 255.f * t * t * ( 3.f - 2.f * t ) ) );
    }
}

inline void bilinear_rgb( const uint8_t * rgb, int w, int h, float u, float v, float out[3] )
{
    const int x0 = std::min( int( u ), w - 1 ), y0 = std::min( int( v ), h - 1 );
    const int x1 = std::min( x0 + 1, w - 1 ), y1 = std::min( y0 + 1, h - 1 );
    const float fx = u - float( x0 ), fy = v - float( y0 );
    const uint8_t * p00 = rgb + ( size_t( y0 ) * w + x0 ) * 3;
    const uint8_t * p10 = rgb + ( size_t( y0 ) * w + x1 ) * 3;
    const uint8_t * p01 = rgb + ( size_t( y1 ) * w + x0 ) * 3;
    const uint8_t * p11 = rgb + ( size_t( y1 ) * w + x1 ) * 3;
    for( int k = 0; k < 3; ++k )
        out[k] = ( 1 - fy ) * ( ( 1 - fx ) * p00[k] + fx * p10[k] ) + fy * ( ( 1 - fx ) * p01[k] + fx * p11[k] );
}

}  // namespace

bool cancel_state::report( float fraction, const char * stage )
{
    if( cancelled.load() )
        return false;
    if( ! progress )
        return true;
    std::lock_guard< std::mutex > lock( mutex );
    if( ! fr::report( progress, user, fraction, stage ) )
        cancelled = true;
    return ! cancelled.load();
}

fr_status check_params( const fr_texture_params * p, const char * where, bool need_texture_size )
{
    if( ! p )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": params is NULL" );
    if( ( need_texture_size && ( p->texture_size < 16 || p->texture_size > 32768 ) ) || ! std::isfinite( p->view_angle_power )
        || p->view_angle_power < 0 || ! std::isfinite( p->min_cos_angle ) || p->min_cos_angle >= 1 || p->blend_views < 1
        || ! std::isfinite( p->occlusion_epsilon_m ) || p->occlusion_epsilon_m <= 0 || p->dilate_px < 0 )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": invalid texture parameters" );
    return FR_OK;
}

fr_status prepare_views( const fr_mesh & mesh, const fr_rgbd_frame * frames, const float * world_from_camera,
                         const float * frame_weights, uint32_t count, const fr_texture_params & params, view_set & out,
                         const char * where )
{
    if( count == 0 )
        return fail( FR_ERR_NOT_ENOUGH_DATA, std::string( where ) + ": no frames" );
    if( ! frames || ! world_from_camera )
        return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": frames or poses are NULL" );
    out.min_cos = std::max( params.min_cos_angle, 1e-3f );
    out.power = params.view_angle_power;
    out.epsilon = params.occlusion_epsilon_m;
    out.blend = std::clamp( params.blend_views, 1, max_blend_views );
    out.views.clear();
    for( uint32_t i = 0; i < count; ++i )
    {
        const fr_rgbd_frame & f = frames[i];
        if( ! f.rgb )
            continue;
        const fr_intrinsics & in = f.intrinsics;
        if( in.width <= 0 || in.height <= 0 || in.width > 16384 || in.height > 16384 || ! ( in.fx > 0 ) || ! ( in.fy > 0 )
            || ! std::isfinite( in.fx ) || ! std::isfinite( in.fy ) || ! std::isfinite( in.cx ) || ! std::isfinite( in.cy )
            || ( f.depth && ( ! std::isfinite( f.depth_scale ) || f.depth_scale < 0 ) ) )
            return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": invalid intrinsics of frame " + std::to_string( i ) );
        float weight = frame_weights ? frame_weights[i] : 1.f;
        weight *= f.weight > 0 ? f.weight : 1.f;
        if( ! std::isfinite( weight ) || weight <= 0 )
            continue;
        const affine pose = affine::from_column_major( world_from_camera + size_t( i ) * 16 );
        frame_view v;
        if( ! pose.is_finite() || ! pose.inverse( v.camera_from_world ) )
            return fail( FR_ERR_INVALID_ARGUMENT, std::string( where ) + ": invalid pose of frame " + std::to_string( i ) );
        v.frame = &f;
        v.fx = in.fx;
        v.fy = in.fy;
        v.cx = in.cx;
        v.cy = in.cy;
        v.width = in.width;
        v.height = in.height;
        v.weight = weight;
        out.views.push_back( std::move( v ) );
    }
    if( out.views.empty() )
        return fail( FR_ERR_NOT_ENOUGH_DATA, std::string( where ) + ": no frame with color and a positive weight" );
    parallel_for( out.views.size(), 1, [&]( size_t begin, size_t end ) {
        for( size_t i = begin; i < end; ++i )
        {
            render_zbuffer( mesh, out.views[i] );
            compute_feather( out.views[i] );
        }
    } );
    return FR_OK;
}

int view_set::evaluate( const vec3 & p, const vec3 & n, sample * out, int max_out ) const
{
    int count = 0;
    for( uint32_t i = 0; i < views.size(); ++i )
    {
        const frame_view & v = views[i];
        const vec3 pc = v.camera_from_world.apply( p );
        if( ! ( pc.z > 1e-3f ) )
            continue;
        const float u = v.fx * pc.x / pc.z + v.cx, y = v.fy * pc.y / pc.z + v.cy;
        if( ! ( u >= 0 && y >= 0 && u <= float( v.width - 1 ) && y <= float( v.height - 1 ) ) )
            continue;
        const float cos_angle = -dot( normalized( v.camera_from_world.rotate( n ) ), pc ) / length( pc );
        if( ! ( cos_angle >= min_cos ) )
            continue;
        const size_t idx = size_t( int( y + 0.5f ) ) * v.width + size_t( int( u + 0.5f ) );
        const uint8_t feather = v.feather[idx];
        const uint16_t zb = v.zbuffer[idx];
        if( ! feather || ! zb || pc.z > float( zb ) * zbuffer_unit + epsilon )
            continue;
        if( v.frame->depth )
        {
            // The frame's own depth also rejects points hidden by something that is not part of the mesh, and
            // points where the registered mesh and this frame disagree (the color there belongs to another surface).
            const uint16_t d = v.frame->depth[idx];
            if( d && std::fabs( pc.z - float( d ) * v.frame->depth_scale ) > epsilon )
                continue;
        }
        const float weight = ( power == 4.f ? ( cos_angle * cos_angle ) * ( cos_angle * cos_angle ) : std::pow( cos_angle, power ) )
                           * v.weight * ( float( feather ) / 255.f );
        if( ! ( weight > 0 ) )
            continue;
        // Insert sorted by decreasing weight, keeping the best max_out.
        int pos = count < max_out ? count : max_out;
        while( pos > 0 && out[pos - 1].weight < weight )
            --pos;
        if( pos >= max_out )
            continue;
        const int last = std::min( count, max_out - 1 );
        for( int k = last; k > pos; --k )
            out[k] = out[k - 1];
        sample & s = out[pos];
        s.weight = weight;
        s.view = i;
        bilinear_rgb( v.frame->rgb, v.width, v.height, u, y, s.rgb );
        for( int k = 0; k < 3; ++k )
            s.rgb[k] = std::min( 255.f, s.rgb[k] * v.gain[k] );
        count = std::min( count + 1, max_out );
    }
    return count;
}

bool view_set::blend_color( const vec3 & p, const vec3 & n, float rgb[3], float * total_weight ) const
{
    sample s[max_blend_views];
    const int count = evaluate( p, n, s, blend );
    float sum = 0, c[3] = { 0, 0, 0 };
    for( int i = 0; i < count; ++i )
    {
        sum += s[i].weight;
        for( int k = 0; k < 3; ++k )
            c[k] += s[i].weight * s[i].rgb[k];
    }
    if( total_weight )
        *total_weight = sum;
    if( count == 0 || ! ( sum > 0 ) )
        return false;
    for( int k = 0; k < 3; ++k )
        rgb[k] = c[k] / sum;
    return true;
}

void estimate_gains( const float * positions, const float * normals, uint32_t n, const uint8_t * synthesized,
                     view_set & vs )
{
    const size_t F = vs.views.size();
    if( F < 2 || n == 0 )
        return;
    struct equation
    {
        uint32_t a, b;
        float d[3];  // log(color_a) - log(color_b)
        float w;
    };
    // A few tens of thousands of surface points constrain a few dozen gains well.
    const uint32_t stride = std::max< uint32_t >( 1, n / 40000 );
    const uint32_t samples = ( n + stride - 1 ) / stride;
    constexpr int max_views = 8;
    std::vector< equation > equations;
    std::mutex mutex;
    parallel_for( samples, 1024, [&]( size_t begin, size_t end ) {
        std::vector< equation > local;
        sample s[max_views];
        for( size_t i = begin; i < end; ++i )
        {
            const size_t v = i * stride;
            if( synthesized && synthesized[v] )
                continue;
            const vec3 p = vec3::load( positions + v * 3 ), nrm = vec3::load( normals + v * 3 );
            const int count = vs.evaluate( p, nrm, s, max_views );
            for( int a = 0; a < count; ++a )
                for( int b = a + 1; b < count; ++b )
                {
                    equation e{ s[a].view, s[b].view, {}, std::min( s[a].weight, s[b].weight ) };
                    bool usable = true;
                    for( int k = 0; k < 3; ++k )
                    {
                        // Clipped and near-black values carry no exposure information.
                        if( s[a].rgb[k] < 6 || s[a].rgb[k] > 250 || s[b].rgb[k] < 6 || s[b].rgb[k] > 250 )
                            usable = false;
                        else
                            e.d[k] = std::log( s[a].rgb[k] ) - std::log( s[b].rgb[k] );
                    }
                    if( usable )
                        local.push_back( e );
                }
        }
        std::lock_guard< std::mutex > lock( mutex );
        equations.insert( equations.end(), local.begin(), local.end() );
    } );
    if( equations.empty() )
        return;

    // Minimizes sum w * (x_a - x_b + d)^2 over the log gains x, with a weak prior toward 0 for frames without
    // overlap; three reweighting rounds (Huber) suppress specular highlights and misregistered samples.
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero( F, 3 );
    for( int round = 0; round < 3; ++round )
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero( F, F ), B = Eigen::MatrixXd::Zero( F, 3 );
        for( const equation & e : equations )
        {
            double w = e.w;
            if( round > 0 )
            {
                double r2 = 0;
                for( int k = 0; k < 3; ++k )
                {
                    const double r = x( e.a, k ) - x( e.b, k ) + e.d[k];
                    r2 += r * r;
                }
                const double r = std::sqrt( r2 / 3 ), delta = 0.05;
                if( r > delta )
                    w *= delta / r;
            }
            A( e.a, e.a ) += w;
            A( e.b, e.b ) += w;
            A( e.a, e.b ) -= w;
            A( e.b, e.a ) -= w;
            for( int k = 0; k < 3; ++k )
            {
                B( e.a, k ) -= w * e.d[k];
                B( e.b, k ) += w * e.d[k];
            }
        }
        const double lambda = 1e-3 * A.diagonal().mean() + 1e-12;
        A.diagonal().array() += lambda;
        x = A.ldlt().solve( B );
        if( ! x.allFinite() )
            return;
    }
    // Keep the overall brightness: the weighted mean log gain is zero.
    double wsum = 0;
    Eigen::RowVector3d mean = Eigen::RowVector3d::Zero();
    for( size_t i = 0; i < F; ++i )
    {
        mean += vs.views[i].weight * x.row( Eigen::Index( i ) );
        wsum += vs.views[i].weight;
    }
    mean /= wsum;
    for( size_t i = 0; i < F; ++i )
        for( int k = 0; k < 3; ++k )
            vs.views[i].gain[k] = float( std::clamp( std::exp( x( Eigen::Index( i ), k ) - mean( k ) ), 1.0 / 3.0, 3.0 ) );
}

bool vertex_colors( const fr_mesh & m, const float * normals, const view_set & vs, const uint8_t * synthesized,
                    std::vector< float > & rgb )
{
    const uint32_t n = m.vertex_count;
    rgb.assign( size_t( n ) * 3, 0.f );
    // Vertices split on UV seams are one surface point: only the lowest-numbered copy (the representative) is colored,
    // and the interpolation into unseen regions runs on the welded topology, so it crosses chart borders. On the split
    // mesh a chart without any seen vertex would otherwise fall back to the global mean color (skin-colored patches
    // in the hair at the open back of a head scan).
    std::vector< uint32_t > rep, welded;
    const uint32_t * idx = m.indices;
    if( m.uvs && n > 1 )
    {
        rep = coincident_representatives( m.positions, n );
        welded = remap_indices( m.indices, size_t( m.triangle_count ) * 3, rep );
        idx = welded.data();
    }
    auto is_representative = [&]( size_t v ) { return rep.empty() || rep[v] == v; };
    std::vector< uint8_t > seen( n, 0 );
    parallel_for( n, 2048, [&]( size_t begin, size_t end ) {
        for( size_t v = begin; v < end; ++v )
            if( is_representative( v ) && ! ( synthesized && synthesized[v] ) )
                seen[v] = vs.blend_color( vec3::load( m.positions + v * 3 ), vec3::load( normals + v * 3 ), &rgb[v * 3] );
    } );
    const csr nb = vertex_neighbors( n, idx, m.triangle_count );

    // Unseen representatives, and whether their connected unseen region touches a seen vertex.
    std::vector< int32_t > unknown( n, -1 );
    std::vector< uint32_t > unknowns;
    for( uint32_t v = 0; v < n; ++v )
        if( ! seen[v] && is_representative( v ) && nb.size( v ) > 0 )
        {
            unknown[v] = int32_t( unknowns.size() );
            unknowns.push_back( v );
        }
    std::vector< uint8_t > anchored( unknowns.size(), 0 );
    {
        std::vector< uint32_t > stack;
        for( uint32_t v = 0; v < n; ++v )
            if( seen[v] )
                for( uint32_t k = nb.begin( v ); k < nb.end( v ); ++k )
                {
                    const int32_t u = unknown[nb.data[k]];
                    if( u >= 0 && ! anchored[u] )
                    {
                        anchored[u] = 1;
                        stack.push_back( nb.data[k] );
                    }
                }
        while( ! stack.empty() )
        {
            const uint32_t v = stack.back();
            stack.pop_back();
            for( uint32_t k = nb.begin( v ); k < nb.end( v ); ++k )
            {
                const int32_t u = unknown[nb.data[k]];
                if( u >= 0 && ! anchored[u] )
                {
                    anchored[u] = 1;
                    stack.push_back( nb.data[k] );
                }
            }
        }
    }
    // Border values: the seen colors within three rings of the unseen regions, averaged with their seen neighbors three
    // times (the colors at a scan's border come from grazing views; a blur keeps their noise out of the interpolation).
    std::vector< float > border = rgb;
    {
        std::vector< uint8_t > band( n, 0 );
        std::vector< uint32_t > frontier;
        for( uint32_t u : unknowns )
            for( uint32_t k = nb.begin( u ); k < nb.end( u ); ++k )
                if( seen[nb.data[k]] && ! band[nb.data[k]] )
                {
                    band[nb.data[k]] = 1;
                    frontier.push_back( nb.data[k] );
                }
        std::vector< uint32_t > members = frontier;
        for( int ring = 1; ring < 3; ++ring )
        {
            std::vector< uint32_t > next;
            for( uint32_t v : frontier )
                for( uint32_t k = nb.begin( v ); k < nb.end( v ); ++k )
                    if( seen[nb.data[k]] && ! band[nb.data[k]] )
                    {
                        band[nb.data[k]] = 1;
                        next.push_back( nb.data[k] );
                    }
            members.insert( members.end(), next.begin(), next.end() );
            frontier.swap( next );
        }
        std::vector< float > blurred( members.size() * 3 );
        for( int it = 0; it < 3; ++it )
        {
            for( size_t i = 0; i < members.size(); ++i )
            {
                const uint32_t v = members[i];
                float sum[3] = { border[size_t( v ) * 3], border[size_t( v ) * 3 + 1], border[size_t( v ) * 3 + 2] };
                int count = 1;
                for( uint32_t k = nb.begin( v ); k < nb.end( v ); ++k )
                    if( seen[nb.data[k]] )
                    {
                        for( int c = 0; c < 3; ++c )
                            sum[c] += border[size_t( nb.data[k] ) * 3 + c];
                        ++count;
                    }
                for( int c = 0; c < 3; ++c )
                    blurred[i * 3 + c] = sum[c] / float( count );
            }
            for( size_t i = 0; i < members.size(); ++i )
                std::copy_n( &blurred[i * 3], 3, &border[size_t( members[i] ) * 3] );
        }
    }
    // Harmonic interpolation (uniform graph Laplacian) over the anchored unseen vertices.
    {
        std::vector< int32_t > col( unknowns.size(), -1 );
        int count = 0;
        for( size_t i = 0; i < unknowns.size(); ++i )
            if( anchored[i] )
                col[i] = count++;
        if( count > 0 )
        {
            using triplet = Eigen::Triplet< double >;
            std::vector< triplet > t;
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero( count, 3 );
            for( size_t i = 0; i < unknowns.size(); ++i )
            {
                if( col[i] < 0 )
                    continue;
                const uint32_t v = unknowns[i];
                double degree = 0;
                for( uint32_t k = nb.begin( v ); k < nb.end( v ); ++k )
                {
                    const uint32_t w = nb.data[k];
                    if( seen[w] )
                    {
                        for( int c = 0; c < 3; ++c )
                            B( col[i], c ) += border[size_t( w ) * 3 + c];
                        ++degree;
                    }
                    else if( unknown[w] >= 0 && col[unknown[w]] >= 0 )
                    {
                        t.emplace_back( col[i], col[unknown[w]], -1.0 );
                        ++degree;
                    }
                }
                t.emplace_back( col[i], col[i], degree );
            }
            Eigen::SparseMatrix< double > A( count, count );
            A.setFromTriplets( t.begin(), t.end() );
            Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > > ldlt( A );
            Eigen::MatrixXd x;
            bool ok = ldlt.info() == Eigen::Success;
            if( ok )
            {
                x = ldlt.solve( B );
                ok = ldlt.info() == Eigen::Success && x.allFinite();
            }
            for( size_t i = 0; i < unknowns.size(); ++i )
                if( col[i] >= 0 && ok )
                {
                    for( int c = 0; c < 3; ++c )
                        rgb[size_t( unknowns[i] ) * 3 + c] = float( std::clamp( x( col[i], c ), 0.0, 255.0 ) );
                    seen[unknowns[i]] = 2;
                }
        }
    }
    // Components no frame sees at all get the mean color.
    double mean[3] = { 0, 0, 0 };
    size_t colored = 0;
    for( uint32_t v = 0; v < n; ++v )
        if( seen[v] == 1 )
        {
            for( int c = 0; c < 3; ++c )
                mean[c] += rgb[size_t( v ) * 3 + c];
            ++colored;
        }
    for( int c = 0; c < 3; ++c )
        mean[c] = colored ? mean[c] / double( colored ) : 128.0;
    for( uint32_t v = 0; v < n; ++v )
        if( ! seen[v] && is_representative( v ) )
            for( int c = 0; c < 3; ++c )
                rgb[size_t( v ) * 3 + c] = float( mean[c] );
    // Seam copies take their representative's color.
    for( uint32_t v = 0; v < uint32_t( rep.size() ); ++v )
        if( rep[v] != v )
            std::copy_n( &rgb[size_t( rep[v] ) * 3], 3, &rgb[size_t( v ) * 3] );
    return colored > 0;
}

}  // namespace texture
}  // namespace fr
