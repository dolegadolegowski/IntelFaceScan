// License: Apache 2.0. See LICENSE file in root directory.
//
// Per-frame visibility and view weights, color compensation and vertex colors (see texture_views.h).

#include "texture_views.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <cstdio>
#include <cstdlib>
#include <numeric>
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

// One shared surface point seen by two views: d = log(color_a) - log(color_b) of the raw samples, a < b.
struct equation
{
    uint32_t a, b, vertex;
    float d[3];
    float w;
};

// Decay length of the correction field away from the overlaps, in cells (fit_field).
constexpr double field_decay_cells = 1.5;
// Smallest field cell: 20 mm (the field only levels low-frequency lighting differences; finer cells would follow
// texture misregistration).
constexpr float field_cell_m = 0.02f;
// Huber scale of the log color residuals (gains and field).
constexpr double huber_delta = 0.05;

// Grid cell of p and the trilinear weights of its 8 corner nodes (corner k: dx = k & 1, dy = k >> 1 & 1,
// dz = k >> 2); the cell is identified by its lowest node. False outside the grid.
bool locate( const correction_field & f, const vec3 & p, size_t & cell, float weights[8] )
{
    const float qx = ( p.x - f.origin.x ) / f.cell, qy = ( p.y - f.origin.y ) / f.cell, qz = ( p.z - f.origin.z ) / f.cell;
    if( ! ( qx >= 0 && qy >= 0 && qz >= 0 && qx < float( f.nodes[0] - 1 ) && qy < float( f.nodes[1] - 1 )
            && qz < float( f.nodes[2] - 1 ) ) )
        return false;
    const int ix = int( qx ), iy = int( qy ), iz = int( qz );
    const float fx = qx - float( ix ), fy = qy - float( iy ), fz = qz - float( iz );
    cell = ( size_t( iz ) * size_t( f.nodes[1] ) + size_t( iy ) ) * size_t( f.nodes[0] ) + size_t( ix );
    for( int k = 0; k < 8; ++k )
        weights[k] = ( k & 1 ? fx : 1 - fx ) * ( k & 2 ? fy : 1 - fy ) * ( k & 4 ? fz : 1 - fz );
    return true;
}

// Node offsets of the 8 corners of a cell (same order as locate).
void corner_offsets( const correction_field & f, size_t out[8] )
{
    const size_t sx = 1, sy = size_t( f.nodes[0] ), sz = size_t( f.nodes[0] ) * size_t( f.nodes[1] );
    for( int k = 0; k < 8; ++k )
        out[k] = ( k & 1 ? sx : 0 ) + ( k & 2 ? sy : 0 ) + ( k & 4 ? sz : 0 );
}

// Grows a binary mask by r pixels (square window, separable running counts).
void dilate_mask( std::vector< uint8_t > & m, int w, int h, int r )
{
    std::vector< uint8_t > rows( m.size() );
    std::vector< int > prefix( size_t( std::max( w, h ) ) + 1 );
    for( int y = 0; y < h; ++y )
    {
        const uint8_t * src = &m[size_t( y ) * w];
        for( int x = 0; x < w; ++x )
            prefix[x + 1] = prefix[x] + ( src[x] ? 1 : 0 );
        for( int x = 0; x < w; ++x )
            rows[size_t( y ) * w + x] = prefix[std::min( w, x + r + 1 )] - prefix[std::max( 0, x - r )] > 0;
    }
    for( int x = 0; x < w; ++x )
    {
        for( int y = 0; y < h; ++y )
            prefix[y + 1] = prefix[y] + rows[size_t( y ) * w + x];
        for( int y = 0; y < h; ++y )
            m[size_t( y ) * w + x] = prefix[std::min( h, y + r + 1 )] - prefix[std::max( 0, y - r )] > 0;
    }
}

float median( std::vector< float > & values )
{
    const auto mid = values.begin() + std::ptrdiff_t( values.size() / 2 );
    std::nth_element( values.begin(), mid, values.end() );
    return *mid;
}

// Camera exposure and white balance per frame (log gain per channel that undoes them) from the static background:
// the guided scan keeps the camera still while the head turns, so pixels away from the head and the body see the
// same scene in every frame, and only the camera's auto exposure / white balance changes them. False (log_gain
// untouched) when the frames differ in size, too little background is usable, or it is not consistent between the
// frames (the camera moved).
//
// Real D455 scan (1280x800, 15 keyframes, color auto exposure): the front frames were brighter and warmer than all
// later ones by 0.20-0.27 log in R and 0.13-0.18 in B, the later frames lie within +-0.035 of each other per
// channel (an independent estimate with a hand-drawn background region agreed within 0.005).
bool camera_gains( const view_set & vs, Eigen::MatrixXd & log_gain, bool debug )
{
    const size_t F = vs.views.size();
    const int w = vs.views[0].width, h = vs.views[0].height;
    for( const frame_view & v : vs.views )
        if( v.width != w || v.height != h )
            return false;
    const size_t count = size_t( w ) * h;
    // The head in any frame (its mask or the mesh drawn at the frame's pose), widened by 3 % of the image width:
    // hair and the mask border.
    std::vector< uint8_t > head( count, 0 );
    std::vector< float > near_m( F, 0.f );
    for( size_t f = 0; f < F; ++f )
    {
        const frame_view & v = vs.views[f];
        uint16_t far = 0;
        for( size_t i = 0; i < count; ++i )
        {
            if( v.zbuffer[i] || ( v.frame->mask && v.frame->mask[i] >= 128 ) )
                head[i] = 1;
            far = std::max( far, v.zbuffer[i] );
        }
        // Anything measured closer than 0.3 m behind the head is the body (it turns with the head) or furniture next
        // to it.
        near_m[f] = float( far ) * zbuffer_unit + 0.3f;
    }
    dilate_mask( head, w, h, std::max( 4, int( 0.03f * float( w ) ) ) );
    // Candidates on a lattice of at most ~64k pixels: away from the head, without near depth, mid-tones in every
    // frame (unclipped; dark pixels follow the camera's tone curve differently: between the front and the side frames
    // of the real scan R changed by 0.46 log on dark background pixels and by 0.22 on bright ones).
    const int step = std::max( 1, int( std::ceil( std::sqrt( double( count ) / 65536.0 ) ) ) );
    std::vector< size_t > pixels;
    for( int y = 0; y < h; y += step )
        for( int x = 0; x < w; x += step )
        {
            const size_t i = size_t( y ) * w + x;
            if( head[i] )
                continue;
            bool usable = true;
            for( size_t f = 0; f < F && usable; ++f )
            {
                const fr_rgbd_frame & fr = *vs.views[f].frame;
                if( fr.depth && fr.depth[i] && float( fr.depth[i] ) * fr.depth_scale < near_m[f] )
                    usable = false;
                for( int k = 0; k < 3 && usable; ++k )
                    usable = fr.rgb[i * 3 + k] >= 40 && fr.rgb[i * 3 + k] <= 235;
            }
            if( usable )
                pixels.push_back( i );
        }
    const size_t lattice = size_t( ( w + step - 1 ) / step ) * size_t( ( h + step - 1 ) / step );
    // At least 2 % of the image consistent (the real scan: 9 %, the wall and the ceiling).
    const size_t min_pixels = std::max< size_t >( 500, lattice / 50 );
    if( pixels.size() < min_pixels )
    {
        if( debug )
            std::fprintf( stderr, "texture: background %zu of %zu pixels usable, too few\n", pixels.size(), lattice );
        return false;
    }
    const size_t P = pixels.size();
    float lut[256];
    for( int i = 0; i < 256; ++i )
        lut[i] = std::log( float( std::max( i, 1 ) ) );
    std::vector< float > L( P * F * 3 );  // [pixel][frame][channel]
    for( size_t p = 0; p < P; ++p )
        for( size_t f = 0; f < F; ++f )
            for( int k = 0; k < 3; ++k )
                L[( p * F + f ) * 3 + k] = lut[vs.views[f].frame->rgb[pixels[p] * 3 + k]];
    // Robust alternation: per-pixel reference (median over the frames of the corrected colors), per-frame gain
    // (median over the consistent pixels), consistent pixels (every frame within 0.08 of the model: windows, screens,
    // moving furniture and shadows change).
    std::vector< float > ref( P * 3 ), x( F * 3, 0.f ), values;
    std::vector< uint8_t > keep( P, 1 );
    auto residual = [&]( size_t p, size_t f, int k ) { return L[( p * F + f ) * 3 + k] - ref[p * 3 + k] - x[f * 3 + k]; };
    size_t kept = P;
    for( int it = 0; it < 4; ++it )
    {
        for( size_t p = 0; p < P; ++p )
            for( int k = 0; k < 3; ++k )
            {
                values.clear();
                for( size_t f = 0; f < F; ++f )
                    values.push_back( L[( p * F + f ) * 3 + k] - x[f * 3 + k] );
                ref[p * 3 + k] = median( values );
            }
        if( it > 0 )
        {
            kept = 0;
            for( size_t p = 0; p < P; ++p )
            {
                float worst = 0;
                for( size_t f = 0; f < F; ++f )
                    for( int k = 0; k < 3; ++k )
                        worst = std::max( worst, std::fabs( residual( p, f, k ) ) );
                keep[p] = worst < 0.08f;
                kept += keep[p];
            }
            if( kept < min_pixels )
                break;
        }
        for( size_t f = 0; f < F; ++f )
            for( int k = 0; k < 3; ++k )
            {
                values.clear();
                for( size_t p = 0; p < P; ++p )
                    if( keep[p] )
                        values.push_back( L[( p * F + f ) * 3 + k] - ref[p * 3 + k] );
                x[f * 3 + k] = median( values );
            }
    }
    // Typical deviation of a consistent pixel from the model, per frame and channel (the real scan: at most 0.017).
    // The share of consistent pixels says little: in the real scan only 37 % of the candidates were (the window with
    // the daylight behind it and the chair the person turns on change between the frames), while a plain wall stays
    // consistent even under a moving camera, and then still shows the camera's gains.
    float spread = 0;
    if( kept >= min_pixels )
        for( size_t f = 0; f < F; ++f )
            for( int k = 0; k < 3; ++k )
            {
                values.clear();
                for( size_t p = 0; p < P; ++p )
                    if( keep[p] )
                        values.push_back( std::fabs( residual( p, f, k ) ) );
                spread = std::max( spread, median( values ) );
            }
    const bool still = kept >= min_pixels && spread < 0.03f;
    if( debug )
        std::fprintf( stderr, "texture: background %zu of %zu pixels usable, %zu consistent, largest per-frame MAD %.3f: %s\n",
                      P, lattice, kept, double( spread ), still ? "camera gains" : "not used" );
    if( ! still )
        return false;
    for( size_t f = 0; f < F; ++f )
        for( int k = 0; k < 3; ++k )
            log_gain( Eigen::Index( f ), k ) = -double( x[f * 3 + k] );
    return true;
}

// One log gain per frame (the same for all channels) from the shared surface points, least squares on the log
// brightness (mean of the channel logs) with a weak prior toward 0 for frames without overlap; three reweighting
// rounds (Huber) suppress specular highlights and misregistered samples.
Eigen::VectorXd brightness_gains( const std::vector< equation > & equations, size_t F )
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero( Eigen::Index( F ) );
    for( int round = 0; round < 3; ++round )
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero( F, F );
        Eigen::VectorXd B = Eigen::VectorXd::Zero( Eigen::Index( F ) );
        for( const equation & e : equations )
        {
            const double d = ( double( e.d[0] ) + e.d[1] + e.d[2] ) / 3;
            double w = e.w;
            if( round > 0 )
            {
                const double r = std::fabs( x( e.a ) - x( e.b ) + d );
                if( r > huber_delta )
                    w *= huber_delta / r;
            }
            A( e.a, e.a ) += w;
            A( e.b, e.b ) += w;
            A( e.a, e.b ) -= w;
            A( e.b, e.a ) -= w;
            B( e.a ) -= w * d;
            B( e.b ) += w * d;
        }
        A.diagonal().array() += 1e-3 * A.diagonal().mean() + 1e-12;
        const Eigen::VectorXd next = A.ldlt().solve( B );
        if( ! next.allFinite() )
            return Eigen::VectorXd::Zero( Eigen::Index( F ) );
        x = next;
    }
    return x;
}

// FR_DEBUG_TEXTURE: weighted RMS of the shared-point log residuals after the compensation (R, G, B and B - R), over
// all pairs and over pairs of views more than 20 deg apart (different poses: where seams form).
void print_residuals( const float * positions, const std::vector< equation > & equations, const view_set & vs,
                      const correction_field & field, const char * label )
{
    double sum[2][4] = {}, wsum[2] = {};
    for( const equation & e : equations )
    {
        const vec3 p = vec3::load( positions + size_t( e.vertex ) * 3 );
        float fa[3] = { 0, 0, 0 }, fb[3] = { 0, 0, 0 };
        int32_t slots[8];
        float weights[8];
        if( ! field.empty() && field.corners( p, slots, weights ) )
        {
            field.at( slots, weights, e.a, fa );
            field.at( slots, weights, e.b, fb );
        }
        double r[3];
        for( int k = 0; k < 3; ++k )
            r[k] = e.d[k] + std::log( vs.views[e.a].gain[k] ) + fa[k] - std::log( vs.views[e.b].gain[k] ) - fb[k];
        const affine & ca = vs.views[e.a].camera_from_world, & cb = vs.views[e.b].camera_from_world;
        const float cos_axes = ca.m[2][0] * cb.m[2][0] + ca.m[2][1] * cb.m[2][1] + ca.m[2][2] * cb.m[2][2];
        for( int group = 0; group < ( cos_axes < 0.94f ? 2 : 1 ); ++group )
        {
            for( int k = 0; k < 3; ++k )
                sum[group][k] += e.w * r[k] * r[k];
            sum[group][3] += e.w * ( r[2] - r[0] ) * ( r[2] - r[0] );
            wsum[group] += e.w;
        }
    }
    for( int group = 0; group < 2; ++group )
        if( wsum[group] > 0 )
            std::fprintf( stderr, "texture: %s, shared-point RMS log R G B (B-R) %s: %.3f %.3f %.3f (%.3f)\n", label,
                          group ? "between poses" : "all pairs", std::sqrt( sum[group][0] / wsum[group] ),
                          std::sqrt( sum[group][1] / wsum[group] ), std::sqrt( sum[group][2] / wsum[group] ),
                          std::sqrt( sum[group][3] / wsum[group] ) );
}

// Per-view log correction field on the grid of `field` (geometry and view_count set): one unknown per view, channel
// and node of the cells the view was sampled in (touched: view * nodes + cell), minimizing
//   sum_e w_e (field_a(p_e) - field_b(p_e) + d_e + base_a - base_b)^2            shared points (Huber reweighted)
//   + lambda_s sum (x_v(j) - x_v(j'))^2 over neighboring nodes j, j' of one view    smoothness
//   + lambda_0 sum x_v(j)^2                                                        zero away from the overlaps
// with lambda_s = 0.5 x the mean data weight of a node and lambda_0 = lambda_s / L^2: away from the overlaps the
// field decays to zero over about L = 1.5 cells (3 cm). A longer decay carries more of the overlap's lighting into
// surface one pose sees alone, without leveling the seams better: on the synthetic sphere under lights that turn with
// the head (TextureBakingTests) the single-pose sides changed by 0.025 / 0.053 / 0.085 log B/R with L = 1.5 / 2 /
// 2.5 cells, the p99 B-R steps across edges were 0.069 / 0.072 / 0.076; on the real scan the residual between poses
// was 0.058 / 0.056 / 0.054 log B/R. Equations of one cell and view pair share their 8 + 8 unknowns (one 8x8 block);
// a sparse LDLT solves the three channels at once (4.5k unknowns per channel on the real scan, 0.3 s for 3 rounds).
// Fills field.slot / field.value; false when the solve fails.
bool fit_field( const float * positions, const std::vector< equation > & equations, const std::vector< uint64_t > & touched,
                const Eigen::MatrixXd & base, correction_field & field )
{
    const size_t F = field.view_count;
    const size_t node_count = size_t( field.nodes[0] ) * size_t( field.nodes[1] ) * size_t( field.nodes[2] );
    size_t offset[8];
    corner_offsets( field, offset );
    // Fitted nodes: the corners of every cell a view was sampled in.
    field.slot.assign( node_count, -1 );
    std::vector< uint8_t > active( node_count, 0 );
    for( uint64_t t : touched )
        for( size_t k : offset )
            active[size_t( t % node_count ) + k] = 1;
    int32_t slots = 0;
    for( size_t i = 0; i < node_count; ++i )
        if( active[i] )
            field.slot[i] = slots++;
    std::vector< int32_t > unknown( size_t( slots ) * F, -1 );
    int32_t U = 0;
    for( uint64_t t : touched )
        for( size_t k : offset )
        {
            int32_t & u = unknown[size_t( field.slot[size_t( t % node_count ) + k] ) * F + size_t( t / node_count )];
            if( u < 0 )
                u = U++;
        }
    const size_t E = equations.size();
    std::vector< size_t > cell( E );
    std::vector< float > phi( E * 8 );
    for( size_t e = 0; e < E; ++e )
        if( ! locate( field, vec3::load( positions + size_t( equations[e].vertex ) * 3 ), cell[e], &phi[e * 8] ) )
            return false;
    // Sorted by cell and view pair: the blocks are accumulated in one pass, in a fixed order (deterministic).
    std::vector< size_t > order( E );
    std::iota( order.begin(), order.end(), size_t( 0 ) );
    std::sort( order.begin(), order.end(), [&]( size_t i, size_t j ) {
        if( cell[i] != cell[j] )
            return cell[i] < cell[j];
        if( equations[i].a != equations[j].a )
            return equations[i].a < equations[j].a;
        if( equations[i].b != equations[j].b )
            return equations[i].b < equations[j].b;
        return i < j;
    } );
    auto unknown_of = [&]( size_t node, uint32_t view ) { return unknown[size_t( field.slot[node] ) * F + view]; };
    auto r0 = [&]( const equation & e, int k ) { return double( e.d[k] ) + base( e.a, k ) - base( e.b, k ); };
    auto field_residual = [&]( const Eigen::MatrixXd & x, size_t e, int k ) {
        const equation & q = equations[e];
        double r = r0( q, k );
        for( int c = 0; c < 8; ++c )
            r += phi[e * 8 + c] * ( x( unknown_of( cell[e] + offset[c], q.a ), k ) - x( unknown_of( cell[e] + offset[c], q.b ), k ) );
        return r;
    };

    using triplet = Eigen::Triplet< double >;
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero( U, 3 );
    std::vector< double > weight( E );
    for( int round = 0; round < 3; ++round )
    {
        for( size_t e = 0; e < E; ++e )
        {
            weight[e] = equations[e].w;
            if( round > 0 )
            {
                double r2 = 0;
                for( int k = 0; k < 3; ++k )
                {
                    const double r = field_residual( x, e, k );
                    r2 += r * r;
                }
                const double r = std::sqrt( r2 / 3 );
                if( r > huber_delta )
                    weight[e] *= huber_delta / r;
            }
        }
        // Lower triangle only (SimplicialLDLT< Lower > reads nothing else).
        std::vector< triplet > t;
        auto add = [&]( int32_t row, int32_t col, double v ) {
            if( row >= col )
                t.emplace_back( row, col, v );
        };
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero( U, 3 );
        std::vector< double > data_diagonal( size_t( U ), 0.0 );
        for( size_t g = 0; g < E; )
        {
            const size_t first = order[g];
            const uint32_t a = equations[first].a, b = equations[first].b;
            double M[8][8] = {}, q[8][3] = {};
            size_t end = g;
            for( ; end < E && cell[order[end]] == cell[first] && equations[order[end]].a == a && equations[order[end]].b == b; ++end )
            {
                const size_t e = order[end];
                const float * ph = &phi[e * 8];
                const double r[3] = { r0( equations[e], 0 ), r0( equations[e], 1 ), r0( equations[e], 2 ) };
                for( int i = 0; i < 8; ++i )
                {
                    const double wi = weight[e] * ph[i];
                    for( int j = 0; j <= i; ++j )
                        M[i][j] += wi * ph[j];
                    for( int k = 0; k < 3; ++k )
                        q[i][k] += wi * r[k];
                }
            }
            int32_t ua[8], ub[8];
            for( int i = 0; i < 8; ++i )
            {
                ua[i] = unknown_of( cell[first] + offset[i], a );
                ub[i] = unknown_of( cell[first] + offset[i], b );
            }
            for( int i = 0; i < 8; ++i )
            {
                for( int j = 0; j < 8; ++j )
                {
                    const double m = i >= j ? M[i][j] : M[j][i];
                    add( ua[i], ua[j], m );
                    add( ub[i], ub[j], m );
                    add( ua[i], ub[j], -m );
                    add( ub[i], ua[j], -m );
                }
                data_diagonal[size_t( ua[i] )] += M[i][i];
                data_diagonal[size_t( ub[i] )] += M[i][i];
                for( int k = 0; k < 3; ++k )
                {
                    B( ua[i], k ) -= q[i][k];
                    B( ub[i], k ) += q[i][k];
                }
            }
            g = end;
        }
        double mean = 0;
        size_t constrained = 0;
        for( double d : data_diagonal )
            if( d > 0 )
            {
                mean += d;
                ++constrained;
            }
        if( constrained == 0 )
            return false;
        const double lambda_s = 0.5 * mean / double( constrained );
        const double lambda_0 = lambda_s / ( field_decay_cells * field_decay_cells );
        const size_t step[3] = { 1, size_t( field.nodes[0] ), size_t( field.nodes[0] ) * size_t( field.nodes[1] ) };
        for( size_t i = 0; i < node_count; ++i )
        {
            if( field.slot[i] < 0 )
                continue;
            const size_t coord[3] = { i % step[1], ( i / step[1] ) % size_t( field.nodes[1] ), i / step[2] };
            for( uint32_t v = 0; v < F; ++v )
            {
                const int32_t u = unknown_of( i, v );
                if( u < 0 )
                    continue;
                add( u, u, lambda_0 );
                for( int axis = 0; axis < 3; ++axis )
                {
                    if( coord[axis] + 1 >= size_t( field.nodes[axis] ) || field.slot[i + step[axis]] < 0 )
                        continue;
                    const int32_t n = unknown_of( i + step[axis], v );
                    if( n < 0 )
                        continue;
                    add( u, u, lambda_s );
                    add( n, n, lambda_s );
                    add( std::max( u, n ), std::min( u, n ), -lambda_s );
                }
            }
        }
        Eigen::SparseMatrix< double > A( U, U );
        A.setFromTriplets( t.begin(), t.end() );
        Eigen::SimplicialLDLT< Eigen::SparseMatrix< double >, Eigen::Lower > ldlt( A );
        if( ldlt.info() != Eigen::Success )
            return false;
        const Eigen::MatrixXd next = ldlt.solve( B );
        if( ldlt.info() != Eigen::Success || ! next.allFinite() )
            return false;
        x = next;
    }
    field.value.assign( size_t( slots ) * F * 4, 0.f );
    for( size_t s = 0; s < size_t( slots ); ++s )
        for( size_t v = 0; v < F; ++v )
        {
            const int32_t u = unknown[s * F + v];
            if( u < 0 )
                continue;
            float * out = &field.value[( s * F + v ) * 4];
            for( int k = 0; k < 3; ++k )
                out[k] = float( x( u, k ) );
            out[3] = 1;
        }
    return true;
}

}  // namespace

void correction_field::at( const int32_t slots[8], const float weights[8], uint32_t view, float out[3] ) const
{
    float wsum = 0, sum[3] = { 0, 0, 0 };
    for( int k = 0; k < 8; ++k )
        if( slots[k] >= 0 )
        {
            const float * f = &value[( size_t( slots[k] ) * view_count + view ) * 4];
            const float w = weights[k] * f[3];
            wsum += w;
            for( int c = 0; c < 3; ++c )
                sum[c] += w * f[c];
        }
    for( int c = 0; c < 3; ++c )
        out[c] = wsum > 1e-6f ? sum[c] / wsum : 0.f;
}

bool correction_field::corners( const vec3 & p, int32_t slots[8], float weights[8] ) const
{
    size_t c;
    if( ! locate( *this, p, c, weights ) )
        return false;
    size_t offset[8];
    corner_offsets( *this, offset );
    for( int k = 0; k < 8; ++k )
        slots[k] = slot[c + offset[k]];
    return true;
}

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
        count = std::min( count + 1, max_out );
    }
    // Color compensation of the selected views: the frame's global gain times its local correction at p.
    int32_t slots[8];
    float corner_weight[8];
    const bool local = count > 0 && ! field.empty() && field.corners( p, slots, corner_weight );
    for( int i = 0; i < count; ++i )
    {
        sample & s = out[i];
        float correction[3] = { 0, 0, 0 };
        if( local )
            field.at( slots, corner_weight, s.view, correction );
        const float * gain = views[s.view].gain;
        for( int c = 0; c < 3; ++c )
            s.rgb[c] = std::min( 255.f, s.rgb[c] * std::clamp( gain[c] * std::exp( correction[c] ), 1.f / 3.f, 3.f ) );
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

// Why a global camera gain plus a local field, and not per-frame per-channel gains fitted on the shared points (the
// earlier method): on a real D455 scan (head turning under a daylight window on one side and a warm lamp on the
// other) the per-channel gains also absorbed the lighting change of the face front between poses and carried it to
// the whole frame. The three frames turned right, the only ones that see the person's left side, got +0.19 log B/R
// beyond the camera change, and that side came out blue (B/R 1.28 around the ear against 0.80 on the face; 413 cm2
// of texture above B/R 1); the frames turned left got -0.21, and the other side went red (B/R 0.65; 174 cm2 below
// 0.66). The shared colors are skin tones only (minor color axes at noise level, 2-3.5 levels), so richer global
// models are not better constrained: a 3x3 matrix per frame lowered the B/R residual only by collapsing chroma
// (singular values 1 : 0.11 : 0.01-0.02, 8-18 % of the color contrast kept, washed-out renders), gain plus offset
// only by compressing contrast (+12..+22 levels of blue offset on a side frame). With the camera gains and the
// field: residual between poses 0.090 / 0.103 / 0.100 log R G B (B/R 0.058) instead of 0.098 / 0.121 / 0.122 (0.070),
// B/R around the left ear 1.09 (no compensation: 1.10, the daylight on that side) and on the right side 0.78,
// 236 cm2 above B/R 1 and 8 cm2 below 0.66. Camera gains alone left the views 0.202 / 0.181 / 0.142 (0.163) apart.
void estimate_gains( const float * positions, const float * normals, uint32_t n, const uint8_t * synthesized,
                     view_set & vs )
{
    const size_t F = vs.views.size();
    // The shared-point equations are of the raw samples (evaluate applies the compensation).
    vs.field = correction_field();
    for( frame_view & v : vs.views )
        std::fill( v.gain, v.gain + 3, 1.f );
    if( F < 2 || n == 0 )
        return;
    const bool debug = std::getenv( "FR_DEBUG_TEXTURE" ) != nullptr;

    // Field grid over the mesh bounds with a one-cell margin, at most ~64 cells per axis.
    vec3 lo( 1e30f, 1e30f, 1e30f ), hi( -1e30f, -1e30f, -1e30f );
    for( uint32_t v = 0; v < n; ++v )
    {
        const vec3 p = vec3::load( positions + size_t( v ) * 3 );
        if( ! finite( p ) )
            continue;
        lo = vec3( std::min( lo.x, p.x ), std::min( lo.y, p.y ), std::min( lo.z, p.z ) );
        hi = vec3( std::max( hi.x, p.x ), std::max( hi.y, p.y ), std::max( hi.z, p.z ) );
    }
    if( ! ( hi.x >= lo.x && hi.y >= lo.y && hi.z >= lo.z ) )
        return;
    correction_field field;
    field.view_count = uint32_t( F );
    field.cell = std::max( field_cell_m, std::max( { hi.x - lo.x, hi.y - lo.y, hi.z - lo.z } ) / 64.f );
    field.origin = lo - vec3( field.cell, field.cell, field.cell );
    field.nodes[0] = int( std::ceil( ( hi.x - lo.x ) / field.cell ) ) + 3;
    field.nodes[1] = int( std::ceil( ( hi.y - lo.y ) / field.cell ) ) + 3;
    field.nodes[2] = int( std::ceil( ( hi.z - lo.z ) / field.cell ) ) + 3;
    const size_t node_count = size_t( field.nodes[0] ) * size_t( field.nodes[1] ) * size_t( field.nodes[2] );

    // Shared points: a few tens of thousands of surface points, every pair among their best 8 views; and the cells
    // every view that can color them is sampled in. Collected per chunk and joined in chunk order (deterministic).
    const uint32_t stride = std::max< uint32_t >( 1, n / 40000 );
    const uint32_t samples = ( n + stride - 1 ) / stride;
    constexpr int max_views = 8;
    constexpr size_t grain = 1024;
    std::vector< std::vector< equation > > chunk_equations( ( samples + grain - 1 ) / grain );
    std::vector< std::vector< uint64_t > > chunk_touched( chunk_equations.size() );
    parallel_for( samples, grain, [&]( size_t begin, size_t end ) {
        std::vector< equation > & local = chunk_equations[begin / grain];
        std::vector< uint64_t > & touched = chunk_touched[begin / grain];
        sample s[max_blend_views];
        float corner_weight[8];
        for( size_t i = begin; i < end; ++i )
        {
            const size_t v = i * stride;
            if( synthesized && synthesized[v] )
                continue;
            const vec3 p = vec3::load( positions + v * 3 ), nrm = vec3::load( normals + v * 3 );
            size_t cell;
            if( ! locate( field, p, cell, corner_weight ) )
                continue;
            const int count = vs.evaluate( p, nrm, s, max_blend_views );
            for( int a = 0; a < count; ++a )
                touched.push_back( uint64_t( s[a].view ) * node_count + cell );
            for( int a = 0; a < std::min( count, max_views ); ++a )
                for( int b = a + 1; b < std::min( count, max_views ); ++b )
                {
                    const bool swap = s[a].view > s[b].view;
                    const sample & first = swap ? s[b] : s[a];
                    const sample & second = swap ? s[a] : s[b];
                    equation e{ first.view, second.view, uint32_t( v ), {}, std::min( s[a].weight, s[b].weight ) };
                    bool usable = true;
                    for( int k = 0; k < 3; ++k )
                    {
                        // Clipped and near-black values carry no exposure information.
                        if( first.rgb[k] < 6 || first.rgb[k] > 250 || second.rgb[k] < 6 || second.rgb[k] > 250 )
                            usable = false;
                        else
                            e.d[k] = std::log( first.rgb[k] ) - std::log( second.rgb[k] );
                    }
                    if( usable )
                        local.push_back( e );
                }
        }
    } );
    std::vector< equation > equations;
    std::vector< uint64_t > touched;
    for( size_t c = 0; c < chunk_equations.size(); ++c )
    {
        equations.insert( equations.end(), chunk_equations[c].begin(), chunk_equations[c].end() );
        touched.insert( touched.end(), chunk_touched[c].begin(), chunk_touched[c].end() );
    }
    std::sort( touched.begin(), touched.end() );
    touched.erase( std::unique( touched.begin(), touched.end() ), touched.end() );

    // Global part: the camera's exposure and white balance, never the lighting of the surface.
    Eigen::MatrixXd base = Eigen::MatrixXd::Zero( F, 3 );
    const char * source = "none";
    if( camera_gains( vs, base, debug ) )
        source = "background";
    else if( ! equations.empty() )
    {
        // Without a usable background, brightness only: a per-channel gain from the shared points would take the
        // lighting's color along.
        const Eigen::VectorXd brightness = brightness_gains( equations, F );
        for( int k = 0; k < 3; ++k )
            base.col( k ) = brightness;
        source = "shared-point brightness";
    }
    // Keep the overall brightness: the weighted mean log gain is zero.
    double wsum = 0;
    Eigen::RowVector3d mean = Eigen::RowVector3d::Zero();
    for( size_t i = 0; i < F; ++i )
    {
        mean += vs.views[i].weight * base.row( Eigen::Index( i ) );
        wsum += vs.views[i].weight;
    }
    mean /= wsum;
    for( size_t i = 0; i < F; ++i )
        for( int k = 0; k < 3; ++k )
        {
            vs.views[i].gain[k] = float( std::clamp( std::exp( base( Eigen::Index( i ), k ) - mean( k ) ), 1.0 / 3.0, 3.0 ) );
            base( Eigen::Index( i ), k ) = std::log( double( vs.views[i].gain[k] ) );
        }
    if( debug )
    {
        std::fprintf( stderr, "texture: global gains (%s), %zu shared-point equations:", source, equations.size() );
        for( size_t i = 0; i < F; ++i )
            std::fprintf( stderr, " [%zu] %.3f %.3f %.3f", i, double( vs.views[i].gain[0] ), double( vs.views[i].gain[1] ),
                          double( vs.views[i].gain[2] ) );
        std::fprintf( stderr, "\n" );
    }
    if( debug )
        print_residuals( positions, equations, vs, correction_field(), "global gains" );
    // Local part: what still differs where views overlap is mostly lighting (the head turns under fixed lights) and
    // varies over the surface; leveled there and faded out away from the overlaps.
    if( ! equations.empty() && fit_field( positions, equations, touched, base, field ) )
    {
        vs.field = std::move( field );
        if( debug )
            print_residuals( positions, equations, vs, vs.field, "with the correction field" );
    }
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
