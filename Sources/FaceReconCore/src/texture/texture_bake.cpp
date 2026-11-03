// License: Apache 2.0. See LICENSE file in root directory.
//
// Texture baking: every atlas texel covered by a triangle gets the blended color of the best views of its surface
// point; uncovered gutter texels are filled by dilation. Vertex coloring uses the same view selection.

#include "fr_texture.h"
#include "texture_views.h"

#include <memory>
#include <string>

using namespace fr::mesh;
using namespace fr::texture;

namespace
{

// Weight of the fallback (diffused vertex) color in the blend: only matters where every view is very grazing or
// feathered out, and makes the transition into unseen regions continuous.
constexpr float fallback_weight = 0.002f;
// Texels whose center lies outside every triangle but within this distance (texels) of one are shaded from the
// nearest point of that triangle, so bilinear lookups at chart borders read computed colors.
constexpr float conservative_distance = 0.75f;

struct uv_triangle
{
    float x[3], y[3];  // texel coordinates (x right, y down)
};

uv_triangle texel_triangle( const fr_mesh & m, uint32_t f, int w, int h )
{
    uv_triangle t;
    for( int k = 0; k < 3; ++k )
    {
        const float * uv = m.uvs + size_t( m.indices[size_t( f ) * 3 + k] ) * 2;
        t.x[k] = uv[0] * float( w );
        t.y[k] = ( 1.f - uv[1] ) * float( h );
    }
    return t;
}

// Barycentric coordinates of (px, py); false for a degenerate triangle.
bool barycentric( const uv_triangle & t, float px, float py, float l[3] )
{
    const float area = ( t.x[1] - t.x[0] ) * ( t.y[2] - t.y[0] ) - ( t.y[1] - t.y[0] ) * ( t.x[2] - t.x[0] );
    if( ! ( std::fabs( area ) > 1e-12f ) )
        return false;
    l[0] = ( ( t.x[2] - t.x[1] ) * ( py - t.y[1] ) - ( t.y[2] - t.y[1] ) * ( px - t.x[1] ) ) / area;
    l[1] = ( ( t.x[0] - t.x[2] ) * ( py - t.y[2] ) - ( t.y[0] - t.y[2] ) * ( px - t.x[2] ) ) / area;
    l[2] = 1.f - l[0] - l[1];
    return true;
}

float segment_distance2( float px, float py, float ax, float ay, float bx, float by )
{
    const float dx = bx - ax, dy = by - ay;
    const float len2 = dx * dx + dy * dy;
    float s = len2 > 0 ? ( ( px - ax ) * dx + ( py - ay ) * dy ) / len2 : 0.f;
    s = std::clamp( s, 0.f, 1.f );
    const float ex = ax + s * dx - px, ey = ay + s * dy - py;
    return ex * ex + ey * ey;
}

// Owner triangle of every texel (-1 = none). Serial on purpose: overlapping UVs (from arbitrary input meshes) then
// resolve deterministically, and the shading pass can write texels in parallel without conflicts.
std::vector< int32_t > rasterize_atlas( const fr_mesh & m, int w, int h )
{
    std::vector< int32_t > owner( size_t( w ) * h, -1 );
    for( int pass = 0; pass < 2; ++pass )
        for( uint32_t f = 0; f < m.triangle_count; ++f )
        {
            const uv_triangle t = texel_triangle( m, f, w, h );
            const float margin = pass == 0 ? 0.f : conservative_distance;
            const float lo_x = std::min( { t.x[0], t.x[1], t.x[2] } ) - margin, hi_x = std::max( { t.x[0], t.x[1], t.x[2] } ) + margin;
            const float lo_y = std::min( { t.y[0], t.y[1], t.y[2] } ) - margin, hi_y = std::max( { t.y[0], t.y[1], t.y[2] } ) + margin;
            if( ! std::isfinite( lo_x ) || ! std::isfinite( hi_x ) || ! std::isfinite( lo_y ) || ! std::isfinite( hi_y ) )
                continue;
            const int x0 = std::max( 0, int( std::ceil( lo_x - 0.5f ) ) ), x1 = std::min( w - 1, int( std::floor( hi_x - 0.5f ) ) );
            const int y0 = std::max( 0, int( std::ceil( lo_y - 0.5f ) ) ), y1 = std::min( h - 1, int( std::floor( hi_y - 0.5f ) ) );
            for( int y = y0; y <= y1; ++y )
                for( int x = x0; x <= x1; ++x )
                {
                    int32_t & o = owner[size_t( y ) * w + x];
                    const float px = float( x ) + 0.5f, py = float( y ) + 0.5f;
                    float l[3];
                    const bool valid = barycentric( t, px, py, l );
                    if( pass == 0 )
                    {
                        if( valid && l[0] >= 0 && l[1] >= 0 && l[2] >= 0 )
                            o = int32_t( f );
                    }
                    else if( o < 0 )
                    {
                        const float d2 = std::min( { segment_distance2( px, py, t.x[0], t.y[0], t.x[1], t.y[1] ),
                                                     segment_distance2( px, py, t.x[1], t.y[1], t.x[2], t.y[2] ),
                                                     segment_distance2( px, py, t.x[2], t.y[2], t.x[0], t.y[0] ) } );
                        if( d2 <= conservative_distance * conservative_distance )
                            o = int32_t( f );
                    }
                }
        }
    return owner;
}

// Grows the filled region by `iterations` texels (8-neighborhood average), then paints every remaining texel with
// the mean color so mipmaps never pull in black.
void dilate( uint8_t * rgba, std::vector< uint8_t > & state, int w, int h, int iterations )
{
    enum : uint8_t { empty = 0, filled = 1, queued = 2 };
    auto push_neighbors = [&]( int x, int y, std::vector< uint32_t > & list ) {
        for( int dy = -1; dy <= 1; ++dy )
            for( int dx = -1; dx <= 1; ++dx )
            {
                const int nx = x + dx, ny = y + dy;
                if( nx < 0 || ny < 0 || nx >= w || ny >= h )
                    continue;
                const size_t j = size_t( ny ) * w + nx;
                if( state[j] == empty )
                {
                    state[j] = queued;
                    list.push_back( uint32_t( j ) );
                }
            }
    };
    std::vector< uint32_t > frontier;
    for( int y = 0; y < h; ++y )
        for( int x = 0; x < w; ++x )
            if( state[size_t( y ) * w + x] == filled )
            {
                bool edge = false;
                for( int dy = -1; dy <= 1 && ! edge; ++dy )
                    for( int dx = -1; dx <= 1 && ! edge; ++dx )
                    {
                        const int nx = x + dx, ny = y + dy;
                        edge = nx >= 0 && ny >= 0 && nx < w && ny < h && state[size_t( ny ) * w + nx] == empty;
                    }
                if( edge )
                    push_neighbors( x, y, frontier );
            }
    std::vector< uint8_t > colors;
    std::vector< uint32_t > next;
    for( int it = 0; it < iterations && ! frontier.empty(); ++it )
    {
        colors.assign( frontier.size() * 3, 0 );
        parallel_for( frontier.size(), 4096, [&]( size_t begin, size_t end ) {
            for( size_t i = begin; i < end; ++i )
            {
                const int x = int( frontier[i] % uint32_t( w ) ), y = int( frontier[i] / uint32_t( w ) );
                int sum[3] = { 0, 0, 0 }, count = 0;
                for( int dy = -1; dy <= 1; ++dy )
                    for( int dx = -1; dx <= 1; ++dx )
                    {
                        const int nx = x + dx, ny = y + dy;
                        if( nx < 0 || ny < 0 || nx >= w || ny >= h || state[size_t( ny ) * w + nx] != filled )
                            continue;
                        const uint8_t * c = rgba + ( size_t( ny ) * w + nx ) * 4;
                        sum[0] += c[0];
                        sum[1] += c[1];
                        sum[2] += c[2];
                        ++count;
                    }
                for( int k = 0; k < 3; ++k )
                    colors[i * 3 + k] = uint8_t( count ? ( sum[k] + count / 2 ) / count : 0 );
            }
        } );
        for( size_t i = 0; i < frontier.size(); ++i )
        {
            uint8_t * c = rgba + size_t( frontier[i] ) * 4;
            c[0] = colors[i * 3];
            c[1] = colors[i * 3 + 1];
            c[2] = colors[i * 3 + 2];
            c[3] = 255;
            state[frontier[i]] = filled;
        }
        next.clear();
        for( uint32_t j : frontier )
            push_neighbors( int( j % uint32_t( w ) ), int( j / uint32_t( w ) ), next );
        frontier.swap( next );
    }

    double mean[3] = { 0, 0, 0 };
    size_t count = 0;
    const size_t total = size_t( w ) * h;
    for( size_t i = 0; i < total; ++i )
        if( state[i] == filled )
        {
            for( int k = 0; k < 3; ++k )
                mean[k] += rgba[i * 4 + k];
            ++count;
        }
    uint8_t fill[3] = { 128, 128, 128 };
    if( count )
        for( int k = 0; k < 3; ++k )
            fill[k] = uint8_t( std::lround( mean[k] / double( count ) ) );
    for( size_t i = 0; i < total; ++i )
        if( state[i] != filled )
        {
            std::copy_n( fill, 3, rgba + i * 4 );
            rgba[i * 4 + 3] = 255;
        }
}

const float * vertex_normals( const fr_mesh & m, std::vector< float > & storage )
{
    if( m.normals )
        return m.normals;
    storage.resize( size_t( m.vertex_count ) * 3 );
    compute_vertex_normals( m.positions, m.vertex_count, m.indices, m.triangle_count, storage.data(), m.uvs != nullptr );
    return storage.data();
}

}  // namespace

extern "C" {

void fr_texture_params_default( fr_texture_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->texture_size = 4096;
    p->view_angle_power = 4.f;
    p->min_cos_angle = 0.2f;
    p->blend_views = 3;
    p->occlusion_epsilon_m = 0.004f;
    p->dilate_px = 8;
    p->exposure_compensation = 1;
}

fr_status fr_bake_texture( const fr_mesh * mesh, const fr_rgbd_frame * frames, const float * world_from_camera,
                           const float * frame_weights, uint32_t count, const fr_texture_params * params,
                           fr_texture ** out, fr_progress_cb progress, void * user )
{
    if( out )
        *out = nullptr;
    return fr::guarded( "fr_bake_texture", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_bake_texture", true );
        if( s != FR_OK )
            return s;
        if( ! mesh->uvs )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_bake_texture: the mesh has no UVs (run fr_mesh_uv_unwrap)" );
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_bake_texture: out is NULL" );
        if( ( s = check_params( params, "fr_bake_texture", true ) ) != FR_OK )
            return s;
        cancel_state cancel;
        cancel.progress = progress;
        cancel.user = user;
        if( ! cancel.report( 0.f, "texture: preparing views" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_bake_texture: cancelled" );

        std::vector< float > normal_storage;
        const float * normals = vertex_normals( *mesh, normal_storage );
        view_set views;
        if( ( s = prepare_views( *mesh, frames, world_from_camera, frame_weights, count, *params, views,
                                 "fr_bake_texture" ) )
            != FR_OK )
            return s;
        if( ! cancel.report( 0.1f, "texture: exposure compensation" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_bake_texture: cancelled" );
        if( params->exposure_compensation )
            estimate_gains( mesh->positions, normals, mesh->vertex_count, views );
        std::vector< float > fallback;
        if( ! vertex_colors( *mesh, normals, views, fallback ) )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_bake_texture: no frame sees the mesh (check the poses)" );
        if( ! cancel.report( 0.2f, "texture: rasterizing the atlas" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_bake_texture: cancelled" );

        const int size = params->texture_size;
        const std::vector< int32_t > owner = rasterize_atlas( *mesh, size, size );
        fr_texture * texture = nullptr;
        if( ( s = fr_texture_create( size, size, &texture ) ) != FR_OK )
            return s;
        std::unique_ptr< fr_texture, void ( * )( fr_texture * ) > guard( texture, fr_texture_free );
        std::vector< uint8_t > state( size_t( size ) * size, 0 );

        const int rows_per_chunk = 16;
        const size_t chunks = size_t( size + rows_per_chunk - 1 ) / rows_per_chunk;
        std::atomic< size_t > done{ 0 };
        parallel_for( chunks, 1, [&]( size_t begin, size_t end ) {
            sample samples[max_blend_views];
            for( size_t chunk = begin; chunk < end; ++chunk )
            {
                if( cancel.cancelled.load() )
                    return;
                const int y_end = std::min( size, int( chunk + 1 ) * rows_per_chunk );
                for( int y = int( chunk ) * rows_per_chunk; y < y_end; ++y )
                    for( int x = 0; x < size; ++x )
                    {
                        const size_t idx = size_t( y ) * size + x;
                        const int32_t f = owner[idx];
                        if( f < 0 )
                            continue;
                        const uv_triangle t = texel_triangle( *mesh, uint32_t( f ), size, size );
                        float l[3];
                        if( ! barycentric( t, float( x ) + 0.5f, float( y ) + 0.5f, l ) )
                            l[0] = l[1] = l[2] = 1.f / 3.f;
                        // Border texels outside the triangle: clamp onto it.
                        float lsum = 0;
                        for( float & li : l )
                        {
                            li = std::max( li, 0.f );
                            lsum += li;
                        }
                        for( float & li : l )
                            li /= lsum;
                        const uint32_t * tri = mesh->indices + size_t( f ) * 3;
                        vec3 p, n;
                        float fb[3] = { 0, 0, 0 };
                        for( int k = 0; k < 3; ++k )
                        {
                            p += vec3::load( mesh->positions + size_t( tri[k] ) * 3 ) * l[k];
                            n += vec3::load( normals + size_t( tri[k] ) * 3 ) * l[k];
                            for( int c = 0; c < 3; ++c )
                                fb[c] += fallback[size_t( tri[k] ) * 3 + c] * l[k];
                        }
                        n = normalized( n );
                        const int found = views.evaluate( p, n, samples, views.blend );
                        float wsum = fallback_weight, rgb[3] = { fb[0] * fallback_weight, fb[1] * fallback_weight,
                                                                 fb[2] * fallback_weight };
                        for( int i = 0; i < found; ++i )
                        {
                            wsum += samples[i].weight;
                            for( int c = 0; c < 3; ++c )
                                rgb[c] += samples[i].weight * samples[i].rgb[c];
                        }
                        uint8_t * px = texture->rgba + idx * 4;
                        for( int c = 0; c < 3; ++c )
                            px[c] = uint8_t( std::clamp( std::lround( rgb[c] / wsum ), 0L, 255L ) );
                        px[3] = 255;
                        state[idx] = 1;
                    }
                const size_t finished = ++done;
                if( ( finished & 7 ) == 0 )
                    cancel.report( 0.25f + 0.7f * float( finished ) / float( chunks ), "texture: blending views" );
            }
        } );
        if( cancel.cancelled.load() )
            return fr::fail( FR_ERR_CANCELLED, "fr_bake_texture: cancelled" );
        if( ! cancel.report( 0.95f, "texture: filling the gutter" ) )
            return fr::fail( FR_ERR_CANCELLED, "fr_bake_texture: cancelled" );
        dilate( texture->rgba, state, size, size, params->dilate_px );
        cancel.report( 1.f, "texture: done" );
        *out = guard.release();
        return FR_OK;
    } );
}

fr_status fr_colorize_vertices( fr_mesh * mesh, const fr_rgbd_frame * frames, const float * world_from_camera,
                                const float * frame_weights, uint32_t count, const fr_texture_params * params )
{
    return fr::guarded( "fr_colorize_vertices", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_colorize_vertices", true );
        if( s != FR_OK )
            return s;
        if( ( s = check_params( params, "fr_colorize_vertices", false ) ) != FR_OK )
            return s;
        std::vector< float > normal_storage;
        const float * normals = vertex_normals( *mesh, normal_storage );
        view_set views;
        if( ( s = prepare_views( *mesh, frames, world_from_camera, frame_weights, count, *params, views,
                                 "fr_colorize_vertices" ) )
            != FR_OK )
            return s;
        if( params->exposure_compensation )
            estimate_gains( mesh->positions, normals, mesh->vertex_count, views );
        std::vector< float > rgb;
        if( ! vertex_colors( *mesh, normals, views, rgb ) )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_colorize_vertices: no frame sees the mesh (check the poses)" );
        const uint32_t flags = FR_MESH_COLORS | ( mesh->normals ? FR_MESH_NORMALS : 0u ) | ( mesh->uvs ? FR_MESH_UVS : 0u );
        if( ( s = fr_mesh_set_attributes( mesh, flags ) ) != FR_OK )
            return s;
        for( size_t i = 0; i < rgb.size(); ++i )
            mesh->colors[i] = uint8_t( std::clamp( std::lround( rgb[i] ), 0L, 255L ) );
        return FR_OK;
    } );
}

}  // extern "C"
