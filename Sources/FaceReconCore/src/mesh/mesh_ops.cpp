// License: Apache 2.0. See LICENSE file in root directory.
//
// Basic mesh operations: normals, cleanup (welding, degenerate/duplicate triangles), connected components,
// plane cropping, rigid transforms and bounds.

#include "fr_mesh.h"
#include "mesh_internal.h"

#include <array>
#include <cfloat>
#include <limits>
#include <numeric>
#include <unordered_map>

using namespace fr::mesh;

namespace
{

uint32_t attribute_flags( const fr_mesh & m )
{
    return ( m.normals ? FR_MESH_NORMALS : 0u ) | ( m.colors ? FR_MESH_COLORS : 0u ) | ( m.uvs ? FR_MESH_UVS : 0u );
}

void recompute_normals( mesh_buffers & b )
{
    if( ! b.has_normals )
        return;
    b.normals.resize( size_t( b.vertex_count() ) * 3 );
    compute_vertex_normals( b.positions.data(), b.vertex_count(), b.indices.data(), b.triangle_count(), b.normals.data() );
}

// Open-addressing hash map from a grid cell key to the first vertex of that cell's chain.
class cell_map
{
public:
    explicit cell_map( size_t expected )
    {
        size_t cap = 16;
        while( cap < expected * 2 )
            cap <<= 1;
        mask_ = cap - 1;
        keys_.assign( cap, empty );
        heads_.assign( cap, ~0u );
    }

    uint32_t find( uint64_t key ) const
    {
        for( size_t i = hash( key );; i = ( i + 1 ) & mask_ )
        {
            if( keys_[i] == key )
                return heads_[i];
            if( keys_[i] == empty )
                return ~0u;
        }
    }

    uint32_t & slot( uint64_t key )
    {
        for( size_t i = hash( key );; i = ( i + 1 ) & mask_ )
        {
            if( keys_[i] == key )
                return heads_[i];
            if( keys_[i] == empty )
            {
                keys_[i] = key;
                return heads_[i];
            }
        }
    }

private:
    static constexpr uint64_t empty = ~0ull;
    size_t hash( uint64_t key ) const { return size_t( ( key * 0x9E3779B97F4A7C15ull ) >> 17 ) & mask_; }
    size_t mask_;
    std::vector< uint64_t > keys_;
    std::vector< uint32_t > heads_;
};

// 21 bits per axis; coordinates outside that range wrap, which only merges far apart cells into one chain (the
// distance test keeps the result exact).
uint64_t cell_key( int64_t x, int64_t y, int64_t z )
{
    return ( ( uint64_t( x ) & 0x1FFFFF ) << 42 ) | ( ( uint64_t( y ) & 0x1FFFFF ) << 21 ) | ( uint64_t( z ) & 0x1FFFFF );
}

int64_t cell_coord( double v )
{
    v = std::floor( v );
    return int64_t( std::clamp( v, -4.0e15, 4.0e15 ) );
}

// Maps every vertex to the first vertex within epsilon (and with the same UV), or to itself.
std::vector< uint32_t > weld_map( const mesh_buffers & b, float epsilon )
{
    const uint32_t n = b.vertex_count();
    std::vector< uint32_t > remap( n );
    std::iota( remap.begin(), remap.end(), 0u );
    if( n == 0 )
        return remap;

    vec3 lo( FLT_MAX, FLT_MAX, FLT_MAX ), hi( -FLT_MAX, -FLT_MAX, -FLT_MAX );
    for( uint32_t v = 0; v < n; ++v )
    {
        vec3 p = b.position( v );
        if( ! finite( p ) )
            continue;
        lo = { std::min( lo.x, p.x ), std::min( lo.y, p.y ), std::min( lo.z, p.z ) };
        hi = { std::max( hi.x, p.x ), std::max( hi.y, p.y ), std::max( hi.z, p.z ) };
    }
    const double extent = std::max( { double( hi.x ) - lo.x, double( hi.y ) - lo.y, double( hi.z ) - lo.z, 0.0 } );
    // With a cell size of at least 2 * epsilon every neighbor within epsilon lies in one of the 2x2x2 cells on the
    // near side of the point, so 8 cells are searched instead of 27.
    double h = epsilon > 0 ? 2.0 * epsilon : 0.0;
    h = std::max( h, extent / double( 1 << 20 ) );
    if( ! ( h > 0 ) )
        h = 1.0;
    const double eps2 = double( epsilon > 0 ? epsilon : 0.f ) * ( epsilon > 0 ? epsilon : 0.f );
    constexpr float uv_tolerance = 1e-6f;

    cell_map cells( n );
    std::vector< uint32_t > next( n, ~0u );
    for( uint32_t v = 0; v < n; ++v )
    {
        const vec3 p = b.position( v );
        if( ! finite( p ) )
            continue;
        const double gx = p.x / h, gy = p.y / h, gz = p.z / h;
        const int64_t cx = cell_coord( gx ), cy = cell_coord( gy ), cz = cell_coord( gz );
        const int64_t nx = gx - double( cx ) < 0.5 ? cx - 1 : cx + 1;
        const int64_t ny = gy - double( cy ) < 0.5 ? cy - 1 : cy + 1;
        const int64_t nz = gz - double( cz ) < 0.5 ? cz - 1 : cz + 1;
        uint32_t found = ~0u;
        for( int corner = 0; corner < 8 && found == ~0u; ++corner )
        {
            const uint64_t key = cell_key( corner & 1 ? nx : cx, corner & 2 ? ny : cy, corner & 4 ? nz : cz );
            for( uint32_t w = cells.find( key ); w != ~0u; w = next[w] )
            {
                const vec3 q = b.position( w );
                const double dx = double( p.x ) - q.x, dy = double( p.y ) - q.y, dz = double( p.z ) - q.z;
                if( dx * dx + dy * dy + dz * dz > eps2 )
                    continue;
                if( b.has_uvs
                    && ( std::fabs( b.uvs[size_t( v ) * 2] - b.uvs[size_t( w ) * 2] ) > uv_tolerance
                         || std::fabs( b.uvs[size_t( v ) * 2 + 1] - b.uvs[size_t( w ) * 2 + 1] ) > uv_tolerance ) )
                    continue;
                found = w;
                break;
            }
        }
        if( found != ~0u )
        {
            remap[v] = found;
            continue;
        }
        uint32_t & head = cells.slot( cell_key( cx, cy, cz ) );
        next[v] = head;
        head = v;
    }
    return remap;
}

bool zero_area( const mesh_buffers & b, const uint32_t * tri )
{
    const float * p0 = &b.positions[size_t( tri[0] ) * 3];
    const float * p1 = &b.positions[size_t( tri[1] ) * 3];
    const float * p2 = &b.positions[size_t( tri[2] ) * 3];
    const double ux = double( p1[0] ) - p0[0], uy = double( p1[1] ) - p0[1], uz = double( p1[2] ) - p0[2];
    const double vx = double( p2[0] ) - p0[0], vy = double( p2[1] ) - p0[1], vz = double( p2[2] ) - p0[2];
    const double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
    return cx == 0 && cy == 0 && cz == 0;
}

struct union_find
{
    std::vector< uint32_t > parent, size;
    explicit union_find( uint32_t n )
        : parent( n )
        , size( n, 1 )
    {
        std::iota( parent.begin(), parent.end(), 0u );
    }
    uint32_t find( uint32_t x )
    {
        while( parent[x] != x )
        {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite( uint32_t a, uint32_t b )
    {
        a = find( a );
        b = find( b );
        if( a == b )
            return;
        if( size[a] < size[b] )
            std::swap( a, b );
        parent[b] = a;
        size[a] += size[b];
    }
};

}  // namespace

extern "C" {

fr_status fr_mesh_compute_normals( fr_mesh * mesh )
{
    return fr::guarded( "fr_mesh_compute_normals", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_compute_normals" );
        if( s != FR_OK )
            return s;
        s = fr_mesh_set_attributes( mesh, attribute_flags( *mesh ) | FR_MESH_NORMALS );
        if( s != FR_OK )
            return s;
        if( mesh->vertex_count )
            compute_vertex_normals( mesh->positions, mesh->vertex_count, mesh->indices, mesh->triangle_count,
                                    mesh->normals );
        return FR_OK;
    } );
}

fr_status fr_mesh_clean( fr_mesh * mesh, float epsilon_m )
{
    return fr::guarded( "fr_mesh_clean", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_clean" );
        if( s != FR_OK )
            return s;
        if( ! std::isfinite( epsilon_m ) || epsilon_m < 0 )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_clean: epsilon must be finite and >= 0" );
        mesh_buffers b = load( *mesh );
        const std::vector< uint32_t > remap = weld_map( b, epsilon_m );
        for( uint32_t & i : b.indices )
            i = remap[i];

        // Degenerate triangles: repeated vertices or exactly zero area. Nearly-collinear triangles with distinct
        // vertices stay, removing them would open slits in the surface.
        const uint32_t t = b.triangle_count();
        std::vector< uint8_t > keep( t, 1 );
        struct tri_key
        {
            uint32_t v[3];
            uint32_t index;
        };
        std::vector< tri_key > keys;
        keys.reserve( t );
        for( uint32_t f = 0; f < t; ++f )
        {
            const uint32_t * tri = &b.indices[size_t( f ) * 3];
            if( tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] || zero_area( b, tri ) )
            {
                keep[f] = 0;
                continue;
            }
            tri_key k{ { tri[0], tri[1], tri[2] }, f };
            std::sort( k.v, k.v + 3 );
            keys.push_back( k );
        }
        // Duplicates have the same vertex set in any order or orientation; the first one is kept.
        std::sort( keys.begin(), keys.end(), []( const tri_key & a, const tri_key & b ) {
            if( a.v[0] != b.v[0] )
                return a.v[0] < b.v[0];
            if( a.v[1] != b.v[1] )
                return a.v[1] < b.v[1];
            if( a.v[2] != b.v[2] )
                return a.v[2] < b.v[2];
            return a.index < b.index;
        } );
        for( size_t i = 1; i < keys.size(); ++i )
            if( std::equal( keys[i].v, keys[i].v + 3, keys[i - 1].v ) )
                keep[keys[i].index] = 0;

        std::vector< uint32_t > indices;
        indices.reserve( b.indices.size() );
        for( uint32_t f = 0; f < t; ++f )
            if( keep[f] )
                indices.insert( indices.end(), &b.indices[size_t( f ) * 3], &b.indices[size_t( f ) * 3] + 3 );
        b.indices.swap( indices );
        b.remove_unreferenced();
        recompute_normals( b );
        return store( b, mesh );
    } );
}

fr_status fr_mesh_remove_small_components( fr_mesh * mesh, float min_relative_size, int32_t keep_largest_only,
                                           uint32_t * removed_triangles )
{
    if( removed_triangles )
        *removed_triangles = 0;
    return fr::guarded( "fr_mesh_remove_small_components", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_remove_small_components" );
        if( s != FR_OK )
            return s;
        if( ! std::isfinite( min_relative_size ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_remove_small_components: min_relative_size is not finite" );
        const uint32_t n = mesh->vertex_count, t = mesh->triangle_count;
        if( t == 0 )
            return FR_OK;
        union_find uf( n );
        for( uint32_t f = 0; f < t; ++f )
        {
            const uint32_t * tri = mesh->indices + size_t( f ) * 3;
            uf.unite( tri[0], tri[1] );
            uf.unite( tri[0], tri[2] );
        }
        std::vector< uint32_t > triangles_of( n, 0 );
        for( uint32_t f = 0; f < t; ++f )
            ++triangles_of[uf.find( mesh->indices[size_t( f ) * 3] )];
        uint32_t largest_root = 0, largest = 0;
        for( uint32_t v = 0; v < n; ++v )
            if( triangles_of[v] > largest )
            {
                largest = triangles_of[v];
                largest_root = v;
            }
        const double threshold = double( std::max( min_relative_size, 0.f ) ) * largest;

        mesh_buffers b = load( *mesh );
        std::vector< uint32_t > indices;
        indices.reserve( b.indices.size() );
        uint32_t removed = 0;
        for( uint32_t f = 0; f < t; ++f )
        {
            const uint32_t root = uf.find( b.indices[size_t( f ) * 3] );
            const bool keep = keep_largest_only ? root == largest_root : double( triangles_of[root] ) >= threshold;
            if( keep )
                indices.insert( indices.end(), &b.indices[size_t( f ) * 3], &b.indices[size_t( f ) * 3] + 3 );
            else
                ++removed;
        }
        if( removed == 0 )
            return FR_OK;
        b.indices.swap( indices );
        b.remove_unreferenced();
        s = store( b, mesh );
        if( s == FR_OK && removed_triangles )
            *removed_triangles = removed;
        return s;
    } );
}

fr_status fr_mesh_crop_plane( fr_mesh * mesh, const float point[3], const float normal[3] )
{
    return fr::guarded( "fr_mesh_crop_plane", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_crop_plane" );
        if( s != FR_OK )
            return s;
        if( ! point || ! normal )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_crop_plane: NULL plane" );
        const vec3 o = vec3::load( point ), nn = vec3::load( normal );
        const float nl = length( nn );
        if( ! finite( o ) || ! std::isfinite( nl ) || nl < 1e-12f )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_crop_plane: invalid plane" );
        const vec3 n = nn / nl;

        mesh_buffers b = load( *mesh );
        const uint32_t vn = b.vertex_count(), t = b.triangle_count();
        // Vertices within a tiny band count as lying on the plane, which avoids microscopic slivers at the cut.
        double extent = 0;
        {
            float lo[3], hi[3];
            if( fr_mesh_bounds( mesh, lo, hi ) == FR_OK )
                extent = std::max( { hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] } );
        }
        const double snap = 1e-7 * std::max( extent, 1e-3 );
        std::vector< double > d( vn );
        for( uint32_t v = 0; v < vn; ++v )
        {
            const vec3 p = b.position( v );
            double dv = double( n.x ) * ( double( p.x ) - o.x ) + double( n.y ) * ( double( p.y ) - o.y )
                      + double( n.z ) * ( double( p.z ) - o.z );
            if( std::fabs( dv ) <= snap )
            {
                b.set_position( v, p - n * float( dv ) );
                dv = 0;
            }
            d[v] = dv;
        }

        std::unordered_map< uint64_t, uint32_t > cut_vertex;
        auto intersect = [&]( uint32_t a, uint32_t c ) -> uint32_t {
            const uint64_t key = a < c ? ( uint64_t( a ) << 32 | c ) : ( uint64_t( c ) << 32 | a );
            auto it = cut_vertex.find( key );
            if( it != cut_vertex.end() )
                return it->second;
            // Interpolate from the lower index so both triangles of the edge produce bit-identical vertices.
            const uint32_t lo = std::min( a, c ), hi = std::max( a, c );
            const float w = float( d[lo] / ( d[lo] - d[hi] ) );
            const uint32_t src[2] = { lo, hi };
            const float weights[2] = { 1.f - w, w };
            const uint32_t v = b.add_blend( src, weights, 2 );
            const vec3 p = b.position( v );
            b.set_position( v, p - n * dot( n, p - o ) );
            cut_vertex.emplace( key, v );
            return v;
        };

        std::vector< uint32_t > indices;
        indices.reserve( b.indices.size() );
        for( uint32_t f = 0; f < t; ++f )
        {
            const uint32_t tri[3] = { b.indices[size_t( f ) * 3], b.indices[size_t( f ) * 3 + 1],
                                      b.indices[size_t( f ) * 3 + 2] };
            const int inside = int( d[tri[0]] >= 0 ) + int( d[tri[1]] >= 0 ) + int( d[tri[2]] >= 0 );
            if( inside == 3 )
            {
                indices.insert( indices.end(), tri, tri + 3 );
                continue;
            }
            if( inside == 0 )
                continue;
            // Sutherland-Hodgman against the half-space d >= 0; a convex polygon of at most 4 vertices remains,
            // fanned in the original winding.
            uint32_t poly[4];
            int count = 0;
            for( int k = 0; k < 3; ++k )
            {
                const uint32_t a = tri[k], c = tri[( k + 1 ) % 3];
                if( d[a] >= 0 )
                    poly[count++] = a;
                if( ( d[a] > 0 && d[c] < 0 ) || ( d[a] < 0 && d[c] > 0 ) )
                    poly[count++] = intersect( a, c );
            }
            for( int k = 1; k + 1 < count; ++k )
                indices.insert( indices.end(), { poly[0], poly[k], poly[k + 1] } );
        }
        b.indices.swap( indices );
        b.remove_unreferenced();
        return store( b, mesh );
    } );
}

fr_status fr_mesh_transform( fr_mesh * mesh, const float transform[16] )
{
    return fr::guarded( "fr_mesh_transform", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_transform" );
        if( s != FR_OK )
            return s;
        if( ! transform )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_transform: transform is NULL" );
        const affine a = affine::from_column_major( transform );
        affine nm;
        if( ! a.is_finite() || ! a.normal_matrix( nm ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_transform: singular or non-finite transform" );
        const bool mirrored = a.det() < 0;
        parallel_for( mesh->vertex_count, 16384, [&]( size_t begin, size_t end ) {
            for( size_t v = begin; v < end; ++v )
            {
                a.apply( vec3::load( mesh->positions + v * 3 ) ).store( mesh->positions + v * 3 );
                if( mesh->normals )
                    normalized( nm.rotate( vec3::load( mesh->normals + v * 3 ) ) ).store( mesh->normals + v * 3 );
            }
        } );
        // A mirroring transform turns the surface inside out; reversing the winding keeps it counter-clockwise
        // seen from outside.
        if( mirrored )
            for( uint32_t f = 0; f < mesh->triangle_count; ++f )
                std::swap( mesh->indices[size_t( f ) * 3 + 1], mesh->indices[size_t( f ) * 3 + 2] );
        return FR_OK;
    } );
}

fr_status fr_mesh_bounds( const fr_mesh * mesh, float min[3], float max[3] )
{
    if( ! mesh || ! min || ! max || ( mesh->vertex_count && ! mesh->positions ) )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_bounds: NULL argument" );
    for( int k = 0; k < 3; ++k )
    {
        min[k] = 0;
        max[k] = 0;
    }
    bool any = false;
    float lo[3] = { FLT_MAX, FLT_MAX, FLT_MAX }, hi[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
    for( uint32_t v = 0; v < mesh->vertex_count; ++v )
    {
        const float * p = mesh->positions + size_t( v ) * 3;
        if( ! std::isfinite( p[0] ) || ! std::isfinite( p[1] ) || ! std::isfinite( p[2] ) )
            continue;
        any = true;
        for( int k = 0; k < 3; ++k )
        {
            lo[k] = std::min( lo[k], p[k] );
            hi[k] = std::max( hi[k], p[k] );
        }
    }
    if( ! any )
        return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_mesh_bounds: the mesh has no vertices" );
    for( int k = 0; k < 3; ++k )
    {
        min[k] = lo[k];
        max[k] = hi[k];
    }
    return FR_OK;
}

}  // extern "C"
