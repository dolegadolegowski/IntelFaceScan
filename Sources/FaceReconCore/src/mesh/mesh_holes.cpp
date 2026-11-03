// License: Apache 2.0. See LICENSE file in root directory.
//
// Hole filling after Liepa 2003 ("Filling Holes in Meshes"): boundary loops are triangulated with the minimum
// (largest dihedral angle, area) weight by dynamic programming, refined to the density of the surrounding mesh
// (centroid splits + Delaunay edge flips) and faired with a bi-Laplacian (thin plate) solve for the new vertices.

#include "fr_mesh.h"
#include "mesh_internal.h"

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include <array>
#include <cfloat>
#include <unordered_map>
#include <unordered_set>

using namespace fr::mesh;

namespace
{

using tri3 = std::array< uint32_t, 3 >;

// Loops larger than this are fanned from their centroid instead (the O(n^3) search gets too slow, O(n^2) memory).
constexpr uint32_t max_dp_loop = 1024;

uint64_t edge_key( uint32_t a, uint32_t b ) { return a < b ? ( uint64_t( a ) << 32 | b ) : ( uint64_t( b ) << 32 | a ); }

struct boundary_loop
{
    std::vector< uint32_t > v;  // v[i] -> v[i + 1] are boundary half-edges (the mesh is on their left)
    double length = 0;
};

// Traces all boundary loops. Paths through non-manifold vertices (a vertex visited twice) are split into simple loops.
std::vector< boundary_loop > find_boundary_loops( const mesh_buffers & b, const csr & out )
{
    const uint32_t n = b.vertex_count();
    csr bnd;
    bnd.offsets.assign( size_t( n ) + 1, 0 );
    for( uint32_t a = 0; a < n; ++a )
        for( uint32_t k = out.begin( a ); k < out.end( a ); ++k )
            if( ! has_edge( out, out.data[k], a ) )
                ++bnd.offsets[a + 1];
    for( uint32_t a = 0; a < n; ++a )
        bnd.offsets[a + 1] += bnd.offsets[a];
    bnd.data.resize( bnd.offsets[n] );
    for( uint32_t a = 0, fill = 0; a < n; ++a )
        for( uint32_t k = out.begin( a ); k < out.end( a ); ++k )
            if( ! has_edge( out, out.data[k], a ) )
                bnd.data[fill++] = out.data[k];
    std::vector< uint8_t > used( bnd.data.size(), 0 );
    auto take = [&]( uint32_t a ) -> uint32_t {
        for( uint32_t k = bnd.begin( a ); k < bnd.end( a ); ++k )
            if( ! used[k] )
            {
                used[k] = 1;
                return bnd.data[k];
            }
        return ~0u;
    };

    std::vector< boundary_loop > loops;
    std::unordered_map< uint32_t, size_t > position;
    std::vector< uint32_t > path;
    for( uint32_t start = 0; start < n; ++start )
    {
        uint32_t cur;
        while( ( cur = take( start ) ) != ~0u )
        {
            path.assign( 1, start );
            position.clear();
            position[start] = 0;
            for( ;; )
            {
                auto it = position.find( cur );
                if( it != position.end() )
                {
                    boundary_loop loop;
                    loop.v.assign( path.begin() + it->second, path.end() );
                    for( size_t i = it->second + 1; i < path.size(); ++i )
                        position.erase( path[i] );
                    path.resize( it->second + 1 );
                    if( loop.v.size() >= 3 )
                        loops.push_back( std::move( loop ) );
                }
                else
                {
                    position[cur] = path.size();
                    path.push_back( cur );
                }
                const uint32_t next = take( cur );
                if( next == ~0u )
                    break;  // either the path closed completely or it is an open chain (inconsistent orientation)
                cur = next;
            }
        }
    }
    for( auto & loop : loops )
        for( size_t i = 0; i < loop.v.size(); ++i )
            loop.length += length( b.position( loop.v[i] ) - b.position( loop.v[( i + 1 ) % loop.v.size()] ) );
    return loops;
}

// Normal of the mesh triangle that contains the directed edge a->b (zero if none).
vec3 edge_triangle_normal( const mesh_buffers & b, const csr & vf, size_t triangle_limit, uint32_t a, uint32_t c )
{
    for( uint32_t k = vf.begin( a ); k < vf.end( a ); ++k )
    {
        const uint32_t f = vf.data[k];
        if( f >= triangle_limit )
            continue;
        const uint32_t * tri = &b.indices[size_t( f ) * 3];
        for( int e = 0; e < 3; ++e )
            if( tri[e] == a && tri[( e + 1 ) % 3] == c )
            {
                const vec3 p0 = b.position( tri[0] ), p1 = b.position( tri[1] ), p2 = b.position( tri[2] );
                return normalized( cross( p1 - p0, p2 - p0 ), vec3() );
            }
    }
    return {};
}

// Liepa's dynamic programming over the loop polygon: minimizes (largest dihedral angle, total area)
// lexicographically. Returns triangles of loop-local indices, oriented to continue the mesh.
std::vector< tri3 > triangulate_loop( const std::vector< vec3 > & P, const std::vector< vec3 > & edge_normal )
{
    const uint32_t n = uint32_t( P.size() );
    std::vector< float > angle( size_t( n ) * n, 0.f ), area( size_t( n ) * n, 0.f );
    std::vector< uint16_t > best( size_t( n ) * n, 0 );
    std::vector< vec3 > top_normal( size_t( n ) * n );
    auto at = [n]( uint32_t i, uint32_t k ) { return size_t( i ) * n + k; };
    // Angle measure: 1 - cos(dihedral), monotonic in the angle and cheaper than acos.
    for( uint32_t len = 2; len < n; ++len )
        for( uint32_t i = 0; i + len < n; ++i )
        {
            const uint32_t k = i + len;
            float best_angle = FLT_MAX, best_area = FLT_MAX;
            uint32_t best_m = i + 1;
            vec3 best_normal;
            for( uint32_t m = i + 1; m < k; ++m )
            {
                // The fill triangle (i, k, m) contains the reversed boundary edges, which keeps the orientation.
                const vec3 c = cross( P[k] - P[i], P[m] - P[i] );
                const float l = length( c );
                const vec3 tn = l > 0 ? c / l : vec3();
                const vec3 & n_im = m == i + 1 ? edge_normal[i] : top_normal[at( i, m )];
                const vec3 & n_mk = k == m + 1 ? edge_normal[m] : top_normal[at( m, k )];
                float a = std::max( { angle[at( i, m )], angle[at( m, k )], 1.f - dot( tn, n_im ), 1.f - dot( tn, n_mk ) } );
                if( i == 0 && k == n - 1 )
                    a = std::max( a, 1.f - dot( tn, edge_normal[n - 1] ) );
                const float ar = area[at( i, m )] + area[at( m, k )] + 0.5f * l;
                if( a < best_angle - 1e-6f || ( a <= best_angle + 1e-6f && ar < best_area ) )
                {
                    best_angle = std::min( a, best_angle );
                    best_area = ar;
                    best_m = m;
                    best_normal = tn;
                }
            }
            angle[at( i, k )] = best_angle;
            area[at( i, k )] = best_area;
            best[at( i, k )] = uint16_t( best_m );
            top_normal[at( i, k )] = best_normal;
        }
    std::vector< tri3 > tris;
    tris.reserve( n - 2 );
    std::vector< std::pair< uint32_t, uint32_t > > stack{ { 0, n - 1 } };
    while( ! stack.empty() )
    {
        auto [i, k] = stack.back();
        stack.pop_back();
        if( k < i + 2 )
            continue;
        const uint32_t m = best[at( i, k )];
        tris.push_back( { i, k, m } );
        stack.push_back( { i, m } );
        stack.push_back( { m, k } );
    }
    return tris;
}

float corner_angle( const vec3 & at, const vec3 & a, const vec3 & b )
{
    const vec3 u = a - at, v = b - at;
    const float d = length( u ) * length( v );
    return d > 0 ? std::acos( std::clamp( dot( u, v ) / d, -1.f, 1.f ) ) : 0.f;
}

class hole_filler
{
public:
    hole_filler( mesh_buffers & b )
        : b_( b )
        , n0_( b.vertex_count() )
        , t0_( b.triangle_count() )
        , out_( outgoing_edges( n0_, b.indices.data(), t0_ ) )
        , vf_( vertex_faces( n0_, b.indices.data(), t0_ ) )
        , nb_( vertex_neighbors( n0_, b.indices.data(), t0_ ) )
    {
    }

    std::vector< boundary_loop > loops() const { return find_boundary_loops( b_, out_ ); }

    bool fill( const boundary_loop & loop )
    {
        const uint32_t n = uint32_t( loop.v.size() );
        const uint32_t first_new = b_.vertex_count();
        sigma_.clear();
        for( uint32_t i = 0; i < n; ++i )
        {
            const vec3 p = b_.position( loop.v[i] );
            const float prev = length( p - b_.position( loop.v[( i + n - 1 ) % n] ) );
            const float next = length( p - b_.position( loop.v[( i + 1 ) % n] ) );
            sigma_[loop.v[i]] = 0.5f * ( prev + next );
        }

        std::vector< tri3 > patch;
        if( n == 3 )
            patch.push_back( { loop.v[0], loop.v[2], loop.v[1] } );
        else if( n <= max_dp_loop )
        {
            std::vector< vec3 > P( n ), edge_normal( n );
            for( uint32_t i = 0; i < n; ++i )
            {
                P[i] = b_.position( loop.v[i] );
                edge_normal[i] = edge_triangle_normal( b_, vf_, t0_, loop.v[i], loop.v[( i + 1 ) % n] );
            }
            for( const tri3 & t : triangulate_loop( P, edge_normal ) )
                patch.push_back( { loop.v[t[0]], loop.v[t[1]], loop.v[t[2]] } );
        }
        else
        {
            std::vector< float > w( n, 1.f / float( n ) );
            const uint32_t c = b_.add_blend( loop.v.data(), w.data(), int( n ) );
            sigma_[c] = float( loop.length / n );
            for( uint32_t i = 0; i < n; ++i )
                patch.push_back( { loop.v[( i + 1 ) % n], loop.v[i], c } );
        }

        refine( patch );
        fair( patch, first_new );
        for( const tri3 & t : patch )
            b_.indices.insert( b_.indices.end(), t.begin(), t.end() );
        return true;
    }

private:
    bool original_edge( uint32_t a, uint32_t c ) const
    {
        return a < n0_ && c < n0_ && ( has_edge( out_, a, c ) || has_edge( out_, c, a ) );
    }

    // Liepa's refinement: split triangles whose centroid is far from its corners relative to the local edge length
    // (sigma), then flip edges to the Delaunay configuration, until no triangle splits.
    void refine( std::vector< tri3 > & patch )
    {
        constexpr float alpha = 1.41421356f;
        for( int pass = 0; pass < 32; ++pass )
        {
            bool split = false;
            const size_t count = patch.size();
            for( size_t i = 0; i < count; ++i )
            {
                const tri3 t = patch[i];
                const vec3 c = ( b_.position( t[0] ) + b_.position( t[1] ) + b_.position( t[2] ) ) * ( 1.f / 3.f );
                const float sc = ( sigma_[t[0]] + sigma_[t[1]] + sigma_[t[2]] ) / 3.f;
                bool ok = true;
                for( uint32_t v : t )
                {
                    const float d = alpha * length( c - b_.position( v ) );
                    if( ! ( d > sc && d > sigma_[v] ) )
                        ok = false;
                }
                if( ! ok )
                    continue;
                const float w[3] = { 1.f / 3.f, 1.f / 3.f, 1.f / 3.f };
                const uint32_t v = b_.add_blend( t.data(), w, 3 );
                sigma_[v] = sc;
                patch[i] = { t[0], t[1], v };
                patch.push_back( { t[1], t[2], v } );
                patch.push_back( { t[2], t[0], v } );
                split = true;
            }
            relax( patch );
            if( ! split )
                break;
        }
    }

    void relax( std::vector< tri3 > & patch )
    {
        for( int pass = 0; pass < 64; ++pass )
        {
            std::unordered_map< uint64_t, std::array< uint32_t, 2 > > edges;
            edges.reserve( patch.size() * 2 );
            for( uint32_t i = 0; i < patch.size(); ++i )
                for( int e = 0; e < 3; ++e )
                {
                    auto [it, inserted] = edges.try_emplace( edge_key( patch[i][e], patch[i][( e + 1 ) % 3] ),
                                                             std::array< uint32_t, 2 >{ i, ~0u } );
                    if( ! inserted )
                        it->second[1] = i;
                }
            std::vector< uint8_t > touched( patch.size(), 0 );
            std::unordered_set< uint64_t > created;
            int flips = 0;
            for( auto & [key, tris] : edges )
            {
                const uint32_t t1 = tris[0], t2 = tris[1];
                if( t2 == ~0u || touched[t1] || touched[t2] )
                    continue;
                // t1 = (a, b, c) with edge a->b; t2 contains b->a and the opposite vertex d.
                int e1 = 0;
                const uint32_t lo = uint32_t( key >> 32 ), hi = uint32_t( key );
                while( e1 < 3 && ! ( ( patch[t1][e1] == lo && patch[t1][( e1 + 1 ) % 3] == hi )
                                     || ( patch[t1][e1] == hi && patch[t1][( e1 + 1 ) % 3] == lo ) ) )
                    ++e1;
                if( e1 == 3 )
                    continue;
                const uint32_t a = patch[t1][e1], b = patch[t1][( e1 + 1 ) % 3], c = patch[t1][( e1 + 2 ) % 3];
                uint32_t d = ~0u;
                for( int e = 0; e < 3; ++e )
                    if( patch[t2][e] == b && patch[t2][( e + 1 ) % 3] == a )
                        d = patch[t2][( e + 2 ) % 3];
                if( d == ~0u || d == c )
                    continue;
                const vec3 pa = b_.position( a ), pb = b_.position( b ), pc = b_.position( c ), pd = b_.position( d );
                if( corner_angle( pc, pa, pb ) + corner_angle( pd, pa, pb ) <= 3.14159265f + 1e-4f )
                    continue;
                if( edges.count( edge_key( c, d ) ) || created.count( edge_key( c, d ) ) || original_edge( c, d ) )
                    continue;
                const vec3 quad = cross( pb - pa, pc - pa ) + cross( pa - pb, pd - pb );
                if( dot( cross( pa - pc, pd - pc ), quad ) <= 0 || dot( cross( pb - pd, pc - pd ), quad ) <= 0 )
                    continue;
                patch[t1] = { c, a, d };
                patch[t2] = { d, b, c };
                touched[t1] = touched[t2] = 1;
                created.insert( edge_key( c, d ) );
                ++flips;
            }
            if( flips == 0 )
                break;
        }
    }

    // Thin-plate fairing: the new vertices satisfy L(L(x)) = 0 with the uniform Laplacian over the combined mesh,
    // the loop and its original neighborhood fixed. Falls back to the membrane (L(x) = 0) if the solve fails.
    void fair( const std::vector< tri3 > & patch, uint32_t first_new )
    {
        const uint32_t count = b_.vertex_count() - first_new;
        if( count == 0 )
            return;
        std::unordered_map< uint32_t, std::vector< uint32_t > > ring;
        for( const tri3 & t : patch )
            for( int e = 0; e < 3; ++e )
            {
                ring[t[e]].push_back( t[( e + 1 ) % 3] );
                ring[t[e]].push_back( t[( e + 2 ) % 3] );
            }
        auto neighbors = [&]( uint32_t v ) -> const std::vector< uint32_t > & {
            auto & list = full_ring_[v];
            if( ! list.empty() )
                return list;
            auto it = ring.find( v );
            if( it != ring.end() )
                list = it->second;
            if( v < n0_ )
                list.insert( list.end(), nb_.data.begin() + nb_.begin( v ), nb_.data.begin() + nb_.end( v ) );
            std::sort( list.begin(), list.end() );
            list.erase( std::unique( list.begin(), list.end() ), list.end() );
            return list;
        };
        full_ring_.clear();

        using triplet = Eigen::Triplet< double >;
        std::vector< triplet > triplets;
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero( count, 3 );
        std::unordered_map< uint32_t, double > row;
        for( uint32_t u = 0; u < count; ++u )
        {
            const uint32_t i = first_new + u;
            row.clear();
            auto add_laplacian = [&]( uint32_t j, double scale ) {
                const auto & nj = neighbors( j );
                row[j] += scale;
                if( nj.empty() )
                    return;
                const double w = -scale / double( nj.size() );
                for( uint32_t k : nj )
                    row[k] += w;
            };
            const auto & ni = neighbors( i );
            add_laplacian( i, 1.0 );
            for( uint32_t j : ni )
                add_laplacian( j, -1.0 / double( ni.size() ) );
            for( auto [col, coef] : row )
            {
                if( col >= first_new )
                    triplets.emplace_back( u, col - first_new, coef );
                else
                {
                    const vec3 p = b_.position( col );
                    rhs( u, 0 ) -= coef * p.x;
                    rhs( u, 1 ) -= coef * p.y;
                    rhs( u, 2 ) -= coef * p.z;
                }
            }
        }
        Eigen::SparseMatrix< double > A( count, count );
        A.setFromTriplets( triplets.begin(), triplets.end() );
        Eigen::SparseLU< Eigen::SparseMatrix< double > > solver;
        solver.compute( A );
        bool solved = false;
        if( solver.info() == Eigen::Success )
        {
            Eigen::MatrixXd x = solver.solve( rhs );
            if( solver.info() == Eigen::Success && x.allFinite() )
            {
                for( uint32_t u = 0; u < count; ++u )
                    b_.set_position( first_new + u, vec3( float( x( u, 0 ) ), float( x( u, 1 ) ), float( x( u, 2 ) ) ) );
                solved = true;
            }
        }
        if( ! solved )
            for( int it = 0; it < 200; ++it )
                for( uint32_t u = 0; u < count; ++u )
                {
                    const auto & ni = neighbors( first_new + u );
                    if( ni.empty() )
                        continue;
                    vec3 sum;
                    for( uint32_t k : ni )
                        sum += b_.position( k );
                    b_.set_position( first_new + u, sum / float( ni.size() ) );
                }
    }

    mesh_buffers & b_;
    const uint32_t n0_, t0_;
    const csr out_, vf_, nb_;
    std::unordered_map< uint32_t, float > sigma_;
    std::unordered_map< uint32_t, std::vector< uint32_t > > full_ring_;
};

}  // namespace

extern "C" {

fr_status fr_mesh_fill_holes( fr_mesh * mesh, uint32_t max_hole_edges, uint32_t * filled_holes )
{
    if( filled_holes )
        *filled_holes = 0;
    return fr::guarded( "fr_mesh_fill_holes", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_fill_holes" );
        if( s != FR_OK )
            return s;
        if( mesh->triangle_count == 0 || max_hole_edges < 3 )
            return FR_OK;
        mesh_buffers b = load( *mesh );
        hole_filler filler( b );
        std::vector< boundary_loop > loops = filler.loops();
        if( loops.size() < 2 )
            return FR_OK;
        // The longest boundary is the open cut of the scan (neck) and stays open.
        size_t largest = 0;
        for( size_t i = 1; i < loops.size(); ++i )
            if( loops[i].length > loops[largest].length )
                largest = i;
        uint32_t filled = 0;
        for( size_t i = 0; i < loops.size(); ++i )
            if( i != largest && loops[i].v.size() <= max_hole_edges && filler.fill( loops[i] ) )
                ++filled;
        if( filled == 0 )
            return FR_OK;
        if( b.has_normals )
            compute_vertex_normals( b.positions.data(), b.vertex_count(), b.indices.data(), b.triangle_count(),
                                    b.normals.data(), b.has_uvs );
        s = store( b, mesh );
        if( s == FR_OK && filled_holes )
            *filled_holes = filled;
        return s;
    } );
}

}  // extern "C"
