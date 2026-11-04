// License: Apache 2.0. See LICENSE file in root directory.
//
// Hole filling after Liepa 2003 ("Filling Holes in Meshes"): boundary loops are triangulated with the minimum
// (largest dihedral angle, area) weight by dynamic programming, refined to the density of the surrounding mesh
// (centroid splits + Delaunay edge flips) and faired with a bi-Laplacian (thin plate) solve for the new vertices.
// Patches are checked for folds against the surrounding surface.
//
// Closing the longest loop (the unobserved back of a head scan joined with its neck cut: ~1.8 m and ~4700 edges at
// 0.75 mm voxels) is a different problem: the patch is 40% of the head, a Cartesian thin plate flattens it, and the
// ragged border folds against anything attached to it. So:
//  * the fusion's fringe along the loop (teeth, folded-over triangles) is trimmed first, and the notches of the
//    zigzag border get triangles of loop vertices only (fill_holes_ex, close, cut_notches). Other loops beyond the
//    perimeter limit (more openings of the same kind) are closed the same way when a watertight result is asked for;
//  * the head center comes from a sphere fitted to the measured band along the loop, the chin crop's plane from the
//    loop's planar run (find_center, find_cut_plane);
//  * the patch is built in the stereographic plane of the directions around the head center: ear clipping (a fan for
//    what a self-crossing loop leaves), then Liepa refinement graded from the loop's edges up to a few millimeters,
//    with Delaunay flips; a planar triangulation does not fold (planar_patch);
//  * the new vertices keep those directions; their distance from the center is a thin plate of the loop's distances
//    (a head is star-shaped around its center and its distance function smooth, so the closing continues the head's
//    roundness), bounded around the membrane solution, and capped where the ray leaves through the planar chin crop
//    (an obstacle problem with an active set): that part becomes a flat cap (fair_radial);
//  * remaining folds are smoothed locally, and measured triangles the closing still folds are trimmed and the
//    closing redone (up to five attempts: on the synthetic heads 48 and 141 folded triangles after three, 37 and 110
//    after five, ~0.7 s each).

#include "fr_mesh.h"
#include "mesh_internal.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <array>
#include <cfloat>
#include <limits>
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
// lexicographically. Returns triangles of loop-local indices, oriented to continue the mesh, or nothing when every
// triangulation needs a forbidden diagonal (forbidden(i, k): an edge between loop vertices i and k already exists, and
// a second pair of triangles on it would make the edge non-manifold).
template< typename Forbidden >
std::vector< tri3 > triangulate_loop( const std::vector< vec3 > & P, const std::vector< vec3 > & edge_normal,
                                      Forbidden && forbidden )
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
            // (0, n - 1) is the loop's closing edge; every other (i, k) here is a diagonal.
            if( ! ( i == 0 && k == n - 1 ) && forbidden( i, k ) )
            {
                angle[at( i, k )] = FLT_MAX;
                area[at( i, k )] = FLT_MAX;
                continue;
            }
            for( uint32_t m = i + 1; m < k; ++m )
            {
                if( angle[at( i, m )] == FLT_MAX || angle[at( m, k )] == FLT_MAX )
                    continue;
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
    if( angle[at( 0, n - 1 )] == FLT_MAX )
        return {};
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

// Target edge length of a closing patch: grows from the loop's own edge lengths (h_j at loop samples q_j) by `grade`
// per unit of distance from the loop, up to a cap: h_max, or in the stereographic plane of a closing (z = 0) the
// length that maps to h_max on the sphere of radius 1 / stereo_scale, h_max * stereo_scale * (1 + |p|^2) / 2. Only
// samples within cap / grade can be below the cap, so they are looked up in a hash grid of that cell size.
class sizing_field
{
public:
    sizing_field( std::vector< vec3 > q, std::vector< float > h, float grade, float h_max, float stereo_scale = 0,
                  float max_radius2 = 0 )
        : q_( std::move( q ) )
        , h_( std::move( h ) )
        , grade_( grade )
        , h_max_( h_max )
        , stereo_( stereo_scale )
        , cell_( double( stereo_scale > 0 ? h_max * stereo_scale * 0.5f * ( 1 + max_radius2 ) : h_max ) / grade )
    {
        for( uint32_t j = 0; j < q_.size(); ++j )
            cells_[key( cell_of( q_[j] ) )].push_back( j );
    }

    float operator()( const vec3 & p ) const
    {
        float best = stereo_ > 0 ? h_max_ * stereo_ * 0.5f * ( 1 + p.x * p.x + p.y * p.y ) : h_max_;
        const std::array< int64_t, 3 > c = cell_of( p );
        for( int dx = -1; dx <= 1; ++dx )
            for( int dy = -1; dy <= 1; ++dy )
                for( int dz = -1; dz <= 1; ++dz )
                {
                    auto it = cells_.find( key( { c[0] + dx, c[1] + dy, c[2] + dz } ) );
                    if( it != cells_.end() )
                        for( uint32_t j : it->second )
                            best = std::min( best, h_[j] + grade_ * length( p - q_[j] ) );
                }
        return best;
    }

private:
    std::array< int64_t, 3 > cell_of( const vec3 & p ) const
    {
        return { int64_t( std::floor( double( p.x ) / cell_ ) ), int64_t( std::floor( double( p.y ) / cell_ ) ),
                 int64_t( std::floor( double( p.z ) / cell_ ) ) };
    }
    static uint64_t key( const std::array< int64_t, 3 > & c )
    {
        return ( uint64_t( c[0] ) & 0x1FFFFF ) << 42 | ( uint64_t( c[1] ) & 0x1FFFFF ) << 21 | ( uint64_t( c[2] ) & 0x1FFFFF );
    }

    std::vector< vec3 > q_;
    std::vector< float > h_;
    float grade_, h_max_, stereo_;
    double cell_;
    std::unordered_map< uint64_t, std::vector< uint32_t > > cells_;
};

// A plane through a run of the loop (the planar chin crop): the closing surface stays on its positive side.
struct cut_plane
{
    vec3 normal;
    float offset = 0;  // plane: dot(normal, x) = offset
    bool valid = false;
};

// A triangle is folded when its normal points against the surrounding surface: the summed area-weighted normals of
// its corners (fold_guard in mesh_smooth.cpp uses the same test). Degenerate triangles (height below 1e-5 of the
// longest edge) have no orientation and do not count.
inline bool folded_against( const vec3 & a, const vec3 & b, const vec3 & c, const vec3 & surrounding )
{
    const vec3 fn = cross( b - a, c - a );
    const float longest = std::max( { length2( b - a ), length2( c - b ), length2( a - c ) } );
    return length( fn ) > 1e-5f * longest && ! ( dot( fn, surrounding ) > 0 );
}

struct tri_hash
{
    size_t operator()( const tri3 & t ) const
    {
        return size_t( ( uint64_t( t[0] ) * 0x9E3779B97F4A7C15ull ) ^ ( uint64_t( t[1] ) * 0xC2B2AE3D27D4EB4Full )
                       ^ ( uint64_t( t[2] ) * 0x165667B19E3779F9ull ) );
    }
};

struct patch_stats
{
    size_t triangles = 0;
    double area = 0;
    uint32_t folded = 0;
};

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

    // A hole within the size limits: Liepa triangulation, refinement and thin-plate fairing. A patch that folds against
    // the surrounding surface is flipped and faired again, then smoothed locally, then replaced by the unrefined
    // triangulation; if that folds too, the hole stays open, unless must_fill (a watertight result): then the variant
    // with the fewest folds closes it.
    bool fill( const boundary_loop & original_loop, patch_stats & stats, bool must_fill )
    {
        const uint32_t first_new = b_.vertex_count();
        // A loop of coincident vertices bounds no area: nothing to fill. (Refining its patch split degenerate
        // triangles forever: their centroid is a rounding error away from the corners, the target length zero; tens of
        // GB within a minute when smoothing had contracted a small hole to a point.)
        if( ! ( original_loop.length > 1e-9 * std::max( 1.0, double( length( b_.position( original_loop.v[0] ) ) ) ) ) )
            return false;
        std::vector< tri3 > coarse;
        const boundary_loop & loop = original_loop;
        const uint32_t n = uint32_t( loop.v.size() );
        set_loop_sigma( loop );
        sizing_ = nullptr;

        const size_t notches = coarse.size();
        bool fan = false;
        if( n == 3 )
            coarse.push_back( { loop.v[0], loop.v[2], loop.v[1] } );
        else if( n <= max_dp_loop )
        {
            std::vector< vec3 > P( n ), edge_normal( n );
            for( uint32_t i = 0; i < n; ++i )
            {
                P[i] = b_.position( loop.v[i] );
                edge_normal[i] = edge_triangle_normal( b_, vf_, t0_, loop.v[i], loop.v[( i + 1 ) % n] );
            }
            const auto forbidden = [&]( uint32_t i, uint32_t k ) { return edge_exists( loop.v[i], loop.v[k] ); };
            for( const tri3 & t : triangulate_loop( P, edge_normal, forbidden ) )
                coarse.push_back( { loop.v[t[0]], loop.v[t[1]], loop.v[t[2]] } );
        }
        if( coarse.size() == notches )
        {
            // Large loops, and loops whose every triangulation would reuse an existing edge: a fan around a new
            // center vertex.
            const std::vector< tri3 > f = fan_patch( loop );
            coarse.insert( coarse.end(), f.begin(), f.end() );
            fan = true;
        }
        const uint32_t coarse_vertices = b_.vertex_count();

        std::vector< tri3 > patch = coarse;
        refine( patch, size_t( 4 ) * n * n + 1024 );
        fair( patch, first_new );
        if( ! keeps_manifold( patch, first_new ) )
        {
            truncate_vertices( first_new );
            return false;
        }
        uint32_t folded = count_folds( patch );
        for( int attempt = 0; attempt < 2 && folded > 0; ++attempt )
        {
            // Delaunay flips in the faired geometry, then fairing again on the new connectivity.
            relax( patch );
            fair( patch, first_new );
            folded = keeps_manifold( patch, first_new ) ? count_folds( patch ) : ~0u;
        }
        if( folded > 0 && folded != ~0u )
        {
            repair_folds( patch, first_new );
            folded = count_folds( patch );
        }
        if( folded > 0 && ! fan )
        {
            // The unrefined minimal triangulation: no new vertices that could fold.
            const mesh_buffers refined_tail = tail( coarse_vertices );
            const std::vector< tri3 > refined = patch;
            const uint32_t refined_folds = folded;
            truncate_vertices( coarse_vertices );
            patch = coarse;
            folded = count_folds( patch );
            if( must_fill && refined_folds < folded )
            {
                append( refined_tail );
                patch = refined;
                folded = refined_folds;
            }
        }
        // A watertight result was asked for: the patch with the fewest folds closes the hole anyway.
        if( folded > 0 && ! must_fill )
        {
            truncate_vertices( first_new );
            return false;
        }
        commit( patch, stats );
        stats.folded = folded;
        return true;
    }

    // The longest loop: a watertight closing surface (see the file comment).
    bool close( const boundary_loop & original_loop, float h_max, patch_stats & stats )
    {
        const uint32_t first_new = b_.vertex_count();
        if( original_loop.v.size() < 3
            || ! ( original_loop.length > 1e-9 * std::max( 1.0, double( length( b_.position( original_loop.v[0] ) ) ) ) ) )
            return false;
        std::vector< tri3 > notches;
        const boundary_loop loop = cut_notches( original_loop, notches );
        const uint32_t n = uint32_t( loop.v.size() );
        if( n < 3 )
            return false;
        set_loop_sigma( loop );
        plane_ = find_cut_plane( original_loop );
        // Bounded whatever the input: a few times the area of a hemisphere with the loop's length as circumference,
        // at the coarsest density, plus the boundary band.
        const double radius = loop.length / ( 2 * 3.14159265 );
        const size_t max_triangles
            = size_t( std::min( 4e6, 8.0 * 2 * 3.14159265 * radius * radius / ( 0.43 * double( h_max ) * h_max ) + 8.0 * n ) );

        vec3 center;
        std::vector< tri3 > patch;
        bool ok;
        if( find_center( loop, center ) && planar_patch( loop, center, h_max, max_triangles, patch ) )
        {
            patch.insert( patch.begin(), notches.begin(), notches.end() );
            // The first solve (uniform weights) gives the geometry for the cotangent weights of the second.
            ok = fair_radial( patch, first_new, center, false ) && fair_radial( patch, first_new, center, true );
        }
        else
        {
            // No center, or the loop does not unfold around it: a fan refined in place, faired in Cartesian
            // coordinates (uniform weights).
            truncate_vertices( first_new );
            std::vector< vec3 > q;
            std::vector< float > h;
            for( uint32_t v : loop.v )
            {
                q.push_back( b_.position( v ) );
                h.push_back( std::min( sigma_[v], h_max ) );
            }
            const sizing_field sizing( std::move( q ), std::move( h ), 0.3f, h_max );
            sizing_ = &sizing;
            patch = fan_patch( loop );
            refine( patch, max_triangles );
            sizing_ = nullptr;
            patch.insert( patch.begin(), notches.begin(), notches.end() );
            ok = fair_cartesian( patch, first_new, false );
        }
        if( ok )
            repair_folds( patch, first_new );
        plane_ = {};
        if( ! ok || ! keeps_manifold( patch, first_new ) )
        {
            truncate_vertices( first_new );
            return false;
        }
        const uint32_t folded = count_folds( patch );
        commit( patch, stats );
        stats.folded = folded;
        return true;
    }

    // A marching-cubes border zigzags: teeth of the measured surface with sharp notches of the hole between them.
    // Triangles across the notches (loop vertices only) come first, so the closing starts from a smoother loop and
    // its triangles at the border do not fold against the teeth. A notch is a loop vertex where the hole's angle is
    // below 100 degrees and the triangle across it faces like the surface there; repeated over the shrinking loop.
    boundary_loop cut_notches( const boundary_loop & loop, std::vector< tri3 > & triangles ) const
    {
        std::vector< uint32_t > v = loop.v;
        const float max_edge = float( 4 * loop.length / std::max< size_t >( loop.v.size(), 1 ) );
        const float cos_limit = std::cos( 100.f * 3.14159265f / 180.f );
        std::unordered_set< uint64_t > added;
        for( int pass = 0; pass < 6 && v.size() > 3; ++pass )
        {
            std::vector< uint8_t > removed( v.size(), 0 );
            size_t cut = 0;
            for( size_t i = 0; i < v.size(); ++i )
            {
                const size_t m = v.size();
                // Neighbors among the vertices not removed in this pass; a removed vertex's neighbors stay.
                size_t ia = ( i + m - 1 ) % m, ib = ( i + 1 ) % m;
                if( removed[i] || removed[ia] || removed[ib] || m - cut <= 3 )
                    continue;
                const uint32_t a = v[ia], c = v[i], b = v[ib];
                const vec3 pa = b_.position( a ), pc = b_.position( c ), pb = b_.position( b );
                const vec3 da = pa - pc, db = pb - pc;
                if( length( pb - pa ) > max_edge || ! ( length( da ) > 0 ) || ! ( length( db ) > 0 ) )
                    continue;
                // The hole's angle at c: between the edges, on the hole's side (the loop runs with the surface on its
                // left; seen along the surface normal, the hole's angle turns from (b - c) to (a - c)).
                vec3 normal;
                if( c < n0_ )
                    for( uint32_t k = vf_.begin( c ); k < vf_.end( c ); ++k )
                        normal += face_normal( &b_.indices[size_t( vf_.data[k] ) * 3] );
                normal = normalized( normal, vec3() );
                const tri3 t = { b, c, a };
                const vec3 tn = face_normal( t.data() );
                if( ! ( dot( tn, normal ) > 0 ) || dot( normalized( da ), normalized( db ) ) < cos_limit )
                    continue;
                if( edge_exists( a, b ) || added.count( edge_key( a, b ) ) || a == b )
                    continue;
                triangles.push_back( t );
                added.insert( edge_key( a, b ) );
                removed[i] = 1;
                ++cut;
            }
            if( cut == 0 )
                break;
            std::vector< uint32_t > kept;
            for( size_t i = 0; i < v.size(); ++i )
                if( ! removed[i] )
                    kept.push_back( v[i] );
            v.swap( kept );
        }
        boundary_loop out;
        out.v = v;
        for( size_t i = 0; i < v.size(); ++i )
            out.length += length( b_.position( v[i] ) - b_.position( v[( i + 1 ) % v.size()] ) );
        return out;
    }

    // Triangulates the loop in the stereographic plane of its directions around c (pole opposite the measured
    // surface): ear clipping of the loop polygon, then refinement and Delaunay flips in the plane, which cannot fold
    // a planar triangulation. The new vertices' directions (dir_) come from their plane positions; their distance
    // from c is left to fair_radial. False when the loop is not a simple polygon there.
    bool planar_patch( const boundary_loop & loop, const vec3 & c, float h_max, size_t max_triangles, std::vector< tri3 > & patch )
    {
        const uint32_t n = uint32_t( loop.v.size() );
        const uint32_t first_new = b_.vertex_count();
        vec3 mean;
        for( uint32_t v = 0; v < n0_; ++v )
            mean += normalized( b_.position( v ) - c, vec3() );
        const vec3 pole = normalized( -mean );
        const vec3 e1 = normalized( std::fabs( pole.x ) < 0.9f ? cross( pole, vec3( 1, 0, 0 ) ) : cross( pole, vec3( 0, 1, 0 ) ) );
        const vec3 e2 = cross( pole, e1 );
        // Plane coordinates of the loop, in counter-clockwise polygon order (the patch lies to the right of the loop's
        // half-edges): w[k] = loop.v[n - 1 - k].
        std::vector< uint32_t > w( n );
        std::vector< vec3 > P( n ), saved( n );
        double mean_distance = 0, max_radius2 = 0;
        for( uint32_t k = 0; k < n; ++k )
        {
            w[k] = loop.v[n - 1 - k];
            const vec3 x = b_.position( w[k] );
            saved[k] = x;
            const vec3 u = normalized( x - c );
            const float d = 1.f + dot( u, pole );
            if( ! ( d > 0.05f ) )
                return false;  // the loop reaches around behind the pole
            P[k] = vec3( dot( u, e1 ) / d, dot( u, e2 ) / d, 0.f );
            mean_distance += length( x - c );
            max_radius2 = std::max( max_radius2, double( length2( P[k] ) ) );
        }
        mean_distance /= n;
        // Seen from c, the ragged border can double back on itself over a few vertices (a marching-cubes zigzag
        // around a rim). Such local loops are straightened in the plane, which only affects the triangulation.
        auto crosses = [&]( uint32_t i, uint32_t j ) {
            const vec3 a = P[i], b = P[( i + 1 ) % n], c2 = P[j], d = P[( j + 1 ) % n];
            auto orient = []( const vec3 & p, const vec3 & q, const vec3 & r ) {
                return ( double( q.x ) - p.x ) * ( double( r.y ) - p.y ) - ( double( q.y ) - p.y ) * ( double( r.x ) - p.x );
            };
            return orient( a, b, c2 ) * orient( a, b, d ) < 0 && orient( c2, d, a ) * orient( c2, d, b ) < 0;
        };
        constexpr uint32_t local_span = 64;
        for( int pass = 0; pass < 8; ++pass )
        {
            uint32_t fixed_crossings = 0;
            for( uint32_t i = 0; i < n; ++i )
                for( uint32_t step = local_span; step >= 2; --step )
                {
                    const uint32_t j = ( i + step ) % n;
                    if( ! crosses( i, j ) )
                        continue;
                    // Vertices i+1 .. j onto the segment from i to j+1.
                    const vec3 a = P[i], b = P[( j + 1 ) % n];
                    for( uint32_t t = 1; t <= step; ++t )
                        P[( i + t ) % n] = a + ( b - a ) * ( float( t ) / float( step + 1 ) );
                    ++fixed_crossings;
                    break;
                }
            if( fixed_crossings == 0 )
                break;
        }
        double area = 0;
        for( uint32_t k = 0; k < n; ++k )
            area += double( P[k].x ) * P[( k + 1 ) % n].y - double( P[( k + 1 ) % n].x ) * P[k].y;
        if( ! ( area > 0 ) )
            return false;

        // Ear clipping (linked list; ears are convex corners without another polygon vertex inside, whose new edge
        // does not exist in the mesh yet).
        std::vector< uint32_t > prev( n ), next( n );
        for( uint32_t k = 0; k < n; ++k )
        {
            prev[k] = ( k + n - 1 ) % n;
            next[k] = ( k + 1 ) % n;
        }
        auto turn = [&]( uint32_t a, uint32_t b, uint32_t d ) {
            return ( double( P[b].x ) - P[a].x ) * ( double( P[d].y ) - P[a].y ) - ( double( P[b].y ) - P[a].y ) * ( double( P[d].x ) - P[a].x );
        };
        auto is_ear = [&]( uint32_t k, bool strict ) {
            const uint32_t a = prev[k], d = next[k];
            if( ! ( turn( a, k, d ) > 0 ) || edge_exists( w[a], w[d] ) )
                return false;
            if( ! strict )
                return true;
            // No remaining vertex inside or on the triangle (reflex vertices are the only candidates, but the
            // polygon is walked from k's neighborhood outward cheaply enough).
            for( uint32_t j = next[d]; j != a; j = next[j] )
            {
                if( turn( prev[j], j, next[j] ) > 0 )
                    continue;  // convex vertices cannot be inside an ear
                if( turn( a, k, j ) >= 0 && turn( k, d, j ) >= 0 && turn( d, a, j ) >= 0 )
                    return false;
            }
            return true;
        };
        uint32_t remaining = n, k = 0, stall = 0;
        bool relaxed = false, stuck = false;
        while( remaining > 3 && ! stuck )
        {
            if( is_ear( k, ! relaxed ) )
            {
                const uint32_t a = prev[k], d = next[k];
                patch.push_back( { w[a], w[k], w[d] } );
                next[a] = d;
                prev[d] = a;
                --remaining;
                k = a;
                stall = 0;
                relaxed = false;
            }
            else
            {
                k = next[k];
                if( ++stall > remaining )
                {
                    // No proper ear: a self-crossing of the loop seen from c. One convex corner goes anyway; when there
                    // is none, the rest is fanned around its centroid below.
                    stuck = relaxed;
                    relaxed = true;
                    stall = 0;
                }
            }
        }
        // The fan's center is a new vertex whose plane position is the centroid of what is left of the polygon.
        uint32_t fan_center = ~0u;
        vec3 fan_position;
        if( stuck )
        {
            std::vector< uint32_t > rest;
            for( uint32_t j = k, count = 0; count < remaining; j = next[j], ++count )
            {
                rest.push_back( w[j] );
                fan_position += P[j];
            }
            fan_position = fan_position / float( remaining );
            const std::vector< float > weights( rest.size(), 1.f / float( rest.size() ) );
            fan_center = b_.add_blend( rest.data(), weights.data(), int( rest.size() ) );
            for( uint32_t j = k, count = 0; count < remaining; j = next[j], ++count )
                patch.push_back( { w[j], w[next[j]], fan_center } );
        }
        else
            patch.push_back( { w[prev[k]], w[k], w[next[k]] } );

        // Refinement and Delaunay flips in the plane: the loop vertices temporarily take their plane positions.
        for( uint32_t j = 0; j < n; ++j )
            b_.set_position( w[j], P[j] );
        if( fan_center != ~0u )
            b_.set_position( fan_center, fan_position );
        sigma_.clear();
        std::vector< vec3 > q( n );
        std::vector< float > h( n );
        double perimeter = 0;
        for( uint32_t j = 0; j < n; ++j )
            perimeter += length( P[( j + 1 ) % n] - P[j] );
        const float floor = float( 0.5 * perimeter / n );
        for( uint32_t j = 0; j < n; ++j )
        {
            const float l = std::max( 0.5f * ( length( P[j] - P[( j + n - 1 ) % n] ) + length( P[j] - P[( j + 1 ) % n] ) ), floor );
            sigma_[w[j]] = l;
            q[j] = P[j];
            h[j] = l;
        }
        // Lengths on the unit sphere relate to the plane by 2 / (1 + |p|^2), and to space by the distance from c.
        const sizing_field planar( q, h, 0.3f, h_max, float( 1.0 / mean_distance ), float( max_radius2 ) );
        sizing_ = &planar;
        relax( patch );
        refine( patch, max_triangles );
        sizing_ = nullptr;
        const uint32_t count = b_.vertex_count() - first_new;
        dir_.assign( count, vec3() );
        for( uint32_t u = 0; u < count; ++u )
        {
            const vec3 s = b_.position( first_new + u );
            const float r2 = s.x * s.x + s.y * s.y;
            dir_[u] = normalized( pole * ( ( 1 - r2 ) / ( 1 + r2 ) ) + e1 * ( 2 * s.x / ( 1 + r2 ) ) + e2 * ( 2 * s.y / ( 1 + r2 ) ) );
        }
        uint32_t inverted = 0;
        for( const tri3 & t : patch )
            if( ! ( dot( face_normal( t.data() ), vec3( 0, 0, 1 ) ) > 0 ) )
                ++inverted;
        for( uint32_t j = 0; j < n; ++j )
            b_.set_position( w[j], saved[j] );
        for( uint32_t u = 0; u < count; ++u )
            b_.set_position( first_new + u, c + dir_[u] * float( mean_distance ) );
        return true;
    }

private:
    static uint64_t directed_key( uint32_t a, uint32_t c ) { return uint64_t( a ) << 32 | c; }
    static tri3 sorted( tri3 t )
    {
        std::sort( t.begin(), t.end() );
        return t;
    }

    // Local edge length at each loop vertex, at least half the loop's mean edge length, so that runs of (nearly)
    // coincident loop vertices do not make the refinement's target length vanish.
    void set_loop_sigma( const boundary_loop & loop )
    {
        const uint32_t n = uint32_t( loop.v.size() );
        sigma_.clear();
        const float sigma_floor = float( 0.5 * loop.length / n );
        for( uint32_t i = 0; i < n; ++i )
        {
            const vec3 p = b_.position( loop.v[i] );
            const float prev = length( p - b_.position( loop.v[( i + n - 1 ) % n] ) );
            const float next = length( p - b_.position( loop.v[( i + 1 ) % n] ) );
            sigma_[loop.v[i]] = std::max( 0.5f * ( prev + next ), sigma_floor );
        }
    }

    std::vector< tri3 > fan_patch( const boundary_loop & loop )
    {
        const uint32_t n = uint32_t( loop.v.size() );
        std::vector< float > w( n, 1.f / float( n ) );
        const uint32_t c = b_.add_blend( loop.v.data(), w.data(), int( n ) );
        sigma_[c] = sizing_ ? ( *sizing_ )( b_.position( c ) ) : float( loop.length / n );
        std::vector< tri3 > patch;
        patch.reserve( n );
        for( uint32_t i = 0; i < n; ++i )
            patch.push_back( { loop.v[( i + 1 ) % n], loop.v[i], c } );
        return patch;
    }

    // The plane of the longest planar run of the loop, if the run is long enough to be a cut rather than chance:
    // crop vertices lie on their plane to float precision, and smoothing moves cut vertices only along the cut.
    // Candidate planes through nearby loop vertices find the run with a loose tolerance (three points a few
    // millimeters apart on a gently curved cut give an imprecise normal); a least-squares fit to the run's vertices
    // then has to hold them within a micrometer-scale tolerance.
    cut_plane find_cut_plane( const boundary_loop & loop ) const
    {
        const uint32_t n = uint32_t( loop.v.size() );
        if( n < 32 )
            return {};
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        for( uint32_t v : loop.v )
            origin += Eigen::Vector3d( b_.position( v ).x, b_.position( v ).y, b_.position( v ).z );
        origin /= double( n );
        std::vector< Eigen::Vector3d > P( n );
        double extent = 0;
        for( uint32_t i = 0; i < n; ++i )
        {
            const vec3 p = b_.position( loop.v[i] );
            P[i] = Eigen::Vector3d( p.x, p.y, p.z ) - origin;
            extent = std::max( extent, P[i].norm() );
        }
        // Longest cyclic run of consecutive loop vertices within tolerance of the plane: (length, first, count).
        // The longest cyclic run of loop edges within tolerance of the plane, and all such edges together (a crop line
        // can be interrupted where the trimmed fringe or a tooth leaves the plane for a few vertices).
        struct run_info
        {
            double length = 0, total_length = 0;
            uint32_t first = 0, count = 0, total_count = 0;
        };
        auto longest_run = [&]( const Eigen::Vector3d & normal, double offset, double tolerance ) {
            run_info best, cur;
            for( uint32_t k = 0; k < 2 * n && cur.count < n; ++k )
            {
                const uint32_t i = k % n, j = ( k + 1 ) % n;
                if( std::fabs( normal.dot( P[i] ) - offset ) < tolerance && std::fabs( normal.dot( P[j] ) - offset ) < tolerance )
                {
                    if( cur.count == 0 )
                        cur.first = i;
                    cur.length += ( P[j] - P[i] ).norm();
                    ++cur.count;
                    if( cur.length > best.length )
                    {
                        const double total_length = best.total_length;
                        const uint32_t total_count = best.total_count;
                        best = cur;
                        best.total_length = total_length;
                        best.total_count = total_count;
                    }
                    if( k < n )
                    {
                        best.total_length += ( P[j] - P[i] ).norm();
                        ++best.total_count;
                    }
                }
                else
                    cur = {};
            }
            return best;
        };
        auto fit = [&]( const run_info & run, Eigen::Vector3d & normal, double & offset ) {
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for( uint32_t k = 0; k <= run.count; ++k )
                mean += P[( run.first + k ) % n];
            mean /= double( run.count + 1 );
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for( uint32_t k = 0; k <= run.count; ++k )
            {
                const Eigen::Vector3d d = P[( run.first + k ) % n] - mean;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > eig( cov );
            normal = eig.eigenvectors().col( 0 );
            offset = normal.dot( mean );
            // A straight run does not determine a plane.
            return eig.eigenvalues()( 1 ) > 1e-6 * eig.eigenvalues()( 2 );
        };

        const double loose = 2e-4 * extent, tight = std::max( 2e-6, 1e-5 * extent );
        const uint32_t step = std::max< uint32_t >( 4, n / 256 );
        run_info best;
        Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
        double best_offset = 0;
        for( uint32_t i = 0; i < n; i += step )
        {
            const Eigen::Vector3d & a = P[i];
            const Eigen::Vector3d nn = ( P[( i + step ) % n] - a ).cross( P[( i + 2 * step ) % n] - a );
            if( ! ( nn.norm() > 1e-12 ) )
                continue;
            const Eigen::Vector3d normal = nn.normalized();
            const run_info run = longest_run( normal, normal.dot( a ), loose );
            if( run.length > best.length )
            {
                best = run;
                best_normal = normal;
                best_offset = normal.dot( a );
            }
        }
        if( best.count < 8 )
            return {};
        for( int pass = 0; pass < 3; ++pass )
        {
            Eigen::Vector3d normal;
            double offset;
            if( ! fit( best, normal, offset ) )
                return {};
            const run_info run = longest_run( normal, offset, pass == 0 ? 4 * tight : tight );
            if( run.count < 8 )
                return {};
            best = run;
            best_normal = normal;
            best_offset = offset;
        }
        // Marching-cubes borders are never planar over more than a few vertices; a cut is (on a head scan 597
        // consecutive loop vertices within 1.3 micrometers, 154 mm of a 1777 mm loop; a ragged 1 mm-voxel border is
        // 3.5 m long around a 165 mm cut).
        if( ! ( best.count >= 24 && best.total_count >= 64 && best.total_length > 0.02 * loop.length ) )
            return {};
        // Orient the normal toward the mesh: the loop's neighborhood lies on the positive side.
        double side = 0;
        for( uint32_t i = 0; i < n; ++i )
        {
            const uint32_t v = loop.v[i];
            for( uint32_t k = nb_.begin( v ); k < nb_.end( v ); ++k )
            {
                const vec3 q = b_.position( nb_.data[k] );
                side += best_normal.dot( Eigen::Vector3d( q.x, q.y, q.z ) - origin ) - best_offset;
            }
        }
        if( side < 0 )
        {
            best_normal = -best_normal;
            best_offset = -best_offset;
        }
        cut_plane plane;
        plane.normal = vec3( float( best_normal.x() ), float( best_normal.y() ), float( best_normal.z() ) );
        plane.offset = float( best_offset + best_normal.dot( origin ) );
        plane.valid = true;
        return plane;
    }

    bool original_edge( uint32_t a, uint32_t c ) const
    {
        return a < n0_ && c < n0_ && ( has_edge( out_, a, c ) || has_edge( out_, c, a ) );
    }
    // Undirected edge of the mesh as filled so far.
    bool edge_exists( uint32_t a, uint32_t c ) const
    {
        return original_edge( a, c ) || patch_undirected_.count( edge_key( a, c ) );
    }
    bool directed_edge_exists( uint32_t a, uint32_t c ) const
    {
        return ( a < n0_ && c < n0_ && has_edge( out_, a, c ) ) || patch_directed_.count( directed_key( a, c ) );
    }
    bool face_exists( const tri3 & t ) const
    {
        if( t[0] < n0_ && t[1] < n0_ && t[2] < n0_ )
            for( uint32_t k = vf_.begin( t[0] ); k < vf_.end( t[0] ); ++k )
            {
                const uint32_t * other = &b_.indices[size_t( vf_.data[k] ) * 3];
                if( sorted( { other[0], other[1], other[2] } ) == sorted( t ) )
                    return true;
            }
        return patch_faces_.count( sorted( t ) ) > 0;
    }

    // A patch may only add directed edges the mesh does not have yet (each once) and no second copy of an existing
    // triangle. A loop that bounds a piece of surface instead of a hole (a flap attached at a vertex, a lone
    // triangle) would otherwise be closed with a reversed copy of that piece: duplicate faces and edges with four
    // triangles.
    bool keeps_manifold( const std::vector< tri3 > & patch, uint32_t first_new ) const
    {
        std::unordered_set< uint64_t > added;
        added.reserve( patch.size() * 3 );
        for( const tri3 & t : patch )
        {
            if( t[0] == t[1] || t[1] == t[2] || t[0] == t[2] )
                return false;
            for( int e = 0; e < 3; ++e )
            {
                const uint32_t a = t[e], c = t[( e + 1 ) % 3];
                if( ( a < first_new && c < first_new && directed_edge_exists( a, c ) ) || ! added.insert( directed_key( a, c ) ).second )
                    return false;
            }
            if( t[0] < first_new && t[1] < first_new && t[2] < first_new && face_exists( t ) )
                return false;
        }
        return true;
    }

    vec3 face_normal( const uint32_t * t ) const
    {
        const vec3 a = b_.position( t[0] );
        return cross( b_.position( t[1] ) - a, b_.position( t[2] ) - a );
    }

    // Folds the patch causes: patch triangles, and original triangles at the loop that were not folded before,
    // whose normal points against the surrounding surface (the summed area-weighted normals of their corners over
    // the original mesh and the patch), as fold_guard in mesh_smooth.cpp and the export checks count them.
    uint32_t count_folds( const std::vector< tri3 > & patch ) const
    {
        std::unordered_map< uint32_t, vec3 > with, without;
        for( const tri3 & t : patch )
        {
            const vec3 fn = face_normal( t.data() );
            for( uint32_t v : t )
                with[v] += fn;
        }
        std::vector< uint32_t > border;
        for( auto & [v, sum] : with )
            if( v < n0_ )
                border.push_back( v );
        std::sort( border.begin(), border.end() );
        // Original triangles at the loop, and the vertices whose normals they need.
        std::vector< uint32_t > faces;
        for( uint32_t v : border )
            for( uint32_t k = vf_.begin( v ); k < vf_.end( v ); ++k )
                faces.push_back( vf_.data[k] );
        std::sort( faces.begin(), faces.end() );
        faces.erase( std::unique( faces.begin(), faces.end() ), faces.end() );
        std::unordered_set< uint32_t > corners;
        for( uint32_t f : faces )
            for( int k = 0; k < 3; ++k )
                corners.insert( b_.indices[size_t( f ) * 3 + k] );
        for( uint32_t v : corners )
        {
            vec3 sum;
            for( uint32_t k = vf_.begin( v ); k < vf_.end( v ); ++k )
                sum += face_normal( &b_.indices[size_t( vf_.data[k] ) * 3] );
            without[v] = sum;
            with[v] += sum;
        }
        uint32_t folded = 0;
        auto folded_with = [&]( const uint32_t * t, std::unordered_map< uint32_t, vec3 > & normals ) {
            return folded_against( b_.position( t[0] ), b_.position( t[1] ), b_.position( t[2] ),
                                   normals[t[0]] + normals[t[1]] + normals[t[2]] );
        };
        for( const tri3 & t : patch )
            if( folded_with( t.data(), with ) )
                ++folded;
        for( uint32_t f : faces )
        {
            const uint32_t * t = &b_.indices[size_t( f ) * 3];
            const bool before = folded_with( t, without );
            const bool after = folded_with( t, with );
            if( after && ! before )
                ++folded;
        }
        return folded;
    }

    // Indices of the patch triangles count_folds counts as folded (patch triangles only).
    std::vector< uint32_t > folded_patch_triangles( const std::vector< tri3 > & patch ) const
    {
        std::unordered_map< uint32_t, vec3 > with;
        for( const tri3 & t : patch )
        {
            const vec3 fn = face_normal( t.data() );
            for( uint32_t v : t )
                with[v] += fn;
        }
        for( auto & [v, sum] : with )
            if( v < n0_ )
                for( uint32_t k = vf_.begin( v ); k < vf_.end( v ); ++k )
                    sum += face_normal( &b_.indices[size_t( vf_.data[k] ) * 3] );
        std::vector< uint32_t > out;
        for( uint32_t i = 0; i < patch.size(); ++i )
        {
            const tri3 & t = patch[i];
            if( folded_against( b_.position( t[0] ), b_.position( t[1] ), b_.position( t[2] ), with[t[0]] + with[t[1]] + with[t[2]] ) )
                out.push_back( i );
        }
        return out;
    }

    // Local repair of the folds a patch still has: the new vertices around folded triangles (two rings) move toward
    // the mean of their neighbors, a few steps at a time, until nothing is folded (or 60 rounds).
    void repair_folds( const std::vector< tri3 > & patch, uint32_t first_new )
    {
        const uint32_t nf = b_.vertex_count() - first_new;
        if( nf == 0 )
            return;
        std::vector< std::vector< uint32_t > > ring( nf );
        for( const tri3 & t : patch )
            for( int e = 0; e < 3; ++e )
                if( t[e] >= first_new )
                {
                    ring[t[e] - first_new].push_back( t[( e + 1 ) % 3] );
                    ring[t[e] - first_new].push_back( t[( e + 2 ) % 3] );
                }
        for( auto & r : ring )
        {
            std::sort( r.begin(), r.end() );
            r.erase( std::unique( r.begin(), r.end() ), r.end() );
        }
        std::vector< uint8_t > active( nf, 0 );
        std::vector< vec3 > next( nf );
        for( int round = 0; round < 60; ++round )
        {
            const std::vector< uint32_t > folded = folded_patch_triangles( patch );
            if( folded.empty() )
                return;
            std::fill( active.begin(), active.end(), 0 );
            std::vector< uint32_t > seeds;
            for( uint32_t i : folded )
                for( uint32_t v : patch[i] )
                    if( v >= first_new && ! active[v - first_new] )
                    {
                        active[v - first_new] = 1;
                        seeds.push_back( v - first_new );
                    }
            for( int grow = 0; grow < 2; ++grow )
            {
                const std::vector< uint32_t > frontier = seeds;
                for( uint32_t u : frontier )
                    for( uint32_t w : ring[u] )
                        if( w >= first_new && ! active[w - first_new] )
                        {
                            active[w - first_new] = 1;
                            seeds.push_back( w - first_new );
                        }
            }
            for( int step = 0; step < 5; ++step )
            {
                for( uint32_t u : seeds )
                {
                    vec3 mean;
                    for( uint32_t w : ring[u] )
                        mean += b_.position( w );
                    next[u] = ring[u].empty() ? b_.position( first_new + u ) : b_.position( first_new + u ) * 0.5f + mean * ( 0.5f / float( ring[u].size() ) );
                }
                for( uint32_t u : seeds )
                    b_.set_position( first_new + u, next[u] );
            }
        }
    }

    void commit( const std::vector< tri3 > & patch, patch_stats & stats )
    {
        stats = {};
        stats.triangles = patch.size();
        for( const tri3 & t : patch )
        {
            b_.indices.insert( b_.indices.end(), t.begin(), t.end() );
            stats.area += 0.5 * double( length( face_normal( t.data() ) ) );
            for( int e = 0; e < 3; ++e )
            {
                patch_directed_.insert( directed_key( t[e], t[( e + 1 ) % 3] ) );
                patch_undirected_.insert( edge_key( t[e], t[( e + 1 ) % 3] ) );
            }
            patch_faces_.insert( sorted( t ) );
        }
    }

    // The vertices from `count` on (positions and attributes), and appending such a copy again.
    mesh_buffers tail( uint32_t count ) const
    {
        mesh_buffers t = b_.empty_like();
        for( uint32_t v = count; v < b_.vertex_count(); ++v )
            t.add_copy( b_, v );
        return t;
    }
    void append( const mesh_buffers & t )
    {
        for( uint32_t v = 0; v < t.vertex_count(); ++v )
            b_.add_copy( t, v );
    }

    void truncate_vertices( uint32_t count )
    {
        b_.positions.resize( size_t( count ) * 3 );
        if( b_.has_normals )
            b_.normals.resize( size_t( count ) * 3 );
        if( b_.has_colors )
            b_.colors.resize( size_t( count ) * 3 );
        if( b_.has_uvs )
            b_.uvs.resize( size_t( count ) * 2 );
    }

    // Liepa's refinement: split triangles whose centroid is far from its corners relative to the local edge length
    // (sigma), then flip edges to the Delaunay configuration, until no triangle splits. New vertices take the mean
    // sigma of their triangle, or the sizing field's value where one is set (closing).
    // max_triangles bounds the patch whatever the input (a round hole with n boundary edges refines to about n^2 / 5
    // triangles).
    void refine( std::vector< tri3 > & patch, size_t max_triangles )
    {
        constexpr float alpha = 1.41421356f;
        for( int pass = 0; pass < 32; ++pass )
        {
            bool split = false;
            const size_t count = patch.size();
            for( size_t i = 0; i < count && patch.size() + 2 <= max_triangles; ++i )
            {
                const tri3 t = patch[i];
                const vec3 c = ( b_.position( t[0] ) + b_.position( t[1] ) + b_.position( t[2] ) ) * ( 1.f / 3.f );
                const float sc = sizing_ ? ( *sizing_ )( c ) : ( sigma_[t[0]] + sigma_[t[1]] + sigma_[t[2]] ) / 3.f;
                bool ok = sc > 0;
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
                if( edges.count( edge_key( c, d ) ) || created.count( edge_key( c, d ) ) || edge_exists( c, d ) )
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

    // Linear operators of a closing patch. Rows are the new vertices (0 .. nf-1), then the loop vertices of the patch
    // (their rings include the original mesh, so a thin plate continues the surface with matching tangents). Each row
    // holds the edge weights to its neighbors: cotangent weights of the current geometry (the Laplace-Beltrami
    // operator, independent of the graded density) or uniform ones for a first, far-from-final geometry.
    struct patch_system
    {
        uint32_t nf = 0, rows = 0;
        std::vector< uint32_t > row_vertex;
        struct entry
        {
            uint32_t row, col;  // col: vertex id
            double w;
        };
        std::vector< entry > entries;      // merged, sorted by (row, col)
        std::vector< double > row_total;   // sum of a row's weights
        std::vector< double > scale;       // per row: 1 (cotangent) or 1 / degree (uniform)
        std::vector< double > energy;      // per row: 1 / vertex area (cotangent) or 1
    };

    patch_system build_system( const std::vector< tri3 > & patch, uint32_t first_new, bool cotangent ) const
    {
        patch_system sys;
        sys.nf = b_.vertex_count() - first_new;
        std::unordered_map< uint32_t, uint32_t > row_of;
        row_of.reserve( size_t( sys.nf ) * 2 );
        for( uint32_t v = first_new; v < b_.vertex_count(); ++v )
        {
            row_of[v] = uint32_t( sys.row_vertex.size() );
            sys.row_vertex.push_back( v );
        }
        for( const tri3 & t : patch )
            for( uint32_t v : t )
                if( v < first_new && row_of.emplace( v, uint32_t( sys.row_vertex.size() ) ).second )
                    sys.row_vertex.push_back( v );
        sys.rows = uint32_t( sys.row_vertex.size() );
        std::vector< const uint32_t * > tris;
        tris.reserve( patch.size() + size_t( sys.rows - sys.nf ) * 6 );
        for( const tri3 & t : patch )
            tris.push_back( t.data() );
        {
            std::vector< uint32_t > faces;
            for( uint32_t r = sys.nf; r < sys.rows; ++r )
            {
                const uint32_t v = sys.row_vertex[r];
                if( v < n0_ )
                    for( uint32_t k = vf_.begin( v ); k < vf_.end( v ); ++k )
                        faces.push_back( vf_.data[k] );
            }
            std::sort( faces.begin(), faces.end() );
            faces.erase( std::unique( faces.begin(), faces.end() ), faces.end() );
            for( uint32_t f : faces )
                tris.push_back( &b_.indices[size_t( f ) * 3] );
        }
        std::vector< patch_system::entry > raw;
        raw.reserve( tris.size() * 6 );
        std::vector< double > area( sys.rows, 0.0 );
        for( const uint32_t * t : tris )
        {
            const vec3 p[3] = { b_.position( t[0] ), b_.position( t[1] ), b_.position( t[2] ) };
            const double twice_area = double( length( cross( p[1] - p[0], p[2] - p[0] ) ) );
            for( int k = 0; k < 3; ++k )
            {
                // Edge (i, j) with the opposite corner o.
                const uint32_t i = t[k], j = t[( k + 1 ) % 3], o = t[( k + 2 ) % 3];
                double w = 1.0;
                if( cotangent )
                {
                    const vec3 u = p[k] - p[( k + 2 ) % 3], v = p[( k + 1 ) % 3] - p[( k + 2 ) % 3];
                    w = twice_area > 0 ? 0.5 * double( dot( u, v ) ) / twice_area : 0.0;
                }
                auto ri = row_of.find( i ), rj = row_of.find( j ), ro = row_of.find( o );
                if( ri != row_of.end() )
                    raw.push_back( { ri->second, j, w } );
                if( rj != row_of.end() )
                    raw.push_back( { rj->second, i, w } );
                if( ro != row_of.end() )
                    area[ro->second] += twice_area / 6.0;
            }
        }
        std::sort( raw.begin(), raw.end(), []( const patch_system::entry & a, const patch_system::entry & b ) {
            return a.row != b.row ? a.row < b.row : a.col < b.col;
        } );
        double mean_area = 0;
        for( double a : area )
            mean_area += a;
        mean_area = sys.rows ? mean_area / sys.rows : 0;
        sys.row_total.assign( sys.rows, 0.0 );
        sys.scale.assign( sys.rows, 1.0 );
        sys.energy.assign( sys.rows, 1.0 );
        for( size_t s = 0; s < raw.size(); )
        {
            const uint32_t r = raw[s].row, col = raw[s].col;
            double w = 0;
            while( s < raw.size() && raw[s].row == r && raw[s].col == col )
                w += raw[s++].w;
            // Cotangent weights of poor triangles can be negative or huge; clamped they stay a positive Laplacian.
            w = cotangent ? std::clamp( w, 0.01, 100.0 ) : 1.0;
            sys.entries.push_back( { r, col, w } );
            sys.row_total[r] += w;
        }
        for( uint32_t r = 0; r < sys.rows; ++r )
        {
            sys.scale[r] = cotangent ? 1.0 : 1.0 / std::max( sys.row_total[r], 1e-12 );
            sys.energy[r] = cotangent ? 1.0 / std::max( area[r], 1e-3 * mean_area + 1e-30 ) : 1.0;
        }
        return sys;
    }

    // Thin plate for k scalar fields: minimizes sum_r energy_r |scale_r (L f)_r|^2 over the rows of the new vertices that
    // are not held (and the loop's, where their energy is not 0). fixed(v) gives the values of the existing vertices,
    // held values those of held new vertices.
    template< typename Fixed >
    bool solve_thin_plate( const patch_system & sys, int k, Fixed && fixed, const std::vector< uint8_t > * held,
                           const Eigen::MatrixXd * held_values, Eigen::MatrixXd & out ) const
    {
        using triplet = Eigen::Triplet< double >;
        const uint32_t nf = sys.nf, first_new = b_.vertex_count() - nf;
        std::vector< int > col_of( nf, -1 );
        int unknowns = 0;
        for( uint32_t u = 0; u < nf; ++u )
            if( ! held || ! ( *held )[u] )
                col_of[u] = unknowns++;
        out.resize( nf, k );
        for( uint32_t u = 0; u < nf; ++u )
            if( col_of[u] < 0 )
                out.row( u ) = held_values->row( u );
        if( unknowns == 0 )
            return true;
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero( sys.rows, k );
        std::vector< triplet > t;
        t.reserve( sys.entries.size() + sys.rows );
        auto add = [&]( uint32_t r, uint32_t v, double c ) {
            if( v >= first_new && col_of[v - first_new] >= 0 )
                t.emplace_back( r, col_of[v - first_new], c );
            else if( v >= first_new )
                B.row( r ) += c * held_values->row( v - first_new );
            else
                B.row( r ) += c * fixed( v );
        };
        for( const auto & e : sys.entries )
            add( e.row, e.col, e.w * sys.scale[e.row] );
        for( uint32_t r = 0; r < sys.rows; ++r )
            add( r, sys.row_vertex[r], -sys.row_total[r] * sys.scale[r] );
        Eigen::SparseMatrix< double > C( sys.rows, unknowns );
        C.setFromTriplets( t.begin(), t.end() );
        // Held vertices are boundary values, like the existing ones: their own rows are not part of the energy (an
        // obstacle's shape, the plane of a cut, is not smooth in the solved field and would bend the free part).
        Eigen::VectorXd W( sys.rows );
        for( uint32_t r = 0; r < sys.rows; ++r )
            W( r ) = r < nf && held && ( *held )[r] ? 0.0 : sys.energy[r];
        const Eigen::SparseMatrix< double > CtW = C.transpose() * W.asDiagonal();
        const Eigen::SparseMatrix< double > K = CtW * C;
        Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > > ldlt( K );
        if( ldlt.info() != Eigen::Success )
            return false;
        const Eigen::MatrixXd x = ldlt.solve( -( CtW * B ) );
        if( ldlt.info() != Eigen::Success || ! x.allFinite() )
            return false;
        for( uint32_t u = 0; u < nf; ++u )
            if( col_of[u] >= 0 )
                out.row( u ) = x.row( col_of[u] );
        return true;
    }

    // Membrane (harmonic) interpolation of k scalar fields over the new vertices from the existing ones.
    template< typename Fixed >
    bool solve_membrane( const patch_system & sys, int k, Fixed && fixed, Eigen::MatrixXd & out ) const
    {
        using triplet = Eigen::Triplet< double >;
        const uint32_t nf = sys.nf, first_new = b_.vertex_count() - nf;
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero( nf, k );
        std::vector< triplet > t;
        for( const auto & e : sys.entries )
        {
            if( e.row >= nf )
                break;  // entries are sorted by row; the new vertices' rows come first
            if( e.col >= first_new )
                t.emplace_back( e.row, e.col - first_new, -e.w );
            else
                B.row( e.row ) += e.w * fixed( e.col );
        }
        for( uint32_t r = 0; r < nf; ++r )
            t.emplace_back( r, r, sys.row_total[r] );
        Eigen::SparseMatrix< double > A( nf, nf );
        A.setFromTriplets( t.begin(), t.end() );
        Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > > ldlt( A );
        if( ldlt.info() != Eigen::Success )
            return false;
        out = ldlt.solve( B );
        return ldlt.info() == Eigen::Success && out.allFinite();
    }

    // Thin-plate fairing in Cartesian coordinates (no obstacle).
    bool fair_cartesian( const std::vector< tri3 > & patch, uint32_t first_new, bool cotangent )
    {
        const patch_system sys = build_system( patch, first_new, cotangent );
        if( sys.nf == 0 )
            return true;
        Eigen::MatrixXd X;
        const auto position = [&]( uint32_t v ) {
            const vec3 p = b_.position( v );
            return Eigen::RowVector3d( p.x, p.y, p.z );
        };
        if( ! solve_thin_plate( sys, 3, position, nullptr, nullptr, X ) )
            return false;
        for( uint32_t u = 0; u < sys.nf; ++u )
            b_.set_position( first_new + u, vec3( float( X( u, 0 ) ), float( X( u, 1 ) ), float( X( u, 2 ) ) ) );
        return true;
    }

    // Thin-plate fairing in spherical coordinates around the head center c: the new vertices keep their directions
    // (dir_, from planar_patch), their distance from c is a thin plate of the existing vertices' distances. A head is
    // star-shaped around its center and its distance function smooth, so the closing continues the head's roundness
    // (a Cartesian thin plate flattens a large opening: on a 100 mm sphere opened above y = 20 mm it stayed 10.9 mm
    // off the sphere on average, 26 mm at most; in spherical coordinates the sphere comes back exactly). With a cut
    // plane, distances are capped where the ray from c leaves through the plane (active set, a few rounds): that part
    // of the closing becomes a flat cap, the central projection of a fold-free direction field.
    bool fair_radial( const std::vector< tri3 > & patch, uint32_t first_new, const vec3 & c, bool cotangent )
    {
        patch_system sys = build_system( patch, first_new, cotangent );
        const uint32_t nf = sys.nf;
        if( nf == 0 )
            return true;
        // Vertices created since the directions were set (edge midpoints, centroids) lie on the spherical segment or
        // triangle of their parents seen from c: their own direction keeps the map fold-free.
        for( uint32_t u = uint32_t( dir_.size() ); u < nf; ++u )
            dir_.push_back( normalized( b_.position( first_new + u ) - c ) );
        const auto distance = [&]( uint32_t v ) { return Eigen::RowVectorXd::Constant( 1, double( length( b_.position( v ) - c ) ) ); };
        Eigen::MatrixXd U( nf, 3 ), R, M;
        for( uint32_t u = 0; u < nf; ++u )
            U.row( u ) << dir_[u].x, dir_[u].y, dir_[u].z;
        // The loop vertices contribute their distance only, not its slope: along a ragged border (an ear's rim, the
        // teeth of a fusion's fringe) the slope is steep and not the head's outline, and matching it overshot into
        // spikes tens of millimeters long. The closing is continuous with the measured surface, and smooth inside.
        double mean_distance = 0;
        for( uint32_t r = sys.nf; r < sys.rows; ++r )
        {
            mean_distance += length( b_.position( sys.row_vertex[r] ) - c );
            sys.energy[r] = 0;
        }
        mean_distance /= std::max< uint32_t >( 1, sys.rows - sys.nf );
        if( ! solve_thin_plate( sys, 1, distance, nullptr, nullptr, R ) || ! solve_membrane( sys, 1, distance, M ) )
            return false;
        // The thin plate stays within a band around the membrane (which keeps to the range of the loop's distances):
        // round, but not overshooting where the border is ragged.
        Eigen::MatrixXd lo( nf, 1 ), hi( nf, 1 );
        const double band = 0.2 * mean_distance;
        for( uint32_t u = 0; u < nf; ++u )
        {
            lo( u, 0 ) = M( u, 0 ) - band;
            hi( u, 0 ) = M( u, 0 ) + band;
        }
        if( plane_.valid )
        {
            const Eigen::Vector3d n( plane_.normal.x, plane_.normal.y, plane_.normal.z );
            const double height = double( dot( plane_.normal, c ) ) - plane_.offset;  // c above the plane
            for( uint32_t u = 0; u < nf; ++u )
            {
                const double down = -U.row( u ).dot( n );
                if( down > 1e-6 && height > 0 )
                    hi( u, 0 ) = std::min( hi( u, 0 ), ( height - 1e-6 ) / down );  // 1 um above the plane
            }
        }
        // Active set: vertices outside their bounds are held on them, until none of the others leaves its bounds.
        std::vector< uint8_t > held( nf, 0 );
        Eigen::MatrixXd target( nf, 1 );
        for( int round = 0; round < 24; ++round )
        {
            bool added = false;
            for( uint32_t u = 0; u < nf; ++u )
                if( ! held[u] && ( R( u, 0 ) > hi( u, 0 ) || R( u, 0 ) < lo( u, 0 ) ) )
                {
                    held[u] = added = true;
                    target( u, 0 ) = R( u, 0 ) > hi( u, 0 ) ? hi( u, 0 ) : lo( u, 0 );
                }
            if( ! added )
                break;
            if( ! solve_thin_plate( sys, 1, distance, &held, &target, R ) )
                return false;
        }
        for( uint32_t u = 0; u < nf; ++u )
            R( u, 0 ) = std::clamp( R( u, 0 ), std::min( lo( u, 0 ), hi( u, 0 ) ), hi( u, 0 ) );
        for( uint32_t u = 0; u < nf; ++u )
        {
            const Eigen::RowVector3d x = Eigen::RowVector3d( c.x, c.y, c.z ) + R( u, 0 ) * U.row( u );
            b_.set_position( first_new + u, vec3( float( x( 0 ) ), float( x( 1 ) ), float( x( 2 ) ) ) );
        }
        return true;
    }

    // Center for the spherical fairing: a least-squares sphere through the measured surface along the loop (a band
    // about a tenth of the loop's extent wide, grown from the loop except the cut, which belongs to the neck; the loop
    // alone can be nearly planar, and a circle fits any sphere on its axis). Accepted when the surface at the loop
    // faces away from the center and the radius is in proportion to the loop.
    bool find_center( const boundary_loop & loop, vec3 & center ) const
    {
        std::vector< uint32_t > points;
        std::vector< uint8_t > visited( n0_, 0 );
        for( uint32_t v : loop.v )
            if( v < n0_ && ! ( plane_.valid && std::fabs( dot( plane_.normal, b_.position( v ) ) - plane_.offset ) < 1e-5f ) )
            {
                visited[v] = 1;
                points.push_back( v );
            }
        double loop_extent = 0, edge = 0;
        for( size_t i = 0; i < loop.v.size(); ++i )
        {
            loop_extent = std::max( loop_extent, double( length( b_.position( loop.v[i] ) - b_.position( loop.v[0] ) ) ) );
            edge += length( b_.position( loop.v[i] ) - b_.position( loop.v[( i + 1 ) % loop.v.size()] ) );
        }
        edge /= std::max< size_t >( 1, loop.v.size() );
        const int rings = int( std::clamp( 0.1 * loop_extent / std::max( edge, 1e-12 ), 2.0, 60.0 ) );
        for( size_t begin = 0, ring = 0; ring < size_t( rings ); ++ring )
        {
            const size_t end = points.size();
            for( size_t i = begin; i < end; ++i )
                for( uint32_t k = nb_.begin( points[i] ); k < nb_.end( points[i] ); ++k )
                    if( ! visited[nb_.data[k]] )
                    {
                        visited[nb_.data[k]] = 1;
                        points.push_back( nb_.data[k] );
                    }
            begin = end;
        }
        if( points.size() < 16 )
            return false;
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        for( uint32_t v : points )
            origin += Eigen::Vector3d( b_.position( v ).x, b_.position( v ).y, b_.position( v ).z );
        origin /= double( points.size() );
        // |p|^2 = 2 p.c + k (Kasa fit), relative to the centroid.
        Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
        Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
        for( uint32_t v : points )
        {
            const Eigen::Vector3d p = Eigen::Vector3d( b_.position( v ).x, b_.position( v ).y, b_.position( v ).z ) - origin;
            const Eigen::Vector4d row( 2 * p.x(), 2 * p.y(), 2 * p.z(), 1 );
            A += row * row.transpose();
            rhs += row * p.squaredNorm();
        }
        const Eigen::Vector4d s = A.ldlt().solve( rhs );
        if( ! s.allFinite() )
            return false;
        const Eigen::Vector3d c = s.head< 3 >();
        const double radius = std::sqrt( std::max( 0.0, s( 3 ) + c.squaredNorm() ) );
        double extent = 0;
        for( uint32_t v : loop.v )
            extent = std::max( extent, ( Eigen::Vector3d( b_.position( v ).x, b_.position( v ).y, b_.position( v ).z ) - origin - c ).norm() );
        if( ! ( radius > 0 ) || radius > 2 * extent + 1e-9 )
            return false;
        center = vec3( float( origin.x() + c.x() ), float( origin.y() + c.y() ), float( origin.z() + c.z() ) );
        // The surface at the loop faces away from the center.
        uint32_t outward = 0, total = 0;
        for( uint32_t v : loop.v )
        {
            if( v >= n0_ )
                continue;
            vec3 normal;
            for( uint32_t k = vf_.begin( v ); k < vf_.end( v ); ++k )
                normal += face_normal( &b_.indices[size_t( vf_.data[k] ) * 3] );
            ++total;
            if( dot( normal, b_.position( v ) - center ) > 0 )
                ++outward;
        }
        return total > 0 && outward >= 0.8 * total;
    }

    mesh_buffers & b_;
    const uint32_t n0_, t0_;
    const csr out_, vf_, nb_;
    std::unordered_map< uint32_t, float > sigma_;
    const sizing_field * sizing_ = nullptr;
    cut_plane plane_;
    std::vector< vec3 > dir_;  // closing: directions of the new vertices around the center
    std::unordered_map< uint32_t, std::vector< uint32_t > > full_ring_;
    // Edges and faces of the patches added so far.
    std::unordered_set< uint64_t > patch_directed_, patch_undirected_;
    std::unordered_set< tri3, tri_hash > patch_faces_;
};

// The face containing the directed edge a->c (~0u if none).
uint32_t face_with_edge( const mesh_buffers & b, const csr & vf, uint32_t a, uint32_t c )
{
    for( uint32_t k = vf.begin( a ); k < vf.end( a ); ++k )
    {
        const uint32_t f = vf.data[k];
        const uint32_t * t = &b.indices[size_t( f ) * 3];
        for( int e = 0; e < 3; ++e )
            if( t[e] == a && t[( e + 1 ) % 3] == c )
                return f;
    }
    return ~0u;
}

// Per triangle: folded against the surrounding surface (the summed area-weighted normals of its corners), the
// criterion of count_folds and fold_guard.
std::vector< uint8_t > fold_flags( const mesh_buffers & b )
{
    const uint32_t n = b.vertex_count(), t = b.triangle_count();
    std::vector< vec3 > fn( t ), vn( n );
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = &b.indices[size_t( f ) * 3];
        const vec3 p0 = b.position( tri[0] );
        fn[f] = cross( b.position( tri[1] ) - p0, b.position( tri[2] ) - p0 );
        for( int k = 0; k < 3; ++k )
            vn[tri[k]] += fn[f];
    }
    std::vector< uint8_t > folded( t );
    for( uint32_t f = 0; f < t; ++f )
    {
        const uint32_t * tri = &b.indices[size_t( f ) * 3];
        folded[f] = folded_against( b.position( tri[0] ), b.position( tri[1] ), b.position( tri[2] ), vn[tri[0]] + vn[tri[1]] + vn[tri[2]] );
    }
    return folded;
}

// Fringe of the loops that are closed (the longest, and every loop at least min_length long): teeth (triangles with
// two border edges) and triangles folded over against a neighbor (more than 75 degrees across an inner edge). The
// low-weight fringe of a fusion is ragged like that, and a closing surface attached to it would fold against it.
// Pieces that the removal cuts off the surface (below 1% of the largest piece) go too. Returns the triangles removed.
// Vertices are not renumbered (unreferenced ones stay until the end).
uint32_t trim_fringe( mesh_buffers & b, double min_length )
{
    uint32_t removed_total = 0;
    const float cos_limit = std::cos( 75.f * 3.14159265f / 180.f );
    for( int pass = 0; pass < 4; ++pass )
    {
        const uint32_t n = b.vertex_count(), t = b.triangle_count();
        const csr out = outgoing_edges( n, b.indices.data(), t );
        const csr vf = vertex_faces( n, b.indices.data(), t );
        const std::vector< boundary_loop > loops = find_boundary_loops( b, out );
        if( loops.empty() )
            break;
        double longest = 0;
        for( const boundary_loop & loop : loops )
            longest = std::max( longest, loop.length );
        auto normal = [&]( uint32_t f ) {
            const uint32_t * tri = &b.indices[size_t( f ) * 3];
            const vec3 p0 = b.position( tri[0] );
            return normalized( cross( b.position( tri[1] ) - p0, b.position( tri[2] ) - p0 ), vec3() );
        };
        std::vector< uint8_t > remove( t, 0 );
        uint32_t removed = 0;
        for( const boundary_loop & loop : loops )
        {
            if( loop.length < longest && loop.length < min_length )
                continue;
            const std::vector< uint32_t > & v = loop.v;
            for( size_t i = 0; i < v.size(); ++i )
            {
                const uint32_t f = face_with_edge( b, vf, v[i], v[( i + 1 ) % v.size()] );
                if( f == ~0u || remove[f] )
                    continue;
                const uint32_t * tri = &b.indices[size_t( f ) * 3];
                int border = 0;
                bool folded = false;
                for( int e = 0; e < 3; ++e )
                {
                    const uint32_t g = face_with_edge( b, vf, tri[( e + 1 ) % 3], tri[e] );
                    if( g == ~0u )
                        ++border;
                    else if( dot( normal( f ), normal( g ) ) < cos_limit )
                        folded = true;
                }
                if( border >= 2 || folded )
                {
                    remove[f] = 1;
                    ++removed;
                }
            }
        }
        if( removed == 0 )
            break;
        std::vector< uint32_t > kept;
        kept.reserve( b.indices.size() );
        for( uint32_t f = 0; f < t; ++f )
            if( ! remove[f] )
                kept.insert( kept.end(), &b.indices[size_t( f ) * 3], &b.indices[size_t( f ) * 3] + 3 );
        b.indices.swap( kept );
        removed_total += removed;
    }
    if( removed_total == 0 )
        return 0;
    // Pieces cut off (connected through edges).
    const uint32_t n = b.vertex_count(), t = b.triangle_count();
    const csr vf = vertex_faces( n, b.indices.data(), t );
    std::vector< uint32_t > piece( t, ~0u ), size;
    std::vector< uint32_t > stack;
    for( uint32_t f0 = 0; f0 < t; ++f0 )
    {
        if( piece[f0] != ~0u )
            continue;
        const uint32_t id = uint32_t( size.size() );
        size.push_back( 0 );
        stack.assign( 1, f0 );
        piece[f0] = id;
        while( ! stack.empty() )
        {
            const uint32_t f = stack.back();
            stack.pop_back();
            ++size[id];
            const uint32_t * tri = &b.indices[size_t( f ) * 3];
            for( int e = 0; e < 3; ++e )
            {
                const uint32_t g = face_with_edge( b, vf, tri[( e + 1 ) % 3], tri[e] );
                if( g != ~0u && piece[g] == ~0u )
                {
                    piece[g] = id;
                    stack.push_back( g );
                }
            }
        }
    }
    const uint32_t largest = size.empty() ? 0 : *std::max_element( size.begin(), size.end() );
    std::vector< uint32_t > kept;
    kept.reserve( b.indices.size() );
    for( uint32_t f = 0; f < t; ++f )
    {
        if( size[piece[f]] * 100ull >= largest )
            kept.insert( kept.end(), &b.indices[size_t( f ) * 3], &b.indices[size_t( f ) * 3] + 3 );
        else
            ++removed_total;
    }
    b.indices.swap( kept );
    return removed_total;
}

}  // namespace

uint32_t fr::mesh::fill_small_holes( mesh_buffers & b, uint32_t max_edges, const std::vector< uint8_t > & marked )
{
    if( b.triangle_count() == 0 || b.has_uvs )
        return 0;
    hole_filler filler( b );
    uint32_t filled = 0;
    for( const boundary_loop & loop : filler.loops() )
    {
        if( loop.v.size() > max_edges || std::none_of( loop.v.begin(), loop.v.end(), [&]( uint32_t v ) { return v < marked.size() && marked[v]; } ) )
            continue;
        patch_stats stats;
        if( filler.fill( loop, stats, true ) )
            ++filled;
    }
    return filled;
}

extern "C" {

void fr_fill_params_default( fr_fill_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->max_hole_perimeter_m = 0.25f;
    p->max_hole_edges = 4096;
    p->close_largest = 0;
    p->closing_edge_m = 0;
}

fr_status fr_mesh_fill_holes_ex( fr_mesh * mesh, const fr_fill_params * params, fr_fill_report * report )
{
    if( report )
        std::memset( report, 0, sizeof( *report ) );
    return fr::guarded( "fr_mesh_fill_holes_ex", [&]() -> fr_status {
        fr_status s = validate( mesh, "fr_mesh_fill_holes_ex" );
        if( s != FR_OK )
            return s;
        if( ! params || ! std::isfinite( params->max_hole_perimeter_m ) || params->max_hole_perimeter_m < 0
            || ! std::isfinite( params->closing_edge_m ) || params->closing_edge_m < 0 )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_mesh_fill_holes_ex: invalid parameters" );
        fr_fill_report r{};
        r.first_new_vertex = mesh->vertex_count;
        auto finish = [&]( fr_status status ) {
            if( report && status == FR_OK )
                *report = r;
            return status;
        };
        if( mesh->triangle_count == 0 )
            return finish( FR_OK );
        const uint32_t max_edges = std::max< uint32_t >( params->max_hole_edges, 3 );
        mesh_buffers b = load( *mesh );
        // With UVs, chart borders are boundaries of the split mesh but not holes: loops are found on the topology
        // with coincident vertices merged, and the original (seam-split) triangles are restored afterwards.
        std::vector< uint32_t > original;
        if( b.has_uvs )
        {
            original = b.indices;
            b.indices = remap_indices( original.data(), original.size(), coincident_representatives( b.positions.data(), b.vertex_count() ) );
        }
        // Loops beyond the perimeter limit are only filled when closing (a watertight result), the way the longest one
        // is closed: openings like the back of the head.
        const double long_loop = params->max_hole_perimeter_m > 0 ? params->max_hole_perimeter_m : DBL_MAX;
        const bool closing = params->close_largest && original.empty();
        if( closing )
            r.trimmed_triangles = trim_fringe( b, long_loop );
        const double mean_edge = mean_edge_length( b.positions.data(), b.indices.data(), b.triangle_count() );
        float h = params->closing_edge_m;
        if( ! ( h > 0 ) )
            h = float( std::clamp( 4.0 * mean_edge, 0.001, 0.005 ) );
        // Holes: every loop but the longest, within the limits.
        {
            hole_filler filler( b );
            const std::vector< boundary_loop > loops = filler.loops();
            if( loops.empty() )
                return finish( FR_OK );
            size_t largest = 0;
            for( size_t i = 1; i < loops.size(); ++i )
                if( loops[i].length > loops[largest].length )
                    largest = i;
            for( size_t i = 0; i < loops.size(); ++i )
            {
                const boundary_loop & loop = loops[i];
                if( i == largest || loop.length > long_loop )
                {
                    if( ! params->close_largest )
                        ++r.open_loops;
                    continue;
                }
                patch_stats stats;
                if( loop.v.size() <= max_edges && filler.fill( loop, stats, params->close_largest != 0 ) )
                {
                    ++r.filled_holes;
                    r.filled_area_m2 += float( stats.area );
                    r.folded_triangles += stats.folded;
                }
                else
                {
                    r.rejected_holes += loop.v.size() <= max_edges;
                    ++r.open_loops;
                }
            }
        }
        // The longest loop and the other long ones are closed. Measured triangles at their borders that the closings
        // fold (a tooth the trimming kept), and those across the loop edges of folded closing triangles, are removed
        // and the loops closed again (at most five attempts). The holes filled above stay.
        for( int attempt = 0; params->close_largest && attempt < 5; ++attempt )
        {
            const uint32_t before_vertices = b.vertex_count(), before_triangles = b.triangle_count();
            const std::vector< uint8_t > folded_before = closing ? fold_flags( b ) : std::vector< uint8_t >();
            hole_filler filler( b );
            const std::vector< boundary_loop > loops = filler.loops();
            if( loops.empty() )
                break;
            double longest = 0;
            for( const boundary_loop & loop : loops )
                longest = std::max( longest, loop.length );
            uint32_t closed = 0, open = 0, folded_now = 0;
            double area = 0;
            for( const boundary_loop & loop : loops )
            {
                patch_stats stats;
                // The longest and the long loops are closed; loops a retry's trimming cut off are filled like holes.
                const bool ok = loop.length >= longest || loop.length > long_loop
                                    ? filler.close( loop, h, stats ) || ( loop.length < longest && filler.fill( loop, stats, true ) )
                                    : loop.v.size() <= max_edges && filler.fill( loop, stats, true );
                if( ok )
                {
                    closed += loop.length >= longest;
                    area += stats.area;
                    folded_now += stats.folded;
                }
                else
                    ++open;
            }
            const bool last = ! closing || attempt == 4 || closed == 0;
            std::vector< uint8_t > remove( before_triangles, 0 );
            size_t removed = 0;
            if( ! last )
            {
                const std::vector< uint8_t > folded = fold_flags( b );
                std::unordered_map< uint64_t, uint32_t > measured_edge;
                for( uint32_t f = 0; f < before_triangles; ++f )
                    for( int e = 0; e < 3; ++e )
                        measured_edge[uint64_t( b.indices[size_t( f ) * 3 + e] ) << 32 | b.indices[size_t( f ) * 3 + ( e + 1 ) % 3]] = f;
                for( uint32_t f = 0; f < before_triangles; ++f )
                    if( folded[f] && ! folded_before[f] && ! remove[f] )
                    {
                        remove[f] = 1;
                        ++removed;
                    }
                for( uint32_t f = before_triangles; f < b.triangle_count(); ++f )
                    if( folded[f] )
                        for( int e = 0; e < 3; ++e )
                        {
                            auto it = measured_edge.find( uint64_t( b.indices[size_t( f ) * 3 + ( e + 1 ) % 3] ) << 32 | b.indices[size_t( f ) * 3 + e] );
                            if( it != measured_edge.end() && ! remove[it->second] )
                            {
                                remove[it->second] = 1;
                                ++removed;
                            }
                        }
            }
            if( removed == 0 )
            {
                r.closed_largest = closed > 0;
                r.closing_area_m2 = float( area );
                r.filled_area_m2 += float( area );
                r.folded_triangles += folded_now;
                r.open_loops += open;
                break;
            }
            // Undo the closings, drop those triangles (and the fringe they expose), and close again.
            b.indices.resize( size_t( before_triangles ) * 3 );
            b.positions.resize( size_t( before_vertices ) * 3 );
            if( b.has_normals )
                b.normals.resize( size_t( before_vertices ) * 3 );
            if( b.has_colors )
                b.colors.resize( size_t( before_vertices ) * 3 );
            std::vector< uint32_t > kept;
            kept.reserve( b.indices.size() );
            for( uint32_t f = 0; f < before_triangles; ++f )
                if( ! remove[f] )
                    kept.insert( kept.end(), &b.indices[size_t( f ) * 3], &b.indices[size_t( f ) * 3] + 3 );
            b.indices.swap( kept );
            r.trimmed_triangles += uint32_t( removed ) + trim_fringe( b, long_loop );
        }
        if( r.filled_holes == 0 && ! r.closed_largest && r.trimmed_triangles == 0 )
            return finish( FR_OK );
        if( ! original.empty() )
            std::copy( original.begin(), original.end(), b.indices.begin() );
        // Vertices the trimming left unreferenced go; the others keep their order, the new ones stay at the end.
        if( r.trimmed_triangles )
        {
            std::vector< uint8_t > used( b.vertex_count(), 0 );
            for( uint32_t i : b.indices )
                used[i] = 1;
            uint32_t kept_original = 0;
            for( uint32_t v = 0; v < mesh->vertex_count; ++v )
                kept_original += used[v];
            b.remove_unreferenced();
            r.first_new_vertex = kept_original;
        }
        if( b.has_normals )
            compute_vertex_normals( b.positions.data(), b.vertex_count(), b.indices.data(), b.triangle_count(),
                                    b.normals.data(), b.has_uvs );
        s = store( b, mesh );
        return finish( s );
    } );
}

fr_status fr_mesh_fill_holes( fr_mesh * mesh, uint32_t max_hole_edges, uint32_t * filled_holes )
{
    if( filled_holes )
        *filled_holes = 0;
    if( max_hole_edges < 3 )
        return validate( mesh, "fr_mesh_fill_holes" );
    fr_fill_params params;
    fr_fill_params_default( &params );
    params.max_hole_perimeter_m = 0;
    params.max_hole_edges = max_hole_edges;
    fr_fill_report report;
    const fr_status s = fr_mesh_fill_holes_ex( mesh, &params, &report );
    if( s == FR_OK && filled_holes )
        *filled_holes = report.filled_holes;
    return s;
}

}  // extern "C"
