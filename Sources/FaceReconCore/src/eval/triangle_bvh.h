// License: Apache 2.0. See LICENSE file in root directory.
//
// Bounding volume hierarchy over the triangles of a mesh for exact closest-point queries (accuracy evaluation).
// Distances are computed in double precision: evaluation compares surfaces that differ by fractions of a
// millimeter at ~0.6 m from the origin, where float rounding would already be visible in the statistics.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace fr
{
namespace eval
{

struct vec3
{
    double x = 0, y = 0, z = 0;
};

inline vec3 operator+( const vec3 & a, const vec3 & b ) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline vec3 operator-( const vec3 & a, const vec3 & b ) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline vec3 operator*( const vec3 & a, double s ) { return { a.x * s, a.y * s, a.z * s }; }
inline double dot( const vec3 & a, const vec3 & b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross( const vec3 & a, const vec3 & b )
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline double length2( const vec3 & a ) { return dot( a, a ); }

inline vec3 closest_point_on_segment( const vec3 & p, const vec3 & a, const vec3 & b )
{
    const vec3 ab = b - a;
    const double len2 = length2( ab );
    if( len2 <= 0 )
        return a;
    const double t = std::min( 1.0, std::max( 0.0, dot( p - a, ab ) / len2 ) );
    return a + ab * t;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection, 5.1.5). Degenerate triangles (slivers
// from marching cubes, collapsed edges) have no well-defined Voronoi regions, so they fall back to the closest point
// on their edges, which is exact for a zero-area triangle.
inline vec3 closest_point_on_triangle( const vec3 & p, const vec3 & a, const vec3 & b, const vec3 & c )
{
    const vec3 ab = b - a, ac = c - a;
    // |ab x ac|^2 = |ab|^2 |ac|^2 sin^2(angle): below sin = 1e-7 the barycentric solve loses all precision, while
    // the edges are then within 1e-7 * edge length of the triangle anyway.
    const double area2 = length2( cross( ab, ac ) );
    if( ! ( area2 > 1e-14 * length2( ab ) * length2( ac ) ) )
    {
        const vec3 q0 = closest_point_on_segment( p, a, b );
        const vec3 q1 = closest_point_on_segment( p, b, c );
        const vec3 q2 = closest_point_on_segment( p, c, a );
        const double d0 = length2( p - q0 ), d1 = length2( p - q1 ), d2 = length2( p - q2 );
        if( d0 <= d1 && d0 <= d2 )
            return q0;
        return d1 <= d2 ? q1 : q2;
    }
    const vec3 ap = p - a;
    const double d1 = dot( ab, ap ), d2 = dot( ac, ap );
    if( d1 <= 0 && d2 <= 0 )
        return a;
    const vec3 bp = p - b;
    const double d3 = dot( ab, bp ), d4 = dot( ac, bp );
    if( d3 >= 0 && d4 <= d3 )
        return b;
    const double vc = d1 * d4 - d3 * d2;
    if( vc <= 0 && d1 >= 0 && d3 <= 0 )
        return a + ab * ( d1 / ( d1 - d3 ) );
    const vec3 cp = p - c;
    const double d5 = dot( ab, cp ), d6 = dot( ac, cp );
    if( d6 >= 0 && d5 <= d6 )
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if( vb <= 0 && d2 >= 0 && d6 <= 0 )
        return a + ac * ( d2 / ( d2 - d6 ) );
    const double va = d3 * d6 - d5 * d4;
    if( va <= 0 && ( d4 - d3 ) >= 0 && ( d5 - d6 ) >= 0 )
        return b + ( c - b ) * ( ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) ) );
    const double denom = 1.0 / ( va + vb + vc );
    return a + ab * ( vb * denom ) + ac * ( vc * denom );
}

class triangle_bvh
{
public:
    // positions: 3 floats per vertex; indices: 3 per triangle, all < vertex_count (validated by the caller).
    triangle_bvh( const float * positions, const uint32_t * indices, uint32_t triangle_count )
    {
        const size_t n = triangle_count;
        std::vector< uint32_t > order( n );
        std::vector< vec3 > centroids( n );
        _vertices.resize( n * 3 );
        std::vector< vec3 > corners( n * 3 );
        for( size_t t = 0; t < n; ++t )
        {
            order[t] = uint32_t( t );
            for( int k = 0; k < 3; ++k )
            {
                const float * v = positions + size_t( indices[3 * t + k] ) * 3;
                corners[3 * t + k] = { v[0], v[1], v[2] };
            }
            centroids[t] = ( corners[3 * t] + corners[3 * t + 1] + corners[3 * t + 2] ) * ( 1.0 / 3.0 );
        }
        if( n == 0 )
            return;
        _nodes.reserve( 2 * n / leaf_size + 1 );

        struct build_item { uint32_t node, begin, end; };
        std::vector< build_item > stack;
        _nodes.push_back( {} );
        stack.push_back( { 0, 0, uint32_t( n ) } );
        while( ! stack.empty() )
        {
            const build_item item = stack.back();
            stack.pop_back();
            vec3 cmin{ inf(), inf(), inf() }, cmax{ -inf(), -inf(), -inf() };
            for( uint32_t i = item.begin; i < item.end; ++i )
            {
                const vec3 & c = centroids[order[i]];
                cmin = { std::min( cmin.x, c.x ), std::min( cmin.y, c.y ), std::min( cmin.z, c.z ) };
                cmax = { std::max( cmax.x, c.x ), std::max( cmax.y, c.y ), std::max( cmax.z, c.z ) };
            }
            const vec3 extent = cmax - cmin;
            const uint32_t count = item.end - item.begin;
            if( count <= leaf_size || ( extent.x <= 0 && extent.y <= 0 && extent.z <= 0 ) )
            {
                node & leaf = _nodes[item.node];
                leaf.first = item.begin;
                leaf.count = count;
                set_bounds( leaf, order, corners, item.begin, item.end );
                continue;
            }
            // Median split along the longest centroid axis keeps the tree balanced (bounded depth, see query()).
            const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : ( extent.y >= extent.z ? 1 : 2 );
            const uint32_t mid = item.begin + count / 2;
            std::nth_element( order.begin() + item.begin, order.begin() + mid, order.begin() + item.end,
                              [&]( uint32_t l, uint32_t r ) {
                                  return component( centroids[l], axis ) < component( centroids[r], axis );
                              } );
            const uint32_t left = uint32_t( _nodes.size() );
            _nodes.push_back( {} );
            const uint32_t right = uint32_t( _nodes.size() );
            _nodes.push_back( {} );
            node & inner = _nodes[item.node];
            inner.first = left;
            inner.second = right;
            inner.count = 0;
            set_bounds( inner, order, corners, item.begin, item.end );
            stack.push_back( { right, mid, item.end } );
            stack.push_back( { left, item.begin, mid } );
        }
        for( size_t i = 0; i < n; ++i )
            for( int k = 0; k < 3; ++k )
                _vertices[3 * i + k] = corners[3 * size_t( order[i] ) + k];
    }

    bool empty() const { return _nodes.empty(); }

    // Exact squared distance from p to the closest triangle; writes the closest surface point when requested.
    double closest( const vec3 & p, vec3 * closest_point = nullptr ) const
    {
        double best = inf();
        vec3 best_point = p;
        if( _nodes.empty() )
            return best;
        // Median splits bound the depth by ~log2(triangles), so 128 entries are never exceeded.
        uint32_t stack[128];
        int top = 0;
        stack[top++] = 0;
        while( top > 0 )
        {
            const node & n = _nodes[stack[--top]];
            if( box_distance2( n, p ) >= best )
                continue;
            if( n.count > 0 )
            {
                for( uint32_t i = n.first; i < n.first + n.count; ++i )
                {
                    const vec3 q = closest_point_on_triangle( p, _vertices[3 * i], _vertices[3 * i + 1],
                                                              _vertices[3 * i + 2] );
                    const double d = length2( p - q );
                    if( d < best )
                    {
                        best = d;
                        best_point = q;
                    }
                }
                continue;
            }
            const double dl = box_distance2( _nodes[n.first], p );
            const double dr = box_distance2( _nodes[n.second], p );
            // Visit the nearer child first so that the far one is usually pruned.
            const uint32_t near_child = dl <= dr ? n.first : n.second;
            const uint32_t far_child = dl <= dr ? n.second : n.first;
            if( std::max( dl, dr ) < best && top < 127 )
                stack[top++] = far_child;
            if( std::min( dl, dr ) < best && top < 127 )
                stack[top++] = near_child;
        }
        if( closest_point )
            *closest_point = best_point;
        return best;
    }

private:
    static constexpr uint32_t leaf_size = 4;

    struct node
    {
        float bmin[3] = { 0, 0, 0 };
        float bmax[3] = { 0, 0, 0 };
        uint32_t first = 0;    // leaf: first triangle (in BVH order); inner: left child
        uint32_t second = 0;   // inner: right child
        uint32_t count = 0;    // leaf: triangle count; 0 = inner node
    };

    static double inf() { return std::numeric_limits< double >::infinity(); }
    static double component( const vec3 & v, int axis ) { return axis == 0 ? v.x : ( axis == 1 ? v.y : v.z ); }

    static void set_bounds( node & n, const std::vector< uint32_t > & order, const std::vector< vec3 > & corners,
                            uint32_t begin, uint32_t end )
    {
        double lo[3] = { inf(), inf(), inf() }, hi[3] = { -inf(), -inf(), -inf() };
        for( uint32_t i = begin; i < end; ++i )
            for( int k = 0; k < 3; ++k )
            {
                const vec3 & v = corners[3 * size_t( order[i] ) + k];
                lo[0] = std::min( lo[0], v.x ); hi[0] = std::max( hi[0], v.x );
                lo[1] = std::min( lo[1], v.y ); hi[1] = std::max( hi[1], v.y );
                lo[2] = std::min( lo[2], v.z ); hi[2] = std::max( hi[2], v.z );
            }
        // Round outward: a box that is a hair too small would prune the true closest triangle.
        for( int k = 0; k < 3; ++k )
        {
            n.bmin[k] = std::nextafter( float( lo[k] ), -std::numeric_limits< float >::infinity() );
            n.bmax[k] = std::nextafter( float( hi[k] ), std::numeric_limits< float >::infinity() );
        }
    }

    static double box_distance2( const node & n, const vec3 & p )
    {
        const double dx = std::max( { double( n.bmin[0] ) - p.x, 0.0, p.x - double( n.bmax[0] ) } );
        const double dy = std::max( { double( n.bmin[1] ) - p.y, 0.0, p.y - double( n.bmax[1] ) } );
        const double dz = std::max( { double( n.bmin[2] ) - p.z, 0.0, p.z - double( n.bmax[2] ) } );
        return dx * dx + dy * dy + dz * dz;
    }

    std::vector< node > _nodes;
    std::vector< vec3 > _vertices;   // 3 per triangle, in BVH leaf order
};

}  // namespace eval
}  // namespace fr
