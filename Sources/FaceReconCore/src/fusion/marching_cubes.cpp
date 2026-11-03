// License: Apache 2.0. See LICENSE file in root directory.
//
// Marching cubes over the dense TSDF, parallel over z slabs, with shared vertices: every voxel owns the three cube
// edges leaving its corner along +x, +y, +z, and a vertex is created once per crossed edge.
//
// The triangle table is generated at startup instead of being transcribed: on each cube face the iso-contour
// segments cut off runs of inside corners ("inside corners are separated" on ambiguous faces). The rule depends
// only on the four corner signs of the face, so neighboring cubes always agree on shared faces and the surface is
// crack-free. Walking each face counter-clockwise as seen from outside the cube, a segment runs from the edge where
// the walk enters the inside region to the edge where it leaves it; chaining the segments gives closed loops that
// are fanned into triangles, counter-clockwise seen from the outside (positive) region.

#include "fusion_internal.h"
#include "../common/fr_internal.h"

#include <array>
#include <cstring>
#include <stdexcept>

namespace fr
{
namespace
{
// Corner offsets (Bourke numbering) and the corner pair of each of the 12 edges.
constexpr int kCorner[8][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
                                { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 } };
constexpr int kEdgeCorners[12][2] = { { 0, 1 }, { 1, 2 }, { 3, 2 }, { 0, 3 }, { 4, 5 }, { 5, 6 },
                                      { 7, 6 }, { 4, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
// Faces as corner cycles, counter-clockwise seen from outside the cube.
constexpr int kFaces[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                               { 3, 7, 6, 2 }, { 0, 4, 7, 3 }, { 1, 2, 6, 5 } };

// Owner of each cube edge: voxel offset of its lower corner and the axis it runs along.
struct EdgeOwner
{
    int dx, dy, dz, axis;
};

struct Tables
{
    std::array< std::vector< int8_t >, 256 > triangles;   // edge indices, 3 per triangle
    EdgeOwner owners[12];

    Tables()
    {
        for( int e = 0; e < 12; ++e )
        {
            const int * a = kCorner[kEdgeCorners[e][0]];
            const int * b = kCorner[kEdgeCorners[e][1]];
            int axis = 0;
            while( a[axis] == b[axis] )
                ++axis;
            owners[e] = { std::min( a[0], b[0] ), std::min( a[1], b[1] ), std::min( a[2], b[2] ), axis };
        }
        auto edge_between = []( int a, int b ) {
            for( int e = 0; e < 12; ++e )
                if( ( kEdgeCorners[e][0] == a && kEdgeCorners[e][1] == b ) || ( kEdgeCorners[e][0] == b && kEdgeCorners[e][1] == a ) )
                    return e;
            throw std::logic_error( "marching cubes: corners are not adjacent" );
        };
        for( int config = 0; config < 256; ++config )
        {
            auto inside = [&]( int corner ) { return ( config >> corner ) & 1; };
            int next[12];
            std::fill( std::begin( next ), std::end( next ), -1 );
            for( const auto & face : kFaces )
            {
                for( int k = 0; k < 4; ++k )
                {
                    // Entry into a run of inside corners at corner k: find where the walk leaves it again.
                    if( ! inside( face[k] ) || inside( face[( k + 3 ) % 4] ) )
                        continue;
                    int last = k;
                    while( inside( face[( last + 1 ) % 4] ) )
                        last = ( last + 1 ) % 4;
                    const int entry = edge_between( face[( k + 3 ) % 4], face[k] );
                    const int exit = edge_between( face[last], face[( last + 1 ) % 4] );
                    if( next[entry] != -1 )
                        throw std::logic_error( "marching cubes: inconsistent contour" );
                    next[entry] = exit;
                }
            }
            std::vector< int8_t > & tris = triangles[size_t( config )];
            bool used[12] = {};
            for( int start = 0; start < 12; ++start )
            {
                if( next[start] == -1 || used[start] )
                    continue;
                std::vector< int > loop;
                for( int e = start; ! used[e]; e = next[e] )
                {
                    if( next[e] == -1 )
                        throw std::logic_error( "marching cubes: open contour" );
                    used[e] = true;
                    loop.push_back( e );
                }
                for( size_t i = 1; i + 1 < loop.size(); ++i )
                    tris.insert( tris.end(), { int8_t( loop[0] ), int8_t( loop[i] ), int8_t( loop[i + 1] ) } );
            }
        }
    }
};

const Tables & tables()
{
    static const Tables t;
    return t;
}

constexpr float kMaxCrossingJump = 1.0f;   // in truncation units; larger jumps are not a surface (see below)

struct Sampler
{
    const VolumeGeometry & g;
    const half_t * tw;
    const uint8_t * colors;
    float min_weight;

    size_t index( int x, int y, int z ) const { return size_t( x ) + size_t( g.nx ) * ( size_t( y ) + size_t( g.ny ) * size_t( z ) ); }
    float value( size_t i ) const { return float( tw[2 * i] ); }
    float weight( size_t i ) const { return float( tw[2 * i + 1] ); }
    bool valid( size_t i ) const { return weight( i ) >= min_weight; }

    // A crossing between two valid voxels. A sign change between a truncated "far in front" value and a value from
    // behind another view's surface (difference near 2) is a fusion artifact at silhouettes, not a surface.
    bool crossing( size_t a, size_t b ) const
    {
        if( ! valid( a ) || ! valid( b ) )
            return false;
        const float va = value( a ), vb = value( b );
        return ( va < 0.f ) != ( vb < 0.f ) && std::abs( va - vb ) <= kMaxCrossingJump;
    }

    // TSDF gradient by central differences (one-sided next to unobserved voxels).
    Eigen::Vector3f gradient( int x, int y, int z ) const
    {
        Eigen::Vector3f grad = Eigen::Vector3f::Zero();
        const int p[3] = { x, y, z };
        const int n[3] = { g.nx, g.ny, g.nz };
        const size_t c = index( x, y, z );
        for( int axis = 0; axis < 3; ++axis )
        {
            int lo[3] = { x, y, z }, hi[3] = { x, y, z };
            lo[axis] = p[axis] - 1;
            hi[axis] = p[axis] + 1;
            const bool has_lo = lo[axis] >= 0 && weight( index( lo[0], lo[1], lo[2] ) ) > 0;
            const bool has_hi = hi[axis] < n[axis] && weight( index( hi[0], hi[1], hi[2] ) ) > 0;
            if( has_lo && has_hi )
                grad[axis] = 0.5f * ( value( index( hi[0], hi[1], hi[2] ) ) - value( index( lo[0], lo[1], lo[2] ) ) );
            else if( has_hi )
                grad[axis] = value( index( hi[0], hi[1], hi[2] ) ) - value( c );
            else if( has_lo )
                grad[axis] = value( c ) - value( index( lo[0], lo[1], lo[2] ) );
        }
        return grad;
    }
};

struct SlabOutput
{
    std::vector< uint32_t > indices;
};
}  // namespace

fr_status marching_cubes( const VolumeGeometry & g, const half_t * tw, const uint8_t * colors, float min_weight, fr_mesh ** out )
{
    const Tables & tab = tables();
    const Sampler s { g, tw, colors, min_weight };
    const int nx = g.nx, ny = g.ny, nz = g.nz;
    const size_t layer = size_t( nx ) * size_t( ny );

    // Pass 1: vertices owned by each z slice.
    std::vector< uint64_t > slice_vertices( size_t( nz ) + 1, 0 );
    parallel_for( size_t( nz ), [&]( size_t zi ) {
        const int z = int( zi );
        uint64_t count = 0;
        for( int y = 0; y < ny; ++y )
            for( int x = 0; x < nx; ++x )
            {
                const size_t i = s.index( x, y, z );
                if( ! s.valid( i ) )
                    continue;
                count += x + 1 < nx && s.crossing( i, i + 1 );
                count += y + 1 < ny && s.crossing( i, i + size_t( nx ) );
                count += z + 1 < nz && s.crossing( i, i + layer );
            }
        slice_vertices[zi + 1] = count;
    } );
    for( size_t z = 1; z < slice_vertices.size(); ++z )
        slice_vertices[z] += slice_vertices[z - 1];
    const uint64_t vertex_count = slice_vertices.back();
    if( vertex_count > UINT32_MAX / 2 )
        return fail( FR_ERR_OUT_OF_MEMORY, "marching cubes: too many vertices" );

    std::vector< float > positions( 3 * vertex_count ), normals( 3 * vertex_count );
    std::vector< uint8_t > vertex_colors( colors ? 3 * vertex_count : 0 );

    // Pass 2: per slab of slices, vertex ids for slices z and z+1 (rebuilt deterministically from the pass-1
    // offsets), vertex attributes for the slab's own slices, and triangles of its cubes.
    const size_t slabs = std::min< size_t >( size_t( nz ), std::max< size_t >( 1, chunk_count( size_t( nz ), 2 ) ) );
    std::vector< SlabOutput > outputs( slabs );
    parallel_chunks( size_t( nz ), slabs, [&]( size_t z0, size_t z1, size_t slab ) {
        // ids[l][3 * (x + nx * y) + axis] for the two slices in flight.
        std::vector< uint32_t > ids[2] = { std::vector< uint32_t >( 3 * layer ), std::vector< uint32_t >( 3 * layer ) };
        auto build_slice = [&]( int z, std::vector< uint32_t > & slice_ids, bool emit ) {
            uint64_t next_id = slice_vertices[size_t( z )];
            for( int y = 0; y < ny; ++y )
                for( int x = 0; x < nx; ++x )
                {
                    const size_t i = s.index( x, y, z );
                    uint32_t * cell = &slice_ids[3 * ( size_t( x ) + size_t( nx ) * size_t( y ) )];
                    cell[0] = cell[1] = cell[2] = UINT32_MAX;
                    if( ! s.valid( i ) )
                        continue;
                    const size_t neighbor[3] = { i + 1, i + size_t( nx ), i + layer };
                    const bool inside_volume[3] = { x + 1 < nx, y + 1 < ny, z + 1 < nz };
                    for( int axis = 0; axis < 3; ++axis )
                    {
                        if( ! inside_volume[axis] || ! s.crossing( i, neighbor[axis] ) )
                            continue;
                        const uint32_t id = uint32_t( next_id++ );
                        cell[axis] = id;
                        if( ! emit )
                            continue;
                        const size_t j = neighbor[axis];
                        const float va = s.value( i ), vb = s.value( j );
                        const float t = va / ( va - vb );
                        float p[3] = { float( x ), float( y ), float( z ) };
                        p[axis] += t;
                        for( int k = 0; k < 3; ++k )
                            positions[3 * id + k] = g.origin[k] + g.voxel * p[k];
                        int q[3] = { x, y, z };
                        ++q[axis];
                        Eigen::Vector3f grad = ( 1.f - t ) * s.gradient( x, y, z ) + t * s.gradient( q[0], q[1], q[2] );
                        const float len = grad.norm();
                        if( len > 1e-12f )
                            grad /= len;
                        else
                        {
                            grad = Eigen::Vector3f::Zero();
                            grad[axis] = vb > va ? 1.f : -1.f;   // along the edge, toward the positive side
                        }
                        for( int k = 0; k < 3; ++k )
                            normals[3 * id + k] = grad[k];
                        if( colors )
                            for( int k = 0; k < 3; ++k )
                                vertex_colors[3 * id + k] = uint8_t( std::lround( ( 1.f - t ) * colors[4 * i + k] + t * colors[4 * j + k] ) );
                    }
                }
        };

        SlabOutput & result = outputs[slab];
        build_slice( int( z0 ), ids[0], true );
        for( size_t zi = z0; zi < z1; ++zi )
        {
            const int z = int( zi );
            std::vector< uint32_t > & lower = ids[( zi - z0 ) & 1];
            std::vector< uint32_t > & upper = ids[( zi - z0 + 1 ) & 1];
            if( z + 1 >= nz )
                break;
            // The next slice's vertices belong to this slab unless it starts the next slab.
            build_slice( z + 1, upper, zi + 1 < z1 );
            for( int y = 0; y + 1 < ny; ++y )
                for( int x = 0; x + 1 < nx; ++x )
                {
                    int config = 0;
                    bool all_valid = true;
                    for( int c = 0; c < 8 && all_valid; ++c )
                    {
                        const size_t i = s.index( x + kCorner[c][0], y + kCorner[c][1], z + kCorner[c][2] );
                        all_valid = s.valid( i );
                        if( s.value( i ) < 0.f )
                            config |= 1 << c;
                    }
                    if( ! all_valid || config == 0 || config == 255 )
                        continue;
                    const std::vector< int8_t > & tris = tab.triangles[size_t( config )];
                    uint32_t edge_ids[12];
                    bool complete = true;
                    for( int e = 0; e < 12; ++e )
                    {
                        const EdgeOwner & o = tab.owners[e];
                        const std::vector< uint32_t > & slice = o.dz ? upper : lower;
                        edge_ids[e] = slice[3 * ( size_t( x + o.dx ) + size_t( nx ) * size_t( y + o.dy ) ) + size_t( o.axis )];
                    }
                    for( int8_t e : tris )
                        complete &= edge_ids[e] != UINT32_MAX;
                    if( ! complete )
                        continue;   // an edge was rejected as a discontinuity: leave a hole rather than a flap
                    for( int8_t e : tris )
                        result.indices.push_back( edge_ids[e] );
                }
        }
    } );

    size_t index_count = 0;
    for( const auto & o : outputs )
        index_count += o.indices.size();
    if( index_count / 3 > UINT32_MAX )
        return fail( FR_ERR_OUT_OF_MEMORY, "marching cubes: too many triangles" );
    fr_mesh * mesh = nullptr;
    const uint32_t attributes = FR_MESH_NORMALS | ( colors ? FR_MESH_COLORS : 0u );
    fr_status status = fr_mesh_create( uint32_t( vertex_count ), uint32_t( index_count / 3 ), attributes, &mesh );
    if( status != FR_OK )
        return status;
    if( vertex_count )
    {
        std::memcpy( mesh->positions, positions.data(), positions.size() * sizeof( float ) );
        std::memcpy( mesh->normals, normals.data(), normals.size() * sizeof( float ) );
        if( colors )
            std::memcpy( mesh->colors, vertex_colors.data(), vertex_colors.size() );
    }
    size_t offset = 0;
    for( const auto & o : outputs )
    {
        if( ! o.indices.empty() )
            std::memcpy( mesh->indices + offset, o.indices.data(), o.indices.size() * sizeof( uint32_t ) );
        offset += o.indices.size();
    }
    *out = mesh;
    return FR_OK;
}
}  // namespace fr
