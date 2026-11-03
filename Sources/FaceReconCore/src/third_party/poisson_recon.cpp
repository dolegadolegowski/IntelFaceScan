// License: Apache 2.0. See LICENSE file in root directory.
//
// Unity wrapper of PoissonRecon (Version 18.76, BSD license, third_party/PoissonRecon/Src). Compiles in ~7 s.

#include "poisson_recon.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include "Reconstructors.h"
#pragma clang diagnostic pop

#include <mutex>
#include <stdexcept>

namespace
{
using namespace PoissonRecon;

template< typename Real >
struct Rgb
{
    Real r = 0, g = 0, b = 0;
    Rgb() = default;
    Rgb( Real r_, Real g_, Real b_ ) : r( r_ ), g( g_ ), b( b_ ) {}
    Rgb & operator+=( const Rgb & c ) { r += c.r; g += c.g; b += c.b; return *this; }
    Rgb & operator*=( Real s ) { r *= s; g *= s; b *= s; return *this; }
    Rgb & operator/=( Real s ) { return operator*=( Real( 1 ) / s ); }
    Rgb operator+( const Rgb & c ) const { return Rgb( r + c.r, g + c.g, b + c.b ); }
    Rgb operator*( Real s ) const { return Rgb( r * s, g * s, b * s ); }
    Rgb operator/( Real s ) const { return operator*( Real( 1 ) / s ); }
};
}  // namespace

namespace PoissonRecon
{
// Splatting color as auxiliary data needs atomic accumulation.
template< typename Real >
struct Atomic< Rgb< Real > >
{
    static void Add( volatile Rgb< Real > & a, const Rgb< Real > & b )
    {
        Atomic< Real >::Add( a.r, b.r );
        Atomic< Real >::Add( a.g, b.g );
        Atomic< Real >::Add( a.b, b.b );
    }
};
}  // namespace PoissonRecon

namespace
{
using Real = float;
constexpr unsigned kDim = 3;

struct OrientedStream : public Reconstructor::InputOrientedSampleStream< Real, kDim >
{
    const fr::poisson::Input & in;
    size_t next = 0;
    explicit OrientedStream( const fr::poisson::Input & input ) : in( input ) {}
    void reset() override { next = 0; }
    bool read( Point< Real, kDim > & p, Point< Real, kDim > & n ) override
    {
        if( next >= in.count )
            return false;
        for( unsigned d = 0; d < kDim; ++d )
        {
            p[d] = in.points[3 * next + d];
            n[d] = in.normals[3 * next + d];
        }
        ++next;
        return true;
    }
};

struct ColoredStream : public Reconstructor::InputOrientedSampleStream< Real, kDim, Rgb< Real > >
{
    const fr::poisson::Input & in;
    size_t next = 0;
    explicit ColoredStream( const fr::poisson::Input & input ) : in( input ) {}
    void reset() override { next = 0; }
    bool read( Point< Real, kDim > & p, Point< Real, kDim > & n, Rgb< Real > & c ) override
    {
        if( next >= in.count )
            return false;
        for( unsigned d = 0; d < kDim; ++d )
        {
            p[d] = in.points[3 * next + d];
            n[d] = in.normals[3 * next + d];
        }
        const uint8_t * rgb = in.colors + 3 * next;
        c = Rgb< Real >( rgb[0] / Real( 255 ), rgb[1] / Real( 255 ), rgb[2] / Real( 255 ) );
        ++next;
        return true;
    }
};

struct VertexSink : public Reconstructor::OutputLevelSetVertexStream< Real, kDim >
{
    fr::poisson::Output & out;
    explicit VertexSink( fr::poisson::Output & o ) : out( o ) {}
    size_t size() const override { return out.density.size(); }
    size_t write( const Point< Real, kDim > & p, const Point< Real, kDim > &, const Real & density ) override
    {
        out.positions.insert( out.positions.end(), { p[0], p[1], p[2] } );
        out.density.push_back( density );
        return out.density.size() - 1;
    }
};

struct ColoredVertexSink : public Reconstructor::OutputLevelSetVertexStream< Real, kDim, Rgb< Real > >
{
    fr::poisson::Output & out;
    explicit ColoredVertexSink( fr::poisson::Output & o ) : out( o ) {}
    size_t size() const override { return out.density.size(); }
    size_t write( const Point< Real, kDim > & p, const Point< Real, kDim > &, const Real & density, const Rgb< Real > & c ) override
    {
        out.positions.insert( out.positions.end(), { p[0], p[1], p[2] } );
        out.density.push_back( density );
        out.colors.insert( out.colors.end(), { c.r, c.g, c.b } );
        return out.density.size() - 1;
    }
};

struct FaceSink : public Reconstructor::OutputFaceStream< 2 >
{
    std::vector< uint32_t > & indices;
    size_t faces = 0;
    explicit FaceSink( std::vector< uint32_t > & i ) : indices( i ) {}
    size_t size() const override { return faces; }
    size_t write( const std::vector< node_index_type > & polygon ) override
    {
        // Triangles are requested; fan anything larger defensively.
        for( size_t k = 1; k + 1 < polygon.size(); ++k )
            indices.insert( indices.end(), { uint32_t( polygon[0] ), uint32_t( polygon[k] ), uint32_t( polygon[k + 1] ) } );
        return faces++;
    }
};

std::mutex g_poisson_mutex;

template< bool Colored >
void solve_and_extract( const fr::poisson::Input & input, const fr::poisson::Options & options, fr::poisson::Output & output )
{
    using Poisson = Reconstructor::Poisson;
    static const unsigned int FEMSig = FEMDegreeAndBType< Poisson::DefaultFEMDegree, Poisson::DefaultFEMBoundary >::Signature;
    using FEMSigs = IsotropicUIntPack< kDim, FEMSig >;
    Poisson::SolutionParameters< Real > sp;
    sp.verbose = false;
    sp.depth = unsigned( options.depth );
    sp.pointWeight = options.point_weight;
    sp.samplesPerNode = options.samples_per_node;
    Reconstructor::LevelSetExtractionParameters ep;
    ep.outputDensity = true;
    ep.forceManifold = true;
    ep.polygonMesh = false;
    ep.verbose = false;

    // Version 18.76 races on a static futures vector when ParallelFor nests inside the ParallelSections of the
    // level-set extraction, so only the solver runs multi-threaded.
    ThreadPool::ParallelizationType = options.parallel ? ThreadPool::ParallelType::ASYNC : ThreadPool::ParallelType::NONE;
    FaceSink faces( output.indices );
    if constexpr( Colored )
    {
        ColoredStream stream( input );
        using Solver = Poisson::Solver< Real, kDim, FEMSigs, Rgb< Real > >;
        std::unique_ptr< Reconstructor::Implicit< Real, kDim, FEMSigs, Rgb< Real > > > implicit( Solver::Solve( stream, sp, Rgb< Real >() ) );
        if( ! implicit )
            throw std::runtime_error( "Poisson solver failed" );
        ThreadPool::ParallelizationType = ThreadPool::ParallelType::NONE;
        ColoredVertexSink vertices( output );
        implicit->extractLevelSet( vertices, faces, ep );
    }
    else
    {
        OrientedStream stream( input );
        using Solver = Poisson::Solver< Real, kDim, FEMSigs >;
        std::unique_ptr< Reconstructor::Implicit< Real, kDim, FEMSigs > > implicit( Solver::Solve( stream, sp ) );
        if( ! implicit )
            throw std::runtime_error( "Poisson solver failed" );
        ThreadPool::ParallelizationType = ThreadPool::ParallelType::NONE;
        VertexSink vertices( output );
        implicit->extractLevelSet( vertices, faces, ep );
    }
}
}  // namespace

namespace fr
{
namespace poisson
{
void reconstruct( const Input & input, const Options & options, Output & output )
{
    std::lock_guard< std::mutex > lock( g_poisson_mutex );
    output = Output();
    struct ResetPool
    {
        ~ResetPool() { ThreadPool::ParallelizationType = ThreadPool::ParallelType::NONE; }
    } reset;
    if( input.colors )
        solve_and_extract< true >( input, options, output );
    else
        solve_and_extract< false >( input, options, output );
}
}  // namespace poisson
}  // namespace fr
