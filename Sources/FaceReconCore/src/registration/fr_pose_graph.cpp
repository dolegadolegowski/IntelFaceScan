// License: Apache 2.0. See LICENSE file in root directory.
//
// SE(3) pose graph optimization: Levenberg-Marquardt over left perturbations of world_from_node poses with one
// fixed reference node, and line processes on uncertain (loop-closure) edges as in Choi, Zhou, Koltun: "Robust
// Reconstruction of Indoor Scenes" (CVPR 2015).
//
// Edge residual: T_t^-1 T_s = exp(e) Z, i.e. e = log(T_t^-1 T_s Z^-1), a left perturbation of the measurement in the
// target frame - the frame in which ICP computes its information matrix. Uncertain edges carry a line process
// l = (mu / (mu + e^T L e))^2 (its closed-form optimum), which turns their cost into mu e^T L e / (mu + e^T L e).
// mu = loop_closure_preference * (1 cm)^2 * mean translational information, i.e. with ICP information matrices
// (translational block = correspondence count * I) and the default preference 0.1, an edge whose correspondences
// disagree with the optimized poses by more than ~sqrt(0.1) cm = 3.2 mm RMS gets l < 0.25 and is pruned.

#include "recon_graph.h"
#include "../common/fr_internal.h"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <cstring>
#include <vector>

namespace fr
{
namespace
{
constexpr double kLineProcessScale2 = 0.01 * 0.01;   // (1 cm)^2, see the file comment

struct EdgeState
{
    Vector6d error;
    double chi2 = 0;
    double mu = 0;
    double line_process = 1;
};

double edge_mu( const fr_pose_graph::Edge & e, double preference )
{
    const double translational = std::max( 1e-12, e.information.block< 3, 3 >( 3, 3 ).trace() / 3.0 );
    return preference * kLineProcessScale2 * translational;
}

EdgeState evaluate_edge( const fr_pose_graph::Edge & e, const std::vector< Eigen::Matrix4d > & poses, double preference )
{
    EdgeState s;
    s.error = se3_log( rigid_inverse( poses[e.target] ) * poses[e.source] * rigid_inverse( e.measurement ) );
    s.chi2 = std::max( 0.0, double( s.error.transpose() * e.information * s.error ) );
    if( e.uncertain )
    {
        s.mu = edge_mu( e, preference );
        const double sq = s.mu / ( s.mu + s.chi2 );
        s.line_process = sq * sq;
    }
    return s;
}

double total_cost( const fr_pose_graph & g, const std::vector< Eigen::Matrix4d > & poses, const std::vector< char > & active,
                   double preference )
{
    double cost = 0;
    for( size_t k = 0; k < g.edges.size(); ++k )
    {
        if( ! active[k] )
            continue;
        const EdgeState s = evaluate_edge( g.edges[k], poses, preference );
        cost += g.edges[k].uncertain ? s.mu * s.chi2 / ( s.mu + s.chi2 ) : s.chi2;
    }
    return cost;
}

struct LmResult
{
    int iterations = 0;
    double initial_cost = 0;
    double final_cost = 0;
};

// Levenberg-Marquardt on the nodes in 'free_index' (index into the variable vector, -1 = fixed).
LmResult levenberg_marquardt( const fr_pose_graph & g, std::vector< Eigen::Matrix4d > & poses, const std::vector< char > & active,
                              const std::vector< int > & free_index, int free_count, int max_iterations, double preference )
{
    LmResult result;
    result.initial_cost = result.final_cost = total_cost( g, poses, active, preference );
    if( free_count == 0 || max_iterations <= 0 )
        return result;
    const int dim = 6 * free_count;
    double lambda = 1e-4;
    double cost = result.initial_cost;
    Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > > solver;
    bool analyzed = false;

    for( int iter = 0; iter < max_iterations; ++iter )
    {
        std::vector< Eigen::Triplet< double > > triplets;
        triplets.reserve( g.edges.size() * 4 * 36 + size_t( dim ) );
        Eigen::VectorXd b = Eigen::VectorXd::Zero( dim );
        Eigen::VectorXd diag = Eigen::VectorXd::Zero( dim );
        auto add_block = [&]( int r, int c, const Matrix6d & m ) {
            for( int i = 0; i < 6; ++i )
                for( int j = 0; j < 6; ++j )
                    triplets.emplace_back( 6 * r + i, 6 * c + j, m( i, j ) );
        };
        for( size_t k = 0; k < g.edges.size(); ++k )
        {
            if( ! active[k] )
                continue;
            const auto & e = g.edges[k];
            const int s = free_index[e.source], t = free_index[e.target];
            if( s < 0 && t < 0 )
                continue;
            const EdgeState st = evaluate_edge( e, poses, preference );
            const double w = e.uncertain ? st.line_process : 1.0;
            // d e / d xi_s = J_l^-1(e) Ad(T_t^-1); d e / d xi_t = -(same).
            const Matrix6d js = ( Matrix6d::Identity() - 0.5 * se3_ad( st.error ) ) * se3_adjoint( rigid_inverse( poses[e.target] ) );
            const Matrix6d jtl = js.transpose() * e.information;
            const Matrix6d hss = w * jtl * js;
            const Vector6d bs = w * jtl * st.error;
            if( s >= 0 )
            {
                add_block( s, s, hss );
                b.segment< 6 >( 6 * s ) += bs;
                diag.segment< 6 >( 6 * s ) += hss.diagonal();
            }
            if( t >= 0 )
            {
                add_block( t, t, hss );
                b.segment< 6 >( 6 * t ) -= bs;
                diag.segment< 6 >( 6 * t ) += hss.diagonal();
            }
            if( s >= 0 && t >= 0 )
            {
                add_block( s, t, -hss );
                add_block( t, s, -hss );
            }
        }
        Eigen::SparseMatrix< double > h_base( dim, dim );
        h_base.setFromTriplets( triplets.begin(), triplets.end() );

        bool improved = false;
        while( ! improved )
        {
            Eigen::SparseMatrix< double > h = h_base;
            for( int i = 0; i < dim; ++i )
                h.coeffRef( i, i ) += lambda * std::max( diag[i], 1e-9 ) + 1e-12;
            if( ! analyzed )
            {
                solver.analyzePattern( h );
                analyzed = true;
            }
            solver.factorize( h );
            if( solver.info() != Eigen::Success )
            {
                lambda *= 10;
                if( lambda > 1e12 )
                    break;
                continue;
            }
            const Eigen::VectorXd delta = -solver.solve( b );
            if( ! delta.allFinite() )
            {
                lambda *= 10;
                if( lambda > 1e12 )
                    break;
                continue;
            }
            std::vector< Eigen::Matrix4d > candidate = poses;
            for( size_t n = 0; n < poses.size(); ++n )
                if( free_index[n] >= 0 )
                    candidate[n] = se3_exp( delta.segment< 6 >( 6 * free_index[n] ) ) * poses[n];
            const double new_cost = total_cost( g, candidate, active, preference );
            if( new_cost < cost )
            {
                const double decrease = cost - new_cost;
                poses.swap( candidate );
                cost = new_cost;
                lambda = std::max( lambda / 3.0, 1e-12 );
                improved = true;
                result.iterations = iter + 1;
                if( decrease < 1e-12 * std::max( 1.0, cost ) || delta.lpNorm< Eigen::Infinity >() < 1e-10 )
                {
                    result.final_cost = cost;
                    return result;
                }
            }
            else
            {
                lambda *= 4;
                if( lambda > 1e12 )
                    break;
            }
        }
        if( ! improved )
            break;   // no step reduces the cost: converged (or numerically stuck)
    }
    result.final_cost = cost;
    return result;
}
}  // namespace
}  // namespace fr

extern "C" {

void fr_pose_graph_params_default( fr_pose_graph_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    p->max_iterations = 100;
    p->edge_prune_threshold = 0.25f;
    p->loop_closure_preference = 0.1f;
    p->reference_node = 0;
}

fr_status fr_pose_graph_create( fr_pose_graph ** out )
{
    return fr::guarded( "fr_pose_graph_create", [&]() -> fr_status {
        if( ! out )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_create: out is NULL" );
        *out = new fr_pose_graph();
        return FR_OK;
    } );
}

uint32_t fr_pose_graph_add_node( fr_pose_graph * graph, const float world_from_node[16] )
{
    if( ! graph )
        return UINT32_MAX;
    try
    {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        if( world_from_node && fr::all_finite( world_from_node, 16 ) )
        {
            t = fr::load_matrix4( world_from_node );
            t.block< 3, 3 >( 0, 0 ) = fr::orthonormalize( t.block< 3, 3 >( 0, 0 ) );
            t.row( 3 ) << 0, 0, 0, 1;
        }
        graph->nodes.push_back( t );
        return uint32_t( graph->nodes.size() - 1 );
    }
    catch( ... )
    {
        fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_pose_graph_add_node: out of memory" );
        return UINT32_MAX;
    }
}

fr_status fr_pose_graph_add_edge( fr_pose_graph * graph, uint32_t source, uint32_t target, const float target_from_source[16],
                                  const double information[36], int32_t uncertain )
{
    return fr::guarded( "fr_pose_graph_add_edge", [&]() -> fr_status {
        if( ! graph || ! target_from_source || ! information )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_add_edge: NULL argument" );
        if( source >= graph->nodes.size() || target >= graph->nodes.size() || source == target )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_add_edge: invalid node index" );
        if( ! fr::all_finite( target_from_source, 16 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_add_edge: measurement is not finite" );
        fr_pose_graph::Edge e;
        e.source = source;
        e.target = target;
        e.measurement = fr::load_matrix4( target_from_source );
        e.measurement.block< 3, 3 >( 0, 0 ) = fr::orthonormalize( e.measurement.block< 3, 3 >( 0, 0 ) );
        e.measurement.row( 3 ) << 0, 0, 0, 1;
        e.information = Eigen::Map< const fr::Matrix6d >( information );
        if( ! e.information.allFinite() )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_add_edge: information is not finite" );
        e.information = 0.5 * ( e.information + e.information.transpose() ).eval();
        e.uncertain = uncertain != 0;
        graph->edges.push_back( e );
        return FR_OK;
    } );
}

fr_status fr_pose_graph_optimize( fr_pose_graph * graph, const fr_pose_graph_params * params, fr_pose_graph_report * report )
{
    return fr::guarded( "fr_pose_graph_optimize", [&]() -> fr_status {
        if( report )
            std::memset( report, 0, sizeof( *report ) );
        if( ! graph )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_optimize: graph is NULL" );
        fr_pose_graph_params p;
        if( params )
            p = *params;
        else
            fr_pose_graph_params_default( &p );
        const size_t n = graph->nodes.size();
        if( n == 0 )
            return fr::fail( FR_ERR_NOT_ENOUGH_DATA, "fr_pose_graph_optimize: no nodes" );
        if( p.reference_node < 0 || size_t( p.reference_node ) >= n )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_optimize: invalid reference node" );
        if( p.max_iterations < 0 || ! ( p.edge_prune_threshold >= 0 && p.edge_prune_threshold < 1 )
            || ! ( p.loop_closure_preference > 0 ) || ! std::isfinite( p.loop_closure_preference ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_optimize: invalid parameters" );
        const double preference = p.loop_closure_preference;

        std::vector< char > active( graph->edges.size() );
        for( size_t k = 0; k < graph->edges.size(); ++k )
            active[k] = ! graph->edges[k].pruned;

        // Only the component connected to the reference is determined by the measurements; other nodes keep their
        // poses (their blocks would make the system singular).
        auto free_variables = [&]( std::vector< int > & free_index ) {
            std::vector< std::vector< uint32_t > > adjacency( n );
            for( size_t k = 0; k < graph->edges.size(); ++k )
                if( active[k] )
                {
                    adjacency[graph->edges[k].source].push_back( graph->edges[k].target );
                    adjacency[graph->edges[k].target].push_back( graph->edges[k].source );
                }
            std::vector< char > reached( n, 0 );
            std::vector< uint32_t > stack { uint32_t( p.reference_node ) };
            reached[size_t( p.reference_node )] = 1;
            while( ! stack.empty() )
            {
                const uint32_t v = stack.back();
                stack.pop_back();
                for( uint32_t u : adjacency[v] )
                    if( ! reached[u] )
                    {
                        reached[u] = 1;
                        stack.push_back( u );
                    }
            }
            free_index.assign( n, -1 );
            int count = 0;
            for( size_t i = 0; i < n; ++i )
                if( reached[i] && int( i ) != p.reference_node )
                    free_index[i] = count++;
            return count;
        };

        std::vector< int > free_index;
        int free_count = free_variables( free_index );
        std::vector< Eigen::Matrix4d > poses = graph->nodes;
        fr::LmResult first = fr::levenberg_marquardt( *graph, poses, active, free_index, free_count, p.max_iterations, preference );
        int iterations = first.iterations;
        double final_cost = first.final_cost;

        uint32_t pruned = 0;
        for( size_t k = 0; k < graph->edges.size(); ++k )
        {
            if( ! active[k] || ! graph->edges[k].uncertain )
                continue;
            const fr::EdgeState s = fr::evaluate_edge( graph->edges[k], poses, preference );
            if( s.line_process < p.edge_prune_threshold )
            {
                graph->edges[k].pruned = true;
                active[k] = 0;
                ++pruned;
            }
        }
        if( pruned )
        {
            // Re-optimize without the inconsistent loop closures (their residual pull biased the first solution).
            free_count = free_variables( free_index );
            fr::LmResult second = fr::levenberg_marquardt( *graph, poses, active, free_index, free_count, p.max_iterations, preference );
            iterations += second.iterations;
            final_cost = second.final_cost;
        }
        graph->nodes = poses;
        if( report )
        {
            report->iterations = iterations;
            report->initial_error = float( first.initial_cost );
            report->final_error = float( final_cost );
            report->pruned_edges = pruned;
        }
        return FR_OK;
    } );
}

uint32_t fr_pose_graph_node_count( const fr_pose_graph * graph )
{
    return graph ? uint32_t( graph->nodes.size() ) : 0;
}

fr_status fr_pose_graph_get_node( const fr_pose_graph * graph, uint32_t node, float world_from_node[16] )
{
    if( ! graph || ! world_from_node )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_get_node: NULL argument" );
    if( node >= graph->nodes.size() )
        return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_pose_graph_get_node: invalid node index" );
    fr::store_matrix4( graph->nodes[node], world_from_node );
    return FR_OK;
}

void fr_pose_graph_free( fr_pose_graph * graph )
{
    delete graph;
}

}  // extern "C"
