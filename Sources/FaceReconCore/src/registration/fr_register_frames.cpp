// License: Apache 2.0. See LICENSE file in root directory.
//
// Whole-scan registration. Frames arrive in scan order (front..., up..., down..., left..., right..., diagonals) with
// initial poses from the Vision head pose. The C API carries no pose labels, so pose groups are recovered from the
// initial poses: consecutive frames whose initial rotations differ by less than kGroupAngleDeg belong to one group.
//
// Edges (all refined by ICP from the initial relative pose):
//   * odometry - consecutive frames of the same group (certain),
//   * bridges  - each group to the frontal group, between the frames with the most similar initial rotation; the
//                best one per group is certain, the others are loop closures,
//   * loop closures - any other pair whose initial rotations differ by less than loop_closure_max_angle_deg
//                (left<->front<->right, up/down<->front, diagonals<->neighbors), uncertain and prunable.
// Edges are rejected by fitness / inlier RMSE and when ICP moved far from the initial guess (a wrong local minimum).
// If a group loses its certain bridge, the best accepted loop closure into the registered part takes its place.
// The pose graph (line processes, fixed frame 0) then distributes the residual error.

#include "recon_cloud.h"
#include "recon_graph.h"
#include "../common/fr_internal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <numeric>

namespace fr
{
namespace
{
constexpr double kGroupAngleDeg = 12.0;        // initial rotation change that starts a new pose group
constexpr size_t kBridgesPerGroup = 3;         // frame pairs tried between a group and the frontal group
constexpr size_t kMaxLoopsPerFrame = 6;        // bound on loop-closure candidates per frame
constexpr double kMaxCorrectionDeg = 30.0;     // ICP result vs. initial relative pose: rotation and
constexpr double kMaxCorrectionM = 0.04;       // displacement of the source cloud's centroid
constexpr size_t kMinFramePoints = 200;
constexpr float kRelativeRmseLimit = 1.5f;     // edges fitting worse than this times the median are rejected

enum class EdgeKind
{
    odometry,
    bridge,
    loop
};

struct Candidate
{
    uint32_t source = 0, target = 0;
    EdgeKind kind = EdgeKind::loop;
    double angle = 0;
    IcpOutput result;
    fr_status status = FR_ERR_FAILED;
    bool accepted = false;
    bool certain = false;
};

struct Progress
{
    fr_progress_cb callback = nullptr;
    void * user = nullptr;
    std::mutex mutex;
    std::atomic< bool > cancelled { false };

    // Serialized so the caller's callback never runs concurrently, even from worker threads.
    void report( float fraction, const char * stage )
    {
        if( ! callback || cancelled.load() )
            return;
        std::lock_guard< std::mutex > lock( mutex );
        if( ! fr::report( callback, user, std::clamp( fraction, 0.f, 1.f ), stage ) )
            cancelled.store( true );
    }
};

double relative_angle( const Eigen::Matrix4d & a, const Eigen::Matrix4d & b )
{
    return rotation_angle( a.block< 3, 3 >( 0, 0 ).transpose() * b.block< 3, 3 >( 0, 0 ) );
}

bool validate_registration_params( const fr_registration_params & p, std::string & error )
{
    if( ! validate_cloud_params( p.cloud, error ) || ! validate_icp_params( p.icp, error ) )
        return false;
    if( p.graph.max_iterations < 0 || ! ( p.graph.edge_prune_threshold >= 0 && p.graph.edge_prune_threshold < 1 )
        || ! ( p.graph.loop_closure_preference > 0 ) )
        error = "invalid pose graph parameters";
    else if( ! ( p.loop_closure_max_angle_deg >= 0 && p.loop_closure_max_angle_deg <= 180 ) )
        error = "loop_closure_max_angle_deg must be in [0, 180]";
    else if( ! ( p.min_fitness >= 0 && p.min_fitness <= 1 ) )
        error = "min_fitness must be in [0, 1]";
    else if( ! ( p.max_rmse_m > 0 ) )
        error = "max_rmse_m must be positive";
    else
        return true;
    return false;
}
}  // namespace
}  // namespace fr

extern "C" {

void fr_registration_params_default( fr_registration_params * p )
{
    if( ! p )
        return;
    std::memset( p, 0, sizeof( *p ) );
    fr_cloud_params_default( &p->cloud );
    fr_icp_params_default( &p->icp );
    fr_pose_graph_params_default( &p->graph );
    p->global_optimization = 1;
    p->loop_closure_max_angle_deg = 70.f;
    p->min_fitness = 0.3f;
    p->max_rmse_m = 0.003f;
}

fr_status fr_register_frames( const fr_rgbd_frame * frames, uint32_t count, const fr_registration_params * params,
                              float * out_world_from_camera, fr_registration_report * report, fr_progress_cb progress,
                              void * user )
{
    return fr::guarded( "fr_register_frames", [&]() -> fr_status {
        using namespace fr;
        if( report )
            std::memset( report, 0, sizeof( *report ) );
        if( ! frames || ! out_world_from_camera || count == 0 )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_register_frames: no frames or no output" );
        fr_registration_params p;
        if( params )
            p = *params;
        else
            fr_registration_params_default( &p );
        std::string error;
        if( ! validate_registration_params( p, error ) )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_register_frames: " + error );
        for( uint32_t i = 0; i < count; ++i )
            if( ! validate_frame( frames[i], error ) )
                return fail( FR_ERR_INVALID_ARGUMENT, "fr_register_frames: frame " + std::to_string( i ) + ": " + error );
        p.graph.reference_node = 0;   // frame 0 defines the world frame

        Progress prog;
        prog.callback = progress;
        prog.user = user;
        const size_t n = count;

        // Initial poses relative to frame 0.
        std::vector< Eigen::Matrix4d > initial( n, Eigen::Matrix4d::Identity() );
        for( size_t i = 0; i < n; ++i )
            if( frames[i].has_initial_pose )
            {
                initial[i] = load_matrix4( frames[i].initial_world_from_camera );
                initial[i].block< 3, 3 >( 0, 0 ) = orthonormalize( initial[i].block< 3, 3 >( 0, 0 ) );
                initial[i].row( 3 ) << 0, 0, 0, 1;
            }
        const Eigen::Matrix4d world_from_first = initial[0];
        for( auto & t : initial )
            t = rigid_inverse( world_from_first ) * t;

        // Per-frame clouds and pyramids (each frame is source and target of some edge).
        prog.report( 0.f, "building clouds" );
        std::vector< CloudPyramid > pyramids( n );
        std::vector< char > usable( n, 0 );
        std::atomic< size_t > built { 0 };
        parallel_for( n, [&]( size_t i ) {
            if( prog.cancelled.load() )
                return;
            const fr_cloud cloud = cloud_from_rgbd( frames[i], p.cloud );
            if( cloud.size() >= kMinFramePoints )
            {
                pyramids[i] = build_pyramid( cloud, p.icp.levels, p.icp.voxel_size_m, true, p.icp.colored != 0 );
                usable[i] = pyramids[i].levels[0]->cloud.size() >= kMinFramePoints / 4;
            }
            prog.report( 0.25f * float( ++built ) / float( n ), "building clouds" );
        } );
        if( prog.cancelled )
            return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );
        std::vector< Eigen::Vector3d > centroids( n, Eigen::Vector3d::Zero() );
        for( size_t i = 0; i < n; ++i )
        {
            if( ! usable[i] )
                continue;
            const fr_cloud & c = pyramids[i].levels[0]->cloud;
            for( size_t k = 0; k < c.size(); ++k )
                centroids[i] += c.point( k ).cast< double >();
            centroids[i] /= double( c.size() );
        }
        if( ! usable[0] )
            return fail( FR_ERR_NOT_ENOUGH_DATA, "fr_register_frames: the reference frame (0) has too few valid depth points" );

        // Pose groups from the initial rotations.
        std::vector< int > group( n, 0 );
        for( size_t i = 1; i < n; ++i )
            group[i] = relative_angle( initial[i - 1], initial[i] ) < deg2rad( kGroupAngleDeg ) ? group[i - 1] : group[i - 1] + 1;
        const int group_count = group[n - 1] + 1;
        const int front = group[0];

        // Candidate edges.
        std::vector< fr::Candidate > edges;
        std::vector< std::vector< char > > paired( n, std::vector< char >( n, 0 ) );
        auto add_edge = [&]( size_t s, size_t t, EdgeKind kind ) {
            fr::Candidate c;
            c.source = uint32_t( s );
            c.target = uint32_t( t );
            c.kind = kind;
            c.angle = relative_angle( initial[s], initial[t] );
            edges.push_back( c );
            paired[s][t] = paired[t][s] = 1;
        };
        for( size_t i = 1; i < n; ++i )
            if( group[i] == group[i - 1] && usable[i] && usable[i - 1] )
                add_edge( i, i - 1, EdgeKind::odometry );
        for( int g = 0; g < group_count; ++g )
        {
            if( g == front )
                continue;
            std::vector< std::pair< double, std::pair< size_t, size_t > > > pairs;
            for( size_t a = 0; a < n; ++a )
                for( size_t b = 0; b < n; ++b )
                    if( group[a] == g && group[b] == front && usable[a] && usable[b] )
                        pairs.push_back( { relative_angle( initial[a], initial[b] ), { a, b } } );
            std::sort( pairs.begin(), pairs.end() );
            std::vector< char > used( n, 0 );
            size_t added = 0;
            for( const auto & pr : pairs )
            {
                const size_t a = pr.second.first, b = pr.second.second;
                if( added >= kBridgesPerGroup || used[a] || used[b] )
                    continue;
                add_edge( a, b, EdgeKind::bridge );
                used[a] = used[b] = 1;
                ++added;
            }
        }
        if( p.global_optimization )
        {
            std::vector< std::pair< double, std::pair< size_t, size_t > > > pairs;
            const double max_angle = deg2rad( p.loop_closure_max_angle_deg );
            for( size_t i = 0; i < n; ++i )
                for( size_t j = i + 1; j < n; ++j )
                {
                    if( ! usable[i] || ! usable[j] || paired[i][j] )
                        continue;
                    // Within a group only skip-one pairs add information; the views are nearly identical.
                    if( group[i] == group[j] && j - i != 2 )
                        continue;
                    const double angle = relative_angle( initial[i], initial[j] );
                    if( angle < max_angle )
                        pairs.push_back( { angle, { j, i } } );
                }
            std::sort( pairs.begin(), pairs.end() );
            std::vector< size_t > per_frame( n, 0 );
            for( const auto & pr : pairs )
            {
                const size_t s = pr.second.first, t = pr.second.second;
                if( per_frame[s] >= kMaxLoopsPerFrame || per_frame[t] >= kMaxLoopsPerFrame )
                    continue;
                add_edge( s, t, EdgeKind::loop );
                ++per_frame[s];
                ++per_frame[t];
            }
        }

        // Pairwise ICP.
        prog.report( 0.25f, "pairwise registration" );
        std::atomic< size_t > done { 0 };
        parallel_for( edges.size(), [&]( size_t k ) {
            if( prog.cancelled.load() )
                return;
            fr::Candidate & c = edges[k];
            auto attempt = [&]( const Eigen::Matrix4d & init, IcpOutput & result ) {
                if( icp_pyramids( pyramids[c.source], pyramids[c.target], init, p.icp, result, &prog.cancelled ) != FR_OK )
                    return false;
                // Judge the correction by how far it moves the observed surface: a few degrees about the head,
                // 0.6 m in front of the camera, already mean centimeters of translation in the camera frame.
                const Eigen::Matrix4d correction = rigid_inverse( init ) * result.target_from_source;
                const Eigen::Vector3d centroid = centroids[c.source];
                const Eigen::Vector3d moved = correction.block< 3, 3 >( 0, 0 ) * centroid + correction.block< 3, 1 >( 0, 3 );
                const bool plausible = rotation_angle( correction.block< 3, 3 >( 0, 0 ) ) < deg2rad( kMaxCorrectionDeg )
                                    && ( moved - centroid ).norm() < kMaxCorrectionM;
                return plausible && result.fitness >= p.min_fitness && result.inlier_rmse <= p.max_rmse_m
                    && result.information.allFinite();
            };
            const Eigen::Matrix4d init = rigid_inverse( initial[c.target] ) * initial[c.source];
            IcpOutput result;
            bool accepted = attempt( init, result );
            if( group[c.source] == group[c.target] )
            {
                // Within a pose group the head is held still: the identity rotation (centroids aligned) is often a
                // better start than two noisy Vision estimates. Keep whichever converges better.
                Eigen::Matrix4d still = Eigen::Matrix4d::Identity();
                still.block< 3, 1 >( 0, 3 ) = centroids[c.target] - centroids[c.source];
                IcpOutput other;
                const bool other_accepted = attempt( still, other );
                const bool better = other_accepted > accepted
                                 || ( other_accepted == accepted && other.fitness / std::max( other.inlier_rmse, 1e-4f )
                                                                    > result.fitness / std::max( result.inlier_rmse, 1e-4f ) );
                if( better && other.correspondences > 0 )
                {
                    result = other;
                    accepted = other_accepted;
                }
            }
            c.result = result;
            c.accepted = accepted;
            c.status = result.correspondences > 0 ? FR_OK : FR_ERR_NOT_ENOUGH_DATA;
            prog.report( 0.25f + 0.6f * float( ++done ) / float( std::max< size_t >( 1, edges.size() ) ), "pairwise registration" );
        } );
        if( prog.cancelled )
            return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );

        // A sliding partial overlap can satisfy the absolute thresholds; it still fits worse than the typical edge,
        // whose RMSE reflects the depth noise.
        {
            std::vector< float > rmse;
            for( const auto & c : edges )
                if( c.accepted )
                    rmse.push_back( c.result.inlier_rmse );
            if( rmse.size() >= 5 )
            {
                std::nth_element( rmse.begin(), rmse.begin() + std::ptrdiff_t( rmse.size() / 2 ), rmse.end() );
                const float limit = kRelativeRmseLimit * rmse[rmse.size() / 2];
                for( auto & c : edges )
                    if( c.accepted && c.result.inlier_rmse > limit )
                        c.accepted = false;
            }
        }
        if( prog.cancelled )
            return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );

        if( std::getenv( "FR_DEBUG_REGISTRATION" ) )
            for( const auto & c : edges )
                std::fprintf( stderr, "edge %u->%u kind %d angle %.1f status %d fit %.3f rmse %.3f mm accepted %d\n", c.source, c.target,
                              int( c.kind ), rad2deg( c.angle ), int( c.status ), c.result.fitness, c.result.inlier_rmse * 1000, int( c.accepted ) );
        // Certain spanning structure: odometry, plus per group the bridge that fits best after ICP (a pre-chosen
        // bridge may have slid into a wrong minimum that still passes the thresholds), repaired with the best
        // accepted alternatives where a group or frame stays disconnected.
        auto quality = []( const fr::Candidate & c ) { return c.result.fitness / std::max( c.result.inlier_rmse, 1e-4f ); };
        for( auto & c : edges )
            c.certain = c.accepted && c.kind == EdgeKind::odometry;
        std::vector< int > best_bridge( size_t( group_count ), -1 );
        for( size_t k = 0; k < edges.size(); ++k )
        {
            const auto & c = edges[k];
            if( ! c.accepted || c.kind != EdgeKind::bridge )
                continue;
            int & b = best_bridge[size_t( group[c.source] )];
            if( b < 0 || quality( c ) > quality( edges[size_t( b )] ) )
                b = int( k );
        }
        for( int b : best_bridge )
            if( b >= 0 )
                edges[size_t( b )].certain = true;
        std::vector< char > reached( n, 0 );
        auto flood = [&]() {
            std::fill( reached.begin(), reached.end(), 0 );
            reached[0] = 1;
            bool changed = true;
            while( changed )
            {
                changed = false;
                for( const auto & c : edges )
                    if( c.certain && reached[c.source] != reached[c.target] )
                    {
                        reached[c.source] = reached[c.target] = 1;
                        changed = true;
                    }
            }
        };
        flood();
        while( true )
        {
            fr::Candidate * best = nullptr;
            for( auto & c : edges )
            {
                if( ! c.accepted || c.certain || reached[c.source] == reached[c.target] )
                    continue;
                const bool bridge = c.kind == EdgeKind::bridge, best_bridge_kind = best && best->kind == EdgeKind::bridge;
                if( ! best || bridge > best_bridge_kind || ( bridge == best_bridge_kind && quality( c ) > quality( *best ) ) )
                    best = &c;
            }
            if( ! best )
                break;
            best->certain = true;
            flood();
        }

        // Poses along the certain edges; frames that could not be connected keep their initial pose.
        std::vector< Eigen::Matrix4d > poses = initial;
        {
            std::vector< char > known( n, 0 );
            known[0] = 1;
            poses[0] = Eigen::Matrix4d::Identity();
            bool changed = true;
            while( changed )
            {
                changed = false;
                for( const auto & c : edges )
                {
                    if( ! c.certain || known[c.source] == known[c.target] )
                        continue;
                    if( known[c.target] )
                        poses[c.source] = poses[c.target] * c.result.target_from_source;
                    else
                        poses[c.target] = poses[c.source] * rigid_inverse( c.result.target_from_source );
                    known[c.source] = known[c.target] = 1;
                    changed = true;
                }
            }
        }

        fr_pose_graph_report graph_report {};
        uint32_t pruned = 0;
        std::vector< char > pruned_edge( edges.size(), 0 );
        if( p.global_optimization )
        {
            prog.report( 0.85f, "pose graph optimization" );
            if( prog.cancelled )
                return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );
            fr_pose_graph * graph = nullptr;
            fr_status status = fr_pose_graph_create( &graph );
            if( status != FR_OK )
                return status;
            std::unique_ptr< fr_pose_graph, void ( * )( fr_pose_graph * ) > owner( graph, fr_pose_graph_free );
            for( size_t i = 0; i < n; ++i )
            {
                float m[16];
                store_matrix4( poses[i], m );
                fr_pose_graph_add_node( graph, m );
            }
            std::vector< size_t > graph_edges;
            for( size_t k = 0; k < edges.size(); ++k )
            {
                const auto & c = edges[k];
                if( ! c.accepted )
                    continue;
                float m[16];
                double info[36];
                store_matrix4( c.result.target_from_source, m );
                Eigen::Map< Matrix6d > info_map( info );
                info_map = c.result.information;
                status = fr_pose_graph_add_edge( graph, c.source, c.target, m, info, c.certain ? 0 : 1 );
                if( status != FR_OK )
                    return status;
                graph_edges.push_back( k );
            }
            status = fr_pose_graph_optimize( graph, &p.graph, &graph_report );
            if( status != FR_OK )
                return status;
            for( size_t i = 0; i < n; ++i )
            {
                float m[16];
                fr_pose_graph_get_node( graph, uint32_t( i ), m );
                poses[i] = load_matrix4( m );
            }
            for( size_t e = 0; e < graph_edges.size(); ++e )
                if( graph->edges[e].pruned )
                {
                    pruned_edge[graph_edges[e]] = 1;
                    ++pruned;
                }
        }

        for( size_t i = 0; i < n; ++i )
        {
            Eigen::Matrix4d t = poses[i];
            t.block< 3, 3 >( 0, 0 ) = orthonormalize( t.block< 3, 3 >( 0, 0 ) );
            t.row( 3 ) << 0, 0, 0, 1;
            store_matrix4( t, out_world_from_camera + 16 * i );
        }

        if( report )
        {
            report->frames = uint32_t( n );
            double rmse_sum = 0, fitness_sum = 0;
            float rmse_max = 0;
            uint32_t used = 0;
            for( size_t k = 0; k < edges.size(); ++k )
            {
                const auto & c = edges[k];
                if( ! c.accepted || pruned_edge[k] )
                {
                    ++report->rejected_edges;
                    continue;
                }
                if( ! c.certain && ! p.global_optimization )
                    continue;   // accepted alternatives are unused without the pose graph
                if( c.certain )
                    ++report->odometry_edges;
                else
                    ++report->loop_closures;
                rmse_sum += c.result.inlier_rmse;
                fitness_sum += c.result.fitness;
                rmse_max = std::max( rmse_max, c.result.inlier_rmse );
                ++used;
            }
            report->mean_rmse_m = used ? float( rmse_sum / used ) : 0.f;
            report->max_rmse_m = rmse_max;
            report->mean_fitness = used ? float( fitness_sum / used ) : 0.f;
            report->graph = graph_report;
            report->graph.pruned_edges = pruned;
        }
        size_t unregistered = 0;
        for( size_t i = 0; i < n; ++i )
            unregistered += ! reached[i];
        prog.report( 1.f, unregistered ? "registration finished (some frames unregistered)" : "registration finished" );
        clear_error();
        return FR_OK;
    } );
}

}  // extern "C"
