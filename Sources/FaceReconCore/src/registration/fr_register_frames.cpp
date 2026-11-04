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
// Edges are accepted by fitness, or below it by a two-way projective overlap test with a guard against sliding (wide
// side views share little surface with a frontal view), and rejected by inlier RMSE and when ICP moved far from the
// initial guess (a wrong local minimum). If a group loses its certain bridge, the best accepted loop closure into the
// registered part takes its place. The pose graph (line processes, fixed frame 0) then distributes the residual error.

#include "recon_cloud.h"
#include "recon_graph.h"
#include "../common/fr_internal.h"

#include <Eigen/Eigenvalues>

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
// Overlap test for edges below min_fitness. Thresholds from 1,259 distinct ICP minima of a real D455 scan's side and
// frontal pairs, labelled correct or wrong (docs/RECONSTRUCTION.md, "Whole-scan registration").
constexpr float kOverlapInlierFraction = 0.75f; // overlap inliers within 0.75 x max_correspondence_m (3 mm)
constexpr float kFreeSpaceTolerance = 0.005f;   // depth difference to the other view's surface
constexpr float kMinOverlapFitness = 0.85f;     // both directions
constexpr float kMaxFreeSpaceRatio = 0.08f;     // both directions
constexpr double kMinOverlapAreaM2 = 0.001;     // 10 cm² of fitting surface, both directions
constexpr double kMinOverlapConstraint = 0.002; // see overlap_constraint()

enum class EdgeKind
{
    odometry,
    bridge,
    loop
};

// The overlap test of an ICP result below min_fitness.
struct OverlapCheck
{
    bool evaluated = false;
    OverlapStats forward, backward;   // source->target and target->source
    double constraint = 0;            // overlap_constraint(), only when the overlap statistics pass
    bool accepted = false;
};

struct Candidate
{
    uint32_t source = 0, target = 0;
    EdgeKind kind = EdgeKind::loop;
    double angle = 0;
    IcpOutput result;
    OverlapCheck overlap;
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

bool overlap_fits( float min_fitness, float max_free_space_ratio, double min_inlier_area_m2 )
{
    return min_fitness >= kMinOverlapFitness && max_free_space_ratio <= kMaxFreeSpaceRatio
        && min_inlier_area_m2 >= kMinOverlapAreaM2;
}

bool overlap_fits( const OverlapStats & forward, const OverlapStats & backward )
{
    return overlap_fits( std::min( forward.fitness(), backward.fitness() ),
                         std::max( forward.violation_ratio(), backward.violation_ratio() ),
                         std::min( forward.inlier_area, backward.inlier_area ) );
}

// How well the shared surface pins all six degrees of freedom: the smallest eigenvalue of the point-to-plane Hessian
// of the inliers (source points within max_d of the target), per inlier, with rotation about their centroid scaled by
// 5 cm. The overlap statistics cannot see a slide: on the ellipsoid test head with side views at 65 deg and no depth
// beyond 60 deg incidence, ICP slid 20-25 deg along the smooth shared patch, even when started at the truth, with
// overlap fitness 0.94-0.99 and free space <= 0.08. Those slides score 0.0004-0.0011; rec4's accepted side edges
// 0.0042-0.0067, the textured synthetic head's 65 deg side edges 0.0049-0.013.
double overlap_constraint( const fr_cloud & source, const PyramidLevel & target, const Eigen::Matrix4d & target_from_source,
                           float max_d )
{
    const Eigen::Matrix3d r = target_from_source.block< 3, 3 >( 0, 0 );
    const Eigen::Vector3d t = target_from_source.block< 3, 1 >( 0, 3 );
    std::vector< std::pair< Eigen::Vector3d, Eigen::Vector3d > > inliers;
    for( size_t i = 0; i < source.size(); ++i )
    {
        const Eigen::Vector3d q = r * source.point( i ).cast< double >() + t;
        const float qf[3] = { float( q.x() ), float( q.y() ), float( q.z() ) };
        uint32_t j;
        float d2;
        if( target.index->nearest( qf, max_d * max_d, j, d2 ) )
            inliers.push_back( { q, target.cloud.normal( j ).cast< double >() } );
    }
    if( inliers.size() < 6 )
        return 0;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for( const auto & e : inliers )
        centroid += e.first;
    centroid /= double( inliers.size() );
    Matrix6d h = Matrix6d::Zero();
    for( const auto & e : inliers )
    {
        Vector6d j;
        j.head< 3 >() = ( e.first - centroid ).cross( e.second ) / 0.05;
        j.tail< 3 >() = e.second;
        h += j * j.transpose();
    }
    return Eigen::SelfAdjointEigenSolver< Matrix6d >( h / double( inliers.size() ) ).eigenvalues()( 0 );
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
        std::vector< ViewMap > views( n );   // where each camera measured surface (overlap test)
        std::vector< char > usable( n, 0 );
        std::atomic< size_t > built { 0 };
        parallel_for( n, [&]( size_t i ) {
            if( prog.cancelled.load() )
                return;
            const fr_cloud cloud = cloud_from_rgbd( frames[i], p.cloud );
            if( cloud.size() >= kMinFramePoints )
            {
                views[i] = build_view_map( cloud, frames[i].intrinsics );
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
        const float cos_max_view_angle = float( std::cos( deg2rad( p.cloud.max_view_angle_deg ) ) );
        const float point_voxel = std::max( p.icp.voxel_size_m, 1e-4f );
        const float overlap_inlier_m = kOverlapInlierFraction * p.icp.max_correspondence_m;
        std::atomic< size_t > done { 0 };
        parallel_for( edges.size(), [&]( size_t k ) {
            if( prog.cancelled.load() )
                return;
            fr::Candidate & c = edges[k];
            auto attempt = [&]( const Eigen::Matrix4d & init, IcpOutput & result, OverlapCheck & overlap ) {
                overlap = OverlapCheck();
                if( icp_pyramids( pyramids[c.source], pyramids[c.target], init, p.icp, result, &prog.cancelled ) != FR_OK )
                    return false;
                // Judge the correction by how far it moves the observed surface: a few degrees about the head,
                // 0.6 m in front of the camera, already mean centimeters of translation in the camera frame.
                const Eigen::Matrix4d correction = rigid_inverse( init ) * result.target_from_source;
                const Eigen::Vector3d centroid = centroids[c.source];
                const Eigen::Vector3d moved = correction.block< 3, 3 >( 0, 0 ) * centroid + correction.block< 3, 1 >( 0, 3 );
                const bool plausible = rotation_angle( correction.block< 3, 3 >( 0, 0 ) ) < deg2rad( kMaxCorrectionDeg )
                                    && ( moved - centroid ).norm() < kMaxCorrectionM;
                if( ! plausible || result.inlier_rmse > p.max_rmse_m || ! result.information.allFinite() )
                    return false;
                if( result.fitness >= p.min_fitness )
                    return true;
                // Fitness counts inliers among all source points, so it also drops when the views share little
                // surface: a view 60-75 deg from the front shares 6-10 % of its cloud with a frontal cloud that holds
                // only the face (rec4's correct side edges: 0.065-0.077). Judge instead what each camera could have
                // measured of the other's surface, in both directions; then make sure that surface pins the pose.
                const PyramidLevel & src = *pyramids[c.source].levels[0];
                const PyramidLevel & dst = *pyramids[c.target].levels[0];
                overlap.evaluated = true;
                overlap.forward = projective_overlap( src.cloud, dst.cloud, *dst.index, views[c.target],
                                                      result.target_from_source, overlap_inlier_m, kFreeSpaceTolerance,
                                                      cos_max_view_angle, frames[c.source].intrinsics.fx, point_voxel,
                                                      p.cloud.stride );
                overlap.backward = projective_overlap( dst.cloud, src.cloud, *src.index, views[c.source],
                                                       rigid_inverse( result.target_from_source ), overlap_inlier_m,
                                                       kFreeSpaceTolerance, cos_max_view_angle, frames[c.target].intrinsics.fx,
                                                       point_voxel, p.cloud.stride );
                if( ! overlap_fits( overlap.forward, overlap.backward ) )
                    return false;
                overlap.constraint = overlap_constraint( src.cloud, dst, result.target_from_source, overlap_inlier_m );
                overlap.accepted = overlap.constraint >= kMinOverlapConstraint;
                return overlap.accepted;
            };
            const Eigen::Matrix4d init = rigid_inverse( initial[c.target] ) * initial[c.source];
            IcpOutput result;
            OverlapCheck overlap;
            bool accepted = attempt( init, result, overlap );
            if( group[c.source] == group[c.target] )
            {
                // Within a pose group the head is held still: the identity rotation (centroids aligned) is often a
                // better start than two noisy Vision estimates. Keep whichever converges better.
                Eigen::Matrix4d still = Eigen::Matrix4d::Identity();
                still.block< 3, 1 >( 0, 3 ) = centroids[c.target] - centroids[c.source];
                IcpOutput other;
                OverlapCheck other_overlap;
                const bool other_accepted = attempt( still, other, other_overlap );
                const bool better = other_accepted > accepted
                                 || ( other_accepted == accepted && other.fitness / std::max( other.inlier_rmse, 1e-4f )
                                                                    > result.fitness / std::max( result.inlier_rmse, 1e-4f ) );
                if( better && other.correspondences > 0 )
                {
                    result = other;
                    overlap = other_overlap;
                    accepted = other_accepted;
                }
            }
            c.result = result;
            c.overlap = overlap;
            c.accepted = accepted;
            c.status = result.correspondences > 0 ? FR_OK : FR_ERR_NOT_ENOUGH_DATA;
            prog.report( 0.25f + 0.6f * float( ++done ) / float( std::max< size_t >( 1, edges.size() ) ), "pairwise registration" );
        } );
        if( prog.cancelled )
            return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );

        // A sliding partial overlap can satisfy the absolute thresholds; it still fits worse than the typical edge,
        // whose RMSE reflects the depth noise. The edges accepted by fitness and those accepted by the overlap test
        // get a limit each, from their own median: overlap edges fit at about 1 mm (rec4: 0.89-1.10 mm; 0.95-1.04 mm
        // in a replay of the whole recording rec4 was cut from). In one shared median they raised that replay's limit
        // from 0.81 to 1.46 mm and let its two down frames in through edges fitting at 1.06-1.19 mm, 36° below the
        // front instead of about 42° (a doubled forehead); held to the fitness edges' 0.81 mm, every overlap edge of
        // the replay would go (3 usable keyframes, as without the overlap test). Their own limit (1.43 mm on rec4,
        // 1.46 mm on the replay) keeps them all and still rejects the replay's 5->2 at 1.6 mm, which missed the
        // overlap test only by its reverse fitness (0.841). The split rejects the replay's wrong down pose only
        // because its within-group edges outnumber the cross-group ones: rec4's correct down edges fit at 1.08-1.25 mm,
        // no better (docs/RECONSTRUCTION.md, "Known limits").
        auto limit_rmse = [&]( bool overlap, size_t min_edges ) {
            std::vector< float > rmse;
            for( const auto & c : edges )
                if( c.accepted && c.overlap.accepted == overlap )
                    rmse.push_back( c.result.inlier_rmse );
            if( rmse.size() < min_edges )
                return;
            std::nth_element( rmse.begin(), rmse.begin() + std::ptrdiff_t( rmse.size() / 2 ), rmse.end() );
            const float limit = kRelativeRmseLimit * rmse[rmse.size() / 2];
            for( auto & c : edges )
                if( c.accepted && c.overlap.accepted == overlap && c.result.inlier_rmse > limit )
                    c.accepted = false;
        };
        limit_rmse( false, 5 );
        limit_rmse( true, 3 );
        if( prog.cancelled )
            return fail( FR_ERR_CANCELLED, "fr_register_frames: cancelled" );

        if( std::getenv( "FR_DEBUG_REGISTRATION" ) )
            for( const auto & c : edges )
            {
                char overlap[160] = "";
                if( c.overlap.evaluated )
                    std::snprintf( overlap, sizeof( overlap ),
                                   " | overlap fit %.3f/%.3f area %.1f/%.1f cm2 free-space %.3f/%.3f constraint %.4f "
                                   "by-overlap %d",
                                   c.overlap.forward.fitness(), c.overlap.backward.fitness(), c.overlap.forward.inlier_area * 1e4,
                                   c.overlap.backward.inlier_area * 1e4, c.overlap.forward.violation_ratio(),
                                   c.overlap.backward.violation_ratio(), c.overlap.constraint, int( c.overlap.accepted ) );
                std::fprintf( stderr, "edge %u->%u kind %d angle %.1f status %d fit %.3f rmse %.3f mm accepted %d%s\n",
                              c.source, c.target, int( c.kind ), rad2deg( c.angle ), int( c.status ), c.result.fitness,
                              c.result.inlier_rmse * 1000, int( c.accepted ), overlap );
            }
        // Certain spanning structure: odometry, plus per group the bridge that fits best after ICP (a pre-chosen
        // bridge may have slid into a wrong minimum that still passes the thresholds), repaired with the best
        // accepted alternatives where a group or frame stays disconnected. Ranked by correspondences, not fitness:
        // fitness is relative to the source cloud, and in rec4 the 1.7k-point eye-band strips of two down frames
        // (fitness 0.78-0.84) outranked the full third down frame (12.9k points, fitness 0.43: four times their
        // correspondences) and put the down group 36 deg below the front instead of 42.6 deg.
        auto quality = []( const fr::Candidate & c ) {
            return float( c.result.correspondences ) / std::max( c.result.inlier_rmse, 1e-4f );
        };
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

fr_status fr_frame_overlap( const fr_rgbd_frame * source, const fr_rgbd_frame * target, const float target_from_source[16],
                            const fr_cloud_params * cloud_params, float voxel_m, float max_correspondence_m,
                            fr_overlap_result * out )
{
    return fr::guarded( "fr_frame_overlap", [&]() -> fr_status {
        using namespace fr;
        if( ! source || ! target || ! target_from_source || ! out )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: NULL argument" );
        std::memset( out, 0, sizeof( *out ) );
        fr_cloud_params p;
        if( cloud_params )
            p = *cloud_params;
        else
            fr_cloud_params_default( &p );
        std::string error;
        if( ! validate_frame( *source, error ) || ! validate_frame( *target, error ) || ! validate_cloud_params( p, error ) )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: " + error );
        if( ! ( voxel_m >= 0 ) || ! std::isfinite( voxel_m ) )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: voxel_m must be >= 0" );
        if( ! ( max_correspondence_m > 0 ) || ! std::isfinite( max_correspondence_m ) )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: max_correspondence_m must be positive" );
        if( ! all_finite( target_from_source, 16 ) )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: transform is not finite" );
        const Eigen::Matrix4d t = load_matrix4( target_from_source );
        // Rigid: the reverse direction uses rigid_inverse, which transposes the rotation. 1e-3 leaves room for float
        // products of poses.
        const Eigen::Matrix3d rotation = t.block< 3, 3 >( 0, 0 );
        if( ( rotation.transpose() * rotation - Eigen::Matrix3d::Identity() ).cwiseAbs().maxCoeff() > 1e-3
            || rotation.determinant() <= 0 )
            return fail( FR_ERR_INVALID_ARGUMENT, "fr_frame_overlap: the transform's rotation is not a proper rotation (orthonormal, determinant +1)" );
        const fr_rgbd_frame * frames[2] = { source, target };
        fr_cloud clouds[2];
        ViewMap views[2];
        for( int k = 0; k < 2; ++k )
        {
            const fr_cloud raw = cloud_from_rgbd( *frames[k], p );
            views[k] = build_view_map( raw, frames[k]->intrinsics );
            clouds[k] = voxel_m > 0 ? voxel_downsample( raw, voxel_m ) : raw;
        }
        if( clouds[0].size() == 0 || clouds[1].size() == 0 )
            return FR_OK;
        const CloudIndex index0( clouds[0] ), index1( clouds[1] );
        const float cos_max = float( std::cos( deg2rad( p.max_view_angle_deg ) ) );
        const OverlapStats stats[2] = {
            projective_overlap( clouds[0], clouds[1], index1, views[1], t, max_correspondence_m, kFreeSpaceTolerance,
                                cos_max, source->intrinsics.fx, voxel_m, p.stride ),
            projective_overlap( clouds[1], clouds[0], index0, views[0], rigid_inverse( t ), max_correspondence_m,
                                kFreeSpaceTolerance, cos_max, target->intrinsics.fx, voxel_m, p.stride ),
        };
        for( int k = 0; k < 2; ++k )
        {
            out->fitness[k] = stats[k].fitness();
            out->free_space_ratio[k] = stats[k].violation_ratio();
            out->expected[k] = uint32_t( std::min< size_t >( stats[k].expected, UINT32_MAX ) );
            out->inliers[k] = uint32_t( std::min< size_t >( stats[k].inliers, UINT32_MAX ) );
            out->inlier_area_m2[k] = float( stats[k].inlier_area );
        }
        return FR_OK;
    } );
}

int32_t fr_overlap_fits( const fr_overlap_result * r )
{
    if( ! r )
        return 0;
    const bool fits = fr::overlap_fits( std::min( r->fitness[0], r->fitness[1] ),
                                        std::max( r->free_space_ratio[0], r->free_space_ratio[1] ),
                                        std::min( r->inlier_area_m2[0], r->inlier_area_m2[1] ) );
    return fits ? 1 : 0;
}

}  // extern "C"
