// License: Apache 2.0. See LICENSE file in root directory.
//
// Post-processing of synchronized framesets (see capture_processing.h).

#include "capture_processing.h"

#include <algorithm>
#include <cmath>
#include <utility>

using nlohmann::json;

namespace
{
void set_block_option( rs2::processing_block & block, rs2_option option, float value )
{
    // Values were validated against the librealsense ranges when the configuration was parsed.
    if( block.supports( option ) )
        block.set_option( option, value );
}

json vec3( const std::array< float, 3 > & v )
{
    return json::array( { v[0], v[1], v[2] } );
}
}  // namespace

namespace ifs
{

void ProcessingControl::set( const ProcessingSettings & settings )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    m_settings = settings;
    ++m_version;
}

void ProcessingControl::set_filters( const FilterConfig & filters )
{
    std::lock_guard< std::mutex > lock( m_mutex );
    m_settings.filters = filters;
    ++m_version;
}

ProcessingSettings ProcessingControl::get() const
{
    std::lock_guard< std::mutex > lock( m_mutex );
    return m_settings;
}

void ImuState::on_frame( const rs2::frame & frame )
{
    auto motion = frame.as< rs2::motion_frame >();
    if( ! motion )
        return;
    rs2_vector v = motion.get_motion_data();
    std::array< float, 3 > sample{ v.x, v.y, v.z };
    rs2_stream stream = frame.get_profile().stream_type();
    std::lock_guard< std::mutex > lock( m_mutex );
    if( stream == RS2_STREAM_ACCEL )
        m_accel = sample;
    else if( stream == RS2_STREAM_GYRO )
        m_gyro = sample;
    else
        return;
    m_timestamp_ms = frame.get_timestamp();
}

void ImuState::reset()
{
    std::lock_guard< std::mutex > lock( m_mutex );
    m_accel.reset();
    m_gyro.reset();
    m_timestamp_ms = 0;
}

json ImuState::snapshot() const
{
    std::lock_guard< std::mutex > lock( m_mutex );
    if( ! m_accel && ! m_gyro )
        return nullptr;
    return json{ { "accel", m_accel ? vec3( *m_accel ) : json( nullptr ) },
                 { "gyro", m_gyro ? vec3( *m_gyro ) : json( nullptr ) },
                 { "timestamp_ms", m_timestamp_ms } };
}

FramesetPairer::FramesetPairer( std::vector< std::pair< rs2_stream, int > > expected )
    : m_expected( std::move( expected ) )
    , m_combine( [this]( rs2::frame, rs2::frame_source & source ) {
        source.frame_ready( source.allocate_composite_frame( m_parts ) );
    } )
{
    m_combine.start( m_out );
}

bool FramesetPairer::complete( const std::vector< rs2::frame > & frames ) const
{
    for( auto & want : m_expected )
    {
        bool found = false;
        for( auto & f : frames )
            if( f.get_profile().stream_type() == want.first && f.get_profile().stream_index() == want.second )
                found = true;
        if( ! found )
            return false;
    }
    return true;
}

rs2::frameset FramesetPairer::pair( const rs2::frameset & frames )
{
    std::vector< rs2::frame > parts;
    for( auto f : frames )
        parts.push_back( f );
    if( parts.empty() )
        return rs2::frameset();
    if( complete( parts ) )
    {
        m_pending.clear();
        return frames;
    }

    // Parts of one set share a timestamp (within half a frame); anything else starts a new set.
    double period_ms = 1000.0 / std::max( 1, parts.front().get_profile().fps() );
    if( ! m_pending.empty()
        && std::fabs( m_pending.front().get_timestamp() - parts.front().get_timestamp() ) > period_ms / 2 )
        m_pending.clear();
    for( auto & f : parts )
    {
        auto same_stream = [&f]( const rs2::frame & p ) {
            return p.get_profile().stream_type() == f.get_profile().stream_type()
                && p.get_profile().stream_index() == f.get_profile().stream_index();
        };
        m_pending.erase( std::remove_if( m_pending.begin(), m_pending.end(), same_stream ), m_pending.end() );
        m_pending.push_back( f );
    }
    if( ! complete( m_pending ) )
        return rs2::frameset();

    m_parts.swap( m_pending );
    m_pending.clear();
    m_combine.invoke( m_parts.front() );  // runs synchronously and composes m_parts into one frameset
    m_parts.clear();
    rs2::frame out;
    if( m_out.poll_for_frame( &out ) )
        return out.as< rs2::frameset >();
    return rs2::frameset();
}

FrameProcessor::FrameProcessor( const rs2_extrinsics & depth_to_color )
    : m_depth_to_color( depth_to_color )
{
}

void FrameProcessor::reset_history()
{
    m_temporal.reset();
    if( m_settings.filters.temporal.enabled )
        configure( m_settings );
}

void FrameProcessor::configure( const ProcessingSettings & s )
{
    const FilterConfig & f = s.filters;

    set_block_option( m_threshold, RS2_OPTION_MIN_DISTANCE, f.threshold.min_m );
    set_block_option( m_threshold, RS2_OPTION_MAX_DISTANCE, f.threshold.max_m );

    set_block_option( m_spatial, RS2_OPTION_FILTER_MAGNITUDE, float( f.spatial.magnitude ) );
    set_block_option( m_spatial, RS2_OPTION_FILTER_SMOOTH_ALPHA, f.spatial.alpha );
    set_block_option( m_spatial, RS2_OPTION_FILTER_SMOOTH_DELTA, float( f.spatial.delta ) );
    set_block_option( m_spatial, RS2_OPTION_HOLES_FILL, float( f.spatial.holes_fill ) );

    // A temporal filter that was off holds an old frame: blending a new frame into it would smear the head, so
    // it starts from scratch whenever it is switched on.
    if( ! f.temporal.enabled )
        m_temporal.reset();
    else if( ! m_temporal || ! m_settings.filters.temporal.enabled )
        m_temporal.emplace();
    if( m_temporal )
    {
        set_block_option( *m_temporal, RS2_OPTION_FILTER_SMOOTH_ALPHA, f.temporal.alpha );
        set_block_option( *m_temporal, RS2_OPTION_FILTER_SMOOTH_DELTA, float( f.temporal.delta ) );
        set_block_option( *m_temporal, RS2_OPTION_HOLES_FILL, float( f.temporal.persistence ) );
    }

    set_block_option( m_hole_filling, RS2_OPTION_HOLES_FILL, float( f.hole_filling.mode ) );
    m_settings = s;
}

std::optional< Frameset > FrameProcessor::process( const rs2::frameset & frames,
                                                   const ProcessingControl & control,
                                                   const json & imu,
                                                   uint64_t sequence )
{
    rs2::depth_frame raw_depth = frames.get_depth_frame();
    rs2::video_frame raw_color = frames.get_color_frame();
    if( ! raw_depth || ! raw_color )
        return std::nullopt;

    if( control.version() != m_version )
    {
        // Read the version first: a change made while copying is picked up with the next frameset.
        m_version = control.version();
        configure( control.get() );
    }

    // The depth-to-disparity transforms and the NEON align cache the depth scale of the first frame they see per
    // stream profile, and a change of the depth units (configure or SET_OPTION while streaming) keeps the profile:
    // the align would then reproject with a scale off by the ratio of the units (misaligned depth), and the
    // disparity-domain filters would compare disparities with thresholds off by the same ratio. New blocks start
    // from the new scale; the temporal history holds disparities of the old one.
    float units = raw_depth.get_units();
    if( m_depth_units > 0 && units != m_depth_units )
    {
        m_to_disparity = rs2::disparity_transform( true );
        m_to_depth = rs2::disparity_transform( false );
        m_align = rs2::align( RS2_STREAM_COLOR );
        reset_history();
    }
    m_depth_units = units;
    const FilterConfig & f = m_settings.filters;

    rs2::frame processed = frames;
    if( f.threshold.enabled )
        processed = m_threshold.process( processed );
    // The edge-preserving filters work on disparity, where stereo noise is uniform; depth noise grows with z^2.
    bool disparity = f.disparity_domain && ( f.spatial.enabled || f.temporal.enabled );
    if( disparity )
        processed = m_to_disparity.process( processed );
    if( f.spatial.enabled )
        processed = m_spatial.process( processed );
    if( f.temporal.enabled && m_temporal )
        processed = m_temporal->process( processed );
    if( disparity )
        processed = m_to_depth.process( processed );
    if( f.hole_filling.enabled )
        processed = m_hole_filling.process( processed );

    rs2::frameset filtered = processed.as< rs2::frameset >();
    if( ! filtered )
        return std::nullopt;
    rs2::depth_frame depth = filtered.get_depth_frame();
    if( ! depth )
        return std::nullopt;

    rs2::frameset aligned = m_align.process( filtered );
    rs2::depth_frame aligned_depth = aligned ? aligned.get_depth_frame() : rs2::depth_frame( rs2::frame() );
    if( ! aligned_depth )
        return std::nullopt;

    rs2_intrinsics color_intrinsics = raw_color.get_profile().as< rs2::video_stream_profile >().get_intrinsics();
    rs2_intrinsics aligned_intrinsics = aligned_depth.get_profile().as< rs2::video_stream_profile >().get_intrinsics();

    Frameset out;
    out.metadata = json{
        { "sequence", sequence },
        { "timestamp_ms", raw_color.get_timestamp() },
        { "timestamp_domain", timestamp_domain_name( raw_color.get_frame_timestamp_domain() ) },
        { "depth_scale", aligned_depth.get_units() },
        { "depth_to_color", extrinsics_json( m_depth_to_color ) },
        { "imu", imu },
        { "metadata", frame_metadata( raw_depth, raw_color ) },
    };

    out.planes.push_back( make_plane( "color", raw_color, color_intrinsics ) );
    out.planes.push_back( make_plane( "depth_aligned", aligned_depth, aligned_intrinsics ) );
    if( m_settings.send_depth )
        out.planes.push_back(
            make_plane( "depth", depth, depth.get_profile().as< rs2::video_stream_profile >().get_intrinsics() ) );
    if( m_settings.send_ir )
    {
        // Left imager = index 1; it shares the depth sensor's frame, so its intrinsics are the depth intrinsics.
        rs2::video_frame ir = frames.get_infrared_frame( 1 );
        if( ir && ir.get_profile().format() == RS2_FORMAT_Y8 )
            out.planes.push_back(
                make_plane( "ir_left", ir, ir.get_profile().as< rs2::video_stream_profile >().get_intrinsics() ) );
    }
    return out;
}

Plane make_plane( const char * kind, const rs2::video_frame & frame, const rs2_intrinsics & intrinsics )
{
    Plane plane;
    plane.desc = json{ { "kind", kind },
                       { "format", format_name( frame.get_profile().format() ) },
                       { "width", frame.get_width() },
                       { "height", frame.get_height() },
                       { "stride", frame.get_stride_in_bytes() },
                       { "frame_number", frame.get_frame_number() },
                       { "timestamp_ms", frame.get_timestamp() },
                       { "intrinsics", intrinsics_json( intrinsics ) } };
    plane.data = static_cast< const uint8_t * >( frame.get_data() );
    plane.bytes = size_t( frame.get_stride_in_bytes() ) * size_t( frame.get_height() );
    // The pixels belong to the librealsense frame: holding a reference keeps them valid (and the frame out of its
    // pool) until the IPC layer has sent the plane.
    plane.keepalive = std::make_shared< rs2::frame >( frame );
    return plane;
}

json frame_metadata( const rs2::frame & depth, const rs2::frame & color )
{
    json md = json::object();
    auto add = [&md]( const rs2::frame & f, rs2_frame_metadata_value key, const char * name, double scale ) {
        try
        {
            if( f && f.supports_frame_metadata( key ) )
                md[name] = double( f.get_frame_metadata( key ) ) * scale;
        }
        catch( const rs2::error & )
        {
        }
    };
    add( depth, RS2_FRAME_METADATA_ACTUAL_EXPOSURE, "depth_exposure_us", 1.0 );
    add( depth, RS2_FRAME_METADATA_GAIN_LEVEL, "depth_gain", 1.0 );
    add( depth, RS2_FRAME_METADATA_FRAME_LASER_POWER, "laser_power", 1.0 );
    add( depth, RS2_FRAME_METADATA_FRAME_EMITTER_MODE, "emitter_mode", 1.0 );
    add( depth, RS2_FRAME_METADATA_AUTO_EXPOSURE, "depth_auto_exposure", 1.0 );
    add( depth, RS2_FRAME_METADATA_FRAME_COUNTER, "depth_frame_counter", 1.0 );
    // The RGB exposure is a UVC control: 100 us units (the depth exposure is already in us).
    add( color, RS2_FRAME_METADATA_ACTUAL_EXPOSURE, "color_exposure_us", 100.0 );
    add( color, RS2_FRAME_METADATA_GAIN_LEVEL, "color_gain", 1.0 );
    add( color, RS2_FRAME_METADATA_AUTO_EXPOSURE, "color_auto_exposure", 1.0 );
    add( color, RS2_FRAME_METADATA_WHITE_BALANCE, "color_white_balance", 1.0 );
    add( color, RS2_FRAME_METADATA_FRAME_COUNTER, "color_frame_counter", 1.0 );
    return md;
}

}  // namespace ifs
