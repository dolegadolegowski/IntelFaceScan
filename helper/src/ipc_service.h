// License: Apache 2.0. See LICENSE file in root directory.
//
// The command side of the helper: validates each command of docs/PROTOCOL.md, runs it on the CaptureEngine and
// answers it exactly once; turns engine callbacks into FRAMESET / EVENT / LOG messages; owns the recording hand-over
// (security_recording.h). Runs on the main thread only, except the engine callbacks.

#pragma once

#include "capture_engine.h"
#include "ipc_connection.h"
#include "security_recording.h"

#include <functional>
#include <memory>
#include <string>

namespace ifs
{

enum class HelperMode
{
    live,
    playback,
    fake,
};

const char * mode_name( HelperMode mode );

class HelperService
{
public:
    using EngineFactory = std::function< std::unique_ptr< CaptureEngine >( CaptureCallbacks ) >;

    HelperService( Connection & connection, HelperMode mode, EngineFactory factory, bool playback_real_time );
    ~HelperService();

    HelperService( const HelperService & ) = delete;
    HelperService & operator=( const HelperService & ) = delete;

    void send_hello();
    // Creates the engine now so device problems are reported right away; a failure is logged and retried by the
    // next command (e.g. after the user plugged the camera in).
    void create_engine();

    // Answers the command (exactly one REPLY when it has a request_id).
    void handle( InMessage & msg );
    bool quit_requested() const { return quit_; }

    // Ends a running recording (the file is still delivered), shuts the engine down and destroys it. After this no
    // engine callback can run.
    void shutdown_engine();

    void log( const std::string & level, const std::string & message );

private:
    nlohmann::json dispatch( InMessage & msg );
    CaptureEngine & engine();
    CaptureCallbacks callbacks();
    void reply( uint64_t request_id, bool ok, const nlohmann::json & data, const std::string & error );
    void on_frameset( Frameset && fs );
    void on_event( const std::string & name, const nlohmann::json & data );

    nlohmann::json record_start( InMessage & msg );
    nlohmann::json record_stop();

    Connection & connection_;
    HelperMode mode_;
    EngineFactory factory_;
    bool playback_real_time_;
    std::unique_ptr< CaptureEngine > engine_;
    std::string engine_error_;
    bool quit_ = false;

    struct Recording
    {
        std::unique_ptr< RecordingSpool > spool;
        UniqueFd destination;
    };
    std::unique_ptr< Recording > recording_;
};

}  // namespace ifs
