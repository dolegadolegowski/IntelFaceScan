// License: Apache 2.0. See LICENSE file in root directory.
//
// The helper's side of the connection: a reader thread that receives and validates commands, and a writer thread
// that sends replies, events, logs and framesets from one ordered queue. Replies, events and logs are never dropped.
// Framesets (docs/PROTOCOL.md, backpressure): at most kMaxPendingFramesets wait to be sent. Lossy delivery (live
// camera, real-time playback) replaces the oldest with a newer one when the app is slower than the camera; lossless
// delivery (non-real-time playback) makes send_frameset() wait for room instead, except while the command thread is
// inside a capture-engine call (EngineCall): the engine may wait for the very thread that is sending (stop() joins its
// processing thread, a paused playback flushes its reader), so the frameset is then queued beyond the limit.

#pragma once

#include "ipc_wire.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace ifs
{

class Connection
{
public:
    static constexpr size_t kMaxPendingFramesets = 2;
    // Lossless delivery during an engine call: framesets are queued without waiting up to this many (the engine
    // produces only the few already in flight while it stops or pauses), then the oldest is dropped after all.
    static constexpr size_t kMaxPendingFramesetsInEngineCall = 8;
    // Commands queue up while a slow one runs (advanced mode re-enumerates the camera for seconds) and a UI may send
    // option changes at display rate meanwhile; far beyond that the peer is not the app.
    static constexpr size_t kMaxPendingCommands = 1024;
    static constexpr size_t kMaxPendingCommandBytes = 16 * 1024 * 1024;

    // 'wake' is called from the reader/writer threads whenever a command arrived or the connection broke.
    Connection( UniqueFd sock, std::function< void() > wake );
    ~Connection();

    Connection( const Connection & ) = delete;
    Connection & operator=( const Connection & ) = delete;

    void start();

    // Thread-safe and never waits. Returns false when the message exceeds the protocol limits (it is not sent) or the
    // connection is closing or broken.
    bool send( uint16_t type, const nlohmann::json & body );
    // Thread-safe. Lossy: queues the frameset, replacing the oldest queued one when kMaxPendingFramesets wait.
    // Lossless: first waits until fewer than kMaxPendingFramesets wait, the connection is closing or broken, or an
    // engine call starts. A frameset that cannot be sent any more is released right away.
    void send_frameset( OutMessage && msg );

    // Lossless frameset delivery on or off (off by default). Thread-safe.
    void set_lossless_framesets( bool lossless );
    bool lossless_framesets() const;

    // Marks a call into the capture engine on the command thread for its lifetime: no send_frameset() waits meanwhile,
    // so the engine can always stop or pause its threads. Nestable.
    class EngineCall
    {
    public:
        explicit EngineCall( Connection & connection );
        ~EngineCall();
        EngineCall( const EngineCall & ) = delete;
        EngineCall & operator=( const EngineCall & ) = delete;

    private:
        Connection & connection_;
    };

    std::optional< InMessage > pop_command();

    // True once the peer closed the connection, a protocol violation was detected, or a write failed.
    bool broken() const;
    std::string broken_reason() const;

    // Statistics for the log.
    uint64_t framesets_sent() const;
    uint64_t framesets_dropped() const;

    // Drops framesets still waiting to be sent (used before BYE so it is not delayed by stale frames).
    void drop_pending_framesets();

    // Sends what is queued (waiting at most 'timeout' for the app to read it), then shuts the socket down and
    // joins both threads, also when the app stopped reading. Nothing can be sent afterwards.
    void finish( std::chrono::milliseconds timeout );

private:
    void reader_loop();
    void writer_loop();
    bool write( const OutMessage & msg );
    void mark_broken( const std::string & reason );
    bool push( OutMessage && msg );

    UniqueFd sock_;
    std::function< void() > wake_;

    mutable std::mutex mutex_;
    std::condition_variable out_cv_;      // writer: queue not empty / closing
    std::condition_variable drained_cv_;  // finish(): queue drained
    std::condition_variable room_cv_;     // lossless send_frameset(): room, engine call, lossy, closing or broken
    std::deque< OutMessage > out_;
    size_t pending_framesets_ = 0;        // queued, not yet being written
    bool lossless_ = false;
    int engine_calls_ = 0;
    bool writing_ = false;
    bool closing_ = false;
    bool writer_done_ = false;
    std::deque< InMessage > in_;
    size_t in_bytes_ = 0;
    bool broken_ = false;
    std::string broken_reason_;
    uint64_t sent_framesets_ = 0;
    uint64_t dropped_framesets_ = 0;

    std::thread reader_;
    std::thread writer_;
    bool finished_ = false;
    std::atomic< bool > abort_writes_{ false };  // finish() gave up waiting: a send that times out fails
};

}  // namespace ifs
