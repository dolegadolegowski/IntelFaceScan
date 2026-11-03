// License: Apache 2.0. See LICENSE file in root directory.
//
// The helper's side of the connection: a reader thread that receives and validates commands, and a writer thread
// that sends replies, events, logs and framesets from one ordered queue. Framesets are the only messages that may
// be dropped: at most kMaxPendingFramesets wait to be sent, a newer one replaces the oldest (backpressure when the
// app is slower than the camera); replies, events and logs are never dropped.

#pragma once

#include "ipc_wire.h"

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
    // A peer that queues this many unanswered commands is not the app (which waits for its replies).
    static constexpr size_t kMaxPendingCommands = 64;

    // 'wake' is called from the reader/writer threads whenever a command arrived or the connection broke.
    Connection( UniqueFd sock, std::function< void() > wake );
    ~Connection();

    Connection( const Connection & ) = delete;
    Connection & operator=( const Connection & ) = delete;

    void start();

    // Thread-safe. Returns false when the message exceeds the protocol limits (it is not sent) or the connection
    // is closing or broken.
    bool send( uint16_t type, const nlohmann::json & body );
    void send_frameset( OutMessage && msg );

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
    // joins both threads. Nothing can be sent afterwards.
    void finish( std::chrono::milliseconds timeout );

private:
    void reader_loop();
    void writer_loop();
    void mark_broken( const std::string & reason );
    bool push( OutMessage && msg );

    UniqueFd sock_;
    std::function< void() > wake_;

    mutable std::mutex mutex_;
    std::condition_variable out_cv_;      // writer: queue not empty / closing
    std::condition_variable drained_cv_;  // finish(): queue drained
    std::deque< OutMessage > out_;
    size_t pending_framesets_ = 0;
    bool writing_ = false;
    bool closing_ = false;
    bool writer_done_ = false;
    std::deque< InMessage > in_;
    bool broken_ = false;
    std::string broken_reason_;
    uint64_t sent_framesets_ = 0;
    uint64_t dropped_framesets_ = 0;

    std::thread reader_;
    std::thread writer_;
    bool finished_ = false;
};

}  // namespace ifs
