// License: Apache 2.0. See LICENSE file in root directory.
//
// What ends a helper session, in one kqueue: the app process exiting (EVFILT_PROC NOTE_EXIT on --parent-pid),
// SIGINT/SIGTERM/SIGHUP (EVFILT_SIGNAL; the signals themselves are ignored so nothing runs in a signal handler),
// and wake-ups from the connection threads (EVFILT_USER: a command arrived or the socket broke).

#pragma once

#include <string>
#include <sys/types.h>

namespace ifs
{

class Lifecycle
{
public:
    // Ignores SIGINT/SIGTERM/SIGHUP/SIGPIPE and starts watching them. Throws std::runtime_error.
    Lifecycle();
    ~Lifecycle();

    Lifecycle( const Lifecycle & ) = delete;
    Lifecycle & operator=( const Lifecycle & ) = delete;

    // Starts watching a process; a process that is already gone ends the session immediately.
    void watch_parent( pid_t pid );

    // Thread-safe.
    void wake();

    enum class Event
    {
        timeout,
        wake,
        terminate,  // parent exited or a termination signal arrived; see reason()
    };

    // Waits up to timeout_ms (-1: forever). A pending termination always wins over a wake-up.
    Event wait( int timeout_ms );
    bool terminating() const { return terminating_; }
    const std::string & reason() const { return reason_; }

private:
    int kq_ = -1;
    bool terminating_ = false;
    std::string reason_;
};

}  // namespace ifs
