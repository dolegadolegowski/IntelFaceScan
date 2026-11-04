// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_lifecycle.h"

#include <sys/event.h>
#include <sys/time.h>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>

namespace ifs
{

namespace
{
constexpr uintptr_t kWakeIdent = 1;
constexpr int kTerminationSignals[] = { SIGINT, SIGTERM, SIGHUP };

const char * signal_name( int sig )
{
    switch( sig )
    {
    case SIGINT: return "SIGINT";
    case SIGTERM: return "SIGTERM";
    case SIGHUP: return "SIGHUP";
    }
    return "signal";
}
}  // namespace

Lifecycle::Lifecycle()
{
    // kqueue records a signal even when it is ignored, and ignoring it means the default action (exit without
    // handing the camera back) can never run.
    std::signal( SIGPIPE, SIG_IGN );
    for( int sig : kTerminationSignals )
        std::signal( sig, SIG_IGN );

    kq_ = ::kqueue();
    if( kq_ < 0 )
        throw std::runtime_error( std::string( "kqueue() failed: " ) + std::strerror( errno ) );
    ::fcntl( kq_, F_SETFD, FD_CLOEXEC );

    struct kevent changes[4];
    int n = 0;
    for( int sig : kTerminationSignals )
        EV_SET( &changes[n++], uintptr_t( sig ), EVFILT_SIGNAL, EV_ADD, 0, 0, nullptr );
    EV_SET( &changes[n++], kWakeIdent, EVFILT_USER, EV_ADD | EV_CLEAR, 0, 0, nullptr );
    if( ::kevent( kq_, changes, n, nullptr, 0, nullptr ) != 0 )
    {
        int e = errno;
        ::close( kq_ );
        throw std::runtime_error( std::string( "kevent() registration failed: " ) + std::strerror( e ) );
    }
}

Lifecycle::~Lifecycle()
{
    if( kq_ >= 0 )
        ::close( kq_ );
}

void Lifecycle::watch_parent( pid_t pid )
{
    struct kevent change;
    EV_SET( &change, uintptr_t( pid ), EVFILT_PROC, EV_ADD | EV_ONESHOT, NOTE_EXIT, 0, nullptr );
    if( ::kevent( kq_, &change, 1, nullptr, 0, nullptr ) != 0 )
    {
        terminating_ = true;
        reason_ = errno == ESRCH ? "the app is not running" : std::string( "cannot watch the app: " ) + std::strerror( errno );
    }
}

void Lifecycle::wake()
{
    struct kevent change;
    EV_SET( &change, kWakeIdent, EVFILT_USER, 0, NOTE_TRIGGER, 0, nullptr );
    ::kevent( kq_, &change, 1, nullptr, 0, nullptr );
}

Lifecycle::Event Lifecycle::wait( int timeout_ms )
{
    if( terminating_ )
        return Event::terminate;
    timespec ts{};
    timespec * tsp = nullptr;
    if( timeout_ms >= 0 )
    {
        ts.tv_sec = timeout_ms / 1000;
        ts.tv_nsec = long( timeout_ms % 1000 ) * 1000000L;
        tsp = &ts;
    }
    struct kevent events[8];
    int n;
    do
        n = ::kevent( kq_, nullptr, 0, events, 8, tsp );
    while( n < 0 && errno == EINTR );
    if( n < 0 )
    {
        terminating_ = true;
        reason_ = std::string( "kevent() failed: " ) + std::strerror( errno );
        return Event::terminate;
    }
    bool woke = false;
    for( int i = 0; i < n; ++i )
    {
        const struct kevent & ev = events[i];
        if( ev.filter == EVFILT_PROC && ( ev.fflags & NOTE_EXIT ) )
        {
            terminating_ = true;
            reason_ = "the app exited";
        }
        else if( ev.filter == EVFILT_SIGNAL )
        {
            terminating_ = true;
            reason_ = std::string( "received " ) + signal_name( int( ev.ident ) );
        }
        else if( ev.filter == EVFILT_USER )
            woke = true;
    }
    if( terminating_ )
        return Event::terminate;
    return woke ? Event::wake : Event::timeout;
}

}  // namespace ifs
