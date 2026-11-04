// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_connection.h"

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <vector>


namespace ifs
{

namespace
{
// How often a writer waiting for an app that does not read checks whether finish() gave up on it.
constexpr timeval kSendTimeout{ 0, 100 * 1000 };
}  // namespace

Connection::Connection( UniqueFd sock, std::function< void() > wake )
    : sock_( std::move( sock ) )
    , wake_( std::move( wake ) )
{
    if( sock_.valid() )
        ::setsockopt( sock_.get(), SOL_SOCKET, SO_SNDTIMEO, &kSendTimeout, sizeof( kSendTimeout ) );
}

Connection::~Connection()
{
    finish( std::chrono::milliseconds( 0 ) );
}

void Connection::start()
{
    reader_ = std::thread( [this] { reader_loop(); } );
    writer_ = std::thread( [this] { writer_loop(); } );
}

bool Connection::send( uint16_t type, const nlohmann::json & body )
{
    OutMessage msg;
    if( ! make_message( msg, type, body ) )
        return false;
    return push( std::move( msg ) );
}

void Connection::send_frameset( OutMessage && msg )
{
    msg.frameset = true;
    push( std::move( msg ) );
}

void Connection::set_lossless_framesets( bool lossless )
{
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        lossless_ = lossless;
    }
    room_cv_.notify_all();
}

bool Connection::lossless_framesets() const
{
    std::lock_guard< std::mutex > lock( mutex_ );
    return lossless_;
}

Connection::EngineCall::EngineCall( Connection & connection )
    : connection_( connection )
{
    {
        std::lock_guard< std::mutex > lock( connection_.mutex_ );
        ++connection_.engine_calls_;
    }
    connection_.room_cv_.notify_all();
}

Connection::EngineCall::~EngineCall()
{
    std::lock_guard< std::mutex > lock( connection_.mutex_ );
    --connection_.engine_calls_;
}

bool Connection::push( OutMessage && msg )
{
    OutMessage dropped;  // released outside the lock: it may hold the last reference to a camera frame
    {
        std::unique_lock< std::mutex > lock( mutex_ );
        if( msg.frameset && lossless_ )
            room_cv_.wait( lock, [this] {
                return closing_ || broken_ || ! lossless_ || engine_calls_ > 0
                    || pending_framesets_ < kMaxPendingFramesets;
            } );
        if( closing_ || broken_ )
        {
            lock.unlock();
            msg = OutMessage();
            return false;
        }
        if( msg.frameset )
        {
            const size_t limit = lossless_ && engine_calls_ > 0 ? kMaxPendingFramesetsInEngineCall
                                                                : kMaxPendingFramesets;
            if( pending_framesets_ >= limit )
            {
                for( auto it = out_.begin(); it != out_.end(); ++it )
                {
                    if( it->frameset )
                    {
                        dropped = std::move( *it );
                        out_.erase( it );
                        --pending_framesets_;
                        ++dropped_framesets_;
                        break;
                    }
                }
            }
            ++pending_framesets_;
        }
        out_.push_back( std::move( msg ) );
    }
    out_cv_.notify_one();
    return true;
}

std::optional< InMessage > Connection::pop_command()
{
    std::lock_guard< std::mutex > lock( mutex_ );
    if( in_.empty() )
        return std::nullopt;
    InMessage msg = std::move( in_.front() );
    in_.pop_front();
    in_bytes_ -= msg.json_bytes;
    return msg;
}

bool Connection::broken() const
{
    std::lock_guard< std::mutex > lock( mutex_ );
    return broken_;
}

std::string Connection::broken_reason() const
{
    std::lock_guard< std::mutex > lock( mutex_ );
    return broken_reason_;
}

uint64_t Connection::framesets_sent() const
{
    std::lock_guard< std::mutex > lock( mutex_ );
    return sent_framesets_;
}

uint64_t Connection::framesets_dropped() const
{
    std::lock_guard< std::mutex > lock( mutex_ );
    return dropped_framesets_;
}

void Connection::drop_pending_framesets()
{
    std::deque< OutMessage > dropped;
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        for( auto it = out_.begin(); it != out_.end(); )
        {
            if( it->frameset )
            {
                dropped.push_back( std::move( *it ) );
                it = out_.erase( it );
                ++dropped_framesets_;
            }
            else
                ++it;
        }
        pending_framesets_ = 0;
    }
    room_cv_.notify_all();
}

void Connection::mark_broken( const std::string & reason )
{
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        if( ! broken_ )
        {
            broken_ = true;
            broken_reason_ = reason;
        }
    }
    out_cv_.notify_all();
    room_cv_.notify_all();
    if( wake_ )
        wake_();
}

void Connection::reader_loop()
{
    for( ;; )
    {
        InMessage msg;
        std::string error;
        ReadResult r = read_message( sock_.get(), msg, error );
        if( r == ReadResult::eof )
        {
            mark_broken( "the app closed the connection" );
            return;
        }
        if( r == ReadResult::error )
        {
            mark_broken( "protocol error: " + error );
            return;
        }
        if( ! msg.has_request_id )
        {
            // Cannot be answered, so the peer does not speak the protocol.
            mark_broken( "protocol error: message without a valid request_id" );
            return;
        }
        bool overflow = false;
        {
            std::lock_guard< std::mutex > lock( mutex_ );
            if( closing_ || broken_ )
                return;
            if( in_.size() >= kMaxPendingCommands || in_bytes_ + msg.json_bytes > kMaxPendingCommandBytes )
                overflow = true;
            else
            {
                in_bytes_ += msg.json_bytes;
                in_.push_back( std::move( msg ) );
            }
        }
        if( overflow )
        {
            mark_broken( "protocol error: too many unanswered commands" );
            return;
        }
        if( wake_ )
            wake_();
    }
}

void Connection::writer_loop()
{
    std::unique_lock< std::mutex > lock( mutex_ );
    for( ;; )
    {
        out_cv_.wait( lock, [this] { return ! out_.empty() || closing_ || broken_; } );
        if( broken_ )
            break;
        if( out_.empty() )
        {
            if( closing_ )
                break;
            continue;
        }
        OutMessage msg = std::move( out_.front() );
        out_.pop_front();
        const bool frameset = msg.frameset;
        if( frameset )
            --pending_framesets_;
        writing_ = true;
        lock.unlock();
        if( frameset )
            room_cv_.notify_all();
        const bool ok = write( msg );
        msg = OutMessage();
        lock.lock();
        writing_ = false;
        if( ! ok )
        {
            if( ! broken_ )
            {
                broken_ = true;
                broken_reason_ = "the app stopped reading (socket write failed)";
            }
            break;
        }
        if( frameset )
            ++sent_framesets_;
        if( out_.empty() )
            drained_cv_.notify_all();
    }
    // Whatever is left can no longer be delivered; free the frames now rather than at destruction.
    std::deque< OutMessage > rest;
    rest.swap( out_ );
    pending_framesets_ = 0;
    writer_done_ = true;
    const bool notify = broken_ && ! closing_;
    lock.unlock();
    drained_cv_.notify_all();
    room_cv_.notify_all();
    rest.clear();
    if( notify && wake_ )
        wake_();
}

// write_message() that can be abandoned: shutdown() does not wake a thread blocked in a send on a macOS AF_UNIX socket
// whose peer has already shut its end down (e.g. the app half-closed the connection while it was not reading), and
// MSG_DONTWAIT is ignored there, so a blocking write could keep finish() - and the helper's exit - waiting forever.
// The socket has a short send timeout (constructor); a send that timed out is retried until finish() gives up.
bool Connection::write( const OutMessage & msg )
{
    const int sock = sock_.get();
    std::vector< iovec > iov;
    iov.reserve( 2 + msg.payload.size() );
    iov.push_back( { const_cast< ifs_wire_header * >( &msg.header ), sizeof( msg.header ) } );
    if( ! msg.json.empty() )
        iov.push_back( { const_cast< char * >( msg.json.data() ), msg.json.size() } );
    for( const iovec & v : msg.payload )
        if( v.iov_len )
            iov.push_back( v );

    size_t index = 0;
    while( index < iov.size() )
    {
        const int count = int( std::min< size_t >( iov.size() - index, IOV_MAX ) );
        const ssize_t n = ::writev( sock, &iov[index], count );
        if( n < 0 )
        {
            if( errno == EINTR )
                continue;
            if( ( errno == EAGAIN || errno == EWOULDBLOCK ) && ! abort_writes_.load() )
                continue;  // the send timeout passed without room: the app is slow, not gone
            return false;
        }
        size_t left = size_t( n );
        while( index < iov.size() && left >= iov[index].iov_len )
        {
            left -= iov[index].iov_len;
            ++index;
        }
        if( left )
        {
            iov[index].iov_base = static_cast< char * >( iov[index].iov_base ) + left;
            iov[index].iov_len -= left;
        }
    }
    return true;
}

void Connection::finish( std::chrono::milliseconds timeout )
{
    {
        std::unique_lock< std::mutex > lock( mutex_ );
        if( finished_ )
            return;
        finished_ = true;
        closing_ = true;
        out_cv_.notify_all();
        room_cv_.notify_all();
        if( writer_.joinable() )
            drained_cv_.wait_for( lock, timeout, [this] { return writer_done_; } );
    }
    // A writer waiting for an app that stopped reading gives up; shutdown() wakes a reader blocked in recvmsg().
    abort_writes_ = true;
    if( sock_.valid() )
        ::shutdown( sock_.get(), SHUT_RDWR );
    if( reader_.joinable() )
        reader_.join();
    if( writer_.joinable() )
        writer_.join();
    sock_.reset();
}

}  // namespace ifs
