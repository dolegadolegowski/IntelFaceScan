// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_connection.h"

#include <sys/socket.h>


namespace ifs
{

Connection::Connection( UniqueFd sock, std::function< void() > wake )
    : sock_( std::move( sock ) )
    , wake_( std::move( wake ) )
{
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

bool Connection::push( OutMessage && msg )
{
    OutMessage dropped;  // released outside the lock: it may hold the last reference to a camera frame
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        if( closing_ || broken_ )
            return false;
        if( msg.frameset )
        {
            if( pending_framesets_ >= kMaxPendingFramesets )
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
        const bool ok = write_message( sock_.get(), msg );
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
    rest.clear();
    if( notify && wake_ )
        wake_();
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
        if( writer_.joinable() )
            drained_cv_.wait_for( lock, timeout, [this] { return writer_done_; } );
    }
    // Wakes a reader blocked in recvmsg() and a writer blocked on an app that stopped reading.
    if( sock_.valid() )
        ::shutdown( sock_.get(), SHUT_RDWR );
    if( reader_.joinable() )
        reader_.join();
    if( writer_.joinable() )
        writer_.join();
    sock_.reset();
}

}  // namespace ifs
