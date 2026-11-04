// License: Apache 2.0. See LICENSE file in root directory.

#include "ipc_wire.h"

#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

static_assert( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the wire header is sent in host byte order" );

namespace ifs
{

UniqueFd & UniqueFd::operator=( UniqueFd && other ) noexcept
{
    if( this != &other )
        reset( other.release() );
    return *this;
}

int UniqueFd::release()
{
    int fd = fd_;
    fd_ = -1;
    return fd;
}

int UniqueFd::reset( int fd )
{
    int result = 0;
    if( fd_ >= 0 )
        result = ::close( fd_ );
    fd_ = fd;
    return result;
}

namespace
{
// More descriptors than this in one read can only come from a misbehaving peer; they still have to be received
// (and closed) instead of being dropped by the kernel as truncated control data, so leave room.
constexpr size_t kMaxFdsPerRead = 16;

enum class Recv
{
    ok,
    eof,
    error
};

void close_all( std::vector< int > & fds )
{
    for( int fd : fds )
        ::close( fd );
    fds.clear();
}

void collect_fds( msghdr & msg, std::vector< int > & fds )
{
    for( cmsghdr * c = CMSG_FIRSTHDR( &msg ); c; c = CMSG_NXTHDR( &msg, c ) )
    {
        if( c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_RIGHTS || c->cmsg_len < CMSG_LEN( 0 ) )
            continue;
        size_t n = ( c->cmsg_len - CMSG_LEN( 0 ) ) / sizeof( int );
        const unsigned char * data = CMSG_DATA( c );
        for( size_t i = 0; i < n; ++i )
        {
            int fd;
            std::memcpy( &fd, data + i * sizeof( int ), sizeof( int ) );
            fds.push_back( fd );
        }
    }
}

// Reads exactly 'size' bytes. SCM_RIGHTS descriptors are delivered together with the first byte of the message
// they were sent with; those arriving with byte 0 of this read go to *first_fds (when the caller accepts any),
// descriptors arriving anywhere else are closed and fail the read. 'eof_ok': a close before the first byte is a
// clean end of the connection.
Recv recv_exact( int sock, uint8_t * buf, size_t size, std::vector< int > * first_fds, bool eof_ok, std::string & error )
{
    size_t done = 0;
    while( done < size )
    {
        alignas( cmsghdr ) unsigned char control[CMSG_SPACE( sizeof( int ) * kMaxFdsPerRead )];
        iovec iov{ buf + done, size - done };
        msghdr msg{};
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control;
        msg.msg_controllen = sizeof( control );
        ssize_t n = ::recvmsg( sock, &msg, 0 );
        if( n < 0 )
        {
            if( errno == EINTR )
                continue;
            error = std::string( "socket read failed: " ) + std::strerror( errno );
            return Recv::error;
        }
        std::vector< int > fds;
        collect_fds( msg, fds );
        if( msg.msg_flags & MSG_CTRUNC )
        {
            close_all( fds );
            error = "too many file descriptors in one message";
            return Recv::error;
        }
        if( ! fds.empty() )
        {
            if( done != 0 || ! first_fds )
            {
                close_all( fds );
                error = "unexpected file descriptor";
                return Recv::error;
            }
            for( int fd : fds )
            {
                ::fcntl( fd, F_SETFD, FD_CLOEXEC );
                first_fds->push_back( fd );
            }
        }
        if( n == 0 )
        {
            if( done == 0 && eof_ok )
                return Recv::eof;
            error = "connection closed in the middle of a message";
            return Recv::error;
        }
        done += size_t( n );
    }
    return Recv::ok;
}

// True when no array or object in 'text' is nested deeper than 'max_depth' (brackets inside strings do not count).
// nlohmann::json parses iteratively, but copying, comparing or dumping a value recurses once per level: a 256 KiB
// body of '[' (about 100000 levels) overflows the stack of whoever copies it (the CONFIGURE handler does), and a
// crash of the root helper leaves the camera captured. Applied to the raw text so nothing deep is ever built.
bool nesting_within( const std::string & text, int max_depth )
{
    int depth = 0;
    bool in_string = false, escaped = false;
    for( char ch : text )
    {
        if( in_string )
        {
            if( escaped )
                escaped = false;
            else if( ch == '\\' )
                escaped = true;
            else if( ch == '"' )
                in_string = false;
        }
        else if( ch == '"' )
            in_string = true;
        else if( ch == '[' || ch == '{' )
        {
            if( ++depth > max_depth )
                return false;
        }
        else if( ch == ']' || ch == '}' )
            --depth;
    }
    return true;
}
}  // namespace

ReadResult read_message( int sock, InMessage & out, std::string & error )
{
    out = InMessage();
    ifs_wire_header header{};
    std::vector< int > raw_fds;
    Recv r = recv_exact( sock, reinterpret_cast< uint8_t * >( &header ), sizeof( header ), &raw_fds, true, error );
    std::vector< UniqueFd > fds;
    for( int fd : raw_fds )
        fds.emplace_back( fd );
    if( r == Recv::eof )
        return ReadResult::eof;
    if( r == Recv::error )
        return ReadResult::error;

    if( ifs_wire_header_valid( &header ) != 0 )
    {
        error = "invalid message header";
        return ReadResult::error;
    }
    const bool fd_flag = ( header.flags & IFS_FLAG_HAS_FD ) != 0;
    if( fds.size() > 1 || ( ! fds.empty() && ( ! fd_flag || header.type != IFS_CMD_RECORD_START ) ) )
    {
        error = "unexpected file descriptor";
        return ReadResult::error;
    }
    if( fd_flag && fds.empty() )
    {
        error = "message flagged with a file descriptor arrived without one";
        return ReadResult::error;
    }
    if( header.payload_bytes != 0 )
    {
        error = "commands carry no binary payload";
        return ReadResult::error;
    }
    if( header.json_bytes == 0 )
    {
        error = "message without a JSON body";
        return ReadResult::error;
    }

    std::string json( header.json_bytes, '\0' );
    r = recv_exact( sock, reinterpret_cast< uint8_t * >( &json[0] ), json.size(), nullptr, false, error );
    if( r != Recv::ok )
        return ReadResult::error;

    if( ! nesting_within( json, kMaxJsonDepth ) )
    {
        error = "message body nested deeper than " + std::to_string( kMaxJsonDepth ) + " levels";
        return ReadResult::error;
    }
    out.body = nlohmann::json::parse( json, nullptr, false );
    if( out.body.is_discarded() || ! out.body.is_object() )
    {
        error = "message body is not a JSON object";
        return ReadResult::error;
    }
    out.type = header.type;
    out.json_bytes = json.size();
    auto id = out.body.find( "request_id" );
    if( id != out.body.end() && id->is_number_unsigned() )
    {
        out.request_id = id->get< uint64_t >();
        out.has_request_id = true;
    }
    if( ! fds.empty() )
        out.fd = std::move( fds.front() );
    return ReadResult::ok;
}

uint64_t OutMessage::payload_bytes() const
{
    uint64_t total = 0;
    for( const iovec & v : payload )
        total += v.iov_len;
    return total;
}

std::string dump_json( const nlohmann::json & body )
{
    return body.dump( -1, ' ', false, nlohmann::json::error_handler_t::replace );
}

bool make_message( OutMessage & msg, uint16_t type, const nlohmann::json & body )
{
    std::string text = dump_json( body );
    const uint64_t payload = msg.payload_bytes();
    if( text.size() > IFS_MAX_JSON_BYTES || payload > IFS_MAX_PAYLOAD_BYTES )
        return false;
    msg.json = std::move( text );
    ifs_wire_header_init( &msg.header, type, uint32_t( msg.json.size() ), payload, 0 );
    return true;
}

bool write_message( int sock, const OutMessage & msg )
{
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
        ssize_t n = ::writev( sock, &iov[index], count );
        if( n < 0 )
        {
            if( errno == EINTR )
                continue;
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

void configure_socket( int sock )
{
    int one = 1;
    ::setsockopt( sock, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof( one ) );
    ::fcntl( sock, F_SETFD, FD_CLOEXEC );
    // The default 8 KiB AF_UNIX buffer turns every 5 MB frameset into hundreds of wakeups.
    int buffer = 4 * 1024 * 1024;
    ::setsockopt( sock, SOL_SOCKET, SO_SNDBUF, &buffer, sizeof( buffer ) );
    ::setsockopt( sock, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof( buffer ) );
}

}  // namespace ifs
