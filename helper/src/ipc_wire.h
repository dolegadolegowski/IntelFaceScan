// License: Apache 2.0. See LICENSE file in root directory.
//
// Message framing of docs/PROTOCOL.md on a connected AF_UNIX stream socket: blocking reads of one complete,
// validated message from the app and gathered writes of messages to the app. Nothing here interprets commands.

#pragma once

#include "ifs_protocol.h"

#include <nlohmann/json.hpp>

#include <sys/uio.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ifs
{

// Owns a file descriptor and closes it exactly once.
class UniqueFd
{
public:
    UniqueFd() = default;
    explicit UniqueFd( int fd ) : fd_( fd ) {}
    UniqueFd( UniqueFd && other ) noexcept : fd_( other.release() ) {}
    UniqueFd & operator=( UniqueFd && other ) noexcept;
    UniqueFd( const UniqueFd & ) = delete;
    UniqueFd & operator=( const UniqueFd & ) = delete;
    ~UniqueFd() { reset(); }

    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
    int release();
    // Closes the descriptor; returns the result of close() (0 when there was nothing to close).
    int reset( int fd = -1 );

private:
    int fd_ = -1;
};

// One command received from the app.
struct InMessage
{
    uint16_t type = 0;
    nlohmann::json body;      // always a JSON object
    uint64_t request_id = 0;  // valid when has_request_id
    bool has_request_id = false;
    UniqueFd fd;              // only for IFS_CMD_RECORD_START
};

enum class ReadResult
{
    ok,
    eof,       // the peer closed the connection cleanly between two messages
    error,     // I/O error or protocol violation (message in 'error'); the connection must be closed
};

// Reads one message from the app. Enforces the framing rules: valid header, JSON object within the size limit,
// no payload (no command carries one), a descriptor only on the first byte of an IFS_CMD_RECORD_START message.
// Every unexpected descriptor is closed.
ReadResult read_message( int sock, InMessage & out, std::string & error );

// One message to the app: header + JSON + payload segments that point into memory kept alive by 'keepalive'.
struct OutMessage
{
    ifs_wire_header header{};
    std::string json;
    std::vector< iovec > payload;
    std::vector< std::shared_ptr< void > > keepalive;
    bool frameset = false;

    uint64_t payload_bytes() const;
};

// Builds a message; returns false (message untouched) when the JSON or payload exceeds the protocol limits.
bool make_message( OutMessage & msg, uint16_t type, const nlohmann::json & body );

// Serializes JSON without throwing on invalid UTF-8 (replaced), which librealsense strings could contain.
std::string dump_json( const nlohmann::json & body );

// Writes the whole message (handles partial writes and EINTR). Returns false on any socket error.
bool write_message( int sock, const OutMessage & msg );

// Applies the socket options both directions need: no SIGPIPE, close-on-exec.
void configure_socket( int sock );

}  // namespace ifs
