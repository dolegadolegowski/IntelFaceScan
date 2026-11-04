// License: Apache 2.0. See LICENSE file in root directory.
//
// Connecting to the app's socket. The helper may run as root, so the path it is given is not trusted: it must be an
// absolute path without symbolic links whose directory belongs to the app's user with mode 0700, and the process
// listening on it must be that user's app (peer uid and pid).

#pragma once

#include "ipc_wire.h"

#include <string>
#include <sys/types.h>

namespace ifs
{

// Validates 'path' (see above) and connects to it. The directory is opened component by component without
// following links and the connection is made relative to that directory, so the checked directory is the one
// connected to. Throws std::runtime_error with the reason.
UniqueFd connect_app_socket( const std::string & path, uid_t uid );

// Checks that the peer of a connected socket runs as 'uid' and is process 'pid'. Throws std::runtime_error.
void verify_app_peer( int sock, uid_t uid, pid_t pid );

}  // namespace ifs
