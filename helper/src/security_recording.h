// License: Apache 2.0. See LICENSE file in root directory.
//
// Recordings without trusting paths from the app: the engine records into a fresh directory that only the helper's
// user can enter (root on the administrator path; the app's user for a live helper that uses the RealSense camera
// service and for the fake camera), and the finished file is copied into a descriptor the app opened as itself and
// passed with SCM_RIGHTS. The helper never opens, creates or deletes a path the app named.

#pragma once

#include "ipc_wire.h"

#include <cstdint>
#include <memory>
#include <string>

namespace ifs
{

// Checks a descriptor received for RECORD_START: a regular file without set-user-ID/set-group-ID bits, opened for
// writing. Returns an empty string when it is acceptable, otherwise the reason.
std::string check_recording_destination( int fd );

class RecordingSpool
{
public:
    // Creates <private base>/ifs-recording.XXXXXX (mkdtemp, mode 0700) and locks it (flock) for the life of the
    // spool. The private base is the effective user's Darwin temporary directory
    // (confstr(_CS_DARWIN_USER_TEMP_DIR): /var/folders/zz/.../T for root, /var/folders/<..>/T for a user), accepted
    // only when it is owned by the effective user and not accessible to anyone else. Spools that helpers of the same
    // user left behind (older than a day and not locked by a running helper) are removed first. Throws
    // std::runtime_error.
    static std::unique_ptr< RecordingSpool > create( const std::string & format );

    ~RecordingSpool();
    RecordingSpool( const RecordingSpool & ) = delete;
    RecordingSpool & operator=( const RecordingSpool & ) = delete;

    // Where the engine writes the recording.
    const std::string & file_path() const { return file_; }
    const std::string & directory() const { return dir_; }

    // Copies the finished recording into 'destination' (from its current offset), fsyncs it and returns the number
    // of bytes. Does not close the descriptor. Throws std::runtime_error.
    uint64_t copy_into( int destination ) const;

    // Deletes the directory and everything the engine left in it (also done by the destructor).
    void remove();

private:
    RecordingSpool( std::string dir, std::string file, UniqueFd lock )
        : dir_( std::move( dir ) )
        , file_( std::move( file ) )
        , lock_( std::move( lock ) )
    {
    }

    std::string dir_;
    std::string file_;
    UniqueFd lock_;  // the spool directory, locked exclusively: this spool is in use
    bool removed_ = false;
};

}  // namespace ifs
