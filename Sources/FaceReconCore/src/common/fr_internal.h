// License: Apache 2.0. See LICENSE file in root directory.
//
// Internal helpers shared by the FaceReconCore implementation files (not part of the public C API).

#pragma once

#include "fr_types.h"

#include <exception>
#include <new>
#include <string>

namespace fr
{
// Records message as the calling thread's fr_last_error() and returns status.
fr_status fail( fr_status status, const std::string & message );
void clear_error();

// Reports progress; returns false when the caller asked to cancel.
inline bool report( fr_progress_cb progress, void * user, float fraction, const char * stage )
{
    return ! progress || progress( user, fraction, stage ) == 0;
}

// Runs f, converting C++ exceptions into an fr_status (C API functions must never throw).
template< typename F >
fr_status guarded( const char * where, F && f ) noexcept
{
    try
    {
        return f();
    }
    catch( const std::bad_alloc & )
    {
        return fail( FR_ERR_OUT_OF_MEMORY, std::string( where ) + ": out of memory" );
    }
    catch( const std::exception & e )
    {
        return fail( FR_ERR_FAILED, std::string( where ) + ": " + e.what() );
    }
    catch( ... )
    {
        return fail( FR_ERR_FAILED, std::string( where ) + ": unknown exception" );
    }
}
}  // namespace fr
