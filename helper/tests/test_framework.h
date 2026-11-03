// License: Apache 2.0. See LICENSE file in root directory.
//
// Minimal test registry for the capture tests (the helper build has no test framework dependency):
//   TEST_CASE( name ) { CHECK( a == b ); REQUIRE( ok ); SKIP( "why" ); }
// capture-tests [substring] runs the tests whose name contains the substring.

#pragma once

#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ifs
{
namespace test
{

struct TestCase
{
    const char * name;
    std::function< void() > body;
};

inline std::vector< TestCase > & registry()
{
    static std::vector< TestCase > tests;
    return tests;
}

struct Registrar
{
    Registrar( const char * name, std::function< void() > body ) { registry().push_back( { name, std::move( body ) } ); }
};

struct Failure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
struct Skipped : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

inline int & failed_checks()
{
    static int n = 0;
    return n;
}

inline void report_check( const char * file, int line, const std::string & what )
{
    ++failed_checks();
    std::fprintf( stderr, "    %s:%d: CHECK failed: %s\n", file, line, what.c_str() );
}

}  // namespace test
}  // namespace ifs

#define IFS_CAT2( a, b ) a##b
#define IFS_CAT( a, b ) IFS_CAT2( a, b )
#define TEST_CASE( name )                                                                                             \
    static void name();                                                                                               \
    static ::ifs::test::Registrar IFS_CAT( registrar_, name )( #name, name );                                         \
    static void name()

#define CHECK( cond )                                                                                                 \
    do                                                                                                                \
    {                                                                                                                 \
        if( ! ( cond ) )                                                                                              \
            ::ifs::test::report_check( __FILE__, __LINE__, #cond );                                                   \
    } while( 0 )

// CHECK with context printed on failure, e.g. CHECK_MSG( n > 3, "n = " << n )
#define CHECK_MSG( cond, msg )                                                                                        \
    do                                                                                                                \
    {                                                                                                                 \
        if( ! ( cond ) )                                                                                              \
        {                                                                                                             \
            std::ostringstream ifs_os;                                                                                \
            ifs_os << #cond << " (" << msg << ")";                                                                    \
            ::ifs::test::report_check( __FILE__, __LINE__, ifs_os.str() );                                            \
        }                                                                                                             \
    } while( 0 )

#define REQUIRE( cond )                                                                                               \
    do                                                                                                                \
    {                                                                                                                 \
        if( ! ( cond ) )                                                                                              \
            throw ::ifs::test::Failure( std::string( __FILE__ ) + ":" + std::to_string( __LINE__ )                     \
                                        + ": REQUIRE failed: " #cond );                                               \
    } while( 0 )

#define SKIP( why ) throw ::ifs::test::Skipped( why )

// Expects the statement to throw std::exception whose message contains 'text'.
#define CHECK_THROWS_WITH( stmt, text )                                                                               \
    do                                                                                                                \
    {                                                                                                                 \
        bool ifs_threw = false;                                                                                       \
        try                                                                                                           \
        {                                                                                                             \
            stmt;                                                                                                     \
        }                                                                                                             \
        catch( const std::exception & ifs_e )                                                                         \
        {                                                                                                             \
            ifs_threw = true;                                                                                         \
            if( std::string( ifs_e.what() ).find( text ) == std::string::npos )                                       \
                ::ifs::test::report_check( __FILE__, __LINE__,                                                        \
                                           std::string( #stmt " threw '" ) + ifs_e.what() + "', expected '" + text   \
                                               + "'" );                                                               \
        }                                                                                                             \
        if( ! ifs_threw )                                                                                             \
            ::ifs::test::report_check( __FILE__, __LINE__, #stmt " did not throw" );                                  \
    } while( 0 )
