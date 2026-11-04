// License: Apache 2.0. See LICENSE file in root directory.
//
// Gives the files a root-run RealSense tool created back to the user who launched it:
//
//   rs-macos-handback <uid> <gid> <stamp-file> <path>...
//
// Each path (a directory is searched up to 3 levels deep) changes owner only if it is a regular file or directory that
// root owns and that was created after <stamp-file> was last modified. Everything is opened without following links and
// changed through that descriptor, and regular files must have a single link, so links or files the user swapped in
// cannot redirect the change to another file.
//
// The stamp must be a regular file owned by root (the root launcher creates it right before the tool runs): a stamp the
// user could backdate would widen "created during the session" to every root-owned file. Paths must be absolute and are
// walked from / one component at a time without following any link: a link in a folder above the path (e.g. the user's
// folder pointing to /private/var/db) would otherwise send the search into system folders.

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static uid_t uid;
static gid_t gid;
static struct timespec since;

static int created_since( const struct timespec * t )
{
    return t->tv_sec > since.tv_sec || ( t->tv_sec == since.tv_sec && t->tv_nsec >= since.tv_nsec );
}

static void hand_back( int dir, const char * name, int depth )
{
    int fd = openat( dir, name, O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC );
    if( fd < 0 )
        return;
    struct stat st;
    if( fstat( fd, &st ) == 0 )
    {
        int ours = st.st_uid == 0 && created_since( &st.st_birthtimespec );
        if( S_ISREG( st.st_mode ) && ours && st.st_nlink == 1 )
            fchown( fd, uid, gid );
        else if( S_ISDIR( st.st_mode ) )
        {
            if( ours )
                fchown( fd, uid, gid );
            DIR * d = depth > 0 ? fdopendir( dup( fd ) ) : NULL;
            if( d )
            {
                struct dirent * entry;
                while( ( entry = readdir( d ) ) )
                    if( strcmp( entry->d_name, "." ) && strcmp( entry->d_name, ".." ) )
                        hand_back( dirfd( d ), entry->d_name, depth - 1 );
                closedir( d );
            }
        }
    }
    close( fd );
}

// Opens the folder that contains 'path' (absolute) without following a link anywhere and returns its descriptor;
// *name points at the last component. -1 when the path is not absolute or a component is a link or no folder.
static int open_parent( const char * path, const char ** name )
{
    if( path[0] != '/' )
        return -1;
    char copy[PATH_MAX];
    if( strlen( path ) >= sizeof( copy ) )
        return -1;
    strcpy( copy, path );
    const char * last = strrchr( path, '/' ) + 1;
    if( ! *last || ! strcmp( last, "." ) || ! strcmp( last, ".." ) )
        return -1;
    *name = last;
    copy[last - path] = '\0';
    int dir = open( "/", O_RDONLY | O_DIRECTORY | O_CLOEXEC );
    for( char *save = NULL, *part = strtok_r( copy, "/", &save ); part && dir >= 0;
         part = strtok_r( NULL, "/", &save ) )
    {
        int next = strcmp( part, ".." ) && strcmp( part, "." )
                     ? openat( dir, part, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC )
                     : -1;
        close( dir );
        dir = next;
    }
    return dir;
}

static int parse_id( const char * text, unsigned long * out )
{
    char * end = NULL;
    errno = 0;
    unsigned long v = strtoul( text, &end, 10 );
    if( ! *text || *text == '-' || errno || *end || v >= UINT_MAX )
        return 0;
    *out = v;
    return 1;
}

int main( int argc, char * argv[] )
{
    struct stat stamp;
    unsigned long u, g;
    if( argc < 4 || ! parse_id( argv[1], &u ) || ! parse_id( argv[2], &g ) )
    {
        fprintf( stderr, "usage: %s <uid> <gid> <stamp-file> <path>...\n", argv[0] );
        return 1;
    }
    if( lstat( argv[3], &stamp ) != 0 || ! S_ISREG( stamp.st_mode ) || stamp.st_uid != 0 )
    {
        fprintf( stderr, "%s: the stamp file must be a regular file owned by root\n", argv[0] );
        return 1;
    }
    uid = (uid_t)u;
    gid = (gid_t)g;
    since = stamp.st_mtimespec;
    int status = 0;
    for( int i = 4; i < argc; ++i )
    {
        const char * name = NULL;
        int parent = open_parent( argv[i], &name );
        if( parent < 0 )
        {
            fprintf( stderr, "%s: skipped %s (not absolute, or a link or missing folder on the way)\n", argv[0],
                     argv[i] );
            status = 1;
            continue;
        }
        hand_back( parent, name, 3 );
        close( parent );
    }
    return status;
}
