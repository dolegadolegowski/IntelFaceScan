// License: Apache 2.0. See LICENSE file in root directory.
//
// 8-bit PNG writing through ImageIO (about twice as fast as stb_image_write on a 4096 x 4096 atlas, with smaller
// files), written atomically like the other exports.

#include "fr_export.h"
#include "export_internal.h"

#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>

#include <cstring>
#include <unistd.h>

namespace
{

// CFRelease for CoreFoundation objects that may be NULL.
template< typename T >
struct cf_holder
{
    T ref = nullptr;
    explicit cf_holder( T r )
        : ref( r )
    {
    }
    ~cf_holder()
    {
        if( ref )
            CFRelease( ref );
    }
    cf_holder( const cf_holder & ) = delete;
    cf_holder & operator=( const cf_holder & ) = delete;
};

}  // namespace

extern "C" {

fr_status fr_write_png( const char * path, const uint8_t * pixels, int32_t width, int32_t height, int32_t channels )
{
    return fr::guarded( "fr_write_png", [&]() -> fr_status {
        if( ! path || ! *path || ! pixels )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_write_png: NULL argument" );
        if( width <= 0 || height <= 0 || width > 65535 || height > 65535 || ( channels != 1 && channels != 3 && channels != 4 ) )
            return fr::fail( FR_ERR_INVALID_ARGUMENT, "fr_write_png: invalid size or channel count" );
        const std::string target( path ), temp = target + ".partial";
        const size_t row_bytes = size_t( width ) * size_t( channels );

        cf_holder< CFURLRef > url( CFURLCreateFromFileSystemRepresentation(
            kCFAllocatorDefault, reinterpret_cast< const UInt8 * >( temp.c_str() ), CFIndex( temp.size() ), false ) );
        cf_holder< CGColorSpaceRef > space( channels == 1 ? CGColorSpaceCreateWithName( kCGColorSpaceGenericGrayGamma2_2 )
                                                          : CGColorSpaceCreateWithName( kCGColorSpaceSRGB ) );
        // The provider does not copy: pixels must outlive the image, which is released before returning.
        cf_holder< CGDataProviderRef > provider(
            CGDataProviderCreateWithData( nullptr, pixels, row_bytes * size_t( height ), nullptr ) );
        if( ! url.ref || ! space.ref || ! provider.ref )
            return fr::fail( FR_ERR_OUT_OF_MEMORY, "fr_write_png: CoreGraphics allocation failed" );
        const CGBitmapInfo info = channels == 4 ? CGBitmapInfo( kCGImageAlphaLast ) : CGBitmapInfo( kCGImageAlphaNone );
        cf_holder< CGImageRef > image( CGImageCreate( size_t( width ), size_t( height ), 8, size_t( 8 * channels ), row_bytes,
                                                      space.ref, info, provider.ref, nullptr, false,
                                                      kCGRenderingIntentDefault ) );
        if( ! image.ref )
            return fr::fail( FR_ERR_FAILED, "fr_write_png: CGImageCreate failed" );
        bool ok;
        {
            cf_holder< CGImageDestinationRef > dest( CGImageDestinationCreateWithURL( url.ref, CFSTR( "public.png" ), 1, nullptr ) );
            if( ! dest.ref )
                return fr::fail( FR_ERR_IO, "fr_write_png: cannot create " + target );
            CGImageDestinationAddImage( dest.ref, image.ref, nullptr );
            ok = CGImageDestinationFinalize( dest.ref );
        }
        if( ! ok || std::rename( temp.c_str(), target.c_str() ) != 0 )
        {
            ::unlink( temp.c_str() );
            return fr::fail( FR_ERR_IO, "fr_write_png: cannot write " + target );
        }
        return FR_OK;
    } );
}

}  // extern "C"
