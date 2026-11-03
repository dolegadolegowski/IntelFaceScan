// License: Apache 2.0. See LICENSE file in root directory.
//
// Lossless PNG reading/writing of the dataset images with ImageIO: 8-bit RGB color, 16-bit grayscale depth, 8-bit
// grayscale masks. Pixels are passed through bit-exactly (no color management, no gamma): the decoded bitmap is
// read directly whenever its layout allows, and only foreign layouts (palette, alpha, ...) are converted by drawing.

import CoreGraphics
import Foundation
import ImageIO

public enum PNGImageIOError: Error, LocalizedError, Equatable {
    case invalidSize(String)
    case encodeFailed(String)
    case decodeFailed(String)
    case unsupportedFormat(String)

    public var errorDescription: String? {
        switch self {
        case .invalidSize(let m): return "Invalid image size: \(m)"
        case .encodeFailed(let m): return "Could not write PNG: \(m)"
        case .decodeFailed(let m): return "Could not read PNG: \(m)"
        case .unsupportedFormat(let m): return "Unsupported PNG format: \(m)"
        }
    }
}

public enum PNGImageIO {
    public struct Image: Sendable, Equatable {
        public var width: Int
        public var height: Int
        /// Tightly packed rows: 3 bytes per pixel (RGB8), 1 (gray 8) or 2 (gray 16, little-endian UInt16).
        public var pixels: Data
    }

    // MARK: Writing

    /// rgb: rows of bytesPerRow bytes (>= 3 * width).
    public static func writeRGB8(_ rgb: Data, width: Int, height: Int, bytesPerRow: Int? = nil, to url: URL) throws {
        let stride = bytesPerRow ?? width * 3
        try checkSize(width, height, stride: stride, minStride: width * 3, bytes: rgb.count, name: url.lastPathComponent)
        let space = CGColorSpace(name: CGColorSpace.sRGB) ?? CGColorSpaceCreateDeviceRGB()
        try write(rgb, width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 24, bytesPerRow: stride,
                  space: space, bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.none.rawValue), to: url)
    }

    /// gray: width * height bytes.
    public static func writeGray8(_ gray: Data, width: Int, height: Int, to url: URL) throws {
        try checkSize(width, height, stride: width, minStride: width, bytes: gray.count, name: url.lastPathComponent)
        try write(gray, width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 8, bytesPerRow: width,
                  space: CGColorSpaceCreateDeviceGray(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.none.rawValue),
                  to: url)
    }

    /// values: width * height little-endian UInt16.
    public static func writeGray16(_ values: Data, width: Int, height: Int, to url: URL) throws {
        try checkSize(width, height, stride: 2 * width, minStride: 2 * width, bytes: values.count, name: url.lastPathComponent)
        // Linear gray: viewers show the raw values, and nothing applies a gamma curve to depth.
        let space = CGColorSpace(name: CGColorSpace.linearGray) ?? CGColorSpaceCreateDeviceGray()
        let info = CGBitmapInfo(rawValue: CGImageAlphaInfo.none.rawValue | CGBitmapInfo.byteOrder16Little.rawValue)
        try write(values, width: width, height: height, bitsPerComponent: 16, bitsPerPixel: 16, bytesPerRow: 2 * width,
                  space: space, bitmapInfo: info, to: url)
    }

    // MARK: Reading

    public static func readRGB8(from url: URL) throws -> Image {
        let image = try load(url)
        let w = image.width, h = image.height
        if image.bitsPerComponent == 8, image.bitsPerPixel == 24, image.colorSpace?.model == .rgb,
           image.alphaInfo == .none, let data = image.dataProvider?.data {
            return Image(width: w, height: h, pixels: packRows(data as Data, rowBytes: 3 * w, stride: image.bytesPerRow, height: h))
        }
        // Foreign layout (alpha, palette, 16-bit, gray): draw into RGBA8 in the image's own RGB space and drop alpha.
        let space = image.colorSpace?.model == .rgb ? image.colorSpace! : CGColorSpaceCreateDeviceRGB()
        let rgba = try draw(image, bytesPerPixel: 4, space: space,
                            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue)
        var rgb = Data(count: w * h * 3)
        rgb.withUnsafeMutableBytes { out in
            rgba.withUnsafeBytes { input in
                let o = out.bindMemory(to: UInt8.self), s = input.bindMemory(to: UInt8.self)
                for i in 0 ..< w * h {
                    o[3 * i] = s[4 * i]; o[3 * i + 1] = s[4 * i + 1]; o[3 * i + 2] = s[4 * i + 2]
                }
            }
        }
        return Image(width: w, height: h, pixels: rgb)
    }

    public static func readGray8(from url: URL) throws -> Image {
        let image = try load(url)
        let w = image.width, h = image.height
        if image.bitsPerComponent == 8, image.bitsPerPixel == 8, image.colorSpace?.model == .monochrome,
           let data = image.dataProvider?.data {
            return Image(width: w, height: h, pixels: packRows(data as Data, rowBytes: w, stride: image.bytesPerRow, height: h))
        }
        let gray = try draw(image, bytesPerPixel: 1, space: CGColorSpaceCreateDeviceGray(),
                            bitmapInfo: CGImageAlphaInfo.none.rawValue)
        return Image(width: w, height: h, pixels: gray)
    }

    /// Returns little-endian UInt16 values. Only 16-bit grayscale PNGs are accepted: converting other formats would
    /// silently change the depth values.
    public static func readGray16(from url: URL) throws -> Image {
        let image = try load(url)
        let w = image.width, h = image.height
        guard image.bitsPerComponent == 16, image.bitsPerPixel == 16, image.colorSpace?.model == .monochrome,
              let data = image.dataProvider?.data else {
            throw PNGImageIOError.unsupportedFormat(
                "\(url.lastPathComponent): expected 16-bit grayscale, got \(image.bitsPerPixel) bits per pixel")
        }
        var packed = packRows(data as Data, rowBytes: 2 * w, stride: image.bytesPerRow, height: h)
        // CoreGraphics' default 16-bit order is big-endian (as stored in PNG).
        if image.byteOrderInfo != .order16Little {
            packed.withUnsafeMutableBytes { raw in
                let v = raw.bindMemory(to: UInt16.self)
                for i in 0 ..< v.count { v[i] = v[i].byteSwapped }
            }
        }
        return Image(width: w, height: h, pixels: packed)
    }

    // MARK: Internals

    private static func checkSize(_ width: Int, _ height: Int, stride: Int, minStride: Int, bytes: Int, name: String) throws {
        guard width > 0, height > 0, width <= 32768, height <= 32768, stride >= minStride else {
            throw PNGImageIOError.invalidSize("\(name): \(width)x\(height), \(stride) bytes per row")
        }
        guard bytes >= stride * (height - 1) + minStride else {
            throw PNGImageIOError.invalidSize("\(name): \(bytes) bytes for \(width)x\(height)")
        }
    }

    private static func write(_ pixels: Data, width: Int, height: Int, bitsPerComponent: Int, bitsPerPixel: Int,
                              bytesPerRow: Int, space: CGColorSpace, bitmapInfo: CGBitmapInfo, to url: URL) throws {
        guard let provider = CGDataProvider(data: pixels as CFData),
              let image = CGImage(width: width, height: height, bitsPerComponent: bitsPerComponent,
                                  bitsPerPixel: bitsPerPixel, bytesPerRow: bytesPerRow, space: space,
                                  bitmapInfo: bitmapInfo, provider: provider, decode: nil, shouldInterpolate: false,
                                  intent: .defaultIntent) else {
            throw PNGImageIOError.encodeFailed("\(url.lastPathComponent): cannot create the image")
        }
        guard let destination = CGImageDestinationCreateWithURL(url as CFURL, "public.png" as CFString, 1, nil) else {
            throw PNGImageIOError.encodeFailed("\(url.path): cannot create the file")
        }
        CGImageDestinationAddImage(destination, image, nil)
        guard CGImageDestinationFinalize(destination) else {
            throw PNGImageIOError.encodeFailed(url.path)
        }
    }

    private static func load(_ url: URL) throws -> CGImage {
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw PNGImageIOError.decodeFailed("\(url.path): no such file")
        }
        let options = [kCGImageSourceShouldCache: false] as CFDictionary
        guard let source = CGImageSourceCreateWithURL(url as CFURL, options),
              let image = CGImageSourceCreateImageAtIndex(source, 0, options) else {
            throw PNGImageIOError.decodeFailed("\(url.path): not a readable image")
        }
        return image
    }

    private static func packRows(_ data: Data, rowBytes: Int, stride: Int, height: Int) -> Data {
        if stride == rowBytes && data.count == rowBytes * height { return data }
        var packed = Data(count: rowBytes * height)
        packed.withUnsafeMutableBytes { out in
            data.withUnsafeBytes { input in
                for y in 0 ..< height where y * stride + rowBytes <= input.count {
                    out.baseAddress!.advanced(by: y * rowBytes)
                        .copyMemory(from: input.baseAddress!.advanced(by: y * stride), byteCount: rowBytes)
                }
            }
        }
        return packed
    }

    private static func draw(_ image: CGImage, bytesPerPixel: Int, space: CGColorSpace, bitmapInfo: UInt32) throws -> Data {
        let w = image.width, h = image.height
        var data = Data(count: w * h * bytesPerPixel)
        let ok = data.withUnsafeMutableBytes { raw -> Bool in
            guard let context = CGContext(data: raw.baseAddress, width: w, height: h, bitsPerComponent: 8,
                                          bytesPerRow: w * bytesPerPixel, space: space, bitmapInfo: bitmapInfo) else {
                return false
            }
            context.interpolationQuality = .none
            context.draw(image, in: CGRect(x: 0, y: 0, width: w, height: h))
            return true
        }
        guard ok else { throw PNGImageIOError.unsupportedFormat("cannot convert a \(image.bitsPerPixel)-bit image") }
        return data
    }
}
