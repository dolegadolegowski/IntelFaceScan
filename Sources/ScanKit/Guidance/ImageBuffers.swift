// License: Apache 2.0. See LICENSE file in root directory.
//
// Image plumbing for the guided scan: RGB8 -> BGRA pixel buffers for Vision (pooled, vImage), Vision masks ->
// binary masks at color resolution, gray crops of the color image.

import Accelerate
import CameraKit
import CoreGraphics
import CoreVideo
import Foundation

extension ColorImage {
    /// The pixel data holds `height` rows of at least `width * 3` bytes.
    public var isValidRGB8: Bool {
        width > 0 && height > 0 && bytesPerRow >= width * 3
            && pixels.count >= bytesPerRow * (height - 1) + width * 3
    }
}

extension DepthImage {
    /// The pixel data holds `width * height` UInt16 values.
    public var isValidZ16: Bool {
        width > 0 && height > 0 && pixels.count >= width * height * 2 && depthScale > 0
    }
}

/// Integer pixel rectangle clipped to an image.
struct PixelRect: Equatable {
    var x: Int, y: Int, width: Int, height: Int

    init(x: Int, y: Int, width: Int, height: Int) {
        self.x = x; self.y = y; self.width = width; self.height = height
    }

    /// The pixels covered by `rect`, clipped to a width x height image; nil when nothing is left.
    init?(_ rect: CGRect, clippedTo width: Int, _ height: Int) {
        guard rect.width.isFinite, rect.height.isFinite, rect.minX.isFinite, rect.minY.isFinite,
              width > 0, height > 0 else { return nil }
        let x0 = max(0, Int(rect.minX.rounded(.down))), y0 = max(0, Int(rect.minY.rounded(.down)))
        let x1 = min(width, Int(rect.maxX.rounded(.up))), y1 = min(height, Int(rect.maxY.rounded(.up)))
        guard x1 > x0, y1 > y0 else { return nil }
        self.init(x: x0, y: y0, width: x1 - x0, height: y1 - y0)
    }

    var area: Int { width * height }
}

extension CGRect {
    /// The rectangle scaled about its center.
    func scaledAboutCenter(_ sx: CGFloat, _ sy: CGFloat) -> CGRect {
        CGRect(x: midX - width * sx / 2, y: midY - height * sy / 2, width: width * sx, height: height * sy)
    }
}

/// Converts RGB8 color images into BGRA CVPixelBuffers from a pool, so steady-state streaming does not allocate.
final class PixelBufferConverter: @unchecked Sendable {
    private let lock = NSLock()
    private var pool: CVPixelBufferPool?
    private var poolWidth = 0, poolHeight = 0

    func makeBGRA(from image: ColorImage) throws -> CVPixelBuffer {
        guard image.isValidRGB8 else {
            throw HeadTrackerError.invalidImage("\(image.width)x\(image.height), \(image.bytesPerRow) bytes per row, \(image.pixels.count) bytes")
        }
        let pool = try bufferPool(width: image.width, height: image.height)
        var out: CVPixelBuffer?
        let status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &out)
        guard status == kCVReturnSuccess, let buffer = out else { throw HeadTrackerError.pixelBuffer(status) }

        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { throw HeadTrackerError.pixelBuffer(kCVReturnInvalidPixelBufferAttributes) }
        var dst = vImage_Buffer(data: base, height: vImagePixelCount(image.height), width: vImagePixelCount(image.width),
                                rowBytes: CVPixelBufferGetBytesPerRow(buffer))
        let error: vImage_Error = image.pixels.withUnsafeBytes { raw in
            guard let src = raw.baseAddress else { return kvImageNullPointerArgument }
            var srcBuffer = vImage_Buffer(data: UnsafeMutableRawPointer(mutating: src),
                                          height: vImagePixelCount(image.height), width: vImagePixelCount(image.width),
                                          rowBytes: image.bytesPerRow)
            return vImageConvert_RGB888toBGRA8888(&srcBuffer, nil, 255, &dst, false, vImage_Flags(kvImageNoFlags))
        }
        guard error == kvImageNoError else { throw HeadTrackerError.invalidImage("vImage error \(error)") }
        return buffer
    }

    private func bufferPool(width: Int, height: Int) throws -> CVPixelBufferPool {
        lock.lock(); defer { lock.unlock() }
        if let pool, poolWidth == width, poolHeight == height { return pool }
        let attributes: [CFString: Any] = [
            kCVPixelBufferPixelFormatTypeKey: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey: width,
            kCVPixelBufferHeightKey: height,
            kCVPixelBufferIOSurfacePropertiesKey: [CFString: Any](),
        ]
        var created: CVPixelBufferPool?
        let status = CVPixelBufferPoolCreate(kCFAllocatorDefault, nil, attributes as CFDictionary, &created)
        guard status == kCVReturnSuccess, let created else { throw HeadTrackerError.pixelBuffer(status) }
        pool = created; poolWidth = width; poolHeight = height
        return created
    }
}

enum MaskResampler {
    /// 0/255 lookup table: Vision masks are soft; the scan needs a binary decision.
    private static let thresholdTable: [Pixel_8] = (0 ..< 256).map { $0 >= 128 ? 255 : 0 }

    /// Resamples a one-component 8-bit Vision mask to width x height and thresholds it at 50 %.
    static func binaryMask(_ buffer: CVPixelBuffer, width: Int, height: Int) throws -> Data {
        guard width > 0, height > 0 else { throw HeadTrackerError.invalidImage("empty mask target") }
        let format = CVPixelBufferGetPixelFormatType(buffer)
        guard format == kCVPixelFormatType_OneComponent8 else {
            throw HeadTrackerError.vision("unexpected segmentation pixel format \(format)")
        }
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { throw HeadTrackerError.vision("empty segmentation buffer") }
        var src = vImage_Buffer(data: base, height: vImagePixelCount(CVPixelBufferGetHeight(buffer)),
                                width: vImagePixelCount(CVPixelBufferGetWidth(buffer)),
                                rowBytes: CVPixelBufferGetBytesPerRow(buffer))
        var out = Data(count: width * height)
        let error: vImage_Error = out.withUnsafeMutableBytes { raw in
            guard let dstBase = raw.baseAddress else { return kvImageNullPointerArgument }
            var dst = vImage_Buffer(data: dstBase, height: vImagePixelCount(height), width: vImagePixelCount(width),
                                    rowBytes: width)
            let scaled = vImageScale_Planar8(&src, &dst, nil, vImage_Flags(kvImageNoFlags))
            guard scaled == kvImageNoError else { return scaled }
            return thresholdTable.withUnsafeBufferPointer {
                vImageTableLookUp_Planar8(&dst, &dst, $0.baseAddress!, vImage_Flags(kvImageNoFlags))
            }
        }
        guard error == kvImageNoError else { throw HeadTrackerError.vision("mask resampling failed (\(error))") }
        return out
    }
}

enum GrayImage {
    /// Luma (BT.601, [0, 255]) of a region of an RGB8 image as floats, row-major `rect.width` per row.
    static func lumaFloats(_ image: ColorImage, in rect: PixelRect) -> [Float]? {
        guard image.isValidRGB8, rect.x >= 0, rect.y >= 0, rect.x + rect.width <= image.width,
              rect.y + rect.height <= image.height, rect.area > 0 else { return nil }
        let w = rect.width, h = rect.height
        var argb = [UInt8](repeating: 0, count: w * h * 4)
        var gray = [UInt8](repeating: 0, count: w * h)
        var floats = [Float](repeating: 0, count: w * h)
        let ok: Bool = image.pixels.withUnsafeBytes { raw in
            guard let base = raw.baseAddress else { return false }
            var src = vImage_Buffer(data: UnsafeMutableRawPointer(mutating: base + rect.y * image.bytesPerRow + rect.x * 3),
                                    height: vImagePixelCount(h), width: vImagePixelCount(w), rowBytes: image.bytesPerRow)
            return argb.withUnsafeMutableBytes { argbRaw in
                gray.withUnsafeMutableBytes { grayRaw in
                    floats.withUnsafeMutableBytes { floatRaw in
                        var argbBuffer = vImage_Buffer(data: argbRaw.baseAddress!, height: vImagePixelCount(h),
                                                       width: vImagePixelCount(w), rowBytes: w * 4)
                        var grayBuffer = vImage_Buffer(data: grayRaw.baseAddress!, height: vImagePixelCount(h),
                                                       width: vImagePixelCount(w), rowBytes: w)
                        var floatBuffer = vImage_Buffer(data: floatRaw.baseAddress!, height: vImagePixelCount(h),
                                                        width: vImagePixelCount(w), rowBytes: w * 4)
                        guard vImageConvert_RGB888toARGB8888(&src, nil, 255, &argbBuffer, false,
                                                             vImage_Flags(kvImageNoFlags)) == kvImageNoError else { return false }
                        // ARGB order; BT.601 weights scaled to a power-of-two divisor.
                        let matrix: [Int16] = [0, 77, 150, 29]
                        guard vImageMatrixMultiply_ARGB8888ToPlanar8(&argbBuffer, &grayBuffer, matrix, 256, nil, 128,
                                                                     vImage_Flags(kvImageNoFlags)) == kvImageNoError else { return false }
                        return vImageConvert_Planar8toPlanarF(&grayBuffer, &floatBuffer, 255, 0,
                                                              vImage_Flags(kvImageNoFlags)) == kvImageNoError
                    }
                }
            }
        }
        return ok ? floats : nil
    }
}
