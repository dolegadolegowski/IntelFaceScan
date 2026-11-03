// License: Apache 2.0. See LICENSE file in root directory.
//
// Head segmentation at color resolution: person mask (or a face-rect ellipse when segmentation is unavailable)
// ∩ depth band around the face ∩ above the neck cut (unless keepShoulders), lightly eroded.

import Accelerate
import CameraKit
import CoreGraphics
import Foundation
import simd

public struct HeadMaskParameters: Sendable, Hashable {
    /// Depth band around the face distance d: [d - nearMarginM, d + farMarginM]. The far margin is larger because
    /// the back of the head and the ears lie behind the face.
    public var nearMarginM: Float = 0.12
    public var farMarginM: Float = 0.20
    /// Keep the neck and shoulders instead of cutting below the chin.
    public var keepShoulders: Bool = false
    /// The neck cut lies this fraction of the face height below the chin, so the underside of the jaw survives.
    public var neckMarginFraction: Float = 0.12
    /// Erosion radius in pixels: drops the silhouette pixels where aligned depth mixes head and background.
    public var erosionRadius: Int = 2

    public init() {}
}

public enum HeadMaskBuilder {
    /// Builds the head mask (width * height bytes, 255 = head, 0 = background).
    /// - Parameters:
    ///   - observation: the face (rectangle, roll and chin landmarks) and the optional person mask.
    ///   - depth: aligned depth (same size as the color image) or nil.
    ///   - faceDistanceM: median face distance; nil disables the depth band.
    public static func build(width: Int, height: Int, observation: HeadObservation, depth: DepthImage?,
                             faceDistanceM: Float?, parameters: HeadMaskParameters = HeadMaskParameters()) -> Data {
        guard width > 0, height > 0 else { return Data() }
        var mask = Data(count: width * height)
        let face = observation.faceRect
        guard face.width > 1, face.height > 1, face.minX.isFinite, face.minY.isFinite else { return mask }

        let personMask = observation.personMask.flatMap { $0.count == width * height ? $0 : nil }
        let usableDepth = depth.flatMap { $0.isValidZ16 && $0.width == width && $0.height == height ? $0 : nil }
        let band: (lo: UInt16, hi: UInt16)? = {
            guard let d = faceDistanceM, d.isFinite, d > 0, let depth = usableDepth else { return nil }
            let lo = max(0, d - parameters.nearMarginM) / depth.depthScale
            let hi = min(Float(UInt16.max), (d + parameters.farMarginM) / depth.depthScale)
            return (UInt16(max(1, lo.rounded())), UInt16(hi.rounded()))
        }()

        let roi = regionOfInterest(face: face, keepShoulders: parameters.keepShoulders, width: width, height: height)
        let ellipse = FallbackEllipse(face: face, pose: observation.pose)
        let down = HeadGeometry.downDirection(rollDegrees: observation.pose.roll)
        let cut = observation.chinPoint + down * Float(face.height) * parameters.neckMarginFraction

        mask.withUnsafeMutableBytes { outRaw in
            let out = outRaw.bindMemory(to: UInt8.self)
            withOptionalBytes(personMask) { personRaw in
                withOptionalBytes(band == nil ? nil : usableDepth?.pixels) { depthRaw in
                    let person = personRaw?.bindMemory(to: UInt8.self)
                    let depthValues = depthRaw?.bindMemory(to: UInt16.self)
                    for y in roi.y ..< roi.y + roi.height {
                        let row = y * width
                        let py = Float(y) + 0.5
                        for x in roi.x ..< roi.x + roi.width {
                            let i = row + x
                            let p = SIMD2(Float(x) + 0.5, py)
                            if let person {
                                if person[i] < 128 { continue }
                            } else if !ellipse.contains(p) {
                                continue
                            }
                            if !parameters.keepShoulders && simd_dot(p - cut, down) > 0 { continue }
                            if let band, let depthValues {
                                let v = UInt16(littleEndian: depthValues[i])
                                if v < band.lo || v > band.hi { continue }
                            }
                            out[i] = 255
                        }
                    }
                }
            }
        }
        if parameters.erosionRadius > 0 { erode(&mask, width: width, height: height, radius: parameters.erosionRadius) }
        return mask
    }

    /// Number of non-zero mask pixels.
    public static func count(_ mask: Data) -> Int {
        mask.withUnsafeBytes { raw in raw.bindMemory(to: UInt8.self).reduce(0) { $0 + ($1 != 0 ? 1 : 0) } }
    }

    /// Pixels that can belong to the head: generous around the face rectangle (hair, ears, back of the head in
    /// profile views), further down when shoulders are kept.
    static func regionOfInterest(face: CGRect, keepShoulders: Bool, width: Int, height: Int) -> PixelRect {
        let below: CGFloat = keepShoulders ? 3.0 : 1.0
        let r = CGRect(x: face.minX - 1.1 * face.width, y: face.minY - 1.2 * face.height,
                       width: 3.2 * face.width, height: (2.2 + below) * face.height)
        return PixelRect(r, clippedTo: width, height) ?? PixelRect(x: 0, y: 0, width: 0, height: 0)
    }

    /// In-place min filter (erosion) with a (2r+1)^2 square kernel.
    static func erode(_ mask: inout Data, width: Int, height: Int, radius: Int) {
        let k = vImagePixelCount(2 * radius + 1)
        let source = mask
        source.withUnsafeBytes { srcRaw in
            mask.withUnsafeMutableBytes { dstRaw in
                guard let s = srcRaw.baseAddress, let d = dstRaw.baseAddress else { return }
                var src = vImage_Buffer(data: UnsafeMutableRawPointer(mutating: s), height: vImagePixelCount(height),
                                        width: vImagePixelCount(width), rowBytes: width)
                var dst = vImage_Buffer(data: d, height: vImagePixelCount(height), width: vImagePixelCount(width),
                                        rowBytes: width)
                _ = vImageMin_Planar8(&src, &dst, nil, 0, 0, k, k, vImage_Flags(kvImageEdgeExtend))
            }
        }
    }
}

/// Runs body with the bytes of data, or nil.
func withOptionalBytes<R>(_ data: Data?, _ body: (UnsafeRawBufferPointer?) -> R) -> R {
    guard let data else { return body(nil) }
    return data.withUnsafeBytes { body($0) }
}

/// Head outline guessed from the Vision face rectangle, which spans roughly eyebrows to chin: the skull extends
/// about 0.7 face heights above it and the ears ~0.2 face widths to each side; in profile views the back of
/// the head extends away from the nose.
struct FallbackEllipse {
    let center: SIMD2<Float>
    let axes: SIMD2<Float>
    let cosR: Float, sinR: Float

    init(face: CGRect, pose: HeadPose) {
        let w = Float(face.width), h = Float(face.height)
        let yaw = max(-80, min(80, pose.yaw)) * .pi / 180
        let roll = pose.roll * .pi / 180
        let s = sin(yaw)
        // Center in the head frame (relative to the face center), then rotated by roll into the image.
        let local = SIMD2(-0.3 * w * s, -0.35 * h)
        cosR = cos(roll); sinR = sin(roll)
        center = SIMD2(Float(face.midX), Float(face.midY)) + SIMD2(cosR * local.x - sinR * local.y,
                                                                   sinR * local.x + cosR * local.y)
        axes = SIMD2(0.75 * w + 0.35 * w * abs(s), 0.95 * h)
    }

    func contains(_ p: SIMD2<Float>) -> Bool {
        let d = p - center
        // Rotate into the head frame (inverse roll).
        let x = cosR * d.x + sinR * d.y
        let y = -sinR * d.x + cosR * d.y
        let nx = x / axes.x, ny = y / axes.y
        return nx * nx + ny * ny <= 1
    }
}
