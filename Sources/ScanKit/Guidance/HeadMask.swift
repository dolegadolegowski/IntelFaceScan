// License: Apache 2.0. See LICENSE file in root directory.
//
// Head segmentation at color resolution: person mask (or a face-rect ellipse when segmentation is unavailable or
// missed the face) ∩ depth band around the face ∩ above the neck cut (unless keepShoulders), silhouette eroded.
// Without a person mask, keepShoulders grows the head below the chin through depth-connected pixels of the band.

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
    /// Holes inside the head (pixels without depth) up to 2 * radius + 1 pixels wide are not widened.
    public var erosionRadius: Int = 2
    /// A person mask that covers less than this fraction of the central face region missed the face (Vision's
    /// person segmentation sometimes returns only the neck or shoulders); the face-rectangle ellipse is used instead.
    public var minPersonMaskFaceCoverage: Float = 0.5
    /// keepShoulders without a person mask (segmentation off, failed or rejected): the ellipse covers only the head,
    /// so neck and shoulders are grown from it: pixels below the chin within this many face widths to each side of
    /// it and face heights below it, with depth in the band, reached from the head through neighbors whose depth
    /// differs by at most shoulderMaxDepthStepM. The step limit stops at occluding edges (a chair back or desk at a
    /// similar distance touching the body in the image), the region limits what a smooth surface can add.
    public var shoulderHalfWidthFaces: Float = 1.6
    public var shoulderExtentFaces: Float = 2.5
    public var shoulderMaxDepthStepM: Float = 0.025

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

        let personMask = observation.personMask.flatMap {
            $0.count == width * height
                && personMaskCoversFace($0, width: width, height: height, face: face,
                                        minCoverage: parameters.minPersonMaskFaceCoverage) ? $0 : nil
        }
        let usableDepth = depth.flatMap { $0.isValidZ16 && $0.width == width && $0.height == height ? $0 : nil }
        let band: (lo: UInt16, hi: UInt16)? = {
            guard let d = faceDistanceM, d.isFinite, d > 0, let depth = usableDepth else { return nil }
            // Clamped to the Z16 range: a distance beyond what the depth unit can represent must not trap.
            let limit = Float(UInt16.max)
            let lo = min(limit, max(1, (max(0, d - parameters.nearMarginM) / depth.depthScale).rounded()))
            let hi = min(limit, max(lo, ((d + parameters.farMarginM) / depth.depthScale).rounded()))
            return (UInt16(lo), UInt16(hi))
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
        if parameters.keepShoulders, personMask == nil, let band, let depth = usableDepth {
            growShoulders(&mask, depth: depth, band: band, roi: roi, face: face, chin: observation.chinPoint, down: down,
                          parameters: parameters)
        }
        if parameters.erosionRadius > 0 {
            erodeSilhouette(&mask, width: width, height: height, radius: parameters.erosionRadius)
        }
        return mask
    }

    /// Adds neck and shoulders to a head mask built without a person mask (see HeadMaskParameters.keepShoulders):
    /// a flood fill from the head's pixels into the region below the chin, through pixels with depth in the band and
    /// small depth steps between neighbors.
    static func growShoulders(_ mask: inout Data, depth: DepthImage, band: (lo: UInt16, hi: UInt16), roi: PixelRect,
                              face: CGRect, chin: SIMD2<Float>, down: SIMD2<Float>, parameters: HeadMaskParameters) {
        let width = depth.width
        guard roi.area > 0, mask.count == width * depth.height, depth.depthScale > 0 else { return }
        let side = SIMD2(-down.y, down.x)
        let halfWidth = parameters.shoulderHalfWidthFaces * Float(face.width)
        let extent = parameters.shoulderExtentFaces * Float(face.height)
        let maxStep = Int((parameters.shoulderMaxDepthStepM / depth.depthScale).rounded())
        mask.withUnsafeMutableBytes { maskRaw in
            depth.pixels.withUnsafeBytes { depthRaw in
                let m = maskRaw.bindMemory(to: UInt8.self)
                let z = depthRaw.bindMemory(to: UInt16.self)
                func inRegion(_ x: Int, _ y: Int) -> Bool {
                    let d = SIMD2(Float(x) + 0.5, Float(y) + 0.5) - chin
                    let below = simd_dot(d, down)
                    return below > 0 && below <= extent && abs(simd_dot(d, side)) <= halfWidth
                }
                var queue: [Int32] = []
                for y in roi.y ..< roi.y + roi.height {
                    for x in roi.x ..< roi.x + roi.width where m[y * width + x] != 0 { queue.append(Int32(y * width + x)) }
                }
                func visit(_ x: Int, _ y: Int, from zi: Int) {
                    guard x >= roi.x, x < roi.x + roi.width, y >= roi.y, y < roi.y + roi.height else { return }
                    let j = y * width + x
                    guard m[j] == 0 else { return }
                    let v = UInt16(littleEndian: z[j])
                    guard v >= band.lo, v <= band.hi, abs(Int(v) - zi) <= maxStep, inRegion(x, y) else { return }
                    m[j] = 255
                    queue.append(Int32(j))
                }
                var head = 0
                while head < queue.count {
                    let i = Int(queue[head]); head += 1
                    let x = i % width, y = i / width
                    let zi = Int(UInt16(littleEndian: z[i]))
                    visit(x - 1, y, from: zi); visit(x + 1, y, from: zi)
                    visit(x, y - 1, from: zi); visit(x, y + 1, from: zi)
                }
            }
        }
    }

    /// Number of non-zero mask pixels.
    public static func count(_ mask: Data) -> Int {
        mask.withUnsafeBytes { raw in raw.bindMemory(to: UInt8.self).reduce(0) { $0 + ($1 != 0 ? 1 : 0) } }
    }

    /// Whether the person mask covers the central face region (where any correct person mask is "person").
    static func personMaskCoversFace(_ person: Data, width: Int, height: Int, face: CGRect, minCoverage: Float) -> Bool {
        guard minCoverage > 0 else { return true }
        guard let core = PixelRect(DepthStatistics.distanceRegion(face), clippedTo: width, height) else { return false }
        var inside = 0
        person.withUnsafeBytes { raw in
            let p = raw.bindMemory(to: UInt8.self)
            for y in core.y ..< core.y + core.height {
                let row = y * width
                for x in core.x ..< core.x + core.width where p[row + x] >= 128 { inside += 1 }
            }
        }
        return Float(inside) >= minCoverage * Float(core.area)
    }

    /// Erodes the outer silhouette by `radius` pixels without widening small holes inside the mask: an erosion of
    /// the closed mask (dilate by r, erode by 2r), intersected with the mask itself. A plain erosion would grow every
    /// pixel without depth (speckle, the eyes) into a (2r+1)^2 hole and drop ~20-30 % of the head's depth points.
    static func erodeSilhouette(_ mask: inout Data, width: Int, height: Int, radius: Int) {
        var shrunk = mask
        morphology(&shrunk, width: width, height: height, kernel: 2 * radius + 1, erode: false)
        morphology(&shrunk, width: width, height: height, kernel: 4 * radius + 1, erode: true)
        mask.withUnsafeMutableBytes { outRaw in
            shrunk.withUnsafeBytes { shrunkRaw in
                let out = outRaw.bindMemory(to: UInt8.self), s = shrunkRaw.bindMemory(to: UInt8.self)
                for i in 0 ..< out.count where s[i] == 0 { out[i] = 0 }
            }
        }
    }

    /// In-place min (erode) or max (dilate) filter with a square kernel.
    static func morphology(_ mask: inout Data, width: Int, height: Int, kernel: Int, erode: Bool) {
        let k = vImagePixelCount(kernel)
        let source = mask
        source.withUnsafeBytes { srcRaw in
            mask.withUnsafeMutableBytes { dstRaw in
                guard let s = srcRaw.baseAddress, let d = dstRaw.baseAddress else { return }
                var src = vImage_Buffer(data: UnsafeMutableRawPointer(mutating: s), height: vImagePixelCount(height),
                                        width: vImagePixelCount(width), rowBytes: width)
                var dst = vImage_Buffer(data: d, height: vImagePixelCount(height), width: vImagePixelCount(width),
                                        rowBytes: width)
                if erode {
                    _ = vImageMin_Planar8(&src, &dst, nil, 0, 0, k, k, vImage_Flags(kvImageEdgeExtend))
                } else {
                    _ = vImageMax_Planar8(&src, &dst, nil, 0, 0, k, k, vImage_Flags(kvImageEdgeExtend))
                }
            }
        }
    }

    /// Pixels that can belong to the head: generous around the face rectangle (hair, ears, back of the head in
    /// profile views), further down when shoulders are kept.
    static func regionOfInterest(face: CGRect, keepShoulders: Bool, width: Int, height: Int) -> PixelRect {
        let below: CGFloat = keepShoulders ? 3.0 : 1.0
        let r = CGRect(x: face.minX - 1.1 * face.width, y: face.minY - 1.2 * face.height,
                       width: 3.2 * face.width, height: (2.2 + below) * face.height)
        return PixelRect(r, clippedTo: width, height) ?? PixelRect(x: 0, y: 0, width: 0, height: 0)
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
