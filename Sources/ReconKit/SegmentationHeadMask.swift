// License: Apache 2.0. See LICENSE file in root directory.
//
// Per-keyframe head mask refinement for the reconstruction: the keyframe's person/head mask intersected with a
// depth band around the face, reduced to the connected region that contains the face, and cut below the chin.
// The neck and shoulders do not rotate with the head, so registration always uses the cut mask; fusion keeps the
// shoulders of the frontal views only when ReconstructionOptions.keepShoulders is set.

import CameraKit
import CoreGraphics
import Foundation
import ScanKit
import simd

/// Depth image of a keyframe in meters with pixel access (copied once; Data access per pixel is slow).
struct DepthMap: Sendable {
    let width: Int
    let height: Int
    let scale: Float
    let raw: [UInt16]
    let intrinsics: Intrinsics

    init?(_ depth: DepthImage) {
        let count = depth.width * depth.height
        guard depth.width > 0, depth.height > 0, depth.pixels.count >= count * 2, depth.depthScale > 0,
              depth.depthScale.isFinite else { return nil }
        width = depth.width
        height = depth.height
        scale = depth.depthScale
        intrinsics = depth.intrinsics
        raw = depth.pixels.withUnsafeBytes { bytes in
            (0 ..< count).map { UInt16(littleEndian: bytes.loadUnaligned(fromByteOffset: 2 * $0, as: UInt16.self)) }
        }
    }

    @inline(__always) func meters(_ x: Int, _ y: Int) -> Float {
        guard x >= 0, y >= 0, x < width, y < height else { return 0 }
        return Float(raw[y * width + x]) * scale
    }

    /// Median of the valid depths within `radius` pixels of `pixel` that fall into `band`.
    func median(around pixel: SIMD2<Float>, radius: Int, band: ClosedRange<Float>) -> Float? {
        guard pixel.x.isFinite, pixel.y.isFinite else { return nil }
        let cx = Int(pixel.x.rounded()), cy = Int(pixel.y.rounded())
        var values: [Float] = []
        for y in (cy - radius) ... (cy + radius) {
            for x in (cx - radius) ... (cx + radius) {
                let z = meters(x, y)
                if z > 0, band.contains(z) { values.append(z) }
            }
        }
        guard values.count >= max(1, (2 * radius + 1) * (2 * radius + 1) / 4) else { return nil }
        values.sort()
        return values[values.count / 2]
    }

    /// Camera-frame point (meters) of a pixel at depth z.
    func point(_ pixel: SIMD2<Float>, depth z: Float) -> SIMD3<Float> {
        intrinsics.deproject(pixel, depth: z)
    }
}

/// Refined masks and statistics of one keyframe.
struct SegmentedKeyframe: Sendable {
    /// Head only, cut below the chin (registration, initial poses).
    var headMask: Data
    /// Mask used for fusion and texturing (head, plus neck/shoulders of frontal views when keepShoulders).
    var fusionMask: Data
    /// Valid depth points inside headMask.
    var validPoints: Int
    /// Median depth of the face (meters).
    var faceDepth: Float
    /// Head roll in the image used for the chin cut (radians, clockwise positive).
    var rollRadians: Float
    /// rollRadians comes from the landmark eye line (not the Vision roll).
    var rollFromLandmarks: Bool
    var depth: DepthMap
}

enum HeadSegmentation {
    /// Depth band around the face depth: the nose is at most this much in front, the back of the visible head
    /// this much behind.
    static let bandInFront: Float = 0.12
    static let bandBehind: Float = 0.25

    static func minimumPoints(width: Int, height: Int) -> Int { max(300, width * height / 500) }

    /// Returns nil when the keyframe has no usable face depth.
    static func segment(_ keyframe: Keyframe, keepShoulders: Bool) -> SegmentedKeyframe? {
        guard let depth = DepthMap(keyframe.frame.alignedDepth) else { return nil }
        let w = depth.width, h = depth.height
        let rect = clampedFaceRect(keyframe.faceRect, width: w, height: h)

        // Face depth: median over the inner half of the face rectangle.
        let inner = rect.insetBy(dx: rect.width / 4, dy: rect.height / 4)
        var faceDepths: [Float] = []
        for y in Int(inner.minY) ..< max(Int(inner.minY) + 1, Int(inner.maxY)) {
            for x in Int(inner.minX) ..< max(Int(inner.minX) + 1, Int(inner.maxX)) {
                let z = depth.meters(x, y)
                if z > 0.15, z < 1.5 { faceDepths.append(z) }
            }
        }
        guard faceDepths.count >= 10 else { return nil }
        faceDepths.sort()
        let faceDepth = faceDepths[faceDepths.count / 2]
        let band = (faceDepth - bandInFront) ... (faceDepth + bandBehind)

        // Candidates: keyframe mask (or a generous region around the face) with depth in the band.
        let personMask: [UInt8]? = keyframe.mask.flatMap { $0.count == w * h ? [UInt8]($0) : nil }
        let rx0 = Int(rect.minX - 0.7 * rect.width), rx1 = Int(rect.maxX + 0.7 * rect.width)
        let ry0 = Int(rect.minY - 0.9 * rect.height), ry1 = Int(rect.maxY + 1.5 * rect.height)
        var candidate = [UInt8](repeating: 0, count: w * h)
        depth.raw.withUnsafeBufferPointer { raw in
            let lo = band.lowerBound / depth.scale, hi = band.upperBound / depth.scale
            for y in 0 ..< h {
                let rowInRegion = y >= ry0 && y <= ry1
                for x in 0 ..< w {
                    let i = y * w + x
                    let r = Float(raw[i])
                    guard r > 0, r >= lo, r <= hi else { continue }
                    if let personMask {
                        if personMask[i] < 128 { continue }
                    } else if !(rowInRegion && x >= rx0 && x <= rx1) {
                        continue
                    }
                    candidate[i] = 1
                }
            }
        }

        // The connected region containing the face removes background objects at a similar depth.
        let component = faceComponent(candidate, width: w, height: h, seedRect: inner)

        // Chin cut, perpendicular to the head's vertical axis in the image.
        let (roll, rollFromLandmarks) = estimateImageRoll(landmarks: keyframe.landmarks, fallbackDegrees: keyframe.headPose.roll)
        let down = SIMD2<Float>(-sin(roll), cos(roll))
        let chin = chinPixel(landmarks: keyframe.landmarks, faceRect: rect, down: down)
        let margin = Float(rect.height) * 0.08
        var head = [UInt8](repeating: 0, count: w * h)
        var validPoints = 0
        for y in 0 ..< h {
            for x in 0 ..< w {
                let i = y * w + x
                guard component[i] != 0 else { continue }
                if simd_dot(SIMD2(Float(x), Float(y)) - chin, down) > margin { continue }
                head[i] = 255
                validPoints += 1
            }
        }
        guard validPoints >= minimumPoints(width: w, height: h) else { return nil }
        let fusion: [UInt8]
        if keepShoulders && keyframe.scanPose == .front {
            fusion = component.map { $0 != 0 ? 255 : 0 }
        } else {
            fusion = head
        }
        return SegmentedKeyframe(headMask: Data(head), fusionMask: Data(fusion), validPoints: validPoints,
                                 faceDepth: faceDepth, rollRadians: roll, rollFromLandmarks: rollFromLandmarks, depth: depth)
    }

    static func clampedFaceRect(_ rect: CGRect, width: Int, height: Int) -> CGRect {
        let bounds = CGRect(x: 0, y: 0, width: width, height: height)
        let r = rect.standardized.intersection(bounds)
        if r.isNull || r.width < 4 || r.height < 4 || !r.width.isFinite {
            // No usable face rectangle: assume a centered face.
            return CGRect(x: Double(width) * 0.35, y: Double(height) * 0.3, width: Double(width) * 0.3, height: Double(height) * 0.4)
        }
        return r
    }

    /// 4-connected component of `candidate` that touches `seedRect` most (largest component when none does).
    static func faceComponent(_ candidate: [UInt8], width w: Int, height h: Int, seedRect: CGRect) -> [UInt8] {
        var label = [Int32](repeating: 0, count: w * h)
        var sizes: [Int] = [0]
        var seedHits: [Int] = [0]
        var stack: [Int] = []
        stack.reserveCapacity(4096)
        let sx0 = Int(seedRect.minX), sx1 = Int(seedRect.maxX), sy0 = Int(seedRect.minY), sy1 = Int(seedRect.maxY)
        for start in 0 ..< w * h where candidate[start] != 0 && label[start] == 0 {
            let id = Int32(sizes.count)
            sizes.append(0)
            seedHits.append(0)
            label[start] = id
            stack.append(start)
            while let i = stack.popLast() {
                sizes[Int(id)] += 1
                let x = i % w, y = i / w
                if x >= sx0, x < sx1, y >= sy0, y < sy1 { seedHits[Int(id)] += 1 }
                if x > 0, candidate[i - 1] != 0, label[i - 1] == 0 { label[i - 1] = id; stack.append(i - 1) }
                if x + 1 < w, candidate[i + 1] != 0, label[i + 1] == 0 { label[i + 1] = id; stack.append(i + 1) }
                if y > 0, candidate[i - w] != 0, label[i - w] == 0 { label[i - w] = id; stack.append(i - w) }
                if y + 1 < h, candidate[i + w] != 0, label[i + w] == 0 { label[i + w] = id; stack.append(i + w) }
            }
        }
        guard sizes.count > 1 else { return [UInt8](repeating: 0, count: w * h) }
        var best = 1
        for id in 1 ..< sizes.count {
            if seedHits[id] > seedHits[best] || (seedHits[id] == seedHits[best] && sizes[id] > sizes[best]) { best = id }
        }
        let keep = Int32(best)
        return label.map { $0 == keep ? 1 : 0 }
    }

    /// Roll of the head in the image (radians, clockwise positive) from the landmark constellation: the line
    /// between the centroids of the upper-left and upper-right landmarks (eyes, brows, temples) is parallel to the
    /// eye line for any symmetric landmark set, whatever the landmark ordering. Falls back to the Vision roll
    /// (second value false).
    static func estimateImageRoll(landmarks: [SIMD2<Float>], fallbackDegrees: Float) -> (Float, Bool) {
        let fallback = ((fallbackDegrees.isFinite ? fallbackDegrees : 0) * .pi / 180, false)
        let pts = landmarks.filter { $0.x.isFinite && $0.y.isFinite }
        guard pts.count >= 8 else { return fallback }
        let minP = pts.reduce(SIMD2<Float>(repeating: .infinity)) { simd_min($0, $1) }
        let maxP = pts.reduce(SIMD2<Float>(repeating: -.infinity)) { simd_max($0, $1) }
        let center = (minP + maxP) / 2
        let size = maxP - minP
        guard size.x > 4, size.y > 4 else { return fallback }
        var roll = fallback.0
        for _ in 0 ..< 3 {
            let c = cos(-roll), s = sin(-roll)
            var left = SIMD2<Float>.zero, right = SIMD2<Float>.zero
            var nl = 0, nr = 0
            for p in pts {
                let d = p - center
                let q = SIMD2(c * d.x - s * d.y, s * d.x + c * d.y)   // leveled
                guard q.y < -0.05 * size.y, abs(q.x) > 0.1 * size.x else { continue }
                if q.x < 0 { left += p; nl += 1 } else { right += p; nr += 1 }
            }
            guard nl >= 2, nr >= 2 else { return fallback }
            let d = right / Float(nr) - left / Float(nl)
            roll = atan2(d.y, d.x)
        }
        // A wildly different estimate means an unusual constellation; trust Vision then.
        return abs(roll - fallback.0) < 15 * .pi / 180 ? (roll, true) : fallback
    }

    /// Lowest point of the face along the head's down direction: the chin landmark, or the face rectangle bottom.
    static func chinPixel(landmarks: [SIMD2<Float>], faceRect: CGRect, down: SIMD2<Float>) -> SIMD2<Float> {
        let pts = landmarks.filter { $0.x.isFinite && $0.y.isFinite }
        if pts.count >= 8, let chin = pts.max(by: { simd_dot($0, down) < simd_dot($1, down) }) { return chin }
        return SIMD2(Float(faceRect.midX), Float(faceRect.maxY))
    }
}
