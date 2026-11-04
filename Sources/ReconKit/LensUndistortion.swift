// License: Apache 2.0. See LICENSE file in root directory.
//
// Lens distortion of the color camera and the one-time undistortion of keyframes before reconstruction.
//
// The D455 RGB sensor reports inverse Brown-Conrady coefficients (e.g. k1 -0.0557, k2 0.0675, k3 -0.0218), while
// FaceReconCore (clouds, TSDF on the CPU and on Metal, texture projection) works with an ideal pinhole. The pipeline
// therefore remaps every keyframe once to the pinhole camera with the same fx, fy, ppx, ppy: color bilinearly, the
// head mask and (when it is distorted) the aligned depth by nearest neighbor, the Vision landmarks and face
// rectangle through the inverse model, and marks the intrinsics distortion-free. docs/RECONSTRUCTION.md, "Lens
// distortion".

import CameraKit
import CoreGraphics
import Foundation
import ScanKit
import simd

/// A librealsense distortion model (rs2_distortion) with the math of rs2_project_point_to_pixel and
/// rs2_deproject_pixel_to_point (librealsense 2.58.4, src/rs.cpp), ported operation by operation in Float, on
/// normalized image coordinates (x / z, y / z before fx, fy and the principal point).
///
/// In librealsense 2.58 the polynomial of every Brown-Conrady variant maps ideal (pinhole) coordinates to the
/// coordinates in the image ("distort", closed form); the inverse ("undistort") iterates 10 times.
/// Brown-Conrady evaluates the tangential terms on the coordinates before the radial factor, the inverse and the
/// modified variants after it. librealsense cannot deproject the modified model; its inverse here is the inverse
/// Brown-Conrady iteration, which inverts the identical forward formula.
public struct LensDistortion: Sendable, Hashable {
    public enum Model: String, Sendable, Hashable, CaseIterable {
        case none
        case modifiedBrownConrady = "modified_brown_conrady"
        case inverseBrownConrady = "inverse_brown_conrady"
        case ftheta
        case brownConrady = "brown_conrady"
        case kannalaBrandt4 = "kannala_brandt4"
    }

    public let model: Model
    /// k1, k2, p1, p2, k3 (Brown-Conrady variants) or the fisheye coefficients, as in rs2_intrinsics.coeffs.
    public let coeffs: [Float]
    // Scalar copies: the remap loops run in debug builds, where array and SIMD access is slow.
    private let c0, c1, c2, c3, c4: Float

    public init(model: Model, coeffs: [Float]) {
        self.model = model
        let c = Array((coeffs + [0, 0, 0, 0, 0]).prefix(5))
        self.coeffs = c
        (c0, c1, c2, c3, c4) = (c[0], c[1], c[2], c[3], c[4])
    }

    /// The model of a stream's intrinsics; nil when the model name is unknown.
    public init?(_ intrinsics: Intrinsics) {
        guard let model = Model(rawValue: intrinsics.model) else { return nil }
        self.init(model: model, coeffs: intrinsics.coeffs)
    }

    /// The D455 color sensor (OV9782) as the helper's synthetic recordings and typical units report it.
    public static let d455Color = LensDistortion(model: .inverseBrownConrady, coeffs: [-0.0557, 0.0675, -0.0006, 0.0006, -0.0218])

    /// librealsense's is_intrinsics_distortion_zero.
    private var coefficientsAreZero: Bool {
        abs(c0) < .ulpOfOne && abs(c1) < .ulpOfOne && abs(c2) < .ulpOfOne && abs(c3) < .ulpOfOne && abs(c4) < .ulpOfOne
    }

    /// True when projecting through the model changes nothing (no model, or Brown-Conrady with zero coefficients).
    public var isIdentity: Bool {
        switch model {
        case .none: return true
        case .modifiedBrownConrady, .inverseBrownConrady, .brownConrady: return coefficientsAreZero
        case .ftheta, .kannalaBrandt4: return false
        }
    }

    /// Whether librealsense's depth-to-color align maps depth through this color distortion, i.e. whether the
    /// aligned depth pixel (u, v) is the surface seen by the distorted color pixel (u, v) or by the pinhole pixel.
    /// The generic align (src/proc/align.cpp) projects with rs2_project_point_to_pixel and so follows every model,
    /// but it is compiled only without SIMD: the NEON align of Apple Silicon (src/proc/neon/neon-align.cpp, built by
    /// default on arm64 and used by the helper) and the SSE align of x86 apply the distortion for modified
    /// Brown-Conrady only and project every other model as an ideal pinhole, although the aligned frame reports the
    /// color intrinsics with their coefficients. Measured with the helper's SDK build: a depth pixel at (1200, 400)
    /// lands on color pixel (1200, 400) with the D455 inverse Brown-Conrady coefficients, on (1193, 399) with the
    /// same coefficients as modified Brown-Conrady (rs2_project_point_to_pixel: 1193.66, 399.72).
    public var alignedDepthFollowsDistortion: Bool {
        model == .modifiedBrownConrady && !isIdentity
    }

    /// rs2_project_point_to_pixel without the camera matrix: ideal normalized coordinates -> distorted ones.
    @inline(__always)
    public func distort(x xIn: Float, y yIn: Float) -> (x: Float, y: Float) {
        var x = xIn, y = yIn
        switch model {
        case .none:
            break
        case .modifiedBrownConrady, .inverseBrownConrady:
            let r2 = x * x + y * y
            let f = 1 + c0 * r2 + c1 * r2 * r2 + c4 * r2 * r2 * r2
            x *= f
            y *= f
            let dx = x + 2 * c2 * x * y + c3 * (r2 + 2 * x * x)
            let dy = y + 2 * c3 * x * y + c2 * (r2 + 2 * y * y)
            x = dx
            y = dy
        case .brownConrady:
            let r2 = x * x + y * y
            let f = 1 + c0 * r2 + c1 * r2 * r2 + c4 * r2 * r2 * r2
            let xf = x * f, yf = y * f
            let dx = xf + 2 * c2 * x * y + c3 * (r2 + 2 * x * x)
            let dy = yf + 2 * c3 * x * y + c2 * (r2 + 2 * y * y)
            x = dx
            y = dy
        case .ftheta:
            var r = (x * x + y * y).squareRoot()
            if r < .ulpOfOne { r = .ulpOfOne }
            let rd = 1 / c0 * atan(2 * r * tan(c0 / 2))
            x *= rd / r
            y *= rd / r
        case .kannalaBrandt4:
            var r = (x * x + y * y).squareRoot()
            if r < .ulpOfOne { r = .ulpOfOne }
            let theta = atan(r)
            let theta2 = theta * theta
            let series = 1 + theta2 * (c0 + theta2 * (c1 + theta2 * (c2 + theta2 * c3)))
            let rd = theta * series
            x *= rd / r
            y *= rd / r
        }
        return (x, y)
    }

    /// rs2_deproject_pixel_to_point without the camera matrix: distorted normalized coordinates -> ideal ones.
    @inline(__always)
    public func undistort(x xIn: Float, y yIn: Float) -> (x: Float, y: Float) {
        var x = xIn, y = yIn
        let xo = x, yo = y
        if !coefficientsAreZero {
            switch model {
            case .inverseBrownConrady, .modifiedBrownConrady:
                for _ in 0 ..< 10 {
                    let r2 = x * x + y * y
                    let icdist = 1 / (1 + ((c4 * r2 + c1) * r2 + c0) * r2)
                    let xq = x / icdist, yq = y / icdist
                    let deltaX = 2 * c2 * xq * yq + c3 * (r2 + 2 * xq * xq)
                    let deltaY = 2 * c3 * xq * yq + c2 * (r2 + 2 * yq * yq)
                    x = (xo - deltaX) * icdist
                    y = (yo - deltaY) * icdist
                }
            case .brownConrady:
                for _ in 0 ..< 10 {
                    let r2 = x * x + y * y
                    let icdist = 1 / (1 + ((c4 * r2 + c1) * r2 + c0) * r2)
                    let deltaX = 2 * c2 * x * y + c3 * (r2 + 2 * x * x)
                    let deltaY = 2 * c3 * x * y + c2 * (r2 + 2 * y * y)
                    x = (xo - deltaX) * icdist
                    y = (yo - deltaY) * icdist
                }
            case .none, .ftheta, .kannalaBrandt4:
                break
            }
        }
        if model == .kannalaBrandt4 {
            var rd = (x * x + y * y).squareRoot()
            if rd < .ulpOfOne { rd = .ulpOfOne }
            var theta = rd
            var theta2 = rd * rd
            for _ in 0 ..< 4 {
                let f = theta * (1 + theta2 * (c0 + theta2 * (c1 + theta2 * (c2 + theta2 * c3)))) - rd
                if abs(f) < .ulpOfOne { break }
                let df = 1 + theta2 * (3 * c0 + theta2 * (5 * c1 + theta2 * (7 * c2 + 9 * theta2 * c3)))
                theta -= f / df
                theta2 = theta * theta
            }
            let r = tan(theta)
            x *= r / rd
            y *= r / rd
        }
        if model == .ftheta {
            var rd = (x * x + y * y).squareRoot()
            if rd < .ulpOfOne { rd = .ulpOfOne }
            let r: Float = abs(c0) < .ulpOfOne ? 0 : tan(c0 * rd) / atan(2 * tan(c0 / 2))
            x *= r / rd
            y *= r / rd
        }
        return (x, y)
    }

    /// Pixel of the distorted image -> pixel of the ideal pinhole image with the same camera matrix.
    public func undistortPixel(_ p: SIMD2<Float>, _ k: Intrinsics) -> SIMD2<Float> {
        let n = undistort(x: (p.x - k.ppx) / k.fx, y: (p.y - k.ppy) / k.fy)
        return SIMD2(n.x * k.fx + k.ppx, n.y * k.fy + k.ppy)
    }

    /// Pixel of the ideal pinhole image -> pixel of the distorted image (where the camera sees that ray).
    public func distortPixel(_ p: SIMD2<Float>, _ k: Intrinsics) -> SIMD2<Float> {
        let n = distort(x: (p.x - k.ppx) / k.fx, y: (p.y - k.ppy) / k.fy)
        return SIMD2(n.x * k.fx + k.ppx, n.y * k.fy + k.ppy)
    }

    /// rs2_project_point_to_pixel: camera-frame point (meters) -> pixel of the distorted image.
    public func project(_ p: SIMD3<Float>, _ k: Intrinsics) -> SIMD2<Float> {
        let n = distort(x: p.x / p.z, y: p.y / p.z)
        return SIMD2(n.x * k.fx + k.ppx, n.y * k.fy + k.ppy)
    }

    /// rs2_deproject_pixel_to_point: pixel of the distorted image at depth z -> camera-frame point.
    public func deproject(_ pixel: SIMD2<Float>, depth z: Float, _ k: Intrinsics) -> SIMD3<Float> {
        let n = undistort(x: (pixel.x - k.ppx) / k.fx, y: (pixel.y - k.ppy) / k.fy)
        return SIMD3(z * n.x, z * n.y, z)
    }
}

extension Intrinsics {
    /// The same camera matrix without distortion.
    var pinhole: Intrinsics {
        Intrinsics(width: width, height: height, fx: fx, fy: fy, ppx: ppx, ppy: ppy, model: "none", coeffs: [0, 0, 0, 0, 0])
    }
}

/// Remaps keyframes from the distorted color camera to the ideal pinhole camera with the same camera matrix.
enum KeyframeUndistortion {
    /// For each pixel of the pinhole image, where the distorted image sees the same ray.
    struct Map: Sendable {
        let width: Int
        let height: Int
        let sourceX: [Float]
        let sourceY: [Float]

        init(intrinsics k: Intrinsics, lens: LensDistortion) {
            width = k.width
            height = k.height
            var xs = [Float](repeating: 0, count: k.width * k.height), ys = xs
            let fx = k.fx, fy = k.fy, cx = k.ppx, cy = k.ppy, w = k.width
            xs.withUnsafeMutableBufferPointer { xb in
                ys.withUnsafeMutableBufferPointer { yb in
                    let px = xb.baseAddress!, py = yb.baseAddress!
                    DispatchQueue.concurrentPerform(iterations: k.height) { y in
                        let ny = (Float(y) - cy) / fy
                        for x in 0 ..< w {
                            let d = lens.distort(x: (Float(x) - cx) / fx, y: ny)
                            px[y * w + x] = d.x * fx + cx
                            py[y * w + x] = d.y * fy + cy
                        }
                    }
                }
            }
            sourceX = xs
            sourceY = ys
        }
    }

    /// Undistorts the keyframes that need it (same order; keyframes without distortion are returned unchanged).
    /// - Parameter depthFollowsDistortion: overrides LensDistortion.alignedDepthFollowsDistortion (tests).
    /// - Returns: the keyframes and the distortion models that were removed.
    static func undistort(_ keyframes: [Keyframe], indices: [Int], depthFollowsDistortion: Bool? = nil)
        -> (keyframes: [Keyframe], removed: [LensDistortion.Model], unknownModels: [String]) {
        var maps: [Intrinsics: Map] = [:]
        var jobs: [(index: Int, lens: LensDistortion)] = []
        var removed = Set<LensDistortion.Model>(), unknown = Set<String>()
        for i in indices {
            let k = keyframes[i].frame.color.intrinsics
            guard let lens = LensDistortion(k) else { unknown.insert(k.model); continue }
            guard !lens.isIdentity else { continue }
            if maps[k] == nil { maps[k] = Map(intrinsics: k, lens: lens) }
            jobs.append((i, lens))
            removed.insert(lens.model)
        }
        var out = keyframes
        if !jobs.isEmpty {
            let results = LockedArray<Keyframe?>(count: jobs.count, value: nil)
            DispatchQueue.concurrentPerform(iterations: jobs.count) { j in
                let (i, lens) = jobs[j]
                let kf = keyframes[i]
                let map = maps[kf.frame.color.intrinsics]!
                results.set(j, undistort(kf, map: map, lens: lens,
                                         depthFollowsDistortion: depthFollowsDistortion ?? lens.alignedDepthFollowsDistortion))
            }
            for (j, kf) in results.values.enumerated() { out[jobs[j].index] = kf! }
        }
        return (out, removed.sorted { $0.rawValue < $1.rawValue }, unknown.sorted())
    }

    /// One keyframe; its color and depth must have the map's size (PipelineRun.isStructurallyValid).
    static func undistort(_ kf: Keyframe, map: Map, lens: LensDistortion, depthFollowsDistortion: Bool) -> Keyframe {
        var out = kf
        let k = kf.frame.color.intrinsics
        let pinhole = k.pinhole
        out.frame.color.pixels = remapColor(kf.frame.color.pixels, map: map)
        out.frame.color.intrinsics = pinhole
        if depthFollowsDistortion {
            let depth = kf.frame.alignedDepth
            out.frame.alignedDepth.pixels = remapDepth(depth.pixels, scale: depth.depthScale, fx: k.fx, map: map)
        }
        out.frame.alignedDepth.intrinsics = pinhole
        if let mask = kf.mask, mask.count == map.width * map.height {
            out.mask = remapNearest(mask, bytesPerPixel: 1, map: map)
        }
        out.landmarks = kf.landmarks.map { $0.x.isFinite && $0.y.isFinite ? lens.undistortPixel($0, k) : $0 }
        out.faceRect = undistortRect(kf.faceRect, lens: lens, intrinsics: k)
        return out
    }

    /// Bounding box of the undistorted rectangle outline (corners and edge midpoints).
    static func undistortRect(_ rect: CGRect, lens: LensDistortion, intrinsics k: Intrinsics) -> CGRect {
        let r = rect.standardized
        guard !r.isNull, r.width.isFinite, r.height.isFinite, r.width > 0, r.height > 0 else { return rect }
        var lo = SIMD2<Float>(repeating: .infinity), hi = SIMD2<Float>(repeating: -.infinity)
        for fx in [0.0, 0.5, 1.0] {
            for fy in [0.0, 0.5, 1.0] where fx != 0.5 || fy != 0.5 {
                let p = lens.undistortPixel(SIMD2(Float(r.minX + fx * r.width), Float(r.minY + fy * r.height)), k)
                lo = simd_min(lo, p)
                hi = simd_max(hi, p)
            }
        }
        return CGRect(x: CGFloat(lo.x), y: CGFloat(lo.y), width: CGFloat(hi.x - lo.x), height: CGFloat(hi.y - lo.y))
    }

    /// Bilinear RGB8 remap; pixels whose source lies outside the image become black.
    static func remapColor(_ pixels: Data, map: Map) -> Data {
        let w = map.width, h = map.height
        var out = Data(count: w * h * 3)
        pixels.withUnsafeBytes { (src: UnsafeRawBufferPointer) in
            out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
                map.sourceX.withUnsafeBufferPointer { xb in
                    map.sourceY.withUnsafeBufferPointer { yb in
                        let s = src.bindMemory(to: UInt8.self).baseAddress!, d = dst.bindMemory(to: UInt8.self).baseAddress!
                        let sx = xb.baseAddress!, sy = yb.baseAddress!
                        let maxX = Float(w - 1), maxY = Float(h - 1)
                        DispatchQueue.concurrentPerform(iterations: h) { y in
                            for x in 0 ..< w {
                                let i = y * w + x
                                var u = sx[i], v = sy[i]
                                guard u > -0.5, v > -0.5, u < maxX + 0.5, v < maxY + 0.5 else { continue }
                                u = min(max(u, 0), maxX)
                                v = min(max(v, 0), maxY)
                                let x0 = Int(u), y0 = Int(v)
                                let x1 = min(x0 + 1, w - 1), y1 = min(y0 + 1, h - 1)
                                let ax = u - Float(x0), ay = v - Float(y0)
                                let w00 = (1 - ax) * (1 - ay), w10 = ax * (1 - ay), w01 = (1 - ax) * ay, w11 = ax * ay
                                let p00 = 3 * (y0 * w + x0), p10 = 3 * (y0 * w + x1), p01 = 3 * (y1 * w + x0), p11 = 3 * (y1 * w + x1)
                                for c in 0 ..< 3 {
                                    let value = w00 * Float(s[p00 + c]) + w10 * Float(s[p10 + c]) + w01 * Float(s[p01 + c])
                                        + w11 * Float(s[p11 + c])
                                    d[3 * i + c] = UInt8(min(255, value + 0.5))
                                }
                            }
                        }
                    }
                }
            }
        }
        return out
    }

    /// Z16 depth remap that never mixes depths of different surfaces: bilinear where the four source neighbors are
    /// valid and lie on one surface (spread within max(5 mm, 2 z / fx tan 75 deg), the silhouette test of
    /// fr_cloud_from_rgbd), the nearest pixel otherwise. Nearest everywhere would put up to half a pixel of lateral
    /// jitter, the same for every keyframe, into sloped surfaces: on the off-center synthetic scan that cost 0.15 deg
    /// mean rotation error instead of 0.04 deg. Outside the image: 0 (no data).
    static func remapDepth(_ pixels: Data, scale: Float, fx: Float, map: Map) -> Data {
        let w = map.width, h = map.height
        var out = Data(count: w * h * 2)
        let slope = 2 * tan(75 * Float.pi / 180) / fx   // same-surface spread per depth unit of z
        let floorUnits = 0.005 / scale
        pixels.withUnsafeBytes { (src: UnsafeRawBufferPointer) in
            out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
                map.sourceX.withUnsafeBufferPointer { xb in
                    map.sourceY.withUnsafeBufferPointer { yb in
                        let s = src.bindMemory(to: UInt16.self).baseAddress!, d = dst.bindMemory(to: UInt16.self).baseAddress!
                        let sx = xb.baseAddress!, sy = yb.baseAddress!
                        DispatchQueue.concurrentPerform(iterations: h) { y in
                            for x in 0 ..< w {
                                let i = y * w + x
                                let u = sx[i], v = sy[i]
                                let nu = (u + 0.5).rounded(.down), nv = (v + 0.5).rounded(.down)
                                guard nu >= 0, nv >= 0, nu < Float(w), nv < Float(h) else { continue }
                                var value = UInt16(littleEndian: s[Int(nv) * w + Int(nu)])
                                let bu = u.rounded(.down), bv = v.rounded(.down)
                                if bu >= 0, bv >= 0, bu + 1 < Float(w), bv + 1 < Float(h) {
                                    let j = Int(bv) * w + Int(bu)
                                    let d00 = Float(UInt16(littleEndian: s[j])), d10 = Float(UInt16(littleEndian: s[j + 1]))
                                    let d01 = Float(UInt16(littleEndian: s[j + w])), d11 = Float(UInt16(littleEndian: s[j + w + 1]))
                                    let lo = min(min(d00, d10), min(d01, d11)), hi = max(max(d00, d10), max(d01, d11))
                                    if lo > 0, hi - lo <= max(floorUnits, slope * lo) {
                                        let ax = u - bu, ay = v - bv
                                        let z = (d00 * (1 - ax) + d10 * ax) * (1 - ay) + (d01 * (1 - ax) + d11 * ax) * ay
                                        value = UInt16(min(65535, z + 0.5))
                                    }
                                }
                                d[i] = value.littleEndian
                            }
                        }
                    }
                }
            }
        }
        return out
    }

    /// Nearest-neighbor remap (masks): never mixes values across edges. Outside the image: 0.
    static func remapNearest(_ pixels: Data, bytesPerPixel bpp: Int, map: Map) -> Data {
        let w = map.width, h = map.height
        var out = Data(count: w * h * bpp)
        pixels.withUnsafeBytes { (src: UnsafeRawBufferPointer) in
            out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
                map.sourceX.withUnsafeBufferPointer { xb in
                    map.sourceY.withUnsafeBufferPointer { yb in
                        let s = src.bindMemory(to: UInt8.self).baseAddress!, d = dst.bindMemory(to: UInt8.self).baseAddress!
                        let sx = xb.baseAddress!, sy = yb.baseAddress!
                        DispatchQueue.concurrentPerform(iterations: h) { y in
                            for x in 0 ..< w {
                                let i = y * w + x
                                // Integer pixel centers: pixel (x0, y0) covers [x0 - 0.5, x0 + 0.5).
                                let u = (sx[i] + 0.5).rounded(.down), v = (sy[i] + 0.5).rounded(.down)
                                guard u >= 0, v >= 0, u < Float(w), v < Float(h) else { continue }
                                let j = Int(v) * w + Int(u)
                                for b in 0 ..< bpp { d[bpp * i + b] = s[bpp * j + b] }
                            }
                        }
                    }
                }
            }
        }
        return out
    }
}
