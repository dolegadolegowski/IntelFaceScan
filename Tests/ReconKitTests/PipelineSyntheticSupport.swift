// License: Apache 2.0. See LICENSE file in root directory.
//
// A minimal synthetic guided scan for the pipeline tests: a head-like union of ellipsoids that rotates about its
// center in front of a static camera (with a static neck and a back wall), rendered to RGB-D keyframes with the
// D455 noise model, Vision-like head poses (with errors), landmarks and person masks. Scalar Double math: tests
// build without optimization.

import CameraKit
import CoreGraphics
import Foundation
import ReconKit
import ScanKit
import simd

struct P3 {
    var x, y, z: Double
    init(_ x: Double, _ y: Double, _ z: Double) { self.x = x; self.y = y; self.z = z }
    static func + (a: P3, b: P3) -> P3 { P3(a.x + b.x, a.y + b.y, a.z + b.z) }
    static func - (a: P3, b: P3) -> P3 { P3(a.x - b.x, a.y - b.y, a.z - b.z) }
    static func * (a: P3, s: Double) -> P3 { P3(a.x * s, a.y * s, a.z * s) }
    func dot(_ b: P3) -> Double { x * b.x + y * b.y + z * b.z }
    var length: Double { dot(self).squareRoot() }
    var simd: SIMD3<Float> { SIMD3(Float(x), Float(y), Float(z)) }
    init(_ v: SIMD3<Float>) { x = Double(v.x); y = Double(v.y); z = Double(v.z) }
}

/// Row-major rotation.
struct Rot {
    var m: [Double]
    static let identity = Rot(m: [1, 0, 0, 0, 1, 0, 0, 0, 1])
    init(m: [Double]) { self.m = m }
    init(_ q: simd_quatf) {
        let r = simd_float3x3(q)
        m = [Double(r.columns.0.x), Double(r.columns.1.x), Double(r.columns.2.x),
             Double(r.columns.0.y), Double(r.columns.1.y), Double(r.columns.2.y),
             Double(r.columns.0.z), Double(r.columns.1.z), Double(r.columns.2.z)]
    }
    func apply(_ p: P3) -> P3 {
        P3(m[0] * p.x + m[1] * p.y + m[2] * p.z, m[3] * p.x + m[4] * p.y + m[5] * p.z, m[6] * p.x + m[7] * p.y + m[8] * p.z)
    }
    var transposed: Rot { Rot(m: [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]) }
}

struct SynthEllipsoid {
    var c: P3
    var r: P3
    /// Ray parameter of the first hit (origin outside).
    func hit(_ o: P3, _ d: P3) -> Double? {
        let ox = (o.x - c.x) / r.x, oy = (o.y - c.y) / r.y, oz = (o.z - c.z) / r.z
        let dx = d.x / r.x, dy = d.y / r.y, dz = d.z / r.z
        let a = dx * dx + dy * dy + dz * dz, b = 2 * (ox * dx + oy * dy + oz * dz), cc = ox * ox + oy * oy + oz * oz - 1
        let disc = b * b - 4 * a * cc
        guard disc >= 0 else { return nil }
        let t = (-b - disc.squareRoot()) / (2 * a)
        return t > 1e-6 ? t : nil
    }
}

/// The head in its canonical frame (camera convention: +Y down, face toward -Z), centered at the origin.
let headParts: [SynthEllipsoid] = [
    SynthEllipsoid(c: P3(0, -0.01, 0.0), r: P3(0.075, 0.095, 0.095)),
    SynthEllipsoid(c: P3(0, 0.035, -0.035), r: P3(0.062, 0.062, 0.062)),
    SynthEllipsoid(c: P3(0, 0.012, -0.093), r: P3(0.011, 0.024, 0.02)),
    SynthEllipsoid(c: P3(0, -0.032, -0.082), r: P3(0.055, 0.011, 0.018)),
    SynthEllipsoid(c: P3(0.038, 0.022, -0.072), r: P3(0.024, 0.02, 0.02)),
    SynthEllipsoid(c: P3(-0.038, 0.022, -0.072), r: P3(0.024, 0.02, 0.02)),
    SynthEllipsoid(c: P3(0, 0.082, -0.062), r: P3(0.024, 0.017, 0.02)),
    SynthEllipsoid(c: P3(0, 0.052, -0.088), r: P3(0.022, 0.007, 0.01)),
    SynthEllipsoid(c: P3(0.077, 0.0, 0.006), r: P3(0.012, 0.03, 0.02)),
    SynthEllipsoid(c: P3(-0.077, 0.0, 0.006), r: P3(0.012, 0.03, 0.02)),
]

func headHit(_ o: P3, _ d: P3) -> Double? {
    let a = d.dot(d), b = 2 * o.dot(d), c = o.dot(o) - 0.13 * 0.13
    if b * b - 4 * a * c < 0 { return nil }
    var best: Double?
    for e in headParts { if let t = e.hit(o, d), t < (best ?? .infinity) { best = t } }
    return best
}

func headColor(_ p: P3) -> (UInt8, UInt8, UInt8) {
    let pattern = 0.5 + 0.25 * sin(230 * p.x + 3 * sin(150 * p.y)) + 0.25 * sin(190 * p.y + 170 * p.z)
    return (UInt8(min(255, 150 + 80 * pattern)), UInt8(min(255, 110 + 70 * pattern)), UInt8(min(255, 90 + 60 * pattern)))
}

/// Landmarks on the canonical head: where rays from the frontal camera through these (x, y) offsets hit the face.
/// A symmetric set (eyes, brows, nose, mouth, jaw, chin) like Vision's constellation.
let landmarkOffsets: [(Double, Double)] = [
    (-0.048, -0.018), (-0.032, -0.02), (-0.016, -0.018), (0.016, -0.018), (0.032, -0.02), (0.048, -0.018),
    (-0.04, -0.035), (-0.02, -0.038), (0.02, -0.038), (0.04, -0.035),
    (0, 0.0), (0, 0.012), (-0.012, 0.025), (0.012, 0.025),
    (-0.025, 0.052), (0, 0.046), (0, 0.058), (0.025, 0.052),
    (-0.05, 0.065), (0.05, 0.065), (-0.03, 0.085), (0.03, 0.085), (0, 0.097),
]

let canonicalLandmarks: [P3] = landmarkOffsets.compactMap { off in
    let o = P3(off.0, off.1, -0.3)
    guard let t = headHit(o, P3(0, 0, 1)) else { return nil }
    return o + P3(0, 0, 1) * t
}

struct SynthScanOptions {
    var width = 640
    var height = 400
    var headCenter = P3(0, 0, 0.6)
    var noiseAt1m = 0.0025
    /// (pose, yaw, pitch) in scan order.
    var views: [(ScanPose, Double, Double)] = [
        (.front, 0, 0), (.front, 1.5, -1), (.front, -1, 1.5),
        (.up, 0, 25), (.up, 2, 24),
        (.down, 0, -25), (.down, -2, -24),
        (.left, 45, 0), (.left, 43, 2),
        (.right, -45, 0), (.right, -44, -2),
        (.upLeft, 30, 20), (.downRight, -30, -20),
    ]
    /// Error of the Vision head pose (degrees, per angle).
    var poseErrorDeg = 2.5
    var seed: UInt64 = 1
}

struct SynthScan {
    var capture: ScanCapture
    /// Ground-truth world_from_camera per keyframe (world = camera frame of keyframe 0, the frontal view).
    var truth: [simd_float4x4]
    /// Head center in the world frame (the frontal view has the identity head rotation: world = canonical + center).
    var center: P3
}

struct Rng {
    var s: UInt64
    mutating func next() -> UInt64 {
        s &+= 0x9E37_79B9_7F4A_7C15
        var z = s
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
    mutating func uniform() -> Double { Double(next() >> 11) / 9_007_199_254_740_992.0 }
    mutating func gaussian() -> Double {
        (-2 * log(max(uniform(), 1e-300))).squareRoot() * cos(2 * .pi * uniform())
    }
}

func makeSyntheticScan(_ o: SynthScanOptions = SynthScanOptions()) -> SynthScan {
    var rng = Rng(s: o.seed)
    let f = Double(o.width) / 2   // 90 degree horizontal field of view, like the D455 color camera
    let cx = Double(o.width - 1) / 2, cy = Double(o.height - 1) / 2
    let intr = Intrinsics(width: o.width, height: o.height, fx: Float(f), fy: Float(f), ppx: Float(cx), ppy: Float(cy))
    let c = o.headCenter
    var keyframes: [Keyframe] = []
    var truth: [simd_float4x4] = []
    for (index, view) in o.views.enumerated() {
        let pose = HeadPose(yaw: Float(view.1), pitch: Float(view.2), roll: 0)
        let r = Rot(pose.cameraFrameRotation)           // head rotation in the camera frame
        let rt = r.transposed
        // world_from_camera = R^T (p - c) + c
        let t = c - rt.apply(c)
        truth.append(simd_float4x4(columns: (
            SIMD4(Float(rt.m[0]), Float(rt.m[3]), Float(rt.m[6]), 0), SIMD4(Float(rt.m[1]), Float(rt.m[4]), Float(rt.m[7]), 0),
            SIMD4(Float(rt.m[2]), Float(rt.m[5]), Float(rt.m[8]), 0), SIMD4(Float(t.x), Float(t.y), Float(t.z), 1))))

        var depth = [UInt16](repeating: 0, count: o.width * o.height)
        var rgb = [UInt8](repeating: 40, count: 3 * o.width * o.height)
        var mask = [UInt8](repeating: 0, count: o.width * o.height)
        for v in 0 ..< o.height {
            for u in 0 ..< o.width {
                let i = v * o.width + u
                let d = P3((Double(u) - cx) / f, (Double(v) - cy) / f, 1)   // camera ray with unit z
                // Head: intersect in the canonical frame (ray rotated by R^T about the center).
                let oc = rt.apply(P3(0, 0, 0) - c), dc = rt.apply(d)
                var z = Double.infinity
                var color: (UInt8, UInt8, UInt8) = (40, 40, 40)
                var person = false
                if let th = headHit(oc, dc) { z = th; color = headColor(oc + dc * th); person = true }
                // Neck: a static vertical cylinder under the head (does not rotate with it).
                let ax = d.x * d.x + d.z * d.z
                let bx = 2 * (-c.x * d.x + -(c.z + 0.02) * d.z)
                let cxn = c.x * c.x + (c.z + 0.02) * (c.z + 0.02) - 0.052 * 0.052
                let disc = bx * bx - 4 * ax * cxn
                if disc >= 0 {
                    let tn = (-bx - disc.squareRoot()) / (2 * ax)
                    let y = d.y * tn
                    if tn > 0, tn < z, y > c.y + 0.07, y < c.y + 0.3 { z = tn; color = (170, 120, 100); person = true }
                }
                if z == .infinity { z = 1.6 }   // back wall
                let noisy = z + o.noiseAt1m * z * z * rng.gaussian()
                depth[i] = UInt16(max(1, min(65000, (noisy / 0.0001).rounded())))
                rgb[3 * i] = color.0; rgb[3 * i + 1] = color.1; rgb[3 * i + 2] = color.2
                mask[i] = person ? 255 : 0
            }
        }
        // Landmarks: projected canonical landmarks (Vision also reports occluded ones, at wrong depths).
        var landmarks: [SIMD2<Float>] = []
        for l in canonicalLandmarks {
            let p = r.apply(l) + c
            landmarks.append(SIMD2(Float(f * p.x / p.z + cx), Float(f * p.y / p.z + cy)))
        }
        let xs = landmarks.map(\.x), ys = landmarks.map(\.y)
        let rect = CGRect(x: Double(xs.min()!) - 10, y: Double(ys.min()!) - 25, width: Double(xs.max()! - xs.min()!) + 20,
                          height: Double(ys.max()! - ys.min()!) + 30)
        let err = o.poseErrorDeg
        let vision = HeadPose(yaw: Float(view.1 + err * rng.gaussian()), pitch: Float(view.2 + err * rng.gaussian()),
                              roll: Float(err * rng.gaussian()), confidence: 0.95)
        let color = ColorImage(width: o.width, height: o.height, bytesPerRow: 3 * o.width, pixels: Data(rgb), intrinsics: intr)
        let depthImage = DepthImage(width: o.width, height: o.height,
                                    pixels: depth.withUnsafeBufferPointer { Data(buffer: $0) }, depthScale: 0.0001,
                                    intrinsics: intr)
        let frame = CameraFrame(sequence: UInt64(index), timestampMs: Double(index) * 500, color: color, alignedDepth: depthImage)
        let quality = FrameQuality(sharpness: 100 + Float(index), angularSpeedDegPerSec: 0, validDepthRatio: 1,
                                   validDepthPoints: 10000, faceDistanceM: 0.55)
        keyframes.append(Keyframe(id: index, frame: frame, scanPose: view.0, headPose: vision, faceRect: rect,
                                  landmarks: landmarks, mask: Data(mask), quality: quality))
    }

    return SynthScan(capture: ScanCapture(keyframes: keyframes, notes: ["source": "synthetic"]), truth: truth, center: c)
}

/// Distance from a world point (e.g. a mesh vertex with its normal) to the ground-truth head surface, measured
/// along the normal from 1 cm outside; nil when the surface is not within reach.
func distanceToHead(_ p: P3, normal n: P3, center: P3) -> Double? {
    let len = n.length
    guard len > 0 else { return nil }
    let dir = n * (1 / len)
    let origin = p - center + dir * 0.01
    guard let t = headHit(origin, dir * -1), t < 0.02 else { return nil }
    return abs(t - 0.01)
}
