// License: Apache 2.0. See LICENSE file in root directory.
//
// Small self-contained synthetic data generators for the reconstruction tests: deterministic noise, rigid poses,
// analytic surfaces (bumpy sphere, textured plane, an ellipsoid-union "head") and a ray-casting RGB-D renderer.
// Scalar Double math on purpose: tests build without optimization and generic SIMD code is slow there.

import FaceReconCore
import Foundation

// MARK: - Random numbers

struct SplitMix64 {
    var state: UInt64
    init(seed: UInt64) { state = seed &+ 0x9E37_79B9_7F4A_7C15 }

    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    mutating func uniform() -> Double { Double(next() >> 11) * (1.0 / 9_007_199_254_740_992.0) }

    mutating func gaussian() -> Double {
        let u1 = max(uniform(), 1e-300), u2 = uniform()
        return (-2 * log(u1)).squareRoot() * cos(2 * Double.pi * u2)
    }
}

// MARK: - Vectors and rigid transforms

struct V3: Equatable {
    var x, y, z: Double
    init(_ x: Double, _ y: Double, _ z: Double) { self.x = x; self.y = y; self.z = z }
    static let zero = V3(0, 0, 0)
    static func + (a: V3, b: V3) -> V3 { V3(a.x + b.x, a.y + b.y, a.z + b.z) }
    static func - (a: V3, b: V3) -> V3 { V3(a.x - b.x, a.y - b.y, a.z - b.z) }
    static func * (a: V3, s: Double) -> V3 { V3(a.x * s, a.y * s, a.z * s) }
    static prefix func - (a: V3) -> V3 { V3(-a.x, -a.y, -a.z) }
    func dot(_ b: V3) -> Double { x * b.x + y * b.y + z * b.z }
    func cross(_ b: V3) -> V3 { V3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x) }
    var length: Double { dot(self).squareRoot() }
    var normalized: V3 { let l = length; return l > 0 ? self * (1 / l) : self }
}

/// Rotation (row-major 3x3) + translation; maps p -> R p + t.
struct Rigid {
    var r: [Double] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    var t = V3.zero

    static let identity = Rigid()

    init() {}
    init(r: [Double], t: V3) { self.r = r; self.t = t }

    /// Rotation of angleDeg degrees about axis (Rodrigues), then translation.
    init(axis: V3, angleDeg: Double, t: V3 = .zero) {
        let a = axis.normalized, th = angleDeg * Double.pi / 180
        let c = cos(th), s = sin(th), C = 1 - c
        r = [c + a.x * a.x * C, a.x * a.y * C - a.z * s, a.x * a.z * C + a.y * s,
             a.y * a.x * C + a.z * s, c + a.y * a.y * C, a.y * a.z * C - a.x * s,
             a.z * a.x * C - a.y * s, a.z * a.y * C + a.x * s, c + a.z * a.z * C]
        self.t = t
    }

    func rotate(_ p: V3) -> V3 {
        V3(r[0] * p.x + r[1] * p.y + r[2] * p.z, r[3] * p.x + r[4] * p.y + r[5] * p.z, r[6] * p.x + r[7] * p.y + r[8] * p.z)
    }
    func apply(_ p: V3) -> V3 { rotate(p) + t }

    static func * (a: Rigid, b: Rigid) -> Rigid {
        var r = [Double](repeating: 0, count: 9)
        for i in 0 ..< 3 { for j in 0 ..< 3 { for k in 0 ..< 3 { r[3 * i + j] += a.r[3 * i + k] * b.r[3 * k + j] } } }
        return Rigid(r: r, t: a.apply(b.t))
    }

    var inverse: Rigid {
        let rt = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
        let inv = Rigid(r: rt, t: .zero)
        return Rigid(r: rt, t: -inv.rotate(t))
    }

    /// Column-major float[16] for the C API.
    var floats: [Float] {
        [Float(r[0]), Float(r[3]), Float(r[6]), 0, Float(r[1]), Float(r[4]), Float(r[7]), 0,
         Float(r[2]), Float(r[5]), Float(r[8]), 0, Float(t.x), Float(t.y), Float(t.z), 1]
    }

    init(floats m: [Float]) {
        r = [Double(m[0]), Double(m[4]), Double(m[8]), Double(m[1]), Double(m[5]), Double(m[9]),
             Double(m[2]), Double(m[6]), Double(m[10])]
        t = V3(Double(m[12]), Double(m[13]), Double(m[14]))
    }

    var rotationAngleDeg: Double {
        let c = min(1, max(-1, (r[0] + r[4] + r[8] - 1) / 2))
        let s = 0.5 * V3(r[7] - r[5], r[2] - r[6], r[3] - r[1]).length
        return atan2(s, c) * 180 / Double.pi
    }

    /// Rotation error (degrees) and translation error (meters) between two poses.
    static func error(_ a: Rigid, _ b: Rigid) -> (rotationDeg: Double, translation: Double) {
        let d = a.inverse * b
        return (d.rotationAngleDeg, (a.t - b.t).length)
    }
}

// MARK: - Clouds

/// Owns an fr_cloud.
final class CoreCloud {
    let pointer: OpaquePointer
    init(_ pointer: OpaquePointer) { self.pointer = pointer }
    deinit { fr_cloud_free(pointer) }
    var count: Int { Int(fr_cloud_size(pointer)) }

    static func make(points: [V3], normals: [V3]? = nil, colors: [(UInt8, UInt8, UInt8)]? = nil) -> CoreCloud? {
        let p = points.flatMap { [Float($0.x), Float($0.y), Float($0.z)] }
        let n = normals?.flatMap { [Float($0.x), Float($0.y), Float($0.z)] }
        let c = colors?.flatMap { [$0.0, $0.1, $0.2] }
        var out: OpaquePointer?
        let status = p.withUnsafeBufferPointer { pp in
            (n ?? []).withUnsafeBufferPointer { np in
                (c ?? []).withUnsafeBufferPointer { cp in
                    fr_cloud_create(pp.baseAddress, n == nil ? nil : np.baseAddress, c == nil ? nil : cp.baseAddress,
                                    UInt32(points.count), &out)
                }
            }
        }
        guard status == FR_OK, let out else { return nil }
        return CoreCloud(out)
    }

    var points: [V3] {
        guard let p = fr_cloud_points(pointer) else { return [] }
        return (0 ..< count).map { V3(Double(p[3 * $0]), Double(p[3 * $0 + 1]), Double(p[3 * $0 + 2])) }
    }

    var normals: [V3] {
        guard let p = fr_cloud_normals(pointer) else { return [] }
        return (0 ..< count).map { V3(Double(p[3 * $0]), Double(p[3 * $0 + 1]), Double(p[3 * $0 + 2])) }
    }
}

/// Points on a sphere with radial bumps, sampled on a jittered Fibonacci lattice.
func bumpySpherePoints(count: Int, radius: Double, center: V3, jitterSeed: UInt64) -> [V3] {
    var rng = SplitMix64(seed: jitterSeed)
    var pts: [V3] = []
    pts.reserveCapacity(count)
    let golden = Double.pi * (3 - 5.0.squareRoot())
    for i in 0 ..< count {
        let y = 1 - 2 * (Double(i) + rng.uniform()) / Double(count)
        let rr = max(0, 1 - y * y).squareRoot()
        let phi = golden * Double(i) + 0.3 * (rng.uniform() - 0.5)
        let dir = V3(rr * cos(phi), y, rr * sin(phi))
        let theta = acos(max(-1, min(1, y)))
        let bump = 1 + 0.06 * sin(5 * theta) * cos(4 * phi) + 0.03 * sin(9 * theta + 1)
        pts.append(center + dir * (radius * bump))
    }
    return pts
}

// MARK: - RGB-D rendering

struct SynthCamera {
    var width: Int
    var height: Int
    var fx: Double
    var fy: Double
    var cx: Double
    var cy: Double

    /// D455 color stream geometry (90 x 65 degrees) scaled to the given width.
    static func d455(width: Int, height: Int) -> SynthCamera {
        let f = Double(width) / 2 / tan(45 * Double.pi / 180)
        return SynthCamera(width: width, height: height, fx: f, fy: f, cx: Double(width - 1) / 2, cy: Double(height - 1) / 2)
    }

    var intrinsics: fr_intrinsics {
        var i = fr_intrinsics()
        i.width = Int32(width); i.height = Int32(height)
        i.fx = Float(fx); i.fy = Float(fy); i.cx = Float(cx); i.cy = Float(cy)
        return i
    }
}

struct SurfaceHit {
    var t: Double
    var normal: V3
}

protocol SynthScene {
    /// Nearest intersection along origin + t * dir (t > 0).
    func intersect(origin: V3, dir: V3) -> SurfaceHit?
    func color(at p: V3) -> (UInt8, UInt8, UInt8)
}

struct Ellipsoid {
    var center: V3
    var radii: V3

    func intersect(origin: V3, dir: V3) -> SurfaceHit? {
        let o = origin - center
        let ox = o.x / radii.x, oy = o.y / radii.y, oz = o.z / radii.z
        let dx = dir.x / radii.x, dy = dir.y / radii.y, dz = dir.z / radii.z
        let a = dx * dx + dy * dy + dz * dz
        let b = 2 * (ox * dx + oy * dy + oz * dz)
        let c = ox * ox + oy * oy + oz * oz - 1
        let disc = b * b - 4 * a * c
        guard disc >= 0 else { return nil }
        let sq = disc.squareRoot()
        var t = (-b - sq) / (2 * a)
        if t <= 1e-6 { t = (-b + sq) / (2 * a) }
        guard t > 1e-6 else { return nil }
        let p = o + dir * t
        let n = V3(p.x / (radii.x * radii.x), p.y / (radii.y * radii.y), p.z / (radii.z * radii.z)).normalized
        return SurfaceHit(t: t, normal: n)
    }
}

/// Sphere with a smooth color pattern (TSDF tests).
struct SphereScene: SynthScene {
    var center: V3
    var radius: Double
    func intersect(origin: V3, dir: V3) -> SurfaceHit? {
        Ellipsoid(center: center, radii: V3(radius, radius, radius)).intersect(origin: origin, dir: dir)
    }
    func color(at p: V3) -> (UInt8, UInt8, UInt8) {
        let d = p - center
        return (UInt8(128 + 100 * sin(40 * d.x)), UInt8(128 + 100 * sin(40 * d.y)), UInt8(128 + 100 * sin(40 * d.z)))
    }
}

/// Head-like union of ellipsoids around `center` (world = camera frame of the frontal view: +Y down, the face
/// looks toward -Z). Textured with a skin tone plus fine patterns, so colored ICP has something to lock onto.
struct HeadScene: SynthScene {
    var center = V3(0, 0, 0.6)
    var parts: [Ellipsoid] = []

    init(center: V3 = V3(0, 0, 0.6)) {
        self.center = center
        let c = center
        parts = [
            Ellipsoid(center: c + V3(0, -0.01, 0.0), radii: V3(0.075, 0.095, 0.095)),      // cranium
            Ellipsoid(center: c + V3(0, 0.035, -0.035), radii: V3(0.062, 0.062, 0.062)),   // face / jaw
            Ellipsoid(center: c + V3(0, 0.012, -0.093), radii: V3(0.011, 0.024, 0.02)),    // nose
            Ellipsoid(center: c + V3(0, -0.032, -0.082), radii: V3(0.055, 0.011, 0.018)),  // brow ridge
            Ellipsoid(center: c + V3(0.038, 0.022, -0.072), radii: V3(0.024, 0.02, 0.02)), // cheeks
            Ellipsoid(center: c + V3(-0.038, 0.022, -0.072), radii: V3(0.024, 0.02, 0.02)),
            Ellipsoid(center: c + V3(0, 0.082, -0.062), radii: V3(0.024, 0.017, 0.02)),    // chin
            Ellipsoid(center: c + V3(0, 0.052, -0.088), radii: V3(0.022, 0.007, 0.01)),    // lips
            Ellipsoid(center: c + V3(0.077, 0.0, 0.006), radii: V3(0.012, 0.03, 0.02)),    // ears
            Ellipsoid(center: c + V3(-0.077, 0.0, 0.006), radii: V3(0.012, 0.03, 0.02)),
        ]
    }

    func intersect(origin: V3, dir: V3) -> SurfaceHit? {
        var best: SurfaceHit?
        for e in parts {
            if let h = e.intersect(origin: origin, dir: dir), h.t < (best?.t ?? .infinity) { best = h }
        }
        return best
    }

    func color(at p: V3) -> (UInt8, UInt8, UInt8) {
        let d = p - center
        let pattern = 0.5 + 0.25 * sin(230 * d.x + 3 * sin(150 * d.y)) + 0.25 * sin(190 * d.y + 170 * d.z)
        let r = 150 + 80 * pattern, g = 110 + 70 * pattern, b = 90 + 60 * pattern
        return (UInt8(min(255, r)), UInt8(min(255, g)), UInt8(min(255, b)))
    }
}

/// One rendered RGB-D frame with stable buffers (the fr_rgbd_frame points into them).
final class RenderedFrame {
    let camera: SynthCamera
    let depthScale: Float
    let depth: UnsafeMutableBufferPointer<UInt16>
    let rgb: UnsafeMutableBufferPointer<UInt8>
    let mask: UnsafeMutableBufferPointer<UInt8>
    var worldFromCamera: Rigid

    init(camera: SynthCamera, depthScale: Float, worldFromCamera: Rigid) {
        self.camera = camera
        self.depthScale = depthScale
        self.worldFromCamera = worldFromCamera
        let n = camera.width * camera.height
        depth = .allocate(capacity: n); depth.initialize(repeating: 0)
        rgb = .allocate(capacity: 3 * n); rgb.initialize(repeating: 0)
        mask = .allocate(capacity: n); mask.initialize(repeating: 0)
    }

    deinit {
        depth.deallocate(); rgb.deallocate(); mask.deallocate()
    }

    /// The C view of this frame; useMask = false passes no mask.
    func coreFrame(initialPose: Rigid? = nil, useMask: Bool = true, useColor: Bool = true) -> fr_rgbd_frame {
        var f = fr_rgbd_frame()
        f.depth = UnsafePointer(depth.baseAddress)
        f.depth_scale = depthScale
        f.rgb = useColor ? UnsafePointer(rgb.baseAddress) : nil
        f.mask = useMask ? UnsafePointer(mask.baseAddress) : nil
        f.intrinsics = camera.intrinsics
        let pose = (initialPose ?? .identity).floats
        withUnsafeMutableBytes(of: &f.initial_world_from_camera) { raw in
            pose.withUnsafeBytes { raw.copyMemory(from: $0) }
        }
        f.has_initial_pose = initialPose == nil ? 0 : 1
        f.weight = 1
        return f
    }
}

/// Ray-casts a scene. Depth is z in the camera frame, quantized to depthScale, with Gaussian noise of
/// sigma = noiseAt1m * z^2 (stereo depth error grows quadratically with distance).
func render(_ scene: SynthScene, camera: SynthCamera, worldFromCamera: Rigid, noiseAt1m: Double = 0,
            depthScale: Float = 0.0001, seed: UInt64 = 1) -> RenderedFrame {
    let frame = RenderedFrame(camera: camera, depthScale: depthScale, worldFromCamera: worldFromCamera)
    var rng = SplitMix64(seed: seed)
    let origin = worldFromCamera.t
    for v in 0 ..< camera.height {
        for u in 0 ..< camera.width {
            let local = V3((Double(u) - camera.cx) / camera.fx, (Double(v) - camera.cy) / camera.fy, 1)
            let dir = worldFromCamera.rotate(local)
            guard let hit = scene.intersect(origin: origin, dir: dir) else { continue }
            let z = hit.t   // dir has unit z in the camera frame
            let noisy = z + (noiseAt1m > 0 ? noiseAt1m * z * z * rng.gaussian() : 0)
            let raw = (noisy / Double(depthScale)).rounded()
            guard raw > 0, raw < 65535 else { continue }
            let i = v * camera.width + u
            frame.depth[i] = UInt16(raw)
            frame.mask[i] = 255
            let c = scene.color(at: origin + dir * hit.t)
            frame.rgb[3 * i] = c.0; frame.rgb[3 * i + 1] = c.1; frame.rgb[3 * i + 2] = c.2
        }
    }
    return frame
}

// MARK: - Meshes

/// Owns an fr_mesh.
final class CoreMesh {
    let pointer: UnsafeMutablePointer<fr_mesh>
    init(_ pointer: UnsafeMutablePointer<fr_mesh>) { self.pointer = pointer }
    deinit { fr_mesh_free(pointer) }

    var vertexCount: Int { Int(pointer.pointee.vertex_count) }
    var triangleCount: Int { Int(pointer.pointee.triangle_count) }

    func vertex(_ i: Int) -> V3 {
        let p = pointer.pointee.positions!
        return V3(Double(p[3 * i]), Double(p[3 * i + 1]), Double(p[3 * i + 2]))
    }

    func normal(_ i: Int) -> V3 {
        let p = pointer.pointee.normals!
        return V3(Double(p[3 * i]), Double(p[3 * i + 1]), Double(p[3 * i + 2]))
    }

    func triangle(_ t: Int) -> (Int, Int, Int) {
        let idx = pointer.pointee.indices!
        return (Int(idx[3 * t]), Int(idx[3 * t + 1]), Int(idx[3 * t + 2]))
    }

    /// Every undirected edge used by exactly two triangles, with opposite directions (closed, consistently oriented).
    var isWatertight: Bool {
        var directed = Set<UInt64>()
        directed.reserveCapacity(3 * triangleCount)
        for t in 0 ..< triangleCount {
            let (a, b, c) = triangle(t)
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let key = UInt64(u) << 32 | UInt64(v)
                if directed.contains(key) { return false }   // the same directed edge twice: non-manifold / flipped
                directed.insert(key)
            }
        }
        for key in directed {
            let u = key >> 32, v = key & 0xFFFF_FFFF
            if !directed.contains(v << 32 | u) { return false }
        }
        return true
    }

    /// Euler characteristic V - E + F over referenced vertices.
    var eulerCharacteristic: Int {
        var used = Set<Int>()
        var edges = Set<UInt64>()
        for t in 0 ..< triangleCount {
            let (a, b, c) = triangle(t)
            used.insert(a); used.insert(b); used.insert(c)
            for (u, v) in [(a, b), (b, c), (c, a)] { edges.insert(UInt64(min(u, v)) << 32 | UInt64(max(u, v))) }
        }
        return used.count - edges.count + triangleCount
    }
}

/// Camera poses on a sphere around `target`, looking at it (world_from_camera; camera +Z toward the target).
func lookAtPoses(target: V3, distance: Double, directions: [V3]) -> [Rigid] {
    directions.map { d in
        let back = d.normalized   // from the target toward the camera
        let eye = target + back * distance
        let z = -back             // camera forward
        let downHint = abs(z.y) > 0.9 ? V3(0, 0, 1) : V3(0, 1, 0)   // camera +Y (down) close to world +Y
        let x = downHint.cross(z).normalized
        let y = z.cross(x)
        // Columns x, y, z of the rotation (camera axes in world).
        return Rigid(r: [x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z], t: eye)
    }
}

/// 26 directions: cube faces, edges and corners.
let cubeDirections: [V3] = {
    var dirs: [V3] = []
    for x in -1 ... 1 { for y in -1 ... 1 { for z in -1 ... 1 where !(x == 0 && y == 0 && z == 0) {
        dirs.append(V3(Double(x), Double(y), Double(z)).normalized)
    } } }
    return dirs
}()
