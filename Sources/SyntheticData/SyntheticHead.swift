// License: Apache 2.0. See LICENSE file in root directory.
//
// The procedural head: a star-shaped surface r(direction) around the head center (smooth union of a cranium
// ellipsoid, a face/jaw ellipsoid and a neck cylinder) plus displacements for the nose, eye sockets and eyes, brow
// ridge, cheekbones, lips, chin and ears; a solid (3D) skin/hair/eye/lip texture with detail down to ~1 mm; and 76
// face landmarks laid out like Vision's face landmark constellation.
//
// Head frame ("local"): origin at the head center, X toward the image right of a frontal view (the person's left),
// U up, F forward (toward the camera of a frontal view). In the world frame (= camera frame of the frontal pose,
// +X right, +Y down, +Z forward) a local point is center + (X, -U, -F), a proper rotation, so triangle winding and
// normals carry over unchanged.

import Foundation
import simd

struct HeadShape: Sendable {
    // Adult head proportions (meters): breadth ~15.5 cm, length ~19.5 cm, eyes-to-chin ~12 cm, eyes-to-vertex ~12 cm.
    static let craniumCenter = Vec3(0, 0.025, -0.0095), craniumAxes = Vec3(0.0775, 0.095, 0.0975)
    static let faceCenter = Vec3(0, -0.04, 0.010), faceAxes = Vec3(0.065, 0.08, 0.08)
    static let neckAxisF: Float = -0.02, neckRadius: Float = 0.055, neckBottomU: Float = -0.5
    /// Head mask boundary: everything above this height (1.5 cm below the chin) is "head".
    static let maskCutU: Float = -0.135
    static let eyeX: Float = 0.032, eyeU: Float = 0.0005

    // MARK: Geometry

    /// Distance from the origin along a unit direction to the surface of an ellipsoid that contains the origin.
    @inline(__always) static func ellipsoidExit(_ d: Vec3, _ e: Vec3, _ a: Vec3) -> Float {
        let qx = d.x / a.x, qy = d.y / a.y, qz = d.z / a.z
        let mx = e.x / a.x, my = e.y / a.y, mz = e.z / a.z
        let qq = qx * qx + qy * qy + qz * qz
        let qm = qx * mx + qy * my + qz * mz
        let mm = mx * mx + my * my + mz * mz
        let disc = qm * qm - qq * (mm - 1)
        return (qm + max(disc, 0).squareRoot()) / qq
    }

    /// Neck: a vertical cylinder closed at the bottom, only for downward directions. For directions at or above the
    /// horizon it returns 0; the head is wider than the neck there by more than the blend width, so this cut-off
    /// never shows in the surface.
    @inline(__always) static func neckExit(_ d: Vec3) -> Float {
        guard d.y < -1e-6 else { return 0 }
        let bottom = neckBottomU / d.y
        let hh = d.x * d.x + d.z * d.z
        if hh < 1e-12 { return bottom }
        let hm = d.z * neckAxisF
        let mm = neckAxisF * neckAxisF - neckRadius * neckRadius
        let wall = (hm + max(hm * hm - hh * mm, 0).squareRoot()) / hh
        return min(wall, bottom)
    }

    @inline(__always) static func baseRadius(_ d: Vec3) -> Float {
        var r = smoothMax(ellipsoidExit(d, craniumCenter, craniumAxes), ellipsoidExit(d, faceCenter, faceAxes), 0.02)
        let neck = neckExit(d)
        if neck > 0 { r = smoothMax(r, neck, 0.012) }
        return r
    }

    @inline(__always) static func gauss(_ t: Float) -> Float { expf(-t * t) }
    @inline(__always) static func gauss2(_ a: Float, _ b: Float) -> Float { expf(-(a * a + b * b)) }
    /// Flat-topped falloff across the mouth width.
    @inline(__always) static func across(_ x: Float, _ w: Float) -> Float {
        let t = x / w
        return expf(-(t * t) * (t * t))
    }

    static func nose(_ x: Float, _ u: Float) -> Float {
        let nasion: Float = 0.012, tip: Float = -0.038, base: Float = -0.050
        var n: Float = 0
        if u < nasion + 0.015 && u > base - 0.006 {
            var h: Float, w: Float
            if u >= tip {
                let s = (nasion - u) / (nasion - tip)   // 0 at the nasion, 1 at the tip
                let sc = max(s, 0)
                h = 0.004 + 0.020 * powf(sc, 1.3)
                w = 0.0075 + 0.004 * sc
                if s < 0 { h *= smoothstep(-0.3, 0, s) }   // fades into the brow above the nasion
            } else {
                let s = min((tip - u) / (tip - base + 0.005), 1)
                let c = cosf(s * Float.pi / 2)
                h = 0.024 * c * c
                w = 0.0115 + 0.006 * s
            }
            let g = x / w
            n = h * expf(-1.4 * g * g)
        }
        // alae (nostril wings)
        n += 0.007 * gauss2((abs(x) - 0.0125) / 0.0055, (u + 0.045) / 0.0055)
        return n
    }

    static func ear(_ u: Float, _ f: Float) -> Float {
        let a = (u + 0.012) / 0.032, b = (f + 0.012) / 0.018
        let e = (a * a + b * b).squareRoot()
        guard e < 1.6 else { return 0 }
        var h = 0.016 * smoothstep(1.05, 0.82, e)                 // auricle
        h += 0.004 * gauss((e - 0.86) / 0.09)                     // helix rim
        h -= 0.009 * gauss2((u + 0.014) / 0.012, (f + 0.010) / 0.0085)   // concha
        return max(h, -0.002)
    }

    /// Radial displacement of the features at base surface point p (direction d).
    static func displacement(_ p: Vec3, _ d: Vec3) -> Float {
        var delta: Float = 0
        let front = smoothstep(0.02, 0.06, p.z)
        if front > 0 {
            let x = p.x, u = p.y, ax = abs(x)
            var f = nose(x, u)
            f -= 0.009 * gauss2((ax - eyeX) / 0.016, (u - 0.002) / 0.011)       // eye socket
            f += 0.0055 * gauss2((ax - eyeX) / 0.0085, u / 0.0075)             // eyeball
            f += 0.0045 * gauss((u - 0.022) / 0.008) * (1 - smoothstep(0.045, 0.062, ax))   // brow ridge
            f += 0.007 * gauss2((ax - 0.047) / 0.018, (u + 0.018) / 0.014)      // cheekbone
            f -= 0.0025 * gauss2((ax - 0.046) / 0.014, (u + 0.058) / 0.014)     // cheek hollow
            f += 0.006 * gauss((u + 0.066) / 0.0055) * across(x, 0.022)         // upper lip
            f += 0.0055 * gauss((u + 0.079) / 0.0055) * across(x, 0.020)        // lower lip
            f -= 0.003 * gauss((u + 0.072) / 0.0018) * across(x, 0.024)         // mouth
            f -= 0.002 * gauss((u + 0.090) / 0.005) * across(x, 0.02)           // mentolabial sulcus
            f += 0.012 * gauss2(x / 0.02, (u + 0.103) / 0.013)                  // chin
            delta += front * f
        }
        let side = smoothstep(0.5, 0.7, abs(d.x))
        if side > 0 { delta += side * ear(p.y, p.z) }
        return delta
    }

    /// Distance from the head center to the surface along a unit direction.
    @inline(__always) static func radius(_ d: Vec3) -> Float {
        let r0 = baseRadius(d)
        return r0 + displacement(d * r0, d)
    }

    /// Surface point in front of the face at the given (X, U): solves |(X, U, F)| = r(direction) for F.
    static func frontPoint(x: Float, u: Float) -> Vec3 {
        var f: Float = 0.08
        for _ in 0 ..< 12 {
            let r = radius(Vec3(x, u, f).normalized)
            f = max((r * r - x * x - u * u), 0.000025).squareRoot()
        }
        return Vec3(x, u, f)
    }

    // MARK: Texture

    /// Albedo (0...1 RGB) of the surface point p (local frame). footprint: surface size of a pixel in meters, used to
    /// fade out detail finer than ~2 pixels.
    static func albedo(_ p: Vec3, footprint s: Float) -> (Float, Float, Float) {
        let x = p.x, u = p.y, f = p.z, ax = abs(x)
        var r: Float = 0.86, g: Float = 0.65, b: Float = 0.54

        // Skin: blotches (4 cm), mottling (9 mm), pores and fine structure (2.6 mm, 1.2 mm) for colored ICP.
        var v = 0.06 * ValueNoise.sample(x * 25, u * 25, f * 25)
        v += 0.05 * ValueNoise.octaveWeight(period: 0.009, footprint: s) * ValueNoise.sample(x * 111 + 17, u * 111, f * 111)
        v += 0.06 * ValueNoise.octaveWeight(period: 0.0026, footprint: s) * ValueNoise.sample(x * 385, u * 385 + 31, f * 385)
        v += 0.045 * ValueNoise.octaveWeight(period: 0.0012, footprint: s) * ValueNoise.sample(x * 833, u * 833, f * 833 + 7)
        r *= 1 + v; g *= 1 + 1.1 * v; b *= 1 + 1.15 * v
        r += 0.03 * ValueNoise.sample(x * 18 + 91, u * 18, f * 18)
        let freckle = ValueNoise.sample(x * 240 + 5, u * 240 + 3, f * 240)
        let dark = 0.16 * smoothstep(0.42, 0.7, freckle) * ValueNoise.octaveWeight(period: 0.004, footprint: s)
        r *= 1 - 0.8 * dark; g *= 1 - dark; b *= 1 - 1.1 * dark

        let front = smoothstep(0.03, 0.06, f)
        if front > 0 {
            // lips and mouth
            let lips = front * min(1, 1.2 * (gauss((u + 0.066) / 0.0062) * across(x, 0.023)
                                         + gauss((u + 0.079) / 0.0062) * across(x, 0.021)))
            r = mix(r, 0.72, lips); g = mix(g, 0.37, lips); b = mix(b, 0.37, lips)
            let mouth = front * gauss((u + 0.072) / 0.0012) * across(x, 0.024)
            r = mix(r, 0.30, mouth); g = mix(g, 0.15, mouth); b = mix(b, 0.15, mouth)

            // eyebrows: arched bands of short dark strands
            if ax > 0.008 && ax < 0.062 {
                let arch = 0.027 + 0.005 * sinf(Float.pi * clamp01((ax - 0.012) / 0.044))
                var brow = gauss((u - arch) / 0.0035) * smoothstep(0.010, 0.016, ax) * (1 - smoothstep(0.050, 0.058, ax))
                brow *= 0.65 + 0.35 * ValueNoise.octaveWeight(period: 0.0015, footprint: s)
                    * ValueNoise.sample(x * 180, u * 900, f * 180)
                brow *= front
                r = mix(r, 0.20, brow); g = mix(g, 0.14, brow); b = mix(b, 0.10, brow)
            }

            // eyes: lid opening with sclera, iris with radial streaks, pupil, lash line
            let ex = (ax - eyeX) / 0.0135, eu = (u - eyeU) / 0.0058
            let opening = ex * ex + eu * eu
            if opening < 1.3 {
                if opening < 1 {
                    let dx = ax - eyeX, du = u - eyeU
                    let rr = (dx * dx + du * du).squareRoot()
                    var er: Float = 0.92, eg: Float = 0.90, eb: Float = 0.86
                    if rr < 0.0058 {
                        let streak = ValueNoise.sample(atan2f(du, dx) * 9, rr * 600, 3)
                        er = 0.32 + 0.08 * streak; eg = 0.24 + 0.07 * streak; eb = 0.15 + 0.05 * streak
                        if rr < 0.0024 { er = 0.03; eg = 0.03; eb = 0.03 }
                    }
                    r = er; g = eg; b = eb
                } else {
                    let lash = smoothstep(1.3, 1.05, opening)
                    r = mix(r, 0.15, lash); g = mix(g, 0.10, lash); b = mix(b, 0.08, lash)
                }
            }

            // nostrils
            let nostril = front * gauss2((ax - 0.0075) / 0.004, (u + 0.0485) / 0.0025)
            r = mix(r, 0.25, nostril); g = mix(g, 0.12, nostril); b = mix(b, 0.10, nostril)
        }

        // Hair above a hairline that runs from the forehead over the temples and behind the ears to the nape.
        var hairline: Float
        if f > 0.04 {
            hairline = 0.072
        } else if f > -0.02 {
            hairline = mix(0.072, 0.03, (0.04 - f) / 0.06)
        } else {
            hairline = mix(0.03, -0.065, clamp01((-0.02 - f) / 0.06))
        }
        var hair = smoothstep(hairline - 0.003, hairline + 0.003, u)
        if hair > 0 {
            if ax > 0.05 {   // keep the ears skin-colored
                let ea = (u + 0.012) / 0.034, eb = (f + 0.012) / 0.02
                hair *= smoothstep(0.95, 1.15, (ea * ea + eb * eb).squareRoot())
            }
            // Strands run down the sides and back, and front-to-back on top of the head.
            let top = smoothstep(0.08, 0.10, u)
            let sides = ValueNoise.sample(x * 350, u * 70, f * 350)
            let crown = ValueNoise.sample(x * 350, u * 350 + 11, f * 70)
            let strands = mix(sides, crown, top) * ValueNoise.octaveWeight(period: 0.003, footprint: s)
            let shade = 0.75 + 0.35 * strands + 0.1 * ValueNoise.sample(x * 40, u * 40, f * 40)
            r = mix(r, 0.23 * shade, hair); g = mix(g, 0.16 * shade, hair); b = mix(b, 0.11 * shade, hair)
        }
        return (clamp01(r), clamp01(g), clamp01(b))
    }

    // MARK: Landmarks

    /// 76 face landmarks (local frame): face contour 17, eyebrows 2x5, eyes 2x8, pupils 2, nose crest 5,
    /// nose base 8, outer lips 12, inner lips 6 - the groups of Vision's 76-point constellation (VNFaceLandmarks2D
    /// allPoints), with an approximate point layout.
    static func landmarkPoints() -> [Vec3] {
        var points: [(Float, Float)] = []
        for i in 0 ..< 17 {   // jaw line through the chin
            let phi = -Float.pi / 2 + Float.pi * Float(i) / 16
            points.append((0.066 * sinf(phi), -0.015 - 0.10 * cosf(phi)))
        }
        for side: Float in [-1, 1] {   // eyebrows
            for i in 0 ..< 5 {
                let ax = 0.014 + 0.040 * Float(i) / 4
                points.append((side * ax, 0.027 + 0.005 * sinf(Float.pi * (ax - 0.012) / 0.044)))
            }
        }
        for side: Float in [-1, 1] {   // eye outlines
            for i in 0 ..< 8 {
                let a = 2 * Float.pi * Float(i) / 8
                points.append((side * (eyeX + 0.0125 * cosf(a)), eyeU + 0.0050 * sinf(a)))
            }
        }
        points.append((-eyeX, eyeU))
        points.append((eyeX, eyeU))
        for i in 0 ..< 5 {   // nose crest, nasion to tip
            points.append((0, 0.010 - 0.048 * Float(i) / 4))
        }
        for i in 0 ..< 8 {   // nose base, ala to ala under the tip
            let a = Float.pi * Float(i) / 7
            points.append((-0.016 * cosf(a), -0.044 - 0.007 * sinf(a)))
        }
        for i in 0 ..< 12 {   // outer lips
            let a = 2 * Float.pi * Float(i) / 12
            points.append((0.025 * cosf(a), -0.0725 + 0.011 * sinf(a)))
        }
        for i in 0 ..< 6 {   // inner lips
            let a = 2 * Float.pi * Float(i) / 6
            points.append((0.017 * cosf(a), -0.072 + 0.0025 * sinf(a)))
        }
        return points.map { frontPoint(x: $0.0, u: $0.1) }
    }
}

/// The tessellated head (local frame), shared by all scenes with the same resolution.
final class HeadModel: @unchecked Sendable {
    let resolution: Int
    let vertexCount: Int
    let triangleCount: Int
    /// xyz per vertex, local frame.
    let positions: [Float]
    /// Unit outward normals, xyz per vertex.
    let normals: [Float]
    /// RGB8 albedo per vertex.
    let colors: [UInt8]
    /// Counter-clockwise seen from outside.
    let indices: [UInt32]
    let landmarks: [Vec3]
    /// Mean edge length (meters).
    let meanEdge: Float

    private static let cacheLock = NSLock()
    nonisolated(unsafe) private static var cache: [Int: HeadModel] = [:]

    /// The model for a cube-sphere resolution (cells per cube face edge; triangles = 12 * resolution^2).
    static func shared(resolution: Int) -> HeadModel {
        let n = min(max(resolution, 8), 512)
        cacheLock.lock()
        defer { cacheLock.unlock() }
        if let model = cache[n] { return model }
        let model = HeadModel(resolution: n)
        cache[n] = model
        return model
    }

    private init(resolution n: Int) {
        resolution = n
        // Cube-sphere tessellation: near-uniform triangles and no poles. Integer cube coordinates in [-n, n] (step 2)
        // identify the vertices shared by neighboring faces.
        var lookup: [Int: UInt32] = [:]
        lookup.reserveCapacity(6 * (n + 1) * (n + 1))
        var cube: [Int32] = []
        var indices: [UInt32] = []
        indices.reserveCapacity(36 * n * n)
        let side = 2 * n + 1
        func vertex(_ c: (Int, Int, Int)) -> UInt32 {
            let key = ((c.0 + n) * side + (c.1 + n)) * side + (c.2 + n)
            if let index = lookup[key] { return index }
            let index = UInt32(cube.count / 3)
            cube += [Int32(c.0), Int32(c.1), Int32(c.2)]
            lookup[key] = index
            return index
        }
        for axis in 0 ..< 3 {
            for sign in [-1, 1] {
                var grid = [UInt32](repeating: 0, count: (n + 1) * (n + 1))
                for j in 0 ... n {
                    for i in 0 ... n {
                        var c = [0, 0, 0]
                        c[axis] = sign * n
                        c[(axis + 1) % 3] = -n + 2 * i
                        c[(axis + 2) % 3] = -n + 2 * j
                        grid[j * (n + 1) + i] = vertex((c[0], c[1], c[2]))
                    }
                }
                // (axis+1) x (axis+2) = axis, so the cell order below faces outward on the positive faces.
                for j in 0 ..< n {
                    for i in 0 ..< n {
                        let p00 = grid[j * (n + 1) + i], p10 = grid[j * (n + 1) + i + 1]
                        let p01 = grid[(j + 1) * (n + 1) + i], p11 = grid[(j + 1) * (n + 1) + i + 1]
                        if sign > 0 {
                            indices += [p00, p10, p11, p00, p11, p01]
                        } else {
                            indices += [p00, p11, p10, p00, p01, p11]
                        }
                    }
                }
            }
        }
        let count = cube.count / 3
        vertexCount = count
        triangleCount = indices.count / 3

        var positions = [Float](repeating: 0, count: count * 3)
        let scale = 1 / Float(n)
        cube.withUnsafeBufferPointer { cb in
            positions.withUnsafeMutableBufferPointer { pb in
                let c = cb.baseAddress!, p = pb.baseAddress!
                parallelChunks(count, minChunk: 512) { range in
                    var i = range.lowerBound
                    while i < range.upperBound {
                        let x = Float(c[3 * i]) * scale, y = Float(c[3 * i + 1]) * scale, z = Float(c[3 * i + 2]) * scale
                        // spherified cube mapping (more uniform than normalizing the cube point)
                        let x2 = x * x, y2 = y * y, z2 = z * z
                        let d = Vec3(x * (1 - y2 / 2 - z2 / 2 + y2 * z2 / 3).squareRoot(),
                                     y * (1 - z2 / 2 - x2 / 2 + z2 * x2 / 3).squareRoot(),
                                     z * (1 - x2 / 2 - y2 / 2 + x2 * y2 / 3).squareRoot()).normalized
                        let r = HeadShape.radius(d)
                        p[3 * i] = d.x * r; p[3 * i + 1] = d.y * r; p[3 * i + 2] = d.z * r
                        i += 1
                    }
                }
            }
        }

        var normals = [Float](repeating: 0, count: count * 3)
        var edgeSum: Double = 0
        positions.withUnsafeBufferPointer { pb in
            normals.withUnsafeMutableBufferPointer { nb in
                indices.withUnsafeBufferPointer { ib in
                    let p = pb.baseAddress!, nrm = nb.baseAddress!, idx = ib.baseAddress!
                    var t = 0
                    let triangles = ib.count / 3
                    while t < triangles {
                        let a = Int(idx[3 * t]), b = Int(idx[3 * t + 1]), c = Int(idx[3 * t + 2])
                        let pa = Vec3(p[3 * a], p[3 * a + 1], p[3 * a + 2])
                        let e1 = Vec3(p[3 * b], p[3 * b + 1], p[3 * b + 2]) - pa
                        let e2 = Vec3(p[3 * c], p[3 * c + 1], p[3 * c + 2]) - pa
                        let fn = e1.cross(e2)   // length = 2 * area: area weighting
                        nrm[3 * a] += fn.x; nrm[3 * a + 1] += fn.y; nrm[3 * a + 2] += fn.z
                        nrm[3 * b] += fn.x; nrm[3 * b + 1] += fn.y; nrm[3 * b + 2] += fn.z
                        nrm[3 * c] += fn.x; nrm[3 * c + 1] += fn.y; nrm[3 * c + 2] += fn.z
                        edgeSum += Double(e1.length)
                        t += 1
                    }
                    var v = 0
                    while v < count {
                        let n = Vec3(nrm[3 * v], nrm[3 * v + 1], nrm[3 * v + 2]).normalized
                        nrm[3 * v] = n.x; nrm[3 * v + 1] = n.y; nrm[3 * v + 2] = n.z
                        v += 1
                    }
                }
            }
        }
        meanEdge = Float(edgeSum / Double(max(triangleCount, 1)))

        var colors = [UInt8](repeating: 0, count: count * 3)
        let footprint = meanEdge
        positions.withUnsafeBufferPointer { pb in
            colors.withUnsafeMutableBufferPointer { cb in
                let p = pb.baseAddress!, col = cb.baseAddress!
                parallelChunks(count, minChunk: 512) { range in
                    for i in range {
                        let (r, g, b) = HeadShape.albedo(Vec3(p[3 * i], p[3 * i + 1], p[3 * i + 2]), footprint: footprint)
                        col[3 * i] = UInt8(r * 255 + 0.5); col[3 * i + 1] = UInt8(g * 255 + 0.5)
                        col[3 * i + 2] = UInt8(b * 255 + 0.5)
                    }
                }
            }
        }

        self.positions = positions
        self.normals = normals
        self.colors = colors
        self.indices = indices
        self.landmarks = HeadShape.landmarkPoints()
    }

    /// The mesh in the world frame for a head centered at `center` (world = local rotated 180 degrees about X).
    func worldMesh(center: Vec3, transform: Rigid = .identity) -> ReconMeshData {
        var positions = [SIMD3<Float>](repeating: .zero, count: vertexCount)
        var normals = [SIMD3<Float>](repeating: .zero, count: vertexCount)
        var colors = [SIMD3<UInt8>](repeating: .zero, count: vertexCount)
        for i in 0 ..< vertexCount {
            let p = transform.apply(Vec3(center.x + self.positions[3 * i], center.y - self.positions[3 * i + 1],
                                         center.z - self.positions[3 * i + 2]))
            let n = transform.r.apply(Vec3(self.normals[3 * i], -self.normals[3 * i + 1], -self.normals[3 * i + 2]))
            positions[i] = SIMD3(p.x, p.y, p.z)
            normals[i] = SIMD3(n.x, n.y, n.z)
            colors[i] = SIMD3(self.colors[3 * i], self.colors[3 * i + 1], self.colors[3 * i + 2])
        }
        return ReconMeshData(positions: positions, normals: normals, colors: colors, indices: indices)
    }
}

/// Plain mesh arrays (converted to ReconMesh by the scene).
struct ReconMeshData: Sendable {
    var positions: [SIMD3<Float>]
    var normals: [SIMD3<Float>]
    var colors: [SIMD3<UInt8>]
    var indices: [UInt32]
}
