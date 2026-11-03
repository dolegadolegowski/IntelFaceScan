// License: Apache 2.0. See LICENSE file in root directory.
//
// Small scalar math, deterministic random numbers and procedural noise for the synthetic head.
//
// The hot loops (mesh generation, rasterization, shading) use these plain Float structs instead of SIMD3<Float>:
// tests run debug builds, where generic SIMD arithmetic is roughly 60x slower than scalar code, which would make
// a single 1280x800 render take seconds instead of milliseconds.

import Dispatch
import Foundation
import simd

// MARK: - Vectors and rigid transforms

struct Vec3: Sendable, Equatable {
    var x: Float
    var y: Float
    var z: Float

    @inline(__always) init(_ x: Float, _ y: Float, _ z: Float) { self.x = x; self.y = y; self.z = z }
    init(_ v: SIMD3<Float>) { x = v.x; y = v.y; z = v.z }

    static let zero = Vec3(0, 0, 0)
    var simd: SIMD3<Float> { SIMD3(x, y, z) }

    @inline(__always) static func + (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x + b.x, a.y + b.y, a.z + b.z) }
    @inline(__always) static func - (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x - b.x, a.y - b.y, a.z - b.z) }
    @inline(__always) static func * (a: Vec3, s: Float) -> Vec3 { Vec3(a.x * s, a.y * s, a.z * s) }
    @inline(__always) func dot(_ b: Vec3) -> Float { x * b.x + y * b.y + z * b.z }
    @inline(__always) func cross(_ b: Vec3) -> Vec3 { Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x) }
    @inline(__always) var length: Float { (x * x + y * y + z * z).squareRoot() }
    @inline(__always) var normalized: Vec3 {
        let l = length
        return l > 0 ? Vec3(x / l, y / l, z / l) : self
    }
}

/// Row-major 3x3 matrix.
struct Mat3: Sendable {
    var m00: Float, m01: Float, m02: Float
    var m10: Float, m11: Float, m12: Float
    var m20: Float, m21: Float, m22: Float

    static let identity = Mat3(m00: 1, m01: 0, m02: 0, m10: 0, m11: 1, m12: 0, m20: 0, m21: 0, m22: 1)

    @inline(__always) func apply(_ v: Vec3) -> Vec3 {
        Vec3(m00 * v.x + m01 * v.y + m02 * v.z, m10 * v.x + m11 * v.y + m12 * v.z, m20 * v.x + m21 * v.y + m22 * v.z)
    }

    var transposed: Mat3 {
        Mat3(m00: m00, m01: m10, m02: m20, m10: m01, m11: m11, m12: m21, m20: m02, m21: m12, m22: m22)
    }

    static func * (a: Mat3, b: Mat3) -> Mat3 {
        let c0 = a.apply(Vec3(b.m00, b.m10, b.m20)), c1 = a.apply(Vec3(b.m01, b.m11, b.m21))
        let c2 = a.apply(Vec3(b.m02, b.m12, b.m22))
        return Mat3(m00: c0.x, m01: c1.x, m02: c2.x, m10: c0.y, m11: c1.y, m12: c2.y, m20: c0.z, m21: c1.z, m22: c2.z)
    }

    init(m00: Float, m01: Float, m02: Float, m10: Float, m11: Float, m12: Float, m20: Float, m21: Float, m22: Float) {
        self.m00 = m00; self.m01 = m01; self.m02 = m02
        self.m10 = m10; self.m11 = m11; self.m12 = m12
        self.m20 = m20; self.m21 = m21; self.m22 = m22
    }

    init(_ q: simd_quatf) {
        let m = simd_float3x3(q)
        self.init(m00: m.columns.0.x, m01: m.columns.1.x, m02: m.columns.2.x,
                  m10: m.columns.0.y, m11: m.columns.1.y, m12: m.columns.2.y,
                  m20: m.columns.0.z, m21: m.columns.1.z, m22: m.columns.2.z)
    }
}

/// p' = r * p + t
struct Rigid: Sendable {
    var r: Mat3
    var t: Vec3

    static let identity = Rigid(r: .identity, t: .zero)

    @inline(__always) func apply(_ p: Vec3) -> Vec3 { r.apply(p) + t }
    var inverse: Rigid {
        let rt = r.transposed
        return Rigid(r: rt, t: rt.apply(t) * -1)
    }
    static func * (a: Rigid, b: Rigid) -> Rigid { Rigid(r: a.r * b.r, t: a.r.apply(b.t) + a.t) }

    init(r: Mat3, t: Vec3) { self.r = r; self.t = t }

    init(_ m: simd_float4x4) {
        r = Mat3(m00: m.columns.0.x, m01: m.columns.1.x, m02: m.columns.2.x,
                 m10: m.columns.0.y, m11: m.columns.1.y, m12: m.columns.2.y,
                 m20: m.columns.0.z, m21: m.columns.1.z, m22: m.columns.2.z)
        t = Vec3(m.columns.3.x, m.columns.3.y, m.columns.3.z)
    }

    var matrix: simd_float4x4 {
        simd_float4x4(columns: (SIMD4(r.m00, r.m10, r.m20, 0), SIMD4(r.m01, r.m11, r.m21, 0),
                                SIMD4(r.m02, r.m12, r.m22, 0), SIMD4(t.x, t.y, t.z, 1)))
    }
}

@inline(__always) func clamp01(_ v: Float) -> Float { v < 0 ? 0 : (v > 1 ? 1 : v) }

@inline(__always) func smoothstep(_ e0: Float, _ e1: Float, _ x: Float) -> Float {
    let t = clamp01((x - e0) / (e1 - e0))
    return t * t * (3 - 2 * t)
}

@inline(__always) func mix(_ a: Float, _ b: Float, _ t: Float) -> Float { a + (b - a) * t }

/// Polynomial smooth maximum (blend width k): equals max(a, b) once |a - b| >= k.
@inline(__always) func smoothMax(_ a: Float, _ b: Float, _ k: Float) -> Float {
    let h = max(k - abs(a - b), 0) / k
    return max(a, b) + h * h * k * 0.25
}

// MARK: - Random numbers

/// splitmix64: a small, fast generator with a fully specified output sequence, so synthetic data is identical on
/// every machine and OS version for a given seed.
public struct SplitMix64: RandomNumberGenerator, Sendable {
    public private(set) var state: UInt64

    public init(seed: UInt64) { state = seed }

    public mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        return SplitMix64.mix(state)
    }

    @inline(__always) static func mix(_ value: UInt64) -> UInt64 {
        var z = value
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    public mutating func uniform() -> Float { Float(next() >> 40) * (1.0 / 16_777_216.0) }
    /// Uniform in [-1, 1).
    public mutating func symmetric() -> Float { uniform() * 2 - 1 }
    /// Standard normal (Box-Muller).
    public mutating func gaussian() -> Float {
        let u1 = max(Double(next() >> 11) * (1.0 / 9_007_199_254_740_992.0), 1e-300)
        let u2 = Double(next() >> 11) * (1.0 / 9_007_199_254_740_992.0)
        return Float((-2 * log(u1)).squareRoot() * cos(2 * Double.pi * u2))
    }
}

/// Counter-based random numbers: the value for (key, index) does not depend on evaluation order, so per-pixel noise
/// is deterministic however the image is split across threads.
enum CounterRandom {
    @inline(__always) static func bits(_ key: UInt64, _ index: UInt64) -> UInt64 {
        SplitMix64.mix(key ^ SplitMix64.mix(index &+ 0x632B_E59B_D9B4_E019))
    }

    @inline(__always) static func uniform(_ key: UInt64, _ index: UInt64) -> Float {
        Float(bits(key, index) >> 40) * (1.0 / 16_777_216.0)
    }

    /// Standard normal from one 64-bit draw (two 26-bit uniforms, Box-Muller).
    @inline(__always) static func gaussian(_ key: UInt64, _ index: UInt64) -> Float {
        let b = bits(key, index)
        let u1 = max(Float(b >> 38) * (1.0 / 67_108_864.0), 1e-7)
        let u2 = Float((b >> 12) & 0x3FF_FFFF) * (1.0 / 67_108_864.0)
        return (-2 * logf(u1)).squareRoot() * cosf(6.283_185_3 * u2)
    }
}

// MARK: - Value noise

/// 3D value noise with quintic interpolation over a 256-periodic lattice; output in [-1, 1].
enum ValueNoise {
    private static let tables: (perm: UnsafeMutablePointer<Int32>, values: UnsafeMutablePointer<Float>) = {
        var random = SplitMix64(seed: 0x1F_ACE5)
        var perm = Array(0 ..< 256).map { Int32($0) }
        for i in stride(from: 255, to: 0, by: -1) {
            let j = Int(random.next() % UInt64(i + 1))
            perm.swapAt(i, j)
        }
        let p = UnsafeMutablePointer<Int32>.allocate(capacity: 768)
        for i in 0 ..< 768 { p[i] = perm[i & 255] }
        let v = UnsafeMutablePointer<Float>.allocate(capacity: 256)
        for i in 0 ..< 256 { v[i] = random.symmetric() }
        return (p, v)
    }()

    @inline(__always) static func sample(_ x: Float, _ y: Float, _ z: Float) -> Float {
        let (perm, values) = tables
        let fx = floorf(x), fy = floorf(y), fz = floorf(z)
        let ix = Int(fx) & 255, iy = Int(fy) & 255, iz = Int(fz) & 255
        let tx = x - fx, ty = y - fy, tz = z - fz
        let ux = tx * tx * tx * (tx * (tx * 6 - 15) + 10)
        let uy = ty * ty * ty * (ty * (ty * 6 - 15) + 10)
        let uz = tz * tz * tz * (tz * (tz * 6 - 15) + 10)
        let a = Int(perm[ix]) + iy, b = Int(perm[ix + 1]) + iy
        let aa = Int(perm[a]) + iz, ab = Int(perm[a + 1]) + iz
        let ba = Int(perm[b]) + iz, bb = Int(perm[b + 1]) + iz
        let x00 = mix(values[Int(perm[aa])], values[Int(perm[ba])], ux)
        let x10 = mix(values[Int(perm[ab])], values[Int(perm[bb])], ux)
        let x01 = mix(values[Int(perm[aa + 1])], values[Int(perm[ba + 1])], ux)
        let x11 = mix(values[Int(perm[ab + 1])], values[Int(perm[bb + 1])], ux)
        return mix(mix(x00, x10, uy), mix(x01, x11, uy), uz)
    }

    /// Weight of a noise octave with the given period (meters) for a pixel footprint (meters): full weight above
    /// 4 pixels per period, none below 2. This is a cheap prefilter (like mip-mapping) that avoids aliasing in the
    /// rendered images, which would otherwise make the texture flicker between views.
    @inline(__always) static func octaveWeight(period: Float, footprint: Float) -> Float {
        clamp01((period / max(footprint, 1e-6) - 2) * 0.5)
    }
}

// MARK: - Parallel loops

/// Runs body(range) on all cores over [0, count) in contiguous chunks.
func parallelChunks(_ count: Int, minChunk: Int = 1024, _ body: (Range<Int>) -> Void) {
    guard count > 0 else { return }
    let cores = max(1, ProcessInfo.processInfo.activeProcessorCount)
    let chunks = max(1, min(cores * 4, (count + minChunk - 1) / minChunk))
    if chunks == 1 {
        body(0 ..< count)
        return
    }
    let size = (count + chunks - 1) / chunks
    DispatchQueue.concurrentPerform(iterations: chunks) { c in
        let start = c * size
        let end = min(count, start + size)
        if start < end { body(start ..< end) }
    }
}
