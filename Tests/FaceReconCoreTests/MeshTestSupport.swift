// License: Apache 2.0. See LICENSE file in root directory.
//
// Synthetic meshes and mesh measurements for the mesh/texture/export tests.

import FaceReconCore
import Foundation
import ReconKit
import simd

/// Deterministic pseudo-random numbers (SplitMix64).
struct TestRandom {
    var state: UInt64
    init(seed: UInt64) { state = seed }
    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    mutating func uniform() -> Float { Float(next() >> 40) / Float(1 << 24) }
    /// Standard normal (Box-Muller).
    mutating func gaussian() -> Float {
        let u1 = max(uniform(), 1e-7), u2 = uniform()
        return sqrt(-2 * log(u1)) * cos(2 * .pi * u2)
    }
}

enum MeshFactory {
    /// Icosahedron subdivided `level` times and projected onto a sphere (20 * 4^level triangles, outward CCW).
    static func icosphere(level: Int, radius: Float = 0.1, center: SIMD3<Float> = .zero) -> ReconMesh {
        let t: Float = (1 + sqrt(5)) / 2
        var positions: [SIMD3<Float>] = [
            [-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t],
            [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1],
        ].map { simd_normalize($0) }
        var faces: [SIMD3<UInt32>] = [
            [0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6],
            [7, 1, 8], [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10],
            [8, 6, 7], [9, 8, 1],
        ]
        for _ in 0 ..< level {
            var midpoint: [UInt64: UInt32] = [:]
            func mid(_ a: UInt32, _ b: UInt32) -> UInt32 {
                let key = a < b ? UInt64(a) << 32 | UInt64(b) : UInt64(b) << 32 | UInt64(a)
                if let m = midpoint[key] { return m }
                positions.append(simd_normalize(positions[Int(a)] + positions[Int(b)]))
                let m = UInt32(positions.count - 1)
                midpoint[key] = m
                return m
            }
            var next: [SIMD3<UInt32>] = []
            next.reserveCapacity(faces.count * 4)
            for f in faces {
                let a = mid(f.x, f.y), b = mid(f.y, f.z), c = mid(f.z, f.x)
                next += [[f.x, a, c], [f.y, b, a], [f.z, c, b], [a, b, c]]
            }
            faces = next
        }
        return ReconMesh(positions: positions.map { $0 * radius + center },
                         indices: faces.flatMap { [$0.x, $0.y, $0.z] })
    }

    /// Latitude/longitude sphere with `rings` x `segments` quads (fast to build for large sizes).
    static func gridSphere(rings: Int, segments: Int, radius: Float = 0.1) -> ReconMesh {
        var positions: [SIMD3<Float>] = [[0, radius, 0]]
        positions.reserveCapacity((rings - 1) * segments + 2)
        for i in 1 ..< rings {
            let theta = Float.pi * Float(i) / Float(rings)
            for j in 0 ..< segments {
                let phi = 2 * Float.pi * Float(j) / Float(segments)
                positions.append(SIMD3(sin(theta) * sin(phi), cos(theta), sin(theta) * cos(phi)) * radius)
            }
        }
        positions.append([0, -radius, 0])
        var indices: [UInt32] = []
        indices.reserveCapacity(rings * segments * 6)
        func ring(_ i: Int, _ j: Int) -> UInt32 { UInt32(1 + (i - 1) * segments + (j % segments)) }
        for j in 0 ..< segments { indices += [0, ring(1, j), ring(1, j + 1)] }
        for i in 1 ..< rings - 1 {
            for j in 0 ..< segments {
                let a = ring(i, j), b = ring(i + 1, j), c = ring(i + 1, j + 1), d = ring(i, j + 1)
                indices += [a, b, c, a, c, d]
            }
        }
        let south = UInt32(positions.count - 1)
        for j in 0 ..< segments { indices += [ring(rings - 1, j), south, ring(rings - 1, j + 1)] }
        return ReconMesh(positions: positions, indices: indices)
    }

    /// Axis-aligned cube [-s/2, s/2]^3 with n x n quads per face. Faces are separate grids (duplicate vertices on
    /// the edges) unless welded afterwards.
    static func cube(n: Int, size: Float = 0.1) -> ReconMesh {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        let h = size / 2
        // (normal axis, sign); u, v axes chosen so that u x v = outward normal.
        let faces: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)] = [
            ([1, 0, 0], [0, 1, 0], [0, 0, 1]), ([-1, 0, 0], [0, 0, 1], [0, 1, 0]),
            ([0, 1, 0], [0, 0, 1], [1, 0, 0]), ([0, -1, 0], [1, 0, 0], [0, 0, 1]),
            ([0, 0, 1], [1, 0, 0], [0, 1, 0]), ([0, 0, -1], [0, 1, 0], [1, 0, 0]),
        ]
        for (normal, u, v) in faces {
            let base = UInt32(positions.count)
            for i in 0 ... n {
                for j in 0 ... n {
                    let a = -h + size * Float(i) / Float(n), b = -h + size * Float(j) / Float(n)
                    positions.append(normal * h + u * a + v * b)
                }
            }
            for i in 0 ..< n {
                for j in 0 ..< n {
                    let p00 = base + UInt32(i * (n + 1) + j), p10 = p00 + UInt32(n + 1), p01 = p00 + 1, p11 = p10 + 1
                    indices += [p00, p10, p11, p00, p11, p01]
                }
            }
        }
        return ReconMesh(positions: positions, indices: indices)
    }

    /// Adds Gaussian noise along the vertex normals (computed from the mesh).
    static func addNormalNoise(_ mesh: inout ReconMesh, sigma: Float, seed: UInt64) throws {
        var rng = TestRandom(seed: seed)
        let normals = try MeshMeasure.normals(of: mesh)
        for i in mesh.positions.indices { mesh.positions[i] += normals[i] * (sigma * rng.gaussian()) }
    }
}

enum MeshMeasure {
    static func normals(of mesh: ReconMesh) throws -> [SIMD3<Float>] {
        var copy = mesh
        try copy.withCoreMesh { try check(fr_mesh_compute_normals($0)) }
        return copy.normals
    }

    /// Directed edges -> number of triangles using them.
    static func directedEdges(_ mesh: ReconMesh) -> [UInt64: Int] {
        var edges: [UInt64: Int] = [:]
        for t in 0 ..< mesh.triangleCount {
            for k in 0 ..< 3 {
                let a = UInt64(mesh.indices[3 * t + k]), b = UInt64(mesh.indices[3 * t + (k + 1) % 3])
                edges[a << 32 | b, default: 0] += 1
            }
        }
        return edges
    }

    /// Number of boundary edges and whether the mesh is an oriented 2-manifold (each directed edge used once,
    /// each interior edge used in both directions).
    static func topology(_ mesh: ReconMesh) -> (boundaryEdges: Int, consistent: Bool) {
        let edges = directedEdges(mesh)
        var boundary = 0, consistent = true
        for (key, count) in edges {
            if count > 1 { consistent = false }
            let reverse = (key & 0xFFFF_FFFF) << 32 | key >> 32
            if edges[reverse] == nil { boundary += 1 }
        }
        return (boundary, consistent)
    }

    /// Boundary loops (as vertex lists) following the directed boundary edges.
    static func boundaryLoops(_ mesh: ReconMesh) -> [[UInt32]] {
        let edges = directedEdges(mesh)
        var next: [UInt32: UInt32] = [:]
        for key in edges.keys {
            let reverse = (key & 0xFFFF_FFFF) << 32 | key >> 32
            if edges[reverse] == nil { next[UInt32(key >> 32)] = UInt32(key & 0xFFFF_FFFF) }
        }
        var loops: [[UInt32]] = []
        var visited = Set<UInt32>()
        for start in next.keys.sorted() where !visited.contains(start) {
            var loop: [UInt32] = []
            var v = start
            while !visited.contains(v), let n = next[v] {
                visited.insert(v); loop.append(v); v = n
            }
            loops.append(loop)
        }
        return loops
    }

    /// Signed volume (positive for a closed outward-oriented surface).
    static func signedVolume(_ mesh: ReconMesh) -> Double {
        var volume = 0.0
        for t in 0 ..< mesh.triangleCount {
            let a = SIMD3<Double>(mesh.positions[Int(mesh.indices[3 * t])])
            let b = SIMD3<Double>(mesh.positions[Int(mesh.indices[3 * t + 1])])
            let c = SIMD3<Double>(mesh.positions[Int(mesh.indices[3 * t + 2])])
            volume += simd_dot(a, simd_cross(b, c)) / 6
        }
        return volume
    }

    static func area(_ mesh: ReconMesh) -> Double {
        var sum = 0.0
        for t in 0 ..< mesh.triangleCount {
            let a = mesh.positions[Int(mesh.indices[3 * t])], b = mesh.positions[Int(mesh.indices[3 * t + 1])]
            let c = mesh.positions[Int(mesh.indices[3 * t + 2])]
            sum += Double(simd_length(simd_cross(b - a, c - a))) / 2
        }
        return sum
    }

    static func rms(_ values: [Float]) -> Float {
        guard !values.isEmpty else { return 0 }
        return sqrt(values.reduce(0) { $0 + $1 * $1 } / Float(values.count))
    }
}

struct CoreError: Error, CustomStringConvertible {
    let status: fr_status
    let message: String
    var description: String { "\(String(cString: fr_status_string(status))): \(message)" }
}

/// Throws when a FaceReconCore call failed.
func check(_ status: fr_status) throws {
    if status != FR_OK { throw CoreError(status: status, message: String(cString: fr_last_error())) }
}

/// Seconds spent in body.
func measure(_ body: () throws -> Void) rethrows -> Double {
    let start = DispatchTime.now().uptimeNanoseconds
    try body()
    return Double(DispatchTime.now().uptimeNanoseconds - start) / 1e9
}

/// A scratch directory removed when the value goes away.
final class TemporaryDirectory {
    let url: URL
    init() throws {
        url = FileManager.default.temporaryDirectory.appendingPathComponent("ifs-mesh-tests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    }
    deinit { try? FileManager.default.removeItem(at: url) }
}
