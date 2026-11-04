// License: Apache 2.0. See LICENSE file in root directory.
//
// Synthetic meshes and keyframes for the ReconKit mesh/texture/export tests: an icosphere and ray-cast RGB-D
// keyframes of an analytically textured sphere.

import CameraKit
import CoreGraphics
import Foundation
import ReconKit
import ScanKit
import simd

enum TestMeshes {
    /// Icosahedron subdivided `level` times, projected onto a sphere (outward CCW).
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
                midpoint[key] = UInt32(positions.count - 1)
                return UInt32(positions.count - 1)
            }
            faces = faces.flatMap { f -> [SIMD3<UInt32>] in
                let a = mid(f.x, f.y), b = mid(f.y, f.z), c = mid(f.z, f.x)
                return [[f.x, a, c], [f.y, b, a], [f.z, c, b], [a, b, c]]
            }
        }
        return ReconMesh(positions: positions.map { $0 * radius + center }, indices: faces.flatMap { [$0.x, $0.y, $0.z] })
    }

    static func boundaryLoopCount(_ mesh: ReconMesh) -> Int {
        var directed = Set<UInt64>()
        for t in 0 ..< mesh.triangleCount {
            for k in 0 ..< 3 {
                directed.insert(UInt64(mesh.indices[3 * t + k]) << 32 | UInt64(mesh.indices[3 * t + (k + 1) % 3]))
            }
        }
        var next: [UInt32: UInt32] = [:]
        for e in directed where !directed.contains((e & 0xFFFF_FFFF) << 32 | e >> 32) {
            next[UInt32(e >> 32)] = UInt32(e & 0xFFFF_FFFF)
        }
        var visited = Set<UInt32>(), loops = 0
        for start in next.keys where !visited.contains(start) {
            var v = start
            while !visited.contains(v), let n = next[v] { visited.insert(v); v = n }
            loops += 1
        }
        return loops
    }

    static func gaussian(_ state: inout UInt64) -> Float {
        func uniform() -> Float {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return max(Float(state >> 40) / Float(1 << 24), 1e-7)
        }
        return sqrt(-2 * log(uniform())) * cos(2 * .pi * uniform())
    }
}

enum SphereScene {
    static let radius: Float = 0.1

    static func albedo(_ d: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3(128 + 90 * sin(2.0 * d.x + 0.5), 128 + 90 * sin(2.5 * d.y + 1.0) * cos(1.5 * d.z),
              128 + 80 * cos(1.8 * d.z - 0.3 + d.x))
    }

    static func lookAt(eye: SIMD3<Float>) -> simd_float4x4 {
        let z = simd_normalize(-eye)
        let x = simd_normalize(simd_cross(SIMD3<Float>(0, 1, 0), z))
        return simd_float4x4(columns: (SIMD4(x, 0), SIMD4(simd_cross(z, x), 0), SIMD4(z, 0), SIMD4(eye, 1)))
    }

    /// Head-pose-like orientations around the sphere: (yaw, pitch) in degrees.
    static let angles: [(Float, Float)] = [(0, 0), (45, 0), (90, 0), (135, 0), (180, 0), (-135, 0), (-90, 0), (-45, 0),
                                           (20, 50), (110, 50), (-160, 50), (-70, 50),
                                           (65, -50), (155, -50), (-115, -50), (-25, -50)]

    static func pose(yaw: Float, pitch: Float, distance: Float = 0.5) -> simd_float4x4 {
        let a = yaw * .pi / 180, e = pitch * .pi / 180
        return lookAt(eye: distance * SIMD3(cos(e) * sin(a), -sin(e), -cos(e) * cos(a)))
    }

    /// A keyframe rendered from the given pose (camera frame +Y down, pixel centers at integer coordinates).
    static func keyframe(id: Int, pose: simd_float4x4, yaw: Float, pitch: Float, width: Int = 320, height: Int = 240,
                         focal: Float = 280, sharpness: Float = 100) -> Keyframe {
        let cx = Float(width - 1) / 2, cy = Float(height - 1) / 2
        var rgb = [UInt8](repeating: 0, count: width * height * 3)
        var depth = [UInt16](repeating: 0, count: width * height)
        var mask = [UInt8](repeating: 0, count: width * height)
        let r = SIMD3(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z)
        let u = SIMD3(pose.columns.1.x, pose.columns.1.y, pose.columns.1.z)
        let f = SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        let eye = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        for y in 0 ..< height {
            for x in 0 ..< width {
                let d = r * ((Float(x) - cx) / focal) + u * ((Float(y) - cy) / focal) + f
                let b = 2 * simd_dot(eye, d), c = simd_dot(eye, eye) - radius * radius, a = simd_dot(d, d)
                let disc = b * b - 4 * a * c
                guard disc >= 0 else { continue }
                let t = (-b - sqrt(disc)) / (2 * a)
                let i = y * width + x
                let color = albedo(simd_normalize(eye + t * d))
                rgb[3 * i] = UInt8(max(0, min(255, color.x.rounded())))
                rgb[3 * i + 1] = UInt8(max(0, min(255, color.y.rounded())))
                rgb[3 * i + 2] = UInt8(max(0, min(255, color.z.rounded())))
                depth[i] = UInt16(min(65535, (t / 0.0001).rounded()))
                mask[i] = 255
            }
        }
        let intrinsics = Intrinsics(width: width, height: height, fx: focal, fy: focal, ppx: cx, ppy: cy)
        let color = ColorImage(width: width, height: height, bytesPerRow: width * 3, pixels: Data(rgb), intrinsics: intrinsics)
        let depthData = depth.withUnsafeBufferPointer { Data(buffer: $0) }
        let aligned = DepthImage(width: width, height: height, pixels: depthData, depthScale: 0.0001, intrinsics: intrinsics)
        let frame = CameraFrame(sequence: UInt64(id), timestampMs: Double(id) * 33, color: color, alignedDepth: aligned)
        let quality = FrameQuality(sharpness: sharpness, angularSpeedDegPerSec: 1, validDepthRatio: 1,
                                   validDepthPoints: width * height / 4, faceDistanceM: 0.5)
        return Keyframe(id: id, frame: frame, scanPose: .front, headPose: HeadPose(yaw: yaw, pitch: pitch, roll: 0),
                        faceRect: .zero, landmarks: [], mask: Data(mask), quality: quality)
    }

    static func keyframes() -> (keyframes: [Keyframe], poses: [simd_float4x4]) {
        var keyframes: [Keyframe] = [], poses: [simd_float4x4] = []
        for (i, (yaw, pitch)) in angles.enumerated() {
            let pose = self.pose(yaw: yaw, pitch: pitch)
            keyframes.append(keyframe(id: i, pose: pose, yaw: yaw, pitch: pitch))
            poses.append(pose)
        }
        return (keyframes, poses)
    }

    /// PSNR (dB) of a texture against the albedo at points spread over every triangle.
    static func psnr(mesh: ReconMesh, texture: TextureImage) -> Double {
        let w = texture.width, h = texture.height
        var sum = 0.0, count = 0
        texture.rgba.withUnsafeBytes { raw in
            func pixel(_ x: Int, _ y: Int) -> SIMD3<Float> {
                let i = (min(h - 1, max(0, y)) * w + min(w - 1, max(0, x))) * 4
                return SIMD3(Float(raw[i]), Float(raw[i + 1]), Float(raw[i + 2]))
            }
            for t in 0 ..< mesh.triangleCount {
                let i = (0 ..< 3).map { Int(mesh.indices[3 * t + $0]) }
                for b: SIMD3<Float> in [[1.0 / 3, 1.0 / 3, 1.0 / 3], [0.6, 0.2, 0.2], [0.2, 0.6, 0.2], [0.2, 0.2, 0.6]] {
                    let p = b.x * mesh.positions[i[0]] + b.y * mesh.positions[i[1]] + b.z * mesh.positions[i[2]]
                    let uv = b.x * mesh.uvs[i[0]] + b.y * mesh.uvs[i[1]] + b.z * mesh.uvs[i[2]]
                    let x = uv.x * Float(w) - 0.5, y = (1 - uv.y) * Float(h) - 0.5
                    let x0 = Int(floor(x)), y0 = Int(floor(y)), fx = x - floor(x), fy = y - floor(y)
                    let c = (1 - fy) * ((1 - fx) * pixel(x0, y0) + fx * pixel(x0 + 1, y0))
                        + fy * ((1 - fx) * pixel(x0, y0 + 1) + fx * pixel(x0 + 1, y0 + 1))
                    let e = c - albedo(simd_normalize(p))
                    sum += Double(simd_dot(e, e))
                    count += 3
                }
            }
        }
        return 10 * log10(255.0 * 255.0 / max(sum / Double(max(count, 1)), 1e-12))
    }
}

/// A scratch directory removed when the value goes away.
final class ScratchDirectory {
    let url: URL
    init() throws {
        url = FileManager.default.temporaryDirectory.appendingPathComponent("ifs-reconkit-tests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    }
    deinit { try? FileManager.default.removeItem(at: url) }
}
