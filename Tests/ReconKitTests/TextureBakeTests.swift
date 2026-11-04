// License: Apache 2.0. See LICENSE file in root directory.

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd
import Testing
@testable import ReconKit

@Suite("Texturing")
struct TextureBakeTests {
    @Test func unwrapAndBakeReproducesTheAlbedo() throws {
        var mesh = TestMeshes.icosphere(level: 4)
        let scene = SphereScene.keyframes()
        var options = ReconstructionOptions()
        options.textureSize = 1024
        let texture = try #require(try Texturing.unwrapAndBake(&mesh, keyframes: scene.keyframes, poses: scene.poses,
                                                               options: options))
        #expect(texture.width == 1024 && texture.height == 1024 && texture.rgba.count == 1024 * 1024 * 4)
        #expect(mesh.uvs.count == mesh.vertexCount && mesh.normals.count == mesh.vertexCount)
        let psnr = SphereScene.psnr(mesh: mesh, texture: texture)
        print("Texturing.unwrapAndBake psnr: \(psnr) dB")
        #expect(psnr > 30)
    }

    @Test func colorizeVerticesMatchesTheAlbedo() throws {
        var mesh = TestMeshes.icosphere(level: 4)
        let scene = SphereScene.keyframes()
        try Texturing.colorizeVertices(&mesh, keyframes: scene.keyframes, poses: scene.poses, options: ReconstructionOptions())
        #expect(mesh.colors.count == mesh.vertexCount)
        let mse = zip(mesh.positions, mesh.colors).map { p, c -> Double in
            let e = SIMD3<Float>(c) - SphereScene.albedo(simd_normalize(p))
            return Double(simd_dot(e, e))
        }.reduce(0, +) / Double(3 * mesh.vertexCount)
        #expect(10 * log10(255 * 255 / mse) > 30)
    }

    /// Keyframes of the sphere scene with everything above y = 0.075 painted pure red, as if the frames saw something
    /// else there than the mesh (a synthesized surface is a guess of where the head is).
    static func keyframesWithRedCap() -> (keyframes: [Keyframe], poses: [simd_float4x4]) {
        var scene = SphereScene.keyframes()
        for (i, pose) in scene.poses.enumerated() {
            var kf = scene.keyframes[i]
            let w = kf.frame.color.width, h = kf.frame.color.height, focal = kf.frame.color.intrinsics.fx
            let cx = Float(w - 1) / 2, cy = Float(h - 1) / 2
            let r = SIMD3(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z), u = SIMD3(pose.columns.1.x, pose.columns.1.y, pose.columns.1.z)
            let f = SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z), eye = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
            var rgb = [UInt8](kf.frame.color.pixels)
            for y in 0 ..< h {
                for x in 0 ..< w {
                    let d = r * ((Float(x) - cx) / focal) + u * ((Float(y) - cy) / focal) + f
                    let b = 2 * simd_dot(eye, d), c = simd_dot(eye, eye) - SphereScene.radius * SphereScene.radius, a = simd_dot(d, d)
                    let disc = b * b - 4 * a * c
                    guard disc >= 0, (eye + (-b - sqrt(disc)) / (2 * a) * d).y > 0.075 else { continue }
                    rgb[3 * (y * w + x)] = 255; rgb[3 * (y * w + x) + 1] = 0; rgb[3 * (y * w + x) + 2] = 0
                }
            }
            kf.frame.color.pixels = Data(rgb)
            scene.keyframes[i] = kf
        }
        return scene
    }

    static func isRed(_ c: SIMD3<UInt8>) -> Bool { c.x > 200 && c.y < 40 && c.z < 40 }

    /// Synthesized vertices (here the cap above y = 0.07) are never colored from the frames: they take a smooth
    /// interpolation of the measured colors around them, within their range, in the vertex colors and the texture.
    @Test func synthesizedSurfaceTakesInterpolatedColors() throws {
        let scene = Self.keyframesWithRedCap()
        var options = ReconstructionOptions()
        options.textureSize = 512
        // Without flags the cap is red: the frames paint it.
        var plain = TestMeshes.icosphere(level: 4)
        try Texturing.colorizeVertices(&plain, keyframes: scene.keyframes, poses: scene.poses, options: options)
        let top = plain.positions.indices.filter { plain.positions[$0].y > 0.085 }
        #expect(top.allSatisfy { Self.isRed(plain.colors[$0]) })

        var mesh = TestMeshes.icosphere(level: 4)
        mesh.synthesized = mesh.positions.map { $0.y > 0.07 }
        try Texturing.colorizeVertices(&mesh, keyframes: scene.keyframes, poses: scene.poses, options: options)
        let synthesized = mesh.positions.indices.filter { mesh.synthesized[$0] }
        #expect(!synthesized.isEmpty && synthesized.allSatisfy { !Self.isRed(mesh.colors[$0]) })
        // Within the range of the measured colors next to the cap (a harmonic interpolation), not one global mean.
        let border = mesh.positions.indices.filter { !mesh.synthesized[$0] && mesh.positions[$0].y > 0.06 }
        let lo = border.map { SIMD3<Float>(mesh.colors[$0]) }.reduce(SIMD3<Float>(repeating: 255)) { simd_min($0, $1) }
        let hi = border.map { SIMD3<Float>(mesh.colors[$0]) }.reduce(SIMD3<Float>(repeating: 0)) { simd_max($0, $1) }
        #expect(synthesized.allSatisfy { i in
            let c = SIMD3<Float>(mesh.colors[i])
            return all(c .>= lo - 1) && all(c .<= hi + 1)
        })
        #expect(Set(synthesized.map { mesh.colors[$0] }).count > 10)

        // The texture: texels of triangles with only synthesized corners are not red either.
        var textured = TestMeshes.icosphere(level: 4)
        textured.synthesized = textured.positions.map { $0.y > 0.07 }
        let texture = try #require(try Texturing.unwrapAndBake(&textured, keyframes: scene.keyframes, poses: scene.poses,
                                                               options: options))
        #expect(textured.synthesized.count == textured.vertexCount, "flags follow the UV seam split")
        var checked = 0, red = 0
        texture.rgba.withUnsafeBytes { raw in
            for t in 0 ..< textured.triangleCount {
                let i = (0 ..< 3).map { Int(textured.indices[3 * t + $0]) }
                guard i.allSatisfy({ textured.synthesized[$0] }) else { continue }
                let uv = (textured.uvs[i[0]] + textured.uvs[i[1]] + textured.uvs[i[2]]) / 3
                let x = min(texture.width - 1, Int(uv.x * Float(texture.width))), y = min(texture.height - 1, Int((1 - uv.y) * Float(texture.height)))
                let o = (y * texture.width + x) * 4
                checked += 1
                if Self.isRed(SIMD3(raw[o], raw[o + 1], raw[o + 2])) { red += 1 }
            }
        }
        #expect(checked > 50 && red == 0)
    }

    @Test func frameWeightsFavorSharpFrontalFrames() {
        let pose = SphereScene.pose(yaw: 0, pitch: 0)
        let sharp = SphereScene.keyframe(id: 0, pose: pose, yaw: 0, pitch: 0, width: 8, height: 8, focal: 8, sharpness: 200)
        let blurry = SphereScene.keyframe(id: 1, pose: pose, yaw: 0, pitch: 0, width: 8, height: 8, focal: 8, sharpness: 20)
        let side = SphereScene.keyframe(id: 2, pose: pose, yaw: 60, pitch: 0, width: 8, height: 8, focal: 8, sharpness: 200)
        let weights = Texturing.frameWeights(for: [sharp, blurry, side])
        #expect(weights.allSatisfy { $0 > 0 && $0 <= 1 })
        #expect(weights[0] > weights[1] && weights[0] > weights[2])
    }

    @Test func rejectsInconsistentKeyframes() throws {
        var mesh = TestMeshes.icosphere(level: 2)
        let scene = SphereScene.keyframes()
        #expect(throws: ReconstructionError.self) {
            _ = try Texturing.unwrapAndBake(&mesh, keyframes: scene.keyframes, poses: Array(scene.poses.dropLast()),
                                            options: ReconstructionOptions())
        }
        var broken = scene.keyframes[0]
        broken.frame.alignedDepth.pixels = Data(count: 16)
        #expect(throws: ReconstructionError.self) {
            try Texturing.colorizeVertices(&mesh, keyframes: [broken], poses: [scene.poses[0]], options: ReconstructionOptions())
        }
        #expect(throws: ReconstructionError.self) {
            try Texturing.colorizeVertices(&mesh, keyframes: [], poses: [], options: ReconstructionOptions())
        }
    }
}
