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
