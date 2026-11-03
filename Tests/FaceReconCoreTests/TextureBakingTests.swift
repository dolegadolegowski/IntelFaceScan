// License: Apache 2.0. See LICENSE file in root directory.
//
// UV unwrapping and texture baking: the baked texture of a sphere seen from synthetic views must reproduce the
// sphere's known albedo.

import FaceReconCore
import Foundation
import ReconKit
import simd
import Testing

@Suite("Texture")
struct TextureBakingTests {
    static func unwrappedSphere(level: Int = 4, size: Int32 = 1024) throws -> ReconMesh {
        var mesh = MeshFactory.icosphere(level: level, radius: TexturedSphere.radius)
        var params = fr_uv_params()
        fr_uv_params_default(&params)
        params.texture_size = size
        try mesh.withCoreMesh {
            try check(fr_mesh_compute_normals($0))
            try check(fr_mesh_uv_unwrap($0, &params, nil, nil))
        }
        return mesh
    }

    static func bake(_ mesh: ReconMesh, views: [SyntheticView], size: Int32 = 1024,
                     configure: (inout fr_texture_params) -> Void = { _ in }) throws -> TextureImage {
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        params.texture_size = size
        configure(&params)
        let store = FrameStore(views)
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var texture: UnsafeMutablePointer<fr_texture>?
        try store.withPointers { frames, poses, count in
            try check(fr_bake_texture(core, frames, poses, nil, count, &params, &texture, nil, nil))
        }
        let t = try #require(texture)
        defer { fr_texture_free(t) }
        return TextureImage(core: t)
    }

    static func psnr(_ mesh: ReconMesh, _ texture: TextureImage) -> Double {
        texture.rgba.withUnsafeBytes {
            TextureMeasure.spherePSNR(mesh: mesh, rgba: $0.bindMemory(to: UInt8.self).baseAddress!,
                                      width: texture.width, height: texture.height)
        }
    }

    @Test func unwrapProducesANonOverlappingAtlas() throws {
        let original = MeshFactory.icosphere(level: 4, radius: TexturedSphere.radius)
        let mesh = try Self.unwrappedSphere()
        #expect(mesh.triangleCount == original.triangleCount)
        #expect(mesh.vertexCount > original.vertexCount)  // seams split vertices
        #expect(mesh.uvs.count == mesh.vertexCount && mesh.normals.count == mesh.vertexCount)
        #expect(mesh.uvs.allSatisfy { $0.x >= 0 && $0.x <= 1 && $0.y >= 0 && $0.y <= 1 })
        // Every split vertex is a copy of an original one.
        let originals = Set(original.positions.map { SIMD3<Int32>(($0 * 1e6).rounded(.toNearestOrEven)) })
        #expect(mesh.positions.allSatisfy { originals.contains(SIMD3<Int32>(($0 * 1e6).rounded(.toNearestOrEven))) })
        // Same surface (topology up to seams): equal area.
        #expect(abs(MeshMeasure.area(mesh) - MeshMeasure.area(original)) < 1e-9)

        // Rasterize the UV triangles on a coarse grid: charts must not overlap and should fill the atlas well.
        let n = 256
        var coverage = [UInt8](repeating: 0, count: n * n)
        var overlaps = 0, covered = 0
        for t in 0 ..< mesh.triangleCount {
            let uv = (0 ..< 3).map { mesh.uvs[Int(mesh.indices[3 * t + $0])] * Float(n) }
            let lo = simd_min(simd_min(uv[0], uv[1]), uv[2]), hi = simd_max(simd_max(uv[0], uv[1]), uv[2])
            let area = (uv[1].x - uv[0].x) * (uv[2].y - uv[0].y) - (uv[1].y - uv[0].y) * (uv[2].x - uv[0].x)
            guard abs(area) > 0 else { continue }
            for y in max(0, Int(lo.y)) ... min(n - 1, Int(hi.y)) {
                for x in max(0, Int(lo.x)) ... min(n - 1, Int(hi.x)) {
                    let p = SIMD2(Float(x) + 0.5, Float(y) + 0.5)
                    let l0 = ((uv[2].x - uv[1].x) * (p.y - uv[1].y) - (uv[2].y - uv[1].y) * (p.x - uv[1].x)) / area
                    let l1 = ((uv[0].x - uv[2].x) * (p.y - uv[2].y) - (uv[0].y - uv[2].y) * (p.x - uv[2].x)) / area
                    let l2 = 1 - l0 - l1
                    guard l0 > 1e-4, l1 > 1e-4, l2 > 1e-4 else { continue }
                    if coverage[y * n + x] > 0 { overlaps += 1 } else { covered += 1 }
                    coverage[y * n + x] += 1
                }
            }
        }
        print("uv atlas: \(mesh.vertexCount) vertices, coverage \(Double(covered) / Double(n * n)), overlaps \(overlaps)")
        #expect(overlaps == 0)
        #expect(Double(covered) / Double(n * n) > 0.5)
    }

    /// Dense smooth closed surfaces used to collapse into one orthographic "planar" xatlas chart (and crash the
    /// packer); the charts must stay small enough to be overlap-free.
    @Test func denseSmoothSurfacesGetProperCharts() throws {
        for closed in [true, false] {
            var mesh = MeshFactory.icosphere(level: 6, radius: TexturedSphere.radius)
            if !closed {
                try mesh.withCoreMesh { try check(fr_mesh_crop_plane($0, [0, 0.07, 0], [0, -1, 0])) }
            }
            var params = fr_uv_params()
            fr_uv_params_default(&params)
            params.texture_size = 2048
            try mesh.withCoreMesh { try check(fr_mesh_uv_unwrap($0, &params, nil, nil)) }
            let n = 512
            var coverage = [UInt8](repeating: 0, count: n * n)
            var overlaps = 0, covered = 0
            for t in 0 ..< mesh.triangleCount {
                let uv = (0 ..< 3).map { mesh.uvs[Int(mesh.indices[3 * t + $0])] * Float(n) }
                let lo = simd_min(simd_min(uv[0], uv[1]), uv[2]), hi = simd_max(simd_max(uv[0], uv[1]), uv[2])
                let area = (uv[1].x - uv[0].x) * (uv[2].y - uv[0].y) - (uv[1].y - uv[0].y) * (uv[2].x - uv[0].x)
                guard abs(area) > 0, lo.x >= 0, lo.y >= 0 else { continue }
                for y in max(0, Int(lo.y)) ... min(n - 1, Int(hi.y)) {
                    for x in max(0, Int(lo.x)) ... min(n - 1, Int(hi.x)) {
                        let p = SIMD2(Float(x) + 0.5, Float(y) + 0.5)
                        let l0 = ((uv[2].x - uv[1].x) * (p.y - uv[1].y) - (uv[2].y - uv[1].y) * (p.x - uv[1].x)) / area
                        let l1 = ((uv[0].x - uv[2].x) * (p.y - uv[2].y) - (uv[0].y - uv[2].y) * (p.x - uv[2].x)) / area
                        guard l0 > 1e-4, l1 > 1e-4, 1 - l0 - l1 > 1e-4 else { continue }
                        if coverage[y * n + x] > 0 { overlaps += 1 } else { covered += 1 }
                        coverage[y * n + x] = 1
                    }
                }
            }
            print("dense \(closed ? "closed" : "open") sphere: \(mesh.vertexCount) vertices, coverage \(Double(covered) / Double(n * n)), overlaps \(overlaps)")
            #expect(overlaps < covered / 1000)
            #expect(Double(covered) / Double(n * n) > 0.4)
        }
    }

    @Test func bakedTextureReproducesTheSurface() throws {
        let mesh = try Self.unwrappedSphere()
        let views = TexturedSphere.surroundingPoses().map { SyntheticView.render(pose: $0) }
        let texture = try Self.bake(mesh, views: views)
        #expect(texture.width == 1024 && texture.height == 1024)
        // Opaque everywhere (gutter included).
        let alphaOK = texture.rgba.withUnsafeBytes { raw in stride(from: 3, to: raw.count, by: 4).allSatisfy { raw[$0] == 255 } }
        #expect(alphaOK)
        let psnr = Self.psnr(mesh, texture)
        print("bake psnr: \(psnr) dB")
        #expect(psnr > 32)
    }

    @Test func exposureCompensationEqualizesFrames() throws {
        let mesh = try Self.unwrappedSphere()
        // Per-frame gains with a geometric mean of ~1, including a white balance shift.
        let gains: [SIMD3<Float>] = [[0.7, 0.75, 0.7], [1.3, 1.25, 1.35], [0.85, 0.9, 0.8], [1.2, 1.15, 1.2]]
        let views = TexturedSphere.surroundingPoses().enumerated().map { i, pose in
            SyntheticView.render(pose: pose, gain: gains[i % gains.count])
        }
        let compensated = Self.psnr(mesh, try Self.bake(mesh, views: views))
        let plain = Self.psnr(mesh, try Self.bake(mesh, views: views) { $0.exposure_compensation = 0 })
        print("exposure: psnr with compensation \(compensated) dB, without \(plain) dB")
        #expect(compensated > plain + 3)
        #expect(compensated > 28)
    }

    @Test func occludersSeenOnlyByTheSensorAreRejected() throws {
        let mesh = try Self.unwrappedSphere()
        let poses = TexturedSphere.surroundingPoses()
        // A green ball between the camera and the sphere in every other equatorial view (not part of the mesh).
        let views = poses.enumerated().map { i, pose -> SyntheticView in
            guard i < 8, i % 2 == 0 else { return SyntheticView.render(pose: pose) }
            let eye = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
            let ball = SyntheticView.Occluder(center: eye * 0.55 + SIMD3(0.02, 0.01, 0), radius: 0.035, color: [0, 255, 0])
            return SyntheticView.render(pose: pose, occluder: ball)
        }
        let psnr = Self.psnr(mesh, try Self.bake(mesh, views: views))
        print("occluder psnr: \(psnr) dB")
        #expect(psnr > 30)
    }

    @Test func vertexColorsMatchTheSurface() throws {
        var mesh = MeshFactory.icosphere(level: 4, radius: TexturedSphere.radius)
        let views = TexturedSphere.surroundingPoses().map { SyntheticView.render(pose: $0) }
        let store = FrameStore(views)
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        try mesh.withCoreMesh { core in
            try store.withPointers { frames, poses, count in
                try check(fr_colorize_vertices(core, frames, poses, nil, count, &params))
            }
        }
        #expect(mesh.colors.count == mesh.vertexCount)
        var sum = 0.0
        for (p, c) in zip(mesh.positions, mesh.colors) {
            let e = SIMD3<Float>(c) - TexturedSphere.color(simd_normalize(p))
            sum += Double(simd_dot(e, e))
        }
        let psnr = TextureMeasure.psnr(mse: sum / Double(3 * mesh.vertexCount))
        print("vertex color psnr: \(psnr) dB")
        #expect(psnr > 32)
    }

    @Test func unseenSurfaceGetsPlausibleColors() throws {
        // Only the front view: the back of the sphere is filled from the seen region instead of staying black.
        let mesh = try Self.unwrappedSphere(level: 3, size: 256)
        let views = [SyntheticView.render(pose: TexturedSphere.surroundingPoses()[0])]
        let texture = try Self.bake(mesh, views: views, size: 256)
        let dark = texture.rgba.withUnsafeBytes { raw in
            stride(from: 0, to: raw.count, by: 4).filter { Int(raw[$0]) + Int(raw[$0 + 1]) + Int(raw[$0 + 2]) < 30 }.count
        }
        #expect(dark == 0)
    }

    @Test func bakingRejectsBadInput() throws {
        let plain = MeshFactory.icosphere(level: 2)
        let core = try plain.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        var texture: UnsafeMutablePointer<fr_texture>?
        let store = FrameStore([SyntheticView.render(pose: TexturedSphere.surroundingPoses()[0], width: 64, height: 48, focal: 60)])
        store.withPointers { frames, poses, count in
            #expect(fr_bake_texture(core, frames, poses, nil, count, &params, &texture, nil, nil) == FR_ERR_INVALID_ARGUMENT)
        }
        #expect(texture == nil)
        let mesh = try Self.unwrappedSphere(level: 2, size: 128)
        let uvCore = try mesh.makeCoreMesh()
        defer { fr_mesh_free(uvCore) }
        // Poses that look away from the mesh.
        var away = TexturedSphere.lookAt(eye: [0, 0, -0.5], target: [0, 0, -1]).columnMajor
        store.withPointers { frames, _, count in
            #expect(fr_bake_texture(uvCore, frames, &away, nil, count, &params, &texture, nil, nil) == FR_ERR_NOT_ENOUGH_DATA)
        }
        // Cancellation from the progress callback.
        params.texture_size = 128
        store.withPointers { frames, poses, count in
            let status = fr_bake_texture(uvCore, frames, poses, nil, count, &params, &texture, { _, _, _ in 1 }, nil)
            #expect(status == FR_ERR_CANCELLED)
        }
        #expect(texture == nil)
    }
}
