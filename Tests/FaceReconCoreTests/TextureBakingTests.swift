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

    static func bake(_ mesh: ReconMesh, views: [SyntheticView], size: Int32 = 1024, withDepth: Bool = true,
                     configure: (inout fr_texture_params) -> Void = { _ in }) throws -> TextureImage {
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        params.texture_size = size
        configure(&params)
        let store = FrameStore(views, withDepth: withDepth)
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

    /// The texture (at least 1 per channel) at every vertex of the unwrapped mesh.
    static func vertexColors(_ mesh: ReconMesh, _ texture: TextureImage) -> [SIMD3<Float>] {
        texture.rgba.withUnsafeBytes { raw in
            let rgba = raw.bindMemory(to: UInt8.self).baseAddress!
            return mesh.uvs.map {
                simd_max(TextureMeasure.sample(rgba, width: texture.width, height: texture.height, uv: $0), SIMD3(repeating: 1))
            }
        }
    }

    /// PSNR (dB) against the albedo at the vertices within 120 deg azimuth of the front (scanPoses sees them all).
    static func seenPSNR(_ mesh: ReconMesh, _ texture: TextureImage) -> Double {
        var sum = 0.0, count = 0
        for (p, c) in zip(mesh.positions, vertexColors(mesh, texture)) where abs(TexturedSphere.azimuth(simd_normalize(p))) <= 120 {
            let e = c - TexturedSphere.color(simd_normalize(p))
            sum += Double(simd_dot(e, e))
            count += 3
        }
        return TextureMeasure.psnr(mse: sum / Double(max(count, 1)))
    }

    /// Mean difference of log(B/R) between two textures of the same mesh over the vertices near the equator
    /// (|elevation| < 15 deg) with an azimuth in `azimuth`.
    static func chromaDifference(_ mesh: ReconMesh, _ texture: TextureImage, _ reference: TextureImage,
                                 azimuth: ClosedRange<Float>) -> Float {
        var sum: Float = 0, count = 0
        for (p, (c, r)) in zip(mesh.positions, zip(vertexColors(mesh, texture), vertexColors(mesh, reference))) {
            let d = simd_normalize(p)
            guard abs(asin(d.y)) * 180 / .pi < 15, azimuth.contains(TexturedSphere.azimuth(d)) else { continue }
            sum += log(c.z / c.x) - log(r.z / r.x)
            count += 1
        }
        return sum / Float(max(count, 1))
    }

    /// 99th percentiles of the steps of log(texture / albedo) across the mesh edges within 120 deg azimuth of the
    /// front, in B - R and in brightness (mean of the channel logs): seams between views lit or exposed differently.
    static func steps(_ mesh: ReconMesh, _ texture: TextureImage) -> (chroma: Float, brightness: Float) {
        let ratio = zip(mesh.positions, vertexColors(mesh, texture)).map { p, c -> SIMD3<Float> in
            let a = TexturedSphere.color(simd_normalize(p))
            return SIMD3(log(c.x / a.x), log(c.y / a.y), log(c.z / a.z))
        }
        var chroma: [Float] = [], brightness: [Float] = []
        for t in 0 ..< mesh.triangleCount {
            for k in 0 ..< 3 {
                let i = Int(mesh.indices[3 * t + k]), j = Int(mesh.indices[3 * t + (k + 1) % 3])
                guard i < j, abs(TexturedSphere.azimuth(simd_normalize(mesh.positions[i]))) <= 120 else { continue }
                let d = ratio[i] - ratio[j]
                chroma.append(abs(d.z - d.x))
                brightness.append(abs(d.x + d.y + d.z) / 3)
            }
        }
        func p99(_ v: [Float]) -> Float { v.sorted()[min(v.count - 1, v.count * 99 / 100)] }
        return (p99(chroma), p99(brightness))
    }

    /// Exposure and white balance per frame of a camera under auto exposure (the real D455 scan: the first frames
    /// brighter and warmer than the rest, small drifts after that), geometric mean 1 per channel.
    static func cameraGains(count: Int) -> [SIMD3<Float>] {
        let raw = (0 ..< count).map { i -> SIMD3<Float> in
            i < 3 ? SIMD3(1.28, 1.22, 1.16) : SIMD3(0.97 + 0.02 * Float(i % 3), 0.98, 1.0 - 0.02 * Float(i % 4))
        }
        let logMean = raw.reduce(SIMD3<Float>.zero) { $0 + SIMD3(log($1.x), log($1.y), log($1.z)) } / Float(count)
        return raw.map { $0 / SIMD3(exp(logMean.x), exp(logMean.y), exp(logMean.z)) }
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

    @Test func normalsStayContinuousAcrossSeams() throws {
        var mesh = try Self.unwrappedSphere(level: 3, size: 512)
        try mesh.withCoreMesh { try check(fr_mesh_compute_normals($0)) }
        var byPosition: [SIMD3<Float>: SIMD3<Float>] = [:]
        var seamVertices = 0
        for (p, n) in zip(mesh.positions, mesh.normals) {
            if let other = byPosition[p] {
                seamVertices += 1
                #expect(other == n)
            } else {
                byPosition[p] = n
            }
            #expect(simd_dot(n, simd_normalize(p)) > 0.99)
        }
        #expect(seamVertices > 0)
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

    /// The head turns under fixed room lights (TexturedSphere.roomLight: they stay with the camera), the camera changes
    /// exposure and white balance, the background stands still. Surface only one pose sees must keep the color of
    /// that view, corrected for the camera only: per-frame per-channel gains fitted on the overlaps carried the
    /// lighting change there to the whole frame (here they shifted the single-pose sides by 0.50 log B/R; a real
    /// scan's side went blue). Where poses meet, the views must agree better than the uncorrected lighting does
    /// (p99 B-R steps: 0.069 compensated, 0.162 for a perfect camera without compensation, 0.134 with the per-channel
    /// gains).
    @Test func compensationKeepsTheLightingOfSurfaceSeenByOnePose() throws {
        let mesh = try Self.unwrappedSphere(level: 5)
        let poses = TexturedSphere.scanPoses()
        let gains = Self.cameraGains(count: poses.count)
        let views = poses.enumerated().map { i, pose in
            SyntheticView.render(pose: pose, gain: gains[i], shading: TexturedSphere.roomLight, background: TexturedSphere.backdrop)
        }
        let texture = try Self.bake(mesh, views: views)
        // Reference: the same lighting seen by a perfect camera (gain 1), without any compensation.
        let lit = poses.map { SyntheticView.render(pose: $0, shading: TexturedSphere.roomLight, background: TexturedSphere.backdrop) }
        let reference = try Self.bake(mesh, views: lit) { $0.exposure_compensation = 0 }
        // Seen only by the frames turned 50 deg to either side (the front ones reach 78 deg, the side ones 128 deg).
        let errors = Self.sides.map { Self.chromaDifference(mesh, texture, reference, azimuth: $0) }
        let compensated = Self.steps(mesh, texture), uncompensated = Self.steps(mesh, reference)
        print("lighting that turns with the head: log B/R change of the single-pose sides \(errors); edge steps p99 B-R / brightness \(compensated) compensated, \(uncompensated) uncompensated perfect camera")
        #expect(errors.allSatisfy { abs($0) < 0.04 })
        #expect(compensated.chroma < 0.6 * uncompensated.chroma)
        #expect(compensated.brightness < uncompensated.brightness)
    }

    /// Azimuth ranges seen by one pose of scanPoses only (away from grazing views).
    static let sides: [ClosedRange<Float>] = [90 ... 115, -115 ... -90]

    /// A still camera's background shows its exposure and white balance changes alone: they are removed exactly, also
    /// on surface only one pose sees (a correction derived from the overlaps fades out there; measured 0.0025 log
    /// B/R, psnr 46.9 dB against 47.1 dB for a perfect camera). Without a usable background (none, or a moving
    /// camera) the brightness gains and the local field take over: 44.8 dB, single-pose sides within 0.02 log B/R.
    @Test func backgroundGivesTheCameraGains() throws {
        let mesh = try Self.unwrappedSphere(level: 5)
        let poses = TexturedSphere.scanPoses()
        let gains = Self.cameraGains(count: poses.count)
        let reference = try Self.bake(mesh, views: poses.map { SyntheticView.render(pose: $0) }) { $0.exposure_compensation = 0 }
        let backgrounds: [(String, (Int) -> ((Int, Int) -> SIMD3<Float>)?)] = [
            ("still", { _ in TexturedSphere.backdrop }),
            ("moving", { i in { x, y in TexturedSphere.backdrop(x + 23 * i, y + 11 * i) } }),
            ("none", { _ in nil }),
        ]
        for (name, background) in backgrounds {
            let views = poses.enumerated().map { i, pose in SyntheticView.render(pose: pose, gain: gains[i], background: background(i)) }
            let texture = try Self.bake(mesh, views: views)
            let psnr = Self.seenPSNR(mesh, texture)
            let errors = Self.sides.map { Self.chromaDifference(mesh, texture, reference, azimuth: $0) }
            print("camera gains, \(name) background: psnr \(psnr) dB (perfect camera \(Self.seenPSNR(mesh, reference)) dB), log B/R error of the single-pose sides \(errors)")
            if name == "still" {
                #expect(psnr > 40)
                #expect(errors.allSatisfy { abs($0) < 0.01 })
            } else {
                #expect(psnr > 35)
                #expect(errors.allSatisfy { abs($0) < 0.04 })
            }
        }
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

    @Test func selfOcclusionUsesTheMeshDepth() throws {
        // A ball in front of the sphere is part of the mesh; the frames carry no depth, so only the mesh z-buffer can
        // keep the ball's green off the sphere behind it.
        var mesh = MeshFactory.icosphere(level: 4, radius: TexturedSphere.radius)
        let sphereTriangles = mesh.triangleCount
        let ballCenter = SIMD3<Float>(0.01, 0, -0.2)
        let ball = MeshFactory.icosphere(level: 3, radius: 0.04, center: ballCenter)
        let offset = UInt32(mesh.vertexCount)
        mesh.positions += ball.positions
        mesh.indices += ball.indices.map { $0 + offset }
        var params = fr_uv_params()
        fr_uv_params_default(&params)
        params.texture_size = 1024
        try mesh.withCoreMesh {
            try check(fr_mesh_compute_normals($0))
            try check(fr_mesh_uv_unwrap($0, &params, nil, nil))
        }
        let occluder = SyntheticView.Occluder(center: ballCenter, radius: 0.04, color: [0, 255, 0])
        let views = TexturedSphere.surroundingPoses().map { SyntheticView.render(pose: $0, occluder: occluder) }
        let texture = try Self.bake(mesh, views: views, withDepth: false)
        // Only the sphere's triangles (xatlas keeps the face order).
        var sphere = mesh
        sphere.indices = Array(mesh.indices.prefix(3 * sphereTriangles))
        let psnr = Self.psnr(sphere, texture)
        // Control: with a huge occlusion tolerance the depth test passes everything and the ball bleeds through.
        let leaky = try Self.bake(mesh, views: views, withDepth: false) { $0.occlusion_epsilon_m = 10 }
        let control = Self.psnr(sphere, leaky)
        print("self-occlusion psnr: \(psnr) dB (without the depth test: \(control) dB)")
        #expect(psnr > 30)
        #expect(control < psnr - 10)
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

    /// Colors of surface no frame sees are diffused from the seen region over the surface, not over the seam-split
    /// atlas mesh: a chart without any seen vertex must not fall back to the global mean color (in a head scan:
    /// skin-colored patches in the hair at the open back). Vertex colors and the baked texture of an unwrapped mesh
    /// must equal those of the same surface without seams.
    @Test func unseenColorsDoNotDependOnUVSeams() throws {
        let whole = MeshFactory.icosphere(level: 3, radius: TexturedSphere.radius)
        let split = try Self.unwrappedSphere(level: 3, size: 512)
        #expect(split.vertexCount > whole.vertexCount)
        // One view from the front: the back half of the sphere is unseen.
        let views = [SyntheticView.render(pose: TexturedSphere.surroundingPoses()[0])]
        let store = FrameStore(views)
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        func colorize(_ mesh: ReconMesh) throws -> ReconMesh {
            var m = mesh
            try m.withCoreMesh { core in
                try store.withPointers { frames, poses, count in
                    try check(fr_colorize_vertices(core, frames, poses, nil, count, &params))
                }
            }
            return m
        }
        let wholeColored = try colorize(whole), splitColored = try colorize(split)
        var reference: [SIMD3<Float>: SIMD3<Float>] = [:]
        for (p, c) in zip(wholeColored.positions, wholeColored.colors) { reference[p] = SIMD3<Float>(c) }
        // Unseen = facing away from the camera at -Z (surroundingPoses()[0] looks at the sphere from there).
        let unseen = split.positions.indices.filter { split.positions[$0].z > 0.02 }
        #expect(unseen.count > split.vertexCount / 4)
        var vertexError: Float = 0
        for i in split.positions.indices {
            let expected = try #require(reference[split.positions[i]])
            vertexError = max(vertexError, simd_reduce_max(simd_abs(SIMD3<Float>(splitColored.colors[i]) - expected)))
        }
        let texture = try Self.bake(split, views: views, size: 512)
        var textureError: Float = 0
        texture.rgba.withUnsafeBytes { raw in
            let rgba = raw.bindMemory(to: UInt8.self).baseAddress!
            for i in unseen {
                let c = TextureMeasure.sample(rgba, width: texture.width, height: texture.height, uv: split.uvs[i])
                textureError += simd_reduce_max(simd_abs(c - reference[split.positions[i]]!))
            }
        }
        textureError /= Float(unseen.count)
        print("unseen colors: max vertex color difference split vs whole \(vertexError), mean texture difference \(textureError)")
        #expect(vertexError <= 1)
        #expect(textureError < 3)
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
