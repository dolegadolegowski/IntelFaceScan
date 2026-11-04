// License: Apache 2.0. See LICENSE file in root directory.
//
// Mesh post-processing on synthetic meshes: normals, cleanup, components, smoothing, holes, decimation, cropping.

import FaceReconCore
import Foundation
import ReconKit
import simd
import Testing

@Suite("Mesh operations")
struct MeshOperationTests {
    @Test func normalsPointOutward() throws {
        let sphere = MeshFactory.icosphere(level: 3)
        let normals = try MeshMeasure.normals(of: sphere)
        #expect(normals.count == sphere.vertexCount)
        for (p, n) in zip(sphere.positions, normals) {
            #expect(abs(simd_length(n) - 1) < 1e-4)
            #expect(simd_dot(n, simd_normalize(p)) > 0.99)
        }
    }

    @Test func cleanWeldsAndRemovesDegenerateAndDuplicateTriangles() throws {
        let n = 6
        var cube = MeshFactory.cube(n: n)
        cube.colors = cube.positions.map { p in SIMD3(UInt8(100 + 1000 * p.x), UInt8(100 + 1000 * p.y), 7) }
        let triangles = cube.triangleCount
        // Extra junk: a duplicate triangle (reversed), a degenerate one and an unreferenced vertex.
        cube.indices += [cube.indices[2], cube.indices[1], cube.indices[0]]
        cube.indices += [cube.indices[3], cube.indices[3], cube.indices[4]]
        cube.positions.append([5, 5, 5]); cube.colors.append([1, 2, 3])
        try cube.withCoreMesh { try check(fr_mesh_clean($0, 1e-6)) }
        #expect(cube.vertexCount == 6 * n * n + 2)
        #expect(cube.triangleCount == triangles)
        #expect(cube.colors.count == cube.vertexCount)
        // Colors stay attached to their positions.
        for (p, c) in zip(cube.positions, cube.colors) {
            #expect(c == SIMD3(UInt8(100 + 1000 * p.x), UInt8(100 + 1000 * p.y), 7))
        }
        let topology = MeshMeasure.topology(cube)
        #expect(topology.boundaryEdges == 0 && topology.consistent)
        #expect(abs(MeshMeasure.signedVolume(cube) - 0.001) < 1e-9)
    }

    @Test func cleanKeepsVerticesBeyondEpsilonAndRespectsUVSeams() throws {
        // Two coincident vertices with different UVs (a texture seam) must not be welded.
        var mesh = ReconMesh(positions: [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [2, 0, 0]],
                             uvs: [[0, 0], [0.5, 0], [0, 0.5], [0.6, 0], [0.6, 0.5], [0, 0.5], [1, 1]],
                             indices: [0, 1, 2, 3, 4, 5, 3, 6, 4])
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 1e-4)) }
        #expect(mesh.vertexCount == 6)  // vertex 5 welds into 2 (same UV), 3 stays apart from 1
        #expect(mesh.triangleCount == 3)
    }

    @Test func removesSmallComponents() throws {
        var mesh = MeshFactory.icosphere(level: 4)
        let big = mesh.triangleCount
        let small = MeshFactory.icosphere(level: 1, radius: 0.01, center: [0.3, 0, 0])
        let offset = UInt32(mesh.vertexCount)
        mesh.positions += small.positions
        mesh.indices += small.indices.map { $0 + offset }
        var removed: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_remove_small_components($0, 0.05, 0, &removed)) }
        #expect(removed == UInt32(small.triangleCount))
        #expect(mesh.triangleCount == big)
        #expect(mesh.positions.allSatisfy { simd_length($0) < 0.11 })

        // A component above the threshold survives unless only the largest is kept.
        mesh.positions += small.positions
        mesh.indices += small.indices.map { $0 + UInt32(mesh.vertexCount - small.vertexCount) }
        try mesh.withCoreMesh { try check(fr_mesh_remove_small_components($0, 0.01, 0, &removed)) }
        #expect(removed == 0)
        try mesh.withCoreMesh { try check(fr_mesh_remove_small_components($0, 0, 1, &removed)) }
        #expect(removed == UInt32(small.triangleCount) && mesh.triangleCount == big)
    }

    @Test func smoothingReducesNoiseWithoutShrinking() throws {
        let radius: Float = 0.1
        var mesh = MeshFactory.icosphere(level: 5, radius: radius)  // edge ~ 2 mm
        try MeshFactory.addNormalNoise(&mesh, sigma: 0.0004, seed: 7)
        let before = MeshMeasure.rms(mesh.positions.map { simd_length($0) - radius })
        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        params.bilateral_sigma_spatial_m = 0  // automatic, from the edge length
        try mesh.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
        let errors = mesh.positions.map { simd_length($0) - radius }
        let after = MeshMeasure.rms(errors)
        let meanRadius = mesh.positions.reduce(0) { $0 + simd_length($1) } / Float(mesh.vertexCount)
        print("smoothing sphere: rms radial error \(before * 1000) mm -> \(after * 1000) mm, mean radius \(meanRadius)")
        #expect(after < 0.4 * before)
        #expect(abs(meanRadius - radius) < 0.002 * radius)
    }

    /// Marching cubes leaves sliver triangles that the bilateral vertex update turned over by the thousands on a real
    /// head scan (49 folded triangles after meshing, 5700 after two bilateral iterations, 1250 after the default
    /// smoothing; on this fused sphere: 9 -> 234). Smoothing must never fold the surface, and still reduce the noise.
    @Test func smoothingNeverFoldsMarchingCubesTriangles() throws {
        let poses = lookAtPoses(target: ReconFusionTests.center, distance: 0.45, directions: cubeDirections)
        let views = poses.enumerated().map { i, pose in
            render(SphereScene(center: ReconFusionTests.center, radius: ReconFusionTests.radius), camera: ReconFusionTests.camera,
                   worldFromCamera: pose, noiseAt1m: 0.008, seed: UInt64(70 + i))
        }
        let (fused, _) = try ReconFusionTests.fuse(views, params: ReconFusionTests.tsdfParams(voxel: 0.0009, gpu: true))
        var mesh = ReconMesh(core: fused.pointer)
        mesh.colors = []
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 1e-7)) }
        let center = SIMD3<Float>(0, 0, 0.5), radius = Float(ReconFusionTests.radius)
        let noise = MeshMeasure.rms(mesh.positions.map { simd_distance($0, center) - radius })
        let before = MeshMeasure.foldedTriangles(mesh)
        for bilateral: Int32 in [2, 4] {
            var params = fr_smooth_params()
            fr_smooth_params_default(&params)
            params.bilateral_sigma_spatial_m = 0
            params.bilateral_iterations = bilateral
            var smoothed = mesh
            try smoothed.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
            let folded = MeshMeasure.foldedTriangles(smoothed)
            let error = MeshMeasure.rms(smoothed.positions.map { simd_distance($0, center) - radius })
            print("smoothing a fused sphere (\(mesh.triangleCount) triangles, \(bilateral) bilateral iterations): folded "
                + "triangles \(before) -> \(folded), rms radial error \(noise * 1000) -> \(error * 1000) mm")
            #expect(folded <= before)
            #expect(error < 0.85 * noise)  // the fused surface is already averaged over 26 views
        }
    }

    /// The neck cut is planar and stays so: smoothing moves boundary vertices only along the boundary and never below
    /// the cut, also where the boundary leaves the plane (the cut joining the open back of a head scan). Before: the
    /// bilateral vertex update pulled cut vertices 0.24 mm out of the plane, and Taubin's inflating steps made the
    /// boundary overshoot below it near the junction, under the chin plane that bounds the export frame's head box.
    @Test(arguments: [false, true])
    func smoothingKeepsAPlanarCutInItsPlane(openBack: Bool) throws {
        var mesh = MeshFactory.icosphere(level: 5)
        try MeshFactory.addNormalNoise(&mesh, sigma: 0.0004, seed: 11)
        let cutY: Float = -0.05
        try mesh.withCoreMesh { try check(fr_mesh_crop_plane($0, [0, cutY, 0], [0, 1, 0])) }
        if openBack {
            // An opening at the back (+Z) from the cut up to the equator, joined to the cut into one boundary loop.
            mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
                let c = (mesh.positions[Int(mesh.indices[t])] + mesh.positions[Int(mesh.indices[t + 1])] + mesh.positions[Int(mesh.indices[t + 2])]) / 3
                return c.z > 0.04 && c.y < 0 && abs(c.x) < 0.04 ? [] : Array(mesh.indices[t ..< t + 3])
            }
            try mesh.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        }
        let loops = MeshMeasure.boundaryLoops(mesh)
        #expect(loops.count == 1)
        let cut = Set(loops.flatMap { $0 }.filter { abs(mesh.positions[Int($0)].y - cutY) < 1e-6 })
        #expect(cut.count > 50)
        for bilateral: Int32 in [0, 2] {
            var params = fr_smooth_params()
            fr_smooth_params_default(&params)
            params.bilateral_sigma_spatial_m = 0
            params.bilateral_iterations = bilateral
            var smoothed = mesh
            try smoothed.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
            let offPlane = cut.map { smoothed.positions[Int($0)].y - cutY }.filter { $0 < 0 }.map { -$0 }.max() ?? 0
            let below = smoothed.positions.map { cutY - $0.y }.max() ?? 0
            print("planar cut (open back: \(openBack)), \(bilateral) bilateral iterations: cut vertices below the plane by "
                + "\(offPlane * 1000) mm, lowest vertex \(below * 1000) mm below it")
            #expect(below < 1e-6)
            if !openBack { #expect(cut.allSatisfy { abs(smoothed.positions[Int($0)].y - cutY) < 1e-6 }) }
        }
    }

    /// Distance of cube vertices to the true surface, split into face interiors and crease (edge) vertices.
    private func cubeErrors(_ mesh: ReconMesh, original: [SIMD3<Float>], half: Float, h: Float) -> (face: Float, crease: Float) {
        var face: [Float] = [], crease: [Float] = []
        for (p, o) in zip(mesh.positions, original) {
            let onFace = (0 ..< 3).filter { abs(abs(o[$0]) - half) < 1e-6 }
            if onFace.count == 1 {
                let axis = onFace[0]
                // Only well inside the face (two rings away from the creases).
                if (0 ..< 3).allSatisfy({ $0 == axis || abs(o[$0]) < half - 2.5 * h }) {
                    face.append(abs(abs(p[axis]) - half))
                }
            } else if onFace.count == 2 {
                // Away from the corners: distance to the edge line.
                let free = (0 ..< 3).first { !onFace.contains($0) }!
                if abs(o[free]) < half - 2.5 * h {
                    let a = onFace[0], b = onFace[1]
                    let da = p[a] - (o[a] > 0 ? half : -half), db = p[b] - (o[b] > 0 ? half : -half)
                    crease.append(sqrt(da * da + db * db))
                }
            }
        }
        return (MeshMeasure.rms(face), MeshMeasure.rms(crease))
    }

    @Test func smoothingKeepsSharpCreases() throws {
        let n = 40, size: Float = 0.1
        let h = size / Float(n)
        var cube = MeshFactory.cube(n: n, size: size)
        try cube.withCoreMesh { try check(fr_mesh_clean($0, 1e-7)) }
        let original = cube.positions
        var noisy = cube
        try MeshFactory.addNormalNoise(&noisy, sigma: 0.08 * h, seed: 3)
        let noise = cubeErrors(noisy, original: original, half: size / 2, h: h)

        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        params.bilateral_sigma_spatial_m = 0
        params.bilateral_iterations = 4
        var smoothed = noisy
        try smoothed.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
        let result = cubeErrors(smoothed, original: original, half: size / 2, h: h)

        // Reference: the same Taubin smoothing without the feature-preserving parts.
        var plain = fr_smooth_params()
        fr_smooth_params_default(&plain)
        plain.bilateral_iterations = 0
        var taubin = noisy
        try taubin.withCoreMesh { try check(fr_mesh_smooth($0, &plain)) }
        let reference = cubeErrors(taubin, original: original, half: size / 2, h: h)
        print("crease test (units of edge length): noise face \(noise.face / h) crease \(noise.crease / h); "
            + "smoothed face \(result.face / h) crease \(result.crease / h); taubin-only face \(reference.face / h) crease \(reference.crease / h)")
        #expect(result.face < 0.5 * noise.face)
        #expect(result.crease < 0.5 * reference.crease)
        #expect(result.crease < 0.15 * h)
    }

    @Test func fillsHolesButLeavesTheLargestBoundaryOpen() throws {
        let radius: Float = 0.1
        var mesh = MeshFactory.icosphere(level: 5, radius: radius)
        mesh.colors = mesh.positions.map { _ in SIMD3<UInt8>(200, 150, 100) }
        // The "neck": everything below y = -0.06; three small holes elsewhere.
        let holeCenters: [SIMD3<Float>] = [simd_normalize([1, 0.3, 0.2]), simd_normalize([-0.5, 0.6, 0.5]), simd_normalize([0, 0.2, -1])]
        let holeRadii: [Float] = [0.006, 0.01, 0.015]
        var kept: [UInt32] = []
        for t in 0 ..< mesh.triangleCount {
            let tri = mesh.indices[3 * t ..< 3 * t + 3]
            let c = tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] } / 3
            if c.y < -0.06 { continue }
            if zip(holeCenters, holeRadii).contains(where: { simd_distance(c, $0.0 * radius) < $0.1 }) { continue }
            kept += tri
        }
        mesh.indices = kept
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        #expect(MeshMeasure.boundaryLoops(mesh).count == 4)
        let vertexCountBefore = mesh.vertexCount

        var filled: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_fill_holes($0, 200, &filled)) }
        #expect(filled == 3)
        let loops = MeshMeasure.boundaryLoops(mesh)
        #expect(loops.count == 1)
        // The remaining boundary is the cut at y = -0.06.
        #expect(loops.first?.allSatisfy { mesh.positions[Int($0)].y < -0.05 } == true)
        #expect(MeshMeasure.topology(mesh).consistent)
        // The patches were refined (new vertices) and faired close to the sphere.
        #expect(mesh.vertexCount > vertexCountBefore + 20)
        let deviation = mesh.positions[vertexCountBefore...].map { abs(simd_length($0) - radius) }
        let maxDeviation = deviation.max() ?? 0
        print("hole filling: \(mesh.vertexCount - vertexCountBefore) new vertices, max deviation \(maxDeviation * 1000) mm")
        #expect(maxDeviation < 0.0015)
        #expect(mesh.colors.count == mesh.vertexCount)
        #expect(mesh.colors[vertexCountBefore...].allSatisfy { $0 == SIMD3<UInt8>(200, 150, 100) })

        // Holes above the size limit stay open.
        var limited = MeshFactory.icosphere(level: 5, radius: radius)
        limited.indices = kept
        try limited.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        try limited.withCoreMesh { try check(fr_mesh_fill_holes($0, 12, &filled)) }
        #expect(filled < 3)
    }

    /// A sphere cut open below y = -0.06 (the "neck"), with one small hole and two flaps that touch the surface only at
    /// a vertex (a lone triangle and a fan of three), as meshing and cropping leave them along ragged borders.
    static func sphereWithFlaps() -> (mesh: ReconMesh, flapTriangles: Int) {
        var mesh = MeshFactory.icosphere(level: 4)
        let hole = simd_normalize(SIMD3<Float>(1, 0.4, 0.3)) * 0.1
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let c = (mesh.positions[Int(mesh.indices[t])] + mesh.positions[Int(mesh.indices[t + 1])] + mesh.positions[Int(mesh.indices[t + 2])]) / 3
            return c.y < -0.06 || simd_distance(c, hole) < 0.012 ? [] : Array(mesh.indices[t ..< t + 3])
        }
        // Anchor vertices on the surface, far from the hole and the cut.
        func nearest(_ d: SIMD3<Float>) -> UInt32 {
            let target = simd_normalize(d) * 0.1
            return UInt32(mesh.positions.indices.min { simd_distance(mesh.positions[$0], target) < simd_distance(mesh.positions[$1], target) }!)
        }
        let a = nearest([-1, 0.5, 0]), b = nearest([0, 0.5, -1])
        func add(_ p: SIMD3<Float>) -> UInt32 { mesh.positions.append(p); return UInt32(mesh.positions.count - 1) }
        let pa = mesh.positions[Int(a)], pb = mesh.positions[Int(b)]
        // Lone triangle sticking out of the surface at vertex a.
        let a1 = add(pa * 1.08 + SIMD3(0, 0.004, 0)), a2 = add(pa * 1.08 + SIMD3(0, -0.004, 0.004))
        mesh.indices += [a, a1, a2]
        // Fan of three triangles at vertex b.
        let b1 = add(pb * 1.1 + SIMD3(0.004, 0, 0)), b2 = add(pb * 1.12), b3 = add(pb * 1.1 + SIMD3(-0.004, 0.002, 0))
        let b4 = add(pb * 1.08 + SIMD3(-0.002, -0.004, 0))
        mesh.indices += [b, b1, b2, b, b2, b3, b, b3, b4]
        return (mesh, 4)
    }

    static func duplicateFaces(_ mesh: ReconMesh) -> Int {
        var seen = Set<SIMD3<UInt32>>(), duplicates = 0
        for t in 0 ..< mesh.triangleCount {
            let s = [mesh.indices[3 * t], mesh.indices[3 * t + 1], mesh.indices[3 * t + 2]].sorted()
            if !seen.insert(SIMD3(s[0], s[1], s[2])).inserted { duplicates += 1 }
        }
        return duplicates
    }

    /// Filling the boundary loop of a flap would close it with a reversed copy (duplicate faces, edges with four
    /// triangles). Measured on a synthetic head reconstruction before the fix: 19 duplicate faces, 24 extra
    /// non-manifold directed edges.
    @Test func fillingHolesNeverDuplicatesFacesOrEdges() throws {
        var (mesh, _) = Self.sphereWithFlaps()
        let before = mesh.triangleCount
        let unreferenced = mesh.vertexCount - Set(mesh.indices).count  // the cut-away part of the sphere
        #expect(MeshMeasure.directedEdges(mesh).values.allSatisfy { $0 == 1 })
        var filled: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_fill_holes($0, 200, &filled)) }
        #expect(mesh.triangleCount > before)
        #expect(Self.duplicateFaces(mesh) == 0)
        #expect(MeshMeasure.directedEdges(mesh).values.allSatisfy { $0 == 1 })
        // The hole is closed: the remaining loops are the neck and at most the flaps' own borders.
        let loops = MeshMeasure.boundaryLoops(mesh)
        #expect(!loops.contains { loop in loop.allSatisfy { simd_distance(mesh.positions[Int($0)], simd_normalize(SIMD3<Float>(1, 0.4, 0.3)) * 0.1) < 0.02 } })
        #expect(filled >= 1)
        // No unreferenced vertices from patches that were discarded.
        #expect(mesh.vertexCount - Set(mesh.indices).count == unreferenced)
    }

    /// Holes whose border collapsed onto one point. Smoothing a scan once contracted a small hole until its three
    /// vertices coincided; refining that patch split degenerate triangles forever (their centroid a rounding error
    /// from the corners, the target edge length zero): 30 GB within a minute in a pipeline test. Also runs of nearly
    /// coincident vertices in a larger border. Patches must stay proportionate to the hole.
    @Test func holesWithCollapsedBordersFillWithBoundedPatches() throws {
        var mesh = MeshFactory.icosphere(level: 4)
        let center = simd_normalize(SIMD3<Float>(0.3, 0.5, 0.8)) * 0.1
        let tiny = simd_normalize(SIMD3<Float>(-0.6, 0.4, -0.5)) * 0.1
        var tinyTriangle: [UInt32] = []
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let tri = Array(mesh.indices[t ..< t + 3])
            let c = tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] } / 3
            if tinyTriangle.isEmpty && simd_distance(c, tiny) < 0.004 { tinyTriangle = tri; return [] }
            return c.y < -0.06 || simd_distance(c, center) < 0.015 ? [] : tri
        }
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        let loops = MeshMeasure.boundaryLoops(mesh)
        let hole = try #require(loops.filter { $0.count > 3 }.min { $0.count < $1.count })
        let three = try #require(loops.first { $0.count == 3 })
        #expect(loops.count == 3 && hole.count >= 12)
        // The three-vertex hole collapses onto one point (exactly), the larger one has two runs of three nearly
        // coincident vertices (1e-9 m apart).
        let point = mesh.positions[Int(three[0])]
        for v in three { mesh.positions[Int(v)] = point }
        for start in [0, hole.count / 2] {
            let anchor = mesh.positions[Int(hole[start + 1])]
            for k in 0 ..< 3 { mesh.positions[Int(hole[start + k])] = anchor + SIMD3(Float(k) * 1e-9, 0, 0) }
        }
        let before = mesh.vertexCount
        var filled: UInt32 = 0
        let seconds = try measure { try mesh.withCoreMesh { try check(fr_mesh_fill_holes($0, 200, &filled)) } }
        print("collapsed hole borders: filled \(filled), \(mesh.vertexCount - before) new vertices in \(seconds) s")
        #expect(filled == 1)  // the zero-length loop bounds nothing
        #expect(mesh.vertexCount - before < 20 * hole.count)
        #expect(mesh.positions.allSatisfy { $0.x.isFinite && $0.y.isFinite && $0.z.isFinite })
        #expect(seconds < 5)
    }

    /// Pieces attached to the surface only at a vertex are separate components (connectivity through edges).
    @Test func flapsTouchingAtAVertexAreSmallComponents() throws {
        var (mesh, flaps) = Self.sphereWithFlaps()
        let before = mesh.triangleCount
        var removed: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_remove_small_components($0, 0.05, 0, &removed)) }
        #expect(removed == UInt32(flaps))
        #expect(mesh.triangleCount == before - flaps)
        #expect(mesh.positions.allSatisfy { simd_length($0) < 0.1001 })
        // The cleaned mesh fills to a surface with only the neck open.
        var filled: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_fill_holes($0, 200, &filled)) }
        #expect(filled == 1 && MeshMeasure.boundaryLoops(mesh).count == 1)
        #expect(MeshMeasure.topology(mesh).consistent)
    }

    @Test func decimationReachesTheTarget() throws {
        var mesh = MeshFactory.icosphere(level: 5)  // 20480 triangles
        mesh.colors = mesh.positions.map { p in SIMD3(UInt8(128 + 1000 * p.x), 90, 60) }
        try mesh.withCoreMesh {
            try check(fr_mesh_compute_normals($0))
            try check(fr_mesh_decimate($0, 5000, 0))
        }
        #expect(mesh.triangleCount <= 5000 && mesh.triangleCount >= 4500)
        #expect(mesh.colors.count == mesh.vertexCount && mesh.normals.count == mesh.vertexCount)
        #expect(mesh.positions.allSatisfy { abs(simd_length($0) - 0.1) < 0.0005 })
        for (p, c) in zip(mesh.positions, mesh.colors) { #expect(c == SIMD3(UInt8(128 + 1000 * p.x), 90, 60)) }
        let topology = MeshMeasure.topology(mesh)
        #expect(topology.boundaryEdges == 0 && topology.consistent)

        // An error limit alone stops early.
        var limited = MeshFactory.icosphere(level: 5)
        try limited.withCoreMesh { try check(fr_mesh_decimate($0, 0, 0.00002)) }
        #expect(limited.triangleCount < 20480 && limited.triangleCount > 1000)
    }

    @Test func decimationKeepsBorders() throws {
        var mesh = MeshFactory.icosphere(level: 5)
        try mesh.withCoreMesh { try check(fr_mesh_crop_plane($0, [0, 0, 0], [0, 0, 1])) }
        let border = Set(MeshMeasure.boundaryLoops(mesh).flatMap { $0 }.map { mesh.positions[Int($0)] })
        try mesh.withCoreMesh { try check(fr_mesh_decimate($0, 2000, 0)) }
        let after = Set(MeshMeasure.boundaryLoops(mesh).flatMap { $0 }.map { mesh.positions[Int($0)] })
        #expect(after == border)
    }

    @Test func cropPlaneLeavesAPlanarCut() throws {
        var mesh = MeshFactory.icosphere(level: 4)
        mesh.uvs = mesh.positions.map { SIMD2($0.x * 5 + 0.5, $0.y * 5 + 0.5) }
        let point: [Float] = [0, -0.03, 0], normal: [Float] = [0, 2, 0]  // keeps y >= -0.03
        try mesh.withCoreMesh {
            try check(fr_mesh_compute_normals($0))
            try check(fr_mesh_crop_plane($0, point, normal))
        }
        #expect(mesh.positions.allSatisfy { $0.y >= -0.03 - 1e-7 })
        let loops = MeshMeasure.boundaryLoops(mesh)
        #expect(loops.count == 1)
        for v in loops.flatMap({ $0 }) { #expect(abs(mesh.positions[Int(v)].y + 0.03) < 1e-6) }
        #expect(MeshMeasure.topology(mesh).consistent)
        // Interpolated attributes on the cut stay consistent with the positions.
        for (p, uv) in zip(mesh.positions, mesh.uvs) {
            #expect(abs(uv.x - (p.x * 5 + 0.5)) < 1e-4 && abs(uv.y - (p.y * 5 + 0.5)) < 1e-4)
        }
        #expect(mesh.normals.allSatisfy { abs(simd_length($0) - 1) < 1e-4 })
        // Area of the kept cap ~ 2 pi r h for the sphere cap above the plane.
        let expected = 2 * Double.pi * 0.1 * 0.13
        #expect(abs(MeshMeasure.area(mesh) - expected) < 0.01 * expected)
    }

    @Test func transformMovesNormalsAndKeepsOrientation() throws {
        var mesh = MeshFactory.icosphere(level: 2)
        try mesh.withCoreMesh { try check(fr_mesh_compute_normals($0)) }
        let rotation = simd_quatf(angle: 0.7, axis: simd_normalize(SIMD3(1, 2, 3)))
        var m = simd_float4x4(rotation)
        m.columns.3 = [0.01, -0.02, 0.5, 1]
        var moved = mesh
        try moved.withCoreMesh { try check(fr_mesh_transform($0, m.columnMajor)) }
        for i in mesh.positions.indices {
            let expected = (m * SIMD4(mesh.positions[i], 1))
            #expect(simd_distance(moved.positions[i], SIMD3(expected.x, expected.y, expected.z)) < 1e-6)
            #expect(simd_distance(moved.normals[i], rotation.act(mesh.normals[i])) < 1e-5)
        }
        // A mirror keeps the surface outward-oriented by reversing the winding.
        var mirrored = mesh
        let mirror = simd_float4x4(diagonal: [-1, 1, 1, 1])
        try mirrored.withCoreMesh { try check(fr_mesh_transform($0, mirror.columnMajor)) }
        #expect(MeshMeasure.signedVolume(mirrored) > 0)
        #expect(mirrored.normals.indices.allSatisfy { simd_dot(mirrored.normals[$0], simd_normalize(mirrored.positions[$0])) > 0.99 })

        var lo = [Float](repeating: 0, count: 3), hi = [Float](repeating: 0, count: 3)
        try moved.withCoreMesh { try check(fr_mesh_bounds($0, &lo, &hi)) }
        let expectedLo = moved.positions.reduce(SIMD3<Float>(repeating: .infinity)) { simd_min($0, $1) }
        let expectedHi = moved.positions.reduce(SIMD3<Float>(repeating: -.infinity)) { simd_max($0, $1) }
        #expect(SIMD3(lo[0], lo[1], lo[2]) == expectedLo && SIMD3(hi[0], hi[1], hi[2]) == expectedHi)
        #expect(lo[2] > 0.39 && hi[2] < 0.61)
    }

    /// After UV unwrapping, seams split vertices: components, holes and smoothing must still see one surface.
    @Test func operationsSeeThroughUVSeams() throws {
        var mesh = MeshFactory.icosphere(level: 4)
        // A neck cut and one small hole.
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let c = (mesh.positions[Int(mesh.indices[t])] + mesh.positions[Int(mesh.indices[t + 1])] + mesh.positions[Int(mesh.indices[t + 2])]) / 3
            return c.y < -0.07 || simd_distance(c, [0.1, 0, 0]) < 0.012 ? [] : Array(mesh.indices[t ..< t + 3])
        }
        var uv = fr_uv_params()
        fr_uv_params_default(&uv)
        uv.texture_size = 512
        try mesh.withCoreMesh {
            try check(fr_mesh_clean($0, 0))
            try check(fr_mesh_compute_normals($0))
            try check(fr_mesh_uv_unwrap($0, &uv, nil, nil))
        }
        let triangles = mesh.triangleCount
        let seamLoops = MeshMeasure.boundaryLoops(mesh).count
        #expect(seamLoops > 2)  // chart borders look like boundaries of the split mesh

        var removed: UInt32 = 99
        try mesh.withCoreMesh { try check(fr_mesh_remove_small_components($0, 0.5, 0, &removed)) }
        #expect(removed == 0 && mesh.triangleCount == triangles)

        var filled: UInt32 = 0
        try mesh.withCoreMesh { try check(fr_mesh_fill_holes($0, 200, &filled)) }
        #expect(filled == 1)
        #expect(mesh.uvs.count == mesh.vertexCount)

        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        params.bilateral_sigma_spatial_m = 0
        let before = mesh.positions
        try mesh.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
        // Vertices that coincided before smoothing still coincide (no cracks along the seams).
        var first: [SIMD3<Float>: Int] = [:]
        var shared = 0
        for (i, p) in before.enumerated() {
            if let j = first[p] {
                shared += 1
                #expect(mesh.positions[i] == mesh.positions[j])
            } else {
                first[p] = i
            }
        }
        #expect(shared > 0)
    }

    @Test func invalidArgumentsAreRejected() throws {
        #expect(fr_mesh_clean(nil, 0) == FR_ERR_INVALID_ARGUMENT)
        #expect(fr_mesh_smooth(nil, nil) == FR_ERR_INVALID_ARGUMENT)
        var mesh = ReconMesh(positions: [[0, 0, 0], [1, 0, 0], [0, 1, 0]], indices: [0, 1, 7])
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        #expect(fr_mesh_clean(core, 0) == FR_ERR_INVALID_ARGUMENT)
        #expect(String(cString: fr_last_error()).contains("out of range"))
        #expect(fr_mesh_crop_plane(core, [0, 0, 0], [0, 0, 0]) == FR_ERR_INVALID_ARGUMENT)
        mesh.indices = [0, 1, 2]
        let ok = try mesh.makeCoreMesh()
        defer { fr_mesh_free(ok) }
        var bad = fr_smooth_params()
        fr_smooth_params_default(&bad)
        bad.taubin_lambda = 2
        #expect(fr_mesh_smooth(ok, &bad) == FR_ERR_INVALID_ARGUMENT)
        #expect(fr_mesh_decimate(ok, 0, 0) == FR_ERR_INVALID_ARGUMENT)
        let singular = [Float](repeating: 0, count: 16)
        #expect(fr_mesh_transform(ok, singular) == FR_ERR_INVALID_ARGUMENT)
    }
}

extension simd_float4x4 {
    /// Column-major floats for the C API.
    var columnMajor: [Float] {
        [columns.0, columns.1, columns.2, columns.3].flatMap { [$0.x, $0.y, $0.z, $0.w] }
    }
}
