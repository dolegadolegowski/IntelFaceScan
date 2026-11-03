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
