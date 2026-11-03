// License: Apache 2.0. See LICENSE file in root directory.

import Foundation
import ReconKit
import simd
import Testing

@Suite("MeshProcessing")
struct MeshProcessingTests {
    /// Noisy sphere with a neck-like cut, three small holes, a floating fragment and duplicated vertices.
    static func damagedSphere() -> ReconMesh {
        var mesh = TestMeshes.icosphere(level: 5)
        var state: UInt64 = 42
        mesh.positions = mesh.positions.map { $0 * (1 + 0.003 * TestMeshes.gaussian(&state)) }
        let holes: [SIMD3<Float>] = [[0.1, 0.02, 0], [-0.05, 0.06, 0.06], [0, 0.03, -0.1]]
        var indices: [UInt32] = []
        for t in 0 ..< mesh.triangleCount {
            let tri = Array(mesh.indices[3 * t ..< 3 * t + 3])
            let c = tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] } / 3
            if c.y < -0.07 || holes.contains(where: { simd_distance(c, simd_normalize($0) * 0.1) < 0.008 }) { continue }
            indices += tri
        }
        // Unshare the first 300 triangles' vertices (as a mesher without vertex sharing would emit them).
        for t in 0 ..< 300 {
            for k in 0 ..< 3 {
                mesh.positions.append(mesh.positions[Int(indices[3 * t + k])])
                indices[3 * t + k] = UInt32(mesh.positions.count - 1)
            }
        }
        mesh.indices = indices
        let fragment = TestMeshes.icosphere(level: 1, radius: 0.005, center: [0.2, 0, 0])
        let offset = UInt32(mesh.positions.count)
        mesh.positions += fragment.positions
        mesh.indices += fragment.indices.map { $0 + offset }
        return mesh
    }

    @Test func cleanRepairsTheMesh() throws {
        var mesh = Self.damagedSphere()
        let noise = mesh.positions.prefix(1000).map { abs(simd_length($0) - 0.1) }.reduce(0, +) / 1000
        var options = ReconstructionOptions()
        options.maxHoleEdges = 100
        try MeshProcessing.clean(&mesh, options: options)
        #expect(mesh.positions.allSatisfy { simd_length($0) < 0.15 })  // fragment removed
        #expect(TestMeshes.boundaryLoopCount(mesh) == 1)                // holes filled, neck open
        #expect(mesh.normals.count == mesh.vertexCount)
        let residual = mesh.positions.prefix(1000).map { abs(simd_length($0) - 0.1) }.reduce(0, +) / 1000
        print("MeshProcessing.clean: mean radial error \(noise * 1000) mm -> \(residual * 1000) mm")
        #expect(residual < 0.6 * noise)
    }

    @Test func cleanDecimatesToTheTarget() throws {
        var mesh = Self.damagedSphere()
        var options = ReconstructionOptions()
        options.targetTriangles = 4000
        options.smoothingIterations = 0
        options.fillHoles = false
        try MeshProcessing.clean(&mesh, options: options)
        #expect(mesh.triangleCount <= 4000 && mesh.triangleCount > 3500)
        #expect(TestMeshes.boundaryLoopCount(mesh) == 4)
    }

    @Test func cleanRejectsInconsistentMeshes() throws {
        var mesh = TestMeshes.icosphere(level: 1)
        mesh.colors = [[1, 2, 3]]
        #expect(throws: ReconstructionError.self) { try MeshProcessing.clean(&mesh, options: ReconstructionOptions()) }
        var empty = ReconMesh()
        try MeshProcessing.clean(&empty, options: ReconstructionOptions())
        #expect(empty.vertexCount == 0)
    }
}
