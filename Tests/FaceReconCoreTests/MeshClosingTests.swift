// License: Apache 2.0. See LICENSE file in root directory.
//
// fr_mesh_fill_holes_ex: closing the longest border into a watertight surface, the perimeter limit for holes, folds of
// filling patches, and the non-manifold edges that welding leaves (fr_mesh_clean).

import FaceReconCore
import Foundation
import ReconKit
import simd
import Testing

@Suite("Mesh closing")
struct MeshClosingTests {
    /// Sphere without its triangles whose centroid lies above y = cut or within a distance of the hole centers.
    static func openSphere(level: Int, radius: Float = 0.1, cut: Float, holes: [(SIMD3<Float>, Float)] = []) throws -> ReconMesh {
        var mesh = MeshFactory.icosphere(level: level, radius: radius)
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let tri = Array(mesh.indices[t ..< t + 3])
            let c = tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] } / 3
            if c.y > cut || holes.contains(where: { simd_distance(c, simd_normalize($0.0) * radius) < $0.1 }) { return [] }
            return tri
        }
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        return mesh
    }

    static func fill(_ mesh: inout ReconMesh, close: Bool, configure: (inout fr_fill_params) -> Void = { _ in }) throws -> fr_fill_report {
        var params = fr_fill_params()
        fr_fill_params_default(&params)
        params.close_largest = close ? 1 : 0
        configure(&params)
        var report = fr_fill_report()
        try mesh.withCoreMesh { try check(fr_mesh_fill_holes_ex($0, &params, &report)) }
        return report
    }

    /// A sphere opened down to 12 degrees below its equator (the longest border by far) plus two small holes closes
    /// into a watertight, outward surface. A head is star-shaped around its center, and so is a sphere: the closing,
    /// faired in spherical coordinates around the fitted center, puts every new vertex back on the sphere.
    @Test func closingTheLongestBorderMakesTheSurfaceWatertight() throws {
        var mesh = try Self.openSphere(level: 5, cut: -0.02, holes: [(SIMD3(1, -0.5, 0.2), 0.012), (SIMD3(-0.3, -0.8, -0.4), 0.008)])
        #expect(MeshMeasure.boundaryLoops(mesh).count == 3)
        let before = mesh.vertexCount
        let report = try Self.fill(&mesh, close: true)
        #expect(report.closed_largest == 1 && report.filled_holes == 2 && report.open_loops == 0)
        // The border cut along triangle centroids has teeth (triangles with two border edges): trimmed first.
        #expect(report.trimmed_triangles > 0 && report.trimmed_triangles < 300)
        let topology = MeshMeasure.topology(mesh)
        #expect(topology.boundaryEdges == 0 && topology.consistent, "watertight and consistently oriented")
        let volume = MeshMeasure.signedVolume(mesh), sphere = 4.0 / 3 * Double.pi * 0.001
        print("closing: \(mesh.vertexCount - Int(report.first_new_vertex)) new vertices, volume \(volume / sphere) of the sphere, "
            + "filled \(report.filled_area_m2 * 1e4) cm2 (closing \(report.closing_area_m2 * 1e4) cm2)")
        #expect(abs(volume - sphere) < 0.01 * sphere, "outward and round: \(volume / sphere) of the sphere")
        #expect(MeshMeasure.foldedTriangles(mesh) == 0)
        #expect(report.folded_triangles == 0)
        // New vertices are reported (appended after the kept ones; the trimming dropped a few): on the sphere.
        let firstNew = Int(report.first_new_vertex)
        #expect(firstNew < before && firstNew > before - 300)
        let deviation = mesh.positions[firstNew...].map { abs(simd_length($0) - 0.1) }.max() ?? 1
        #expect(deviation < 0.0005, "closing within \(deviation * 1000) mm of the sphere")
        // The closing area is the opened cap's (2 pi r h with h = 0.12).
        #expect(abs(Double(report.closing_area_m2) - 2 * Double.pi * 0.1 * 0.12) < 0.02 * 2 * Double.pi * 0.1 * 0.12)
    }

    /// A head-like cut: the sphere opened at the back (z > 0.03) and cropped by a plane (y < -0.06, exactly planar
    /// vertices as fr_mesh_crop_plane leaves them). The closing stays above the plane and caps it flat.
    @Test func closingCapsAPlanarCutFlat() throws {
        var mesh = try Self.openSphere(level: 5, cut: 2)  // the whole sphere
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let tri = Array(mesh.indices[t ..< t + 3])
            let c = tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] } / 3
            return c.z > 0.03 ? [] : tri
        }
        try mesh.withCoreMesh { m in
            try check(fr_mesh_crop_plane(m, [0, -0.06, 0], [0, 1, 0]))
            try check(fr_mesh_clean(m, 0))
        }
        #expect(MeshMeasure.boundaryLoops(mesh).count == 1)
        let report = try Self.fill(&mesh, close: true)
        #expect(report.closed_largest == 1)
        let topology = MeshMeasure.topology(mesh)
        #expect(topology.boundaryEdges == 0 && topology.consistent)
        #expect(MeshMeasure.signedVolume(mesh) > 0)
        #expect(MeshMeasure.foldedTriangles(mesh) == 0)
        let lowest = mesh.positions.map(\.y).min() ?? -1
        #expect(lowest > -0.06 - 1e-5, "nothing below the cut: lowest \(lowest)")
        let added = mesh.positions[Int(report.first_new_vertex)...]
        let cap = added.filter { abs($0.y + 0.06) < 1e-5 }
        // Away from the cap the closing stays on the sphere; toward it, it bends down onto the plane.
        let high = added.filter { $0.y > -0.02 }.map { abs(simd_length($0) - 0.1) }.max() ?? 1
        let low = added.filter { $0.y > -0.055 }.map { abs(simd_length($0) - 0.1) }.max() ?? 1
        print("planar cut: \(cap.count) of \(added.count) new vertices on the plane; off the sphere \(high * 1000) mm "
            + "above y = -0.02, \(low * 1000) mm above y = -0.055")
        #expect(cap.count > 50, "a flat cap on the cut")
        #expect(high < 0.002 && low < 0.005)
    }

    /// Without close_largest the longest border stays open, as fr_mesh_fill_holes always did.
    @Test func withoutClosingTheLongestBorderStaysOpen() throws {
        var mesh = try Self.openSphere(level: 4, cut: 0.02, holes: [(SIMD3(1, -0.5, 0.2), 0.012)])
        let report = try Self.fill(&mesh, close: false)
        #expect(report.closed_largest == 0 && report.filled_holes == 1 && report.open_loops == 1)
        #expect(MeshMeasure.boundaryLoops(mesh).count == 1)
    }

    /// The openings behind the ears of a 0.75 mm scan are 160-220 mm around with 420-480 border edges: above the old
    /// 400-edge limit. The limit is the perimeter now (250 mm by default), with an edge cap for safety.
    @Test func holesAreLimitedByTheirPerimeter() throws {
        // Level 7 on a 50 mm sphere: ~0.47 mm edges, so an ear-sized hole has a ragged border of more than 400 edges.
        let make = { try Self.openSphere(level: 7, radius: 0.05, cut: 0.01,
                                        holes: [(SIMD3(1, -0.8, 0.1), 0.026), (SIMD3(-0.5, -0.8, -0.6), 0.015)]) }
        var mesh = try make()
        func perimeter(_ loop: [UInt32]) -> Float {
            zip(loop, loop.dropFirst() + loop.prefix(1)).map { simd_distance(mesh.positions[Int($0)], mesh.positions[Int($1)]) }.reduce(0, +)
        }
        let holes = MeshMeasure.boundaryLoops(mesh).sorted { perimeter($0) < perimeter($1) }.dropLast()  // the cap is longest
        #expect(holes.count == 2)
        let small = try #require(holes.first), ear = try #require(holes.last)
        print("holes: \(ear.count) edges / \(perimeter(ear) * 1000) mm and \(small.count) edges / \(perimeter(small) * 1000) mm")
        #expect(ear.count > 400 && perimeter(ear) > 0.15 && perimeter(ear) < 0.25)
        var old = mesh
        var filled: UInt32 = 0
        try old.withCoreMesh { try check(fr_mesh_fill_holes($0, 400, &filled)) }
        #expect(filled == 1, "the edge limit of 400 left the ear-sized hole open")
        let report = try Self.fill(&mesh, close: false)
        #expect(report.filled_holes == 2 && report.open_loops == 1, "both holes are filled, the cap stays open")
        // A perimeter limit between the two holes fills only the smaller one; the edge cap still applies.
        var limited = try make()
        let between = (perimeter(ear) + perimeter(small)) / 2
        let byPerimeter = try Self.fill(&limited, close: false) { $0.max_hole_perimeter_m = between }
        #expect(byPerimeter.filled_holes == 1 && byPerimeter.open_loops == 2)
        var capped = try make()
        let byEdges = try Self.fill(&capped, close: false) { $0.max_hole_edges = UInt32(ear.count - 1) }
        #expect(byEdges.filled_holes == 1)
    }

    /// Filling patches are checked against the surrounding normals: on a noisy sphere with many small holes the
    /// filling adds no folded triangles (before the check, a synthetic head scan went from 11 folded triangles after
    /// smoothing to about 68 after filling its 580 holes).
    @Test func fillingAddsNoFoldedTriangles() throws {
        var rng = TestRandom(seed: 5)
        var centers: [(SIMD3<Float>, Float)] = []
        for _ in 0 ..< 60 {
            let d = SIMD3<Float>(rng.gaussian(), rng.gaussian(), rng.gaussian())
            if d.y < 0.5 { centers.append((simd_normalize(d), 0.002 + 0.006 * rng.uniform())) }
        }
        var mesh = try Self.openSphere(level: 6, cut: 0.07, holes: centers)
        try MeshFactory.addNormalNoise(&mesh, sigma: 0.0004, seed: 9)
        let foldsBefore = MeshMeasure.foldedTriangles(mesh)
        let holes = MeshMeasure.boundaryLoops(mesh).count - 1
        let report = try Self.fill(&mesh, close: false)
        print("noisy sphere: \(holes) holes, filled \(report.filled_holes), rejected \(report.rejected_holes), "
            + "folded \(foldsBefore) -> \(MeshMeasure.foldedTriangles(mesh))")
        #expect(report.filled_holes >= UInt32(holes * 8 / 10))
        #expect(MeshMeasure.foldedTriangles(mesh) <= foldsBefore)
        #expect(MeshMeasure.topology(mesh).consistent)
    }

    /// Four marching-cubes vertices within a micrometer of a voxel corner merged by the weld left an edge with three
    /// triangles on a head scan. fr_mesh_clean keeps the largest triangle per direction on such edges.
    @Test func weldingLeavesNoNonManifoldEdges() throws {
        // A strip of two triangles on edge (0, 1), and a sliver whose corner 4 lies 0.3 um from vertex 0: welded,
        // it uses the directed edge 0->1 a second time.
        var mesh = ReconMesh(positions: [[0, 0, 0], [0.001, 0, 0], [0.0005, 0.001, 0], [0.0005, -0.001, 0],
                                         [0.0000003, 0, 0], [0.0005, 0.0000001, 0.0000001]],
                             indices: [0, 1, 2, 1, 0, 3, 4, 1, 5])
        #expect(MeshMeasure.topology(mesh).consistent)
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 1e-6)) }
        #expect(mesh.triangleCount == 2, "the sliver is gone")
        let edges = MeshMeasure.directedEdges(mesh)
        #expect(edges.values.allSatisfy { $0 == 1 })
        var undirected: [UInt64: Int] = [:]
        for key in edges.keys {
            let a = key >> 32, b = key & 0xFFFF_FFFF
            undirected[min(a, b) << 32 | max(a, b), default: 0] += 1
        }
        #expect(undirected.values.allSatisfy { $0 <= 2 })
    }

    /// meshoptimizer collapses thin closed pieces into fins: two opposed copies of a triangle, every edge used twice
    /// but enclosing nothing (two at 60k triangles on the closed synthetic head; a viewer's weld turns them into a lone
    /// triangle with three open edges). A flat tetrahedron next to a sphere is such a piece: its apex is the cheapest
    /// collapse. fr_mesh_decimate removes the fins and leaves the rest closed.
    @Test func decimationLeavesNoFins() throws {
        var mesh = MeshFactory.icosphere(level: 3)
        let base = UInt32(mesh.vertexCount)
        mesh.positions += [[0.2, 0, 0], [0.21, 0, 0], [0.2, 0.01, 0], [0.203, 0.003, 0.00001]]
        mesh.indices += [base, base + 2, base + 1, base, base + 1, base + 3, base + 1, base + 2, base + 3, base + 2, base, base + 3]
        #expect(MeshMeasure.topology(mesh).boundaryEdges == 0 && MeshMeasure.topology(mesh).consistent)
        let target = UInt32(mesh.triangleCount - 2)
        try mesh.withCoreMesh { try check(fr_mesh_decimate($0, target, 0)) }
        var copies: [[UInt32]: Int] = [:]
        for t in 0 ..< mesh.triangleCount { copies[Array(mesh.indices[3 * t ..< 3 * t + 3]).sorted(), default: 0] += 1 }
        #expect(copies.values.allSatisfy { $0 == 1 }, "no opposed copies of a triangle")
        let topology = MeshMeasure.topology(mesh)
        #expect(topology.boundaryEdges == 0 && topology.consistent)
        #expect(mesh.positions.allSatisfy { simd_length($0) < 0.15 }, "the flat tetrahedron is gone, the sphere stays")
        #expect(mesh.triangleCount > 1200)
    }

    /// The synthesized flags follow their vertices through withCoreMesh: xatlas renumbers the vertices of a small open
    /// cap (one chart, no seam split, the same vertex count), smoothing moves them in place.
    @Test func synthesizedFlagsFollowTheirVertices() throws {
        var mesh = MeshFactory.icosphere(level: 3)
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let tri = Array(mesh.indices[t ..< t + 3])
            return tri.reduce(SIMD3<Float>.zero) { $0 + mesh.positions[Int($1)] }.y / 3 > 0.07 ? tri : []
        }
        try mesh.withCoreMesh { try check(fr_mesh_clean($0, 0)) }
        mesh.synthesized = mesh.positions.map { $0.y > 0.09 }
        let flagged = Set(zip(mesh.positions, mesh.synthesized).filter(\.1).map(\.0))
        let count = mesh.vertexCount
        var uv = fr_uv_params()
        fr_uv_params_default(&uv)
        uv.texture_size = 256
        try mesh.withCoreMesh { m in
            try check(fr_mesh_compute_normals(m))
            try check(fr_mesh_uv_unwrap(m, &uv, nil, nil))
        }
        #expect(mesh.vertexCount == count, "no seam split")
        #expect(mesh.positions.indices.allSatisfy { mesh.synthesized[$0] == flagged.contains(mesh.positions[$0]) })
        let before = mesh.synthesized
        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        try mesh.withCoreMesh { try check(fr_mesh_smooth($0, &params)) }
        #expect(mesh.synthesized == before)
    }

    /// A hole that no patch can close (smoothing contracted it to a point: its border vertices coincide) stays open and
    /// is counted once, also when the closing pass tries it a second time.
    @Test func unfillableHoleIsCountedOnce() throws {
        var mesh = try Self.openSphere(level: 4, cut: 0.05)
        // Remove the fan of the vertex nearest to (0, -0.1, 0) and pull its ring onto it.
        let center = mesh.positions.indices.min { simd_distance(mesh.positions[$0], [0, -0.1, 0]) < simd_distance(mesh.positions[$1], [0, -0.1, 0]) }!
        var ring = Set<UInt32>()
        var kept: [UInt32] = []
        for t in 0 ..< mesh.triangleCount {
            let tri = Array(mesh.indices[3 * t ..< 3 * t + 3])
            if tri.contains(UInt32(center)) { ring.formUnion(tri) } else { kept += tri }
        }
        mesh.indices = kept
        for v in ring { mesh.positions[Int(v)] = mesh.positions[center] }
        #expect(MeshMeasure.boundaryLoops(mesh).count == 2)
        for close in [false, true] {
            var copy = mesh
            let report = try Self.fill(&copy, close: close)
            let open = close ? 1 : 2  // the hole, and without closing the cap
            #expect(report.open_loops == open && report.rejected_holes == 1, "close \(close): open \(report.open_loops), rejected \(report.rejected_holes)")
            #expect(MeshMeasure.boundaryLoops(copy).count == open)
        }
    }

    @Test func invalidParametersAreRejected() throws {
        var mesh = try Self.openSphere(level: 2, cut: 0.05)
        try mesh.withCoreMesh { m in
            var params = fr_fill_params()
            fr_fill_params_default(&params)
            params.max_hole_perimeter_m = -1
            #expect(fr_mesh_fill_holes_ex(m, &params, nil) == FR_ERR_INVALID_ARGUMENT)
            params.max_hole_perimeter_m = .nan
            #expect(fr_mesh_fill_holes_ex(m, &params, nil) == FR_ERR_INVALID_ARGUMENT)
            #expect(fr_mesh_fill_holes_ex(m, nil, nil) == FR_ERR_INVALID_ARGUMENT)
            #expect(fr_mesh_fill_holes_ex(nil, &params, nil) == FR_ERR_INVALID_ARGUMENT)
        }
    }
}
