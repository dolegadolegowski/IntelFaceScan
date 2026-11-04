// License: Apache 2.0. See LICENSE file in root directory.
//
// ReconstructionOptions.closeHead end to end: the synthetic head scan closes into a watertight, outward bust whose
// measured part is the same as without closing; the synthesized part is flagged, reported and exported.

import FaceReconCore
import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

@Suite("Mesh closing (pipeline)", .enabled(if: CoreAvailability.reconstruction, "fr_register_frames / fr_tsdf_create are placeholders"),
       .timeLimit(.minutes(30)), .heavyReconstruction)
struct MeshClosingPipelineTests {
    static func run(closeHead: Bool, scan: SyntheticScanResult) async throws -> ReconstructionResult {
        var options = ReconstructionOptions()
        options.closeHead = closeHead
        options.targetTriangles = 60_000
        options.textureSize = 512
        return try await ReconstructionPipeline(options: options).run(scan.capture)
    }

    /// Topology of an exported OBJ as a viewer sees it: vertices welded by position (UV seams joined). The weld is
    /// done here, not by fr_mesh_clean, which would drop duplicate triangles and those on non-manifold edges first.
    /// fins: triangles whose three welded vertices another triangle also has (opposed copies enclose nothing).
    static func exportedTopology(_ url: URL) throws -> (boundaryEdges: Int, nonManifoldEdges: Int, fins: Int, volume: Double) {
        var imported: UnsafeMutablePointer<fr_mesh>?
        guard fr_import_obj(url.path, &imported) == FR_OK, let core = imported else { throw CocoaError(.fileReadCorruptFile) }
        defer { fr_mesh_free(core) }
        let mesh = ReconMesh(core: core)
        var welded: [SIMD3<Float>: UInt32] = [:]
        let weld = mesh.positions.map { p in welded[p] ?? { welded[p] = UInt32(welded.count); return UInt32(welded.count - 1) }() }
        var directed: [UInt64: Int] = [:]
        var triangles: [[UInt32]: Int] = [:]
        for t in 0 ..< mesh.triangleCount {
            for k in 0 ..< 3 {
                let a = weld[Int(mesh.indices[3 * t + k])], b = weld[Int(mesh.indices[3 * t + (k + 1) % 3])]
                directed[UInt64(a) << 32 | UInt64(b), default: 0] += 1
            }
            triangles[(0 ..< 3).map { weld[Int(mesh.indices[3 * t + $0])] }.sorted(), default: 0] += 1
        }
        var boundary = 0, nonManifold = 0
        for (key, count) in directed {
            if count > 1 { nonManifold += 1 }
            if directed[(key & 0xFFFF_FFFF) << 32 | key >> 32] == nil { boundary += 1 }
        }
        var volume = 0.0
        for t in 0 ..< mesh.triangleCount {
            let p = (0 ..< 3).map { SIMD3<Double>(mesh.positions[Int(mesh.indices[3 * t + $0])]) }
            volume += simd_dot(p[0], simd_cross(p[1], p[2])) / 6
        }
        return (boundary, nonManifold, triangles.values.filter { $0 > 1 }.reduce(0, +), volume)
    }

    @Test func closedHeadIsWatertightAndKeepsTheMeasuredSurface() async throws {
        let scan = SyntheticScan.make()
        let open = try await Self.run(closeHead: false, scan: scan)
        let closed = try await Self.run(closeHead: true, scan: scan)

        // Synthesized vertices are flagged (through decimation and UV seams) and reported.
        #expect(open.mesh.synthesized.filter { $0 }.count < open.mesh.vertexCount / 50)  // small holes only
        #expect(closed.mesh.synthesized.count == closed.mesh.vertexCount)
        let share = closed.report.synthesizedAreaMM2 / closed.report.surfaceAreaMM2
        print("closeHead: \(closed.report.synthesizedAreaMM2 / 100) of \(closed.report.surfaceAreaMM2 / 100) cm2 synthesized, "
            + "cleanup \(closed.report.durations["cleanup"] ?? 0) s (open \(open.report.durations["cleanup"] ?? 0) s), notes \(closed.report.notes)")
        #expect(share > 0.2 && share < 0.6)
        #expect(closed.report.notes.contains { $0.contains("synthesized") })

        // The exported OBJ is watertight and outward, as viewers weld it.
        let dir = try ScratchDirectory()
        let url = dir.url.appendingPathComponent("head.obj")
        try MeshExport.writeOBJ(closed, to: url)
        let topology = try Self.exportedTopology(url)
        #expect(topology.boundaryEdges == 0 && topology.nonManifoldEdges == 0 && topology.fins == 0, "\(topology)")
        #expect(topology.volume > 0)
        let openURL = dir.url.appendingPathComponent("open.obj")
        try MeshExport.writeOBJ(open, to: openURL)
        #expect(try Self.exportedTopology(openURL).boundaryEdges > 100)

        // The measured surface is the same: closing trims a little fringe off the border and adds nothing to it.
        let measuredOpen = try MeshEvaluation.distance(from: open.mesh.measuredSurface, to: scan.groundTruthMesh)
        let measuredClosed = try MeshEvaluation.distance(from: closed.mesh.measuredSurface, to: scan.groundTruthMesh)
        let whole = try MeshEvaluation.distance(from: closed.mesh, to: scan.groundTruthMesh)
        print("surface to ground truth (mm): measured open mean \(measuredOpen.meanMm) p95 \(measuredOpen.p95Mm), "
            + "measured closed mean \(measuredClosed.meanMm) p95 \(measuredClosed.p95Mm); "
            + "whole closed mesh mean \(whole.meanMm) p95 \(whole.p95Mm) max \(whole.maxMm)")
        #expect(abs(measuredClosed.meanMm - measuredOpen.meanMm) < 0.03)
        #expect(abs(measuredClosed.p95Mm - measuredOpen.p95Mm) < 0.1)
        #expect(measuredClosed.meanMm < 1)
    }
}

