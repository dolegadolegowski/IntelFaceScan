// License: Apache 2.0. See LICENSE file in root directory.
//
// End to end: synthetic scan -> ReconstructionPipeline -> comparison with the ground truth. Skipped automatically
// while the registration or fusion core is a placeholder (CoreAvailability probes); active after integration.
//
// Recordings: set IFS_TEST_RECORDINGS to a folder with .db3/.bag files (outside git, see docs/TESTING.md) and build
// the helper (build/helper/IntelFaceScanHelper or IFS_HELPER) to reconstruct each of them through ifscan.

import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

private func d455Scan(seed: UInt64 = 1) -> SyntheticScanResult {
    var options = SyntheticScan.Options()
    options.width = 1280
    options.height = 800
    options.framesPerPose = 2
    options.seed = seed
    return SyntheticScan.make(options)
}

private func temporaryDirectory() throws -> URL {
    let url = FileManager.default.temporaryDirectory.appendingPathComponent("ifs-e2e-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    return url
}

private final class Lines: @unchecked Sendable {
    private let lock = NSLock()
    private var lines: [String] = []
    func append(_ line: String) { lock.withLock { lines.append(line) } }
    var text: String { lock.withLock { lines.joined(separator: "\n") } }
}

@Suite(.enabled(if: CoreAvailability.reconstruction, "fr_register_frames / fr_tsdf_create are still placeholders"),
       .timeLimit(.minutes(10)))
struct EndToEndSyntheticTests {
    @Test func reconstructionMatchesTheGroundTruth() async throws {
        let scan = d455Scan()
        let result = try await ReconstructionPipeline().run(scan.capture)

        // Registration: every keyframe's pose relative to keyframe 0 (the world frame).
        #expect(result.cameraPoses.count == scan.groundTruthPoses.count)
        let errors = MeshEvaluation.relativePoseErrors(estimated: result.cameraPoses, groundTruth: scan.groundTruthPoses)
        let rotation = errors.map(\.rotationDeg).max() ?? .infinity
        let translation = errors.map(\.translationM).max() ?? .infinity
        print("e2e pose error: max rotation \(rotation) deg, max translation \(translation * 1000) mm")
        #expect(rotation < 1.0)
        #expect(translation < 0.002)

        // Surface: the reconstruction lies on the true surface (one-sided: unseen parts are not penalized).
        #expect(result.mesh.triangleCount > 20_000)
        let distance = try MeshEvaluation.distance(from: result.mesh, to: scan.groundTruthMesh)
        print("e2e surface distance (mm): mean \(distance.meanMm), median \(distance.medianMm), p95 \(distance.p95Mm), max \(distance.maxMm)")
        #expect(distance.meanMm < 1.0)
        #expect(distance.p95Mm < 3.0)

        // Coverage: the face, forehead and both sides are there (head ~15.5 cm wide, ~20 cm above the neck cut).
        let xs = result.mesh.positions.map(\.x), ys = result.mesh.positions.map(\.y)
        #expect(xs.max()! - xs.min()! > 0.13)
        #expect(ys.max()! - ys.min()! > 0.15)

        // Export frame: rigid, head upright (+Y up), face toward +Z.
        let t = result.exportTransform
        let r = simd_float3x3(SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z), SIMD3(t.columns.1.x, t.columns.1.y, t.columns.1.z),
                              SIMD3(t.columns.2.x, t.columns.2.y, t.columns.2.z))
        #expect(abs(simd_determinant(r) - 1) < 1e-3)
        #expect(simd_dot(r * SIMD3(0, -1, 0), SIMD3(0, 1, 0)) > 0.9)   // world up (-Y) -> export +Y
        #expect(simd_dot(r * SIMD3(0, 0, -1), SIMD3(0, 0, 1)) > 0.9)   // the face looks toward the camera (-Z)
    }

    @Test(.enabled(if: CoreAvailability.objExport, "fr_export_obj is still a placeholder"))
    func cliReconstructsASyntheticDatasetAndEvaluatesIt() async throws {
        let root = try temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: root) }
        let dataset = root.appendingPathComponent("head.ifsdataset").path
        let head = root.appendingPathComponent("head.obj").path, report = root.appendingPathComponent("report.json").path
        let err = Lines(), out = Lines()
        let io = IFScanOutput(out: { out.append($0) }, err: { err.append($0) })
        #expect(await IFScanTool.run(["synth", "-o", dataset, "--seed", "3"], output: io) == 0, "\(err.text)")
        let status = await IFScanTool.run(["reconstruct", dataset, "-o", head, "--report", report,
                                           "--ply", root.appendingPathComponent("head.ply").path], output: io)
        #expect(status == 0, "\(err.text)")
        print(out.text)

        let json = try #require(try JSONSerialization.jsonObject(with: Data(contentsOf: URL(fileURLWithPath: report))) as? [String: Any])
        let truth = try #require(json["ground_truth"] as? [String: Any])
        #expect((truth["max_rotation_deg"] as? Double ?? 99) < 1.0)
        #expect((truth["max_translation_mm"] as? Double ?? 99) < 2.0)
        let surface = try #require(truth["surface_to_ground_truth"] as? [String: Any])
        #expect((surface["mean_mm"] as? Double ?? 99) < 1.0)
        #expect(FileManager.default.fileExists(atPath: root.appendingPathComponent("head.ply").path))

        // The exported OBJ (millimeters, export frame) evaluated against the ground truth through the report.
        let eval = await IFScanTool.run(["e" + "val", head, "--gt", dataset + "/ground_truth.obj", "--report", report, "--json"],
                                        output: io)
        #expect(eval == 0, "\(err.text)")
    }
}

private let recordingsFolder: URL? = {
    guard let path = ProcessInfo.processInfo.environment["IFS_TEST_RECORDINGS"], !path.isEmpty else { return nil }
    return URL(fileURLWithPath: path)
}()

private func recordings() -> [URL] {
    guard let folder = recordingsFolder,
          let names = try? FileManager.default.contentsOfDirectory(atPath: folder.path) else { return [] }
    return names.filter { $0.hasSuffix(".db3") || $0.hasSuffix(".bag") }.sorted().map { folder.appendingPathComponent($0) }
}

@Test(.enabled(if: !recordings().isEmpty, "set IFS_TEST_RECORDINGS to a folder of .db3/.bag recordings"),
      .timeLimit(.minutes(30)))
func endToEndRecordingsReconstruct() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    for recording in recordings() {
        let name = recording.deletingPathExtension().lastPathComponent
        let report = root.appendingPathComponent("\(name).json")
        let err = Lines()
        let status = await IFScanTool.run(["reconstruct", recording.path, "-o", root.appendingPathComponent("\(name).obj").path,
                                           "--report", report.path, "-q"],
                                          output: IFScanOutput(out: { print($0) }, err: { err.append($0) }))
        #expect(status == 0, "\(recording.lastPathComponent): \(err.text)")
        guard status == 0 else { continue }
        let json = try JSONSerialization.jsonObject(with: Data(contentsOf: report)) as? [String: Any]
        #expect((json?["keyframes"] as? Int ?? 0) >= 5, "\(recording.lastPathComponent)")
    }
}
