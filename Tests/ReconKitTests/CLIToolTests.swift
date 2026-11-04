// License: Apache 2.0. See LICENSE file in root directory.
//
// ifscan commands run in-process: synth -> info -> eval, error handling and exit codes.

import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

/// Captured output of one ifscan run.
private final class Captured: @unchecked Sendable {
    private let lock = NSLock()
    private var outLines: [String] = []
    private var errLines: [String] = []
    var out: String { lock.withLock { outLines.joined(separator: "\n") } }
    var err: String { lock.withLock { errLines.joined(separator: "\n") } }
    var output: IFScanOutput {
        IFScanOutput(out: { line in self.lock.withLock { self.outLines.append(line) } },
                     err: { line in self.lock.withLock { self.errLines.append(line) } })
    }
}

private func ifscan(_ arguments: [String]) async -> (status: Int32, out: String, err: String) {
    let captured = Captured()
    let status = await IFScanTool.run(arguments, output: captured.output)
    return (status, captured.out, captured.err)
}

private func temporaryDirectory() throws -> URL {
    let url = FileManager.default.temporaryDirectory.appendingPathComponent("ifscan-tests-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    return url
}

private let smallSynth = ["--width", "320", "--height", "200", "--frames-per-pose", "1", "--poses", "front,left,right",
                          "--mesh-resolution", "32"]

@Test func cliSynthWritesADatasetWithGroundTruth() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let dataset = root.appendingPathComponent("s.ifsdataset"), gt = root.appendingPathComponent("gt.obj")
    let run = await ifscan(["synth", "-o", dataset.path, "--gt-mesh", gt.path, "--seed", "9"] + smallSynth)
    #expect(run.status == 0, "\(run.err)")
    #expect(run.out.contains("3 keyframes 320x200"))

    let capture = try ScanDataset.read(from: dataset)
    #expect(capture.keyframes.map(\.scanPose) == [.front, .left, .right])
    #expect(capture.notes["seed"] == "9")
    let truth = try #require(try SyntheticGroundTruth.load(fromDataset: dataset))
    #expect(truth.frames.count == 3 && truth.scene.seed == 9 && truth.worldFromCamera[0] == matrix_identity_float4x4)
    let mesh = try #require(try truth.loadMesh(fromDataset: dataset))
    #expect(mesh.triangleCount == 12 * 32 * 32)
    // The OBJ is in millimeters; loadMesh converts back to meters.
    let exported = try WavefrontOBJ.read(gt)
    #expect(abs(exported.positions[0].x - 1000 * mesh.positions[0].x) < 0.01)
    let zs = mesh.positions.map(\.z)
    #expect(zs.min()! > 0.5 && zs.max()! < 0.9)

    let info = await ifscan(["info", dataset.path])
    #expect(info.status == 0)
    #expect(info.out.contains("Frames: 3 (front 1, left 1, right 1)"))
    #expect(info.out.contains("Synthetic D455"))
    #expect(info.out.contains("Ground truth: synthetic (seed 9"))
    let json = await ifscan(["info", "--json", dataset.path])
    let decoded = try JSONSerialization.jsonObject(with: Data(json.out.utf8)) as? [String: Any]
    #expect(decoded?["format"] as? String == "ifsdataset")
    let objInfo = await ifscan(["info", gt.path])
    #expect(objInfo.out.contains("\(mesh.vertexCount) vertices, \(mesh.triangleCount) triangles"))
    let pngInfo = await ifscan(["info", dataset.appendingPathComponent("depth/000000.png").path])
    #expect(pngInfo.out.contains("320x200, 16-bit Gray"))
}

@Test func cliEvalMeasuresSurfaceDistances() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    var options = SyntheticScan.Options()
    options.meshResolution = 48
    let truth = SyntheticScene(options: options).groundTruthMesh()
    let gtURL = root.appendingPathComponent("gt.obj")
    try WavefrontOBJ.write(truth, to: gtURL, scale: 1000)

    // The same surface pushed out 0.3 mm along its normals.
    var offset = truth
    offset.positions = zip(truth.positions, truth.normals).map { $0 + 0.0003 * $1 }
    let offsetURL = root.appendingPathComponent("offset.obj")
    try WavefrontOBJ.write(offset, to: offsetURL, scale: 1000)
    let run = await ifscan(["eval", offsetURL.path, "--gt", gtURL.path, "--samples", "50000", "--json"])
    #expect(run.status == 0, "\(run.err)")
    let json = try #require(try JSONSerialization.jsonObject(with: Data(run.out.utf8)) as? [String: Any])
    let forward = try #require(json["mesh_to_ground_truth"] as? [String: Any])
    let mean = try #require(forward["mean_mm"] as? Double)
    #expect(abs(mean - 0.3) < 0.02)
    #expect(forward["samples"] as? Int == 50000)
    #expect(json["ground_truth_to_mesh"] == nil)

    let table = await ifscan(["eval", offsetURL.path, "--gt", gtURL.path, "--samples", "20000", "--symmetric"])
    #expect(table.out.contains("mesh -> ground truth") && table.out.contains("ground truth -> mesh"))

    // A mesh in the export frame (rotated, recentered, millimeters) maps back to the world frame with --report.
    let exportTransform = simd_float4x4(simd_quatf(angle: .pi, axis: [1, 0, 0])) * simd_float4x4(columns: (
        [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0.03, -0.68, 1]))
    var exported = truth
    exported.positions = truth.positions.map { p in
        let q = exportTransform * SIMD4(p, 1)
        return SIMD3(q.x, q.y, q.z)
    }
    let exportedURL = root.appendingPathComponent("export.obj")
    try WavefrontOBJ.write(exported, to: exportedURL, scale: 1000)
    let reportURL = root.appendingPathComponent("report.json")
    let m = exportTransform
    let report: [String: Any] = ["export_transform": [m.columns.0, m.columns.1, m.columns.2, m.columns.3].flatMap { [$0.x, $0.y, $0.z, $0.w] },
                                 "export_scale": 1000]
    try JSONSerialization.data(withJSONObject: report).write(to: reportURL)
    let unaligned = await ifscan(["eval", exportedURL.path, "--gt", gtURL.path, "--samples", "5000", "--json"])
    let aligned = await ifscan(["eval", exportedURL.path, "--gt", gtURL.path, "--samples", "5000", "--json", "--report", reportURL.path])
    func meanOf(_ text: String) throws -> Double {
        let object = try #require(try JSONSerialization.jsonObject(with: Data(text.utf8)) as? [String: Any])
        return try #require((object["mesh_to_ground_truth"] as? [String: Any])?["mean_mm"] as? Double)
    }
    #expect(try meanOf(unaligned.out) > 50)
    #expect(try meanOf(aligned.out) < 0.01)
}

@Test func cliReportsErrorsWithExitCodes() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let usage = await ifscan(["synth"])
    #expect(usage.status == 64 && usage.err.contains("missing -o"))
    let help = await ifscan(["help", "eval"])
    #expect(help.status == 0 && help.out.contains("--samples"))

    let missing = await ifscan(["reconstruct", root.appendingPathComponent("none.ifsdataset").path, "-o", "h.obj"])
    #expect(missing.status == 66)
    let notDataset = await ifscan(["reconstruct", root.path, "-o", "h.obj"])
    #expect(notDataset.status == 65 && notDataset.err.contains("manifest.json"))
    let text = root.appendingPathComponent("notes.txt")
    try "x".write(to: text, atomically: true, encoding: .utf8)
    let wrongType = await ifscan(["reconstruct", text.path, "-o", "h.obj"])
    #expect(wrongType.status == 65)
    let recording = root.appendingPathComponent("scan.db3")
    try Data().write(to: recording)
    let noHelper = await ifscan(["reconstruct", recording.path, "-o", "h.obj", "--helper", root.appendingPathComponent("nohelper").path])
    #expect(noHelper.status == 66 && noHelper.err.contains("not executable"))

    let broken = root.appendingPathComponent("broken.obj")
    try "v 0 0 0\nv 1 0 0\nf 1 2 3\n".write(to: broken, atomically: true, encoding: .utf8)
    let badObj = await ifscan(["eval", broken.path, "--gt", broken.path])
    #expect(badObj.status == 65 && badObj.err.contains("broken.obj:3"))
    let unknown = await ifscan(["info", text.path])
    #expect(unknown.status == 65)
}

@Test func cliReconstructRefusesUnwritableOutputsBeforeWorking() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let dataset = root.appendingPathComponent("s.ifsdataset")
    #expect(await ifscan(["synth", "-o", dataset.path] + smallSynth).status == 0)
    let file = root.appendingPathComponent("file")
    try "x".write(to: file, atomically: true, encoding: .utf8)
    // Every case fails before the dataset is even read (no "Read N keyframes" progress line): a reconstruction
    // takes minutes on real data and its result must not be lost at the export.
    func expectRefused(_ arguments: [String], status: Int32, _ message: String) async {
        let run = await ifscan(["reconstruct", dataset.path] + arguments)
        #expect(run.status == status, "\(arguments): \(run.err)")
        #expect(run.err.contains(message), "\(arguments): \(run.err)")
        #expect(!run.err.contains("Read "), "\(arguments): \(run.err)")
    }
    await expectRefused(["-o", file.appendingPathComponent("head.obj").path], status: 73, "output")
    await expectRefused(["-o", root.appendingPathComponent("h.obj").path, "--ply", file.appendingPathComponent("h.ply").path],
                        status: 73, "--ply")
    await expectRefused(["-o", root.appendingPathComponent("h.obj").path, "--report", root.path + "/"], status: 73, "is a folder")
    await expectRefused(["-o", root.appendingPathComponent("h.obj").path, "--save-dataset", root.path], status: 73,
                        "not a dataset or an empty folder")
    // Saving the keyframes over the input dataset would delete its ground truth.
    await expectRefused(["-o", root.appendingPathComponent("h.obj").path, "--save-dataset", dataset.path + "/."], status: 64,
                        "must not be the input dataset")
    #expect(FileManager.default.fileExists(atPath: dataset.appendingPathComponent(SyntheticGroundTruth.fileName).path))

    let missing = await ifscan(["eval", root.appendingPathComponent("none.obj").path, "--gt", file.path])
    #expect(missing.status == 66 && missing.err.contains("none.obj"))
}

/// A dataset that does not start with a frontal keyframe: the reconstruction's world is the camera frame of its first
/// frontal keyframe, the ground truth's that of keyframe 0. The evaluation must bring both into one frame.
@Test func cliGroundTruthEvaluationAlignsTheWorldFrames() async throws {
    var options = SyntheticScan.Options()
    options.width = 160
    options.height = 100
    options.meshResolution = 24
    options.framesPerPose = 1
    options.poses = [.left, .front, .right]
    let scan = SyntheticScan.make(options)
    let truth = SyntheticGroundTruth(result: scan, options: options)
    // A perfect reconstruction as the pipeline returns it: world = camera frame of keyframe 1 (front).
    let worldFromGroundTruth = scan.groundTruthPoses[1].inverse
    var mesh = scan.groundTruthMesh
    mesh.positions = mesh.positions.map { p in
        let q = worldFromGroundTruth * SIMD4(p, 1)
        return SIMD3(q.x, q.y, q.z)
    }
    let exportTransform = simd_float4x4(simd_quatf(angle: .pi, axis: [1, 0, 0]))
    let result = ReconstructionResult(mesh: mesh, texture: nil, cameraPoses: scan.groundTruthPoses.map { worldFromGroundTruth * $0 },
                                      exportTransform: exportTransform, report: ReconstructionReport())
    let evaluation = GroundTruthEvaluation.evaluate(result: result, groundTruth: truth, keyframeIDs: [0, 1, 2],
                                                    mesh: scan.groundTruthMesh, samples: 20_000)
    #expect(evaluation.maxRotationDeg < 0.01 && evaluation.maxHeadMisalignmentMm < 0.01)
    let surface = try #require(evaluation.surfaceToGroundTruth)
    #expect(surface.meanMm < 0.01, "surface mean \(surface.meanMm) mm")
    #expect(evaluation.groundTruthFromWorld?.count == 16)

    // ifscan eval --report maps the exported mesh (export frame, millimeters) into the ground truth's world frame.
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    var exported = mesh
    exported.positions = mesh.positions.map { p in
        let q = exportTransform * SIMD4(p, 1)
        return SIMD3(q.x, q.y, q.z)
    }
    let exportedURL = root.appendingPathComponent("head.obj"), gtURL = root.appendingPathComponent("gt.obj")
    try WavefrontOBJ.write(exported, to: exportedURL, scale: 1000)
    try WavefrontOBJ.write(scan.groundTruthMesh, to: gtURL, scale: 1000)
    let encoder = JSONEncoder()
    encoder.keyEncodingStrategy = .convertToSnakeCase
    let groundTruthJSON = try JSONSerialization.jsonObject(with: encoder.encode(evaluation))
    let m = exportTransform
    let report: [String: Any] = [
        "export_transform": [m.columns.0, m.columns.1, m.columns.2, m.columns.3].flatMap { [$0.x, $0.y, $0.z, $0.w] },
        "export_scale": 1000, "ground_truth": groundTruthJSON]
    let reportURL = root.appendingPathComponent("report.json")
    try JSONSerialization.data(withJSONObject: report).write(to: reportURL)
    let run = await ifscan(["e" + "val", exportedURL.path, "--gt", gtURL.path, "--samples", "20000", "--json", "--report", reportURL.path])
    #expect(run.status == 0, "\(run.err)")
    let json = try #require(try JSONSerialization.jsonObject(with: Data(run.out.utf8)) as? [String: Any])
    let mean = try #require((json["mesh_to_ground_truth"] as? [String: Any])?["mean_mm"] as? Double)
    #expect(mean < 0.01, "mean \(mean) mm")
}

/// `ifscan info x | head`: writing to a pipe whose reader is gone must fail softly (ifscan then exits 0), not raise
/// the NSException of FileHandle.write(_: Data) that aborted the tool.
@Test func cliOutputSurvivesAClosedPipe() throws {
    let pipe = Pipe()
    // EPIPE instead of SIGPIPE for this descriptor only (ifscan ignores SIGPIPE process-wide).
    #expect(fcntl(pipe.fileHandleForWriting.fileDescriptor, F_SETNOSIGPIPE, 1) == 0)
    #expect(IFScanOutput.writeLine("first", to: pipe.fileHandleForWriting))
    try pipe.fileHandleForReading.close()
    #expect(!IFScanOutput.writeLine("after the reader left", to: pipe.fileHandleForWriting))
}

/// The animated synthetic head loops forever; the guided scan must not keep `reconstruct synthetic-camera` running
/// when it cannot capture every pose (here: 5 s cover at most the first poses). Without the limit the command never
/// ends, which the time limit trait catches. No tighter wall-clock bound: the tool runs in-process next to the other
/// ReconKit tests, whose synchronous reconstructions can occupy every thread of Swift's cooperative pool on a
/// small machine; on the GitHub runner the scan with its 5 s watchdog then took 184 s (CI run 41765277127). The
/// limit includes the wait for a heavy test slot (HeavyTestSlots.swift).
@Test(.timeLimit(.minutes(30)), .heavyReconstruction) func cliSyntheticCameraScanStopsAtTheTimeLimit() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let run = await ifscan(["reconstruct", "synthetic-camera", "-o", root.appendingPathComponent("head.obj").path,
                            "--time-limit", "5", "--triangles", "20000", "--texture-size", "512"])
    #expect(run.err.contains("within the time limit of 5 s"), "\(run.err)")
    #expect(run.status == 0 || run.status == 65, "\(run.err)")   // reconstructed what it had, or too few keyframes
}

@Test(.enabled(if: !CoreAvailability.reconstruction, "the reconstruction core is integrated"))
func cliNamesPlaceholderStagesWhenReconstructing() async throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let dataset = root.appendingPathComponent("s.ifsdataset")
    #expect(await ifscan(["synth", "-o", dataset.path] + smallSynth).status == 0)
    let run = await ifscan(["reconstruct", dataset.path, "-o", root.appendingPathComponent("head.obj").path, "-q"])
    #expect(run.status == 69)
    #expect(run.err.contains("placeholder"))
    #expect(run.err.contains("fr_register_frames") || run.err.contains("fr_tsdf_create"))
}

@Test func cliObjReaderHandlesTheCommonDialects() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let url = root.appendingPathComponent("dialects.obj")
    let text = """
        # comment\r
        mtllib head.mtl\r
        o head
        v 0 0 0 1 0 0
        v 1 0 0 0 1 0
        v 1 1 0 0 0 1
        v 0 1 0 1 1 1
        vt 0 0
        vn 0 0 1
        usemtl skin
        f 1/1/1 2/1/1 3/1/1 4/1/1
        f -4//1 -2//1 -1//1
        \tf 1 3 4
        """
    try text.write(to: url, atomically: true, encoding: .utf8)
    let mesh = try WavefrontOBJ.read(url)
    #expect(mesh.vertexCount == 4)
    #expect(mesh.indices == [0, 1, 2, 0, 2, 3, 0, 2, 3, 0, 2, 3])
    #expect(mesh.colors == [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 255]])
    let stats = try WavefrontOBJ.statistics(url)
    #expect(stats.polygons == 3 && stats.normals == 1 && stats.textureCoordinates == 1 && stats.materialLibraries == ["head.mtl"])

    try "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 0\n".write(to: url, atomically: true, encoding: .utf8)
    #expect(throws: WavefrontOBJError.parse(file: "dialects.obj", line: 4, message: "face index 0 out of range (3 vertices so far)")) {
        try WavefrontOBJ.read(url)
    }
    try "v 0 0\n".write(to: url, atomically: true, encoding: .utf8)
    #expect(throws: WavefrontOBJError.self) { try WavefrontOBJ.read(url) }

    // write -> read keeps geometry to 0.1 um and colors exactly
    let original = ReconMesh(positions: [[0.1, -0.2, 0.65], [0.1234567, 0, 0.6], [0, 0.0001, 0.7]],
                             normals: [[0, 0, -1], [0, 0, -1], [0, 0, -1]], colors: [[1, 2, 3], [250, 128, 0], [77, 77, 77]],
                             indices: [0, 1, 2])
    try WavefrontOBJ.write(original, to: url, scale: 1000)
    let back = try WavefrontOBJ.read(url)
    for (a, b) in zip(original.positions, back.positions) { #expect(simd_length(a * 1000 - b) < 1e-3) }
    #expect(back.colors == original.colors && back.indices == original.indices)
}
