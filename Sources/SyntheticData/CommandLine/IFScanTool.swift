// License: Apache 2.0. See LICENSE file in root directory.
//
// The ifscan commands. `IFScanTool.run(arguments)` is the whole tool (the executable only forwards its arguments),
// so tests run commands in-process. Exit codes follow sysexits(3).

import CameraKit
import FaceReconCore
import Foundation
import ImageIO
import ReconKit
import ScanKit
import simd

/// Where the tool prints: standard output for results, standard error for progress and diagnostics.
public struct IFScanOutput: Sendable {
    public var out: @Sendable (String) -> Void
    public var err: @Sendable (String) -> Void

    public init(out: @escaping @Sendable (String) -> Void, err: @escaping @Sendable (String) -> Void) {
        self.out = out
        self.err = err
    }

    /// When standard output's reader has gone away (`ifscan info x | head`), the tool ends quietly with status 0, as
    /// the comment in main.swift promises: ifscan ignores SIGPIPE (the helper socket must not kill it), and
    /// FileHandle.write(_: Data) raised an uncaught NSException on EPIPE (abort, exit 134, stack trace).
    public static let standard = IFScanOutput(
        out: { line in if !writeLine(line, to: .standardOutput) { exit(0) } },
        err: { line in writeLine(line, to: .standardError) })

    /// Writes one line; false when the write failed (EPIPE: the reader closed the pipe). Never raises.
    @discardableResult
    public static func writeLine(_ line: String, to handle: FileHandle) -> Bool {
        do {
            try handle.write(contentsOf: Data((line + "\n").utf8))
            return true
        } catch {
            return false
        }
    }
}

public enum IFScanTool {
    public enum ExitCode: Int32 {
        case ok = 0
        case usage = 64          // EX_USAGE
        case dataError = 65      // EX_DATAERR: input is not valid
        case noInput = 66        // EX_NOINPUT
        case unavailable = 69    // EX_UNAVAILABLE: a dependency is missing or still a placeholder
        case software = 70       // EX_SOFTWARE
        case cannotCreate = 73   // EX_CANTCREAT
    }

    struct Failure: Error {
        var code: ExitCode
        var message: String
        var notes: [String] = []
    }

    public static func run(_ arguments: [String], output: IFScanOutput = .standard) async -> Int32 {
        let command: IFScanCommand
        do {
            command = try IFScanArguments.parse(arguments)
        } catch {
            output.err("ifscan: \(error.localizedDescription)")
            output.err("Run 'ifscan help' for usage.")
            return ExitCode.usage.rawValue
        }
        do {
            switch command {
            case .help(let topic):
                output.out(topic.flatMap(IFScanArguments.usage) ?? IFScanArguments.overview)
            case .version:
                output.out("ifscan \(IFScanArguments.version)")
            case .synth(let c):
                try synth(c, output)
            case .reconstruct(let c):
                try await reconstruct(c, output)
            case .eval(let c):
                try evaluate(c, output)
            case .info(let c):
                try await info(c, output)
            }
            return ExitCode.ok.rawValue
        } catch let failure as Failure {
            output.err("ifscan: error: \(failure.message)")
            for note in failure.notes { output.err("note: \(note)") }
            return failure.code.rawValue
        } catch let error as ArgumentError {
            output.err("ifscan: \(error.message)")
            return ExitCode.usage.rawValue
        } catch {
            output.err("ifscan: error: \(error.localizedDescription)")
            return ExitCode.software.rawValue
        }
    }

    // MARK: - synth

    static func synth(_ c: SynthCommand, _ io: IFScanOutput) throws {
        var options = SyntheticScan.Options()
        options.width = c.width
        options.height = c.height
        options.framesPerPose = c.framesPerPose
        options.depthNoiseAt1m = c.noiseMM / 1000
        options.seed = c.seed
        options.distanceM = c.distanceM
        options.poses = c.poses
        options.backgroundDistanceM = c.background ? max(1.1, c.distanceM + 0.5) : nil
        options.meshResolution = c.meshResolution
        options.lighting = c.lighting
        let start = Date()
        let result = SyntheticScan.make(options)
        let url = URL(fileURLWithPath: c.output)
        do {
            try ScanDataset.write(result.capture, to: url)
            try SyntheticGroundTruth(result: result, options: options).write(toDataset: url, mesh: result.groundTruthMesh)
            if let path = c.groundTruthMesh {
                try WavefrontOBJ.write(result.groundTruthMesh, to: URL(fileURLWithPath: path), scale: 1000,
                                       comments: SyntheticGroundTruth.meshComments)
            }
        } catch {
            throw Failure(code: .cannotCreate, message: error.localizedDescription)
        }
        io.out("Wrote \(url.path): \(result.capture.keyframes.count) keyframes \(c.width)x\(c.height) "
            + "(\(c.poses.map(\.rawValue).joined(separator: ", ")) x \(c.framesPerPose)), "
            + "noise \(format(c.noiseMM, 2)) mm at 1 m, seed \(c.seed), "
            + "ground truth \(result.groundTruthMesh.triangleCount) triangles, \(format(Date().timeIntervalSince(start), 1)) s")
        if let path = c.groundTruthMesh { io.out("Ground-truth mesh: \(path) (millimeters, world frame)") }
    }

    // MARK: - reconstruct

    enum InputKind: String { case dataset, recording, syntheticCamera }

    /// Input name that selects the animated synthetic head (SyntheticCameraSource) instead of a file.
    static let syntheticCameraInput = "synthetic-camera"

    static func inputKind(_ url: URL) throws -> InputKind {
        if url.lastPathComponent == syntheticCameraInput && !FileManager.default.fileExists(atPath: url.path) {
            return .syntheticCamera
        }
        var isDirectory: ObjCBool = false
        guard FileManager.default.fileExists(atPath: url.path, isDirectory: &isDirectory) else {
            throw Failure(code: .noInput, message: "\(url.path): no such file or folder")
        }
        if isDirectory.boolValue {
            guard FileManager.default.fileExists(atPath: url.appendingPathComponent(ScanDataset.manifestFileName).path) else {
                throw Failure(code: .dataError, message: "\(url.path) is a folder without \(ScanDataset.manifestFileName) (not an .ifsdataset)")
            }
            return .dataset
        }
        switch url.pathExtension.lowercased() {
        case "db3", "bag": return .recording
        default:
            throw Failure(code: .dataError, message: "\(url.lastPathComponent): expected an .ifsdataset folder or a .db3/.bag recording")
        }
    }

    /// Why frames were held back, per pose (most frequent reasons first).
    static func logRejections(_ scan: RecordingScan.Result, _ log: (String) -> Void) {
        for pose in ScanPose.allCases {
            guard let counts = scan.rejections[pose], !counts.isEmpty else { continue }
            let text = counts.sorted { $0.value > $1.value }.map { "\($0.key) \($0.value)" }.joined(separator: ", ")
            log("  \(pose.rawValue): frames held back: \(text)")
        }
    }

    /// Fails before any work when an output cannot be written: the guided scan and the reconstruction take minutes and
    /// their result would be lost at the end. Creates missing parent folders (the export would create them too).
    static func checkWritableOutputs(_ c: ReconstructCommand, input: URL, kind: InputKind) throws {
        let fm = FileManager.default
        func parentFolder(of path: String, _ what: String) throws -> URL {
            let url = URL(fileURLWithPath: path).standardizedFileURL
            let parent = url.deletingLastPathComponent()
            do {
                try fm.createDirectory(at: parent, withIntermediateDirectories: true)
            } catch {
                throw Failure(code: .cannotCreate, message: "\(what) \(url.path): cannot create its folder: \(error.localizedDescription)")
            }
            guard fm.isWritableFile(atPath: parent.path) else {
                throw Failure(code: .cannotCreate, message: "\(what) \(url.path): the folder \(parent.path) is not writable")
            }
            return url
        }
        for (path, what) in [(c.output, "output"), (c.ply, "--ply"), (c.report, "--report")] {
            guard let path else { continue }
            let url = try parentFolder(of: path, what)
            var isDirectory: ObjCBool = false
            if fm.fileExists(atPath: url.path, isDirectory: &isDirectory), isDirectory.boolValue {
                throw Failure(code: .cannotCreate, message: "\(what) \(url.path) is a folder")
            }
        }
        if let path = c.saveDataset {
            let url = try parentFolder(of: path, "--save-dataset")
            if kind == .dataset, url.resolvingSymlinksInPath().path == input.standardizedFileURL.resolvingSymlinksInPath().path {
                // ScanDataset.write would replace the input folder, dropping files it does not know (ground truth).
                throw Failure(code: .usage, message: "--save-dataset must not be the input dataset")
            }
            // ScanDataset.write only replaces an existing dataset or an empty folder.
            var isDirectory: ObjCBool = false
            if fm.fileExists(atPath: url.path, isDirectory: &isDirectory) {
                let contents = (try? fm.contentsOfDirectory(atPath: url.path)) ?? []
                guard isDirectory.boolValue, contents.isEmpty || contents.contains(ScanDataset.manifestFileName) else {
                    throw Failure(code: .cannotCreate,
                                  message: "--save-dataset \(url.path) exists and is not a dataset or an empty folder")
                }
            }
        }
    }

    static func reconstruct(_ c: ReconstructCommand, _ io: IFScanOutput) async throws {
        let input = URL(fileURLWithPath: c.input)
        let kind = try inputKind(input)
        try checkWritableOutputs(c, input: input, kind: kind)
        let begin = Date()
        var timings: [String: Double] = [:]
        var notes: [String] = []
        let log: @Sendable (String) -> Void = { message in if !c.quiet { io.err(message) } }

        var capture: ScanCapture
        switch kind {
        case .dataset:
            do {
                capture = try ScanDataset.read(from: input)
            } catch {
                throw Failure(code: .dataError, message: error.localizedDescription)
            }
            log("Read \(capture.keyframes.count) keyframes from \(input.lastPathComponent)")
        case .recording:
            let helper = try resolveHelper(c.helper)
            log("Playing \(input.lastPathComponent) through \(helper.path)")
            var configuration = ScanConfiguration.offline
            configuration.keepShoulders = c.keepShoulders
            let scan: RecordingScan.Result
            do {
                scan = try await RecordingScan.run(recording: input, helper: helper, configuration: configuration, log: log)
            } catch {
                var failure = Failure(code: .unavailable, message: "playback failed: \(error.localizedDescription)")
                if let cameraError = error as? CameraError, case .unsupported = cameraError {
                    failure.notes.append("PlaybackCameraSource (CameraKit) is not implemented in this build yet")
                }
                throw failure
            }
            capture = scan.capture
            log("Guided scan: \(capture.keyframes.count) keyframes from \(scan.framesProcessed) frames"
                + (scan.completed ? " (all poses captured)" : ""))
            if let reason = scan.failure { notes.append("guided scan failed: \(reason)") }
            if !scan.completed { notes.append("the recording ended before every scan pose was captured") }
            logRejections(scan, log)
            guard !capture.keyframes.isEmpty else {
                var failure = Failure(code: .dataError,
                                      message: "the guided scan accepted no keyframes in \(scan.framesProcessed) frames of \(input.lastPathComponent)")
                if scan.framesProcessed == 0 {
                    failure.notes.append("no frames arrived from the helper")
                } else {
                    failure.notes.append("check that the recording shows a face at 0.4-0.9 m turning through the scan poses; "
                        + "ScanController (ScanKit) may also still be a placeholder in this build")
                }
                throw failure
            }
        case .syntheticCamera:
            // The app's live flow without the UI: 30 fps frames of the animated head -> Vision -> ScanController.
            log("Scanning the animated synthetic head in real time (Vision head pose, guided scan, "
                + "time limit \(format(c.timeLimitSeconds, 0)) s)")
            var configuration = ScanConfiguration.offline
            configuration.keepShoulders = c.keepShoulders
            // The animated head loops forever and the guided scan waits for every pose: without a limit a pose that
            // Vision never detects would keep the command running indefinitely. Shutting the source down ends the
            // frame stream, and RecordingScan returns the keyframes accepted so far.
            let source = SyntheticCameraSource(holdSeconds: 2.5)
            let limit = c.timeLimitSeconds
            let watchdog = Task {
                try await Task.sleep(nanoseconds: UInt64(limit * 1e9))
                await source.shutdown()
            }
            let scan: RecordingScan.Result
            do {
                scan = try await RecordingScan.run(source: source, label: syntheticCameraInput,
                                                   adoptRecordedProfiles: false, configuration: configuration, log: log)
                watchdog.cancel()
            } catch {
                watchdog.cancel()
                throw Failure(code: .unavailable, message: "synthetic camera failed: \(error.localizedDescription)")
            }
            capture = scan.capture
            log("Guided scan: \(capture.keyframes.count) keyframes from \(scan.framesProcessed) frames"
                + (scan.completed ? " (all poses captured)" : ""))
            if let reason = scan.failure { notes.append("guided scan failed: \(reason)") }
            if !scan.completed && scan.failure == nil {
                let note = "the guided scan did not capture every pose within the time limit of \(format(limit, 0)) s (--time-limit)"
                notes.append(note)
                io.err("warning: \(note)")
            }
            logRejections(scan, log)
            guard !capture.keyframes.isEmpty else {
                throw Failure(code: .dataError, message: "the guided scan accepted no keyframes from the synthetic camera",
                              notes: notes)
            }
        }
        timings["input"] = Date().timeIntervalSince(begin)

        if let path = c.saveDataset {
            do {
                try ScanDataset.write(capture, to: URL(fileURLWithPath: path))
                log("Saved the keyframes to \(path)")
            } catch {
                throw Failure(code: .cannotCreate, message: error.localizedDescription)
            }
        }

        var options = ReconstructionOptions()
        options.voxelSizeMM = c.voxelMM
        options.method = c.method
        options.keepShoulders = c.keepShoulders
        options.closeHead = !c.openBack
        options.useColoredICP = c.coloredICP
        options.targetTriangles = c.targetTriangles
        options.textureSize = c.textureSize
        let progress = ProgressPrinter(enabled: !c.quiet, io: io)
        let reconstructionStart = Date()
        let result: ReconstructionResult
        do {
            result = try await ReconstructionPipeline(options: options).run(capture) { progress.update($0) }
        } catch {
            throw pipelineFailure(error, stage: "reconstruction")
        }
        timings["reconstruction"] = Date().timeIntervalSince(reconstructionStart)

        let exportStart = Date()
        let output = URL(fileURLWithPath: c.output)
        var written: [URL]
        do {
            try FileManager.default.createDirectory(at: output.deletingLastPathComponent(), withIntermediateDirectories: true)
            written = try MeshExport.writeOBJ(result, to: output)
            if let ply = c.ply { written.append(try MeshExport.writePLY(result, to: URL(fileURLWithPath: ply))) }
        } catch {
            var failure = pipelineFailure(error, stage: "export")
            // A file that cannot be written is EX_CANTCREAT (73), like every other output of the tool.
            if error is CocoaError || isCoreStatus(error, FR_ERR_IO) { failure.code = .cannotCreate }
            throw failure
        }
        timings["export"] = Date().timeIntervalSince(exportStart)

        var groundTruth: GroundTruthEvaluation?
        if kind == .dataset, let truth = try? SyntheticGroundTruth.load(fromDataset: input) {
            let mesh = try? truth.loadMesh(fromDataset: input)
            groundTruth = GroundTruthEvaluation.evaluate(result: result, groundTruth: truth,
                                                         keyframeIDs: capture.keyframes.map(\.id), mesh: mesh)
        }
        timings["total"] = Date().timeIntervalSince(begin)

        if let path = c.report {
            let report = ReconstructionRunReport(
                tool: "ifscan \(IFScanArguments.version)", createdAt: ISO8601DateFormatter().string(from: Date()),
                input: input.path, inputKind: kind.rawValue, outputs: written.map(\.path), keyframes: capture.keyframes.count,
                options: options, report: result.report, exportTransform: result.exportTransform.columnMajor,
                exportScale: ExportOptions().unitsScale, cameraPoses: result.cameraPoses.map(\.columnMajor),
                timings: timings, groundTruth: groundTruth, notes: notes)
            do {
                let encoder = JSONEncoder()
                encoder.keyEncodingStrategy = .convertToSnakeCase
                encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
                encoder.nonConformingFloatEncodingStrategy = .convertToString(positiveInfinity: "inf", negativeInfinity: "-inf", nan: "nan")
                try encoder.encode(report).write(to: URL(fileURLWithPath: path), options: .atomic)
            } catch {
                throw Failure(code: .cannotCreate, message: "\(path): \(error.localizedDescription)")
            }
        }

        let mesh = result.mesh
        io.out("Reconstructed \(mesh.vertexCount) vertices, \(mesh.triangleCount) triangles from \(capture.keyframes.count) keyframes "
            + "(\(result.report.method.rawValue), voxel \(format(result.report.voxelSizeMM > 0 ? result.report.voxelSizeMM : c.voxelMM, 2)) mm) "
            + "in \(format(timings["total"] ?? 0, 1)) s")
        for url in written { io.out("  \(url.path)") }
        if let gt = groundTruth {
            if !gt.poses.isEmpty {
                io.out("Pose error vs ground truth: rotation mean \(format(gt.meanRotationDeg, 3))°, max \(format(gt.maxRotationDeg, 3))°; "
                    + "head misalignment mean \(format(gt.meanHeadMisalignmentMm, 3)) mm, max \(format(gt.maxHeadMisalignmentMm, 3)) mm; "
                    + "camera center max \(format(gt.maxTranslationMm, 2)) mm")
            }
            if let s = gt.surfaceToGroundTruth {
                io.out("Surface distance to ground truth\(gt.wholeMeshToGroundTruth == nil ? "" : " (measured part)"): "
                    + "mean \(format(s.meanMm, 3)) mm, median \(format(s.medianMm, 3)), "
                    + "p95 \(format(s.p95Mm, 3)), max \(format(s.maxMm, 3)) mm")
            }
            if let s = gt.wholeMeshToGroundTruth {
                io.out("Whole mesh, synthesized surface included: mean \(format(s.meanMm, 3)) mm, "
                    + "p95 \(format(s.p95Mm, 3)), max \(format(s.maxMm, 3)) mm")
            }
        }
        for note in notes { io.err("note: \(note)") }
    }

    static func isCoreStatus(_ error: Error, _ status: fr_status) -> Bool {
        if case ReconstructionError.core(_, let code, _) = error { return code == Int32(bitPattern: status.rawValue) }
        return false
    }

    static func pipelineFailure(_ error: Error, stage: String) -> Failure {
        var failure = Failure(code: .software, message: "\(stage) failed: \(error.localizedDescription)")
        if isCoreStatus(error, FR_ERR_NOT_IMPLEMENTED) {
            failure.code = .unavailable
            failure.notes.append(stage == "export" ? "MeshExport / fr_export_obj is still a placeholder in this build"
                                                   : "a FaceReconCore stage is still a placeholder in this build")
        }
        if case ReconstructionError.notEnoughKeyframes = error { failure.code = .dataError }
        let missing = CoreAvailability.missingComponents
        if !missing.isEmpty {
            failure.code = .unavailable
            failure.notes.append("FaceReconCore placeholders in this build: \(missing.joined(separator: ", "))")
        }
        return failure
    }

    /// The camera helper for playback: --helper, $IFS_HELPER, next to ifscan, inside an app bundle, the build tree.
    static func resolveHelper(_ explicit: String?) throws -> URL {
        let fm = FileManager.default
        if let explicit {
            let url = URL(fileURLWithPath: explicit)
            guard fm.isExecutableFile(atPath: url.path) else {
                throw Failure(code: .noInput, message: "helper \(url.path) does not exist or is not executable")
            }
            return url
        }
        var candidates: [URL] = []
        if let env = ProcessInfo.processInfo.environment["IFS_HELPER"], !env.isEmpty {
            candidates.append(URL(fileURLWithPath: env))
        }
        if let executable = Bundle.main.executableURL?.resolvingSymlinksInPath() {
            let folder = executable.deletingLastPathComponent()
            candidates.append(folder.appendingPathComponent("IntelFaceScanHelper"))
            candidates.append(folder.deletingLastPathComponent().appendingPathComponent("Helpers/IntelFaceScanHelper"))
        }
        let cwd = URL(fileURLWithPath: fm.currentDirectoryPath)
        candidates.append(cwd.appendingPathComponent("build/helper/IntelFaceScanHelper"))
        candidates.append(cwd.appendingPathComponent("build/IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper"))
        candidates.append(URL(fileURLWithPath: "/Applications/IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper"))
        if let found = candidates.first(where: { fm.isExecutableFile(atPath: $0.path) }) { return found }
        throw Failure(code: .unavailable, message: "IntelFaceScanHelper not found (needed to play recordings)",
                      notes: ["build it with scripts/build.sh, or: cmake -S helper -B build/helper && cmake --build build/helper",
                              "or pass --helper <path> / set IFS_HELPER"]
                          + candidates.map { "looked at \($0.path)" })
    }

    // MARK: - eval

    static func evaluate(_ c: EvalCommand, _ io: IFScanOutput) throws {
        for path in [c.mesh, c.groundTruth] + (c.report.map { [$0] } ?? []) where !FileManager.default.fileExists(atPath: path) {
            throw Failure(code: .noInput, message: "\(URL(fileURLWithPath: path).path): no such file")
        }
        var mesh: ReconMesh, truth: ReconMesh
        do {
            mesh = try WavefrontOBJ.read(URL(fileURLWithPath: c.mesh))
            truth = try WavefrontOBJ.read(URL(fileURLWithPath: c.groundTruth))
        } catch {
            throw Failure(code: .dataError, message: error.localizedDescription)
        }
        if let path = c.report {
            let frame: ExportFrame
            do {
                frame = try ExportFrame.load(URL(fileURLWithPath: path))
            } catch {
                throw Failure(code: .dataError, message: "\(path): \(error.localizedDescription)")
            }
            mesh.positions = mesh.positions.map(frame.worldMillimeters)
        }
        let forward: SurfaceDistance, backward: SurfaceDistance?
        do {
            forward = try MeshEvaluation.distance(from: mesh, to: truth, samples: c.samples, unitsToMM: 1)
            backward = c.symmetric ? try MeshEvaluation.distance(from: truth, to: mesh, samples: c.samples, unitsToMM: 1) : nil
        } catch {
            throw Failure(code: .dataError, message: error.localizedDescription)
        }
        if c.json {
            struct Output: Encodable { var mesh: String; var groundTruth: String; var meshToGroundTruth: SurfaceDistance
                                       var groundTruthToMesh: SurfaceDistance? }
            let encoder = JSONEncoder()
            encoder.keyEncodingStrategy = .convertToSnakeCase
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            let data = try encoder.encode(Output(mesh: c.mesh, groundTruth: c.groundTruth, meshToGroundTruth: forward,
                                                 groundTruthToMesh: backward))
            io.out(String(decoding: data, as: UTF8.self))
            return
        }
        io.out("Distance (mm)           mean     rms  median     p95     max  samples")
        io.out(row("mesh -> ground truth", forward))
        if let backward { io.out(row("ground truth -> mesh", backward)) }
    }

    private static func row(_ label: String, _ d: SurfaceDistance) -> String {
        label.padding(toLength: 20, withPad: " ", startingAt: 0)
            + [d.meanMm, d.rmsMm, d.medianMm, d.p95Mm, d.maxMm].map { format($0, 3).leftPadded(8) }.joined()
            + String(d.samples).leftPadded(9)
    }

    // MARK: - info

    static func info(_ c: InfoCommand, _ io: IFScanOutput) async throws {
        let url = URL(fileURLWithPath: c.path)
        var isDirectory: ObjCBool = false
        guard FileManager.default.fileExists(atPath: url.path, isDirectory: &isDirectory) else {
            throw Failure(code: .noInput, message: "\(url.path): no such file or folder")
        }
        if isDirectory.boolValue {
            try datasetInfo(url, json: c.json, io)
            return
        }
        switch url.pathExtension.lowercased() {
        case "obj": try objInfo(url, json: c.json, io)
        case "ply": try plyInfo(url, io)
        case "png": try pngInfo(url, io)
        case "db3", "bag": try await recordingInfo(url, helper: c.helper, io)
        case "json":
            if url.lastPathComponent == ScanDataset.manifestFileName {
                try datasetInfo(url.deletingLastPathComponent(), json: c.json, io)
            } else {
                throw Failure(code: .dataError, message: "\(url.lastPathComponent): unknown file type")
            }
        default:
            throw Failure(code: .dataError, message: "\(url.lastPathComponent): unknown file type (expected .ifsdataset, .db3, .bag, .obj, .ply, .png)")
        }
    }

    static func datasetInfo(_ url: URL, json: Bool, _ io: IFScanOutput) throws {
        let manifest: ScanDataset.Manifest
        do {
            manifest = try ScanDataset.readManifest(from: url)
        } catch {
            throw Failure(code: .dataError, message: error.localizedDescription)
        }
        let truth = try? SyntheticGroundTruth.load(fromDataset: url)
        if json {
            var summary = manifest
            summary.frames = []
            let encoder = JSONEncoder()
            encoder.keyEncodingStrategy = .convertToSnakeCase
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            io.out(String(decoding: try encoder.encode(summary), as: UTF8.self))
            return
        }
        io.out("\(url.lastPathComponent): IntelFaceScan dataset v\(manifest.version), created \(manifest.createdAt)")
        if let device = manifest.device {
            io.out("Device: \(device.name) (serial \(device.serial.isEmpty ? "-" : device.serial), firmware \(device.firmware.isEmpty ? "-" : device.firmware), \(device.source))")
        }
        if let config = manifest.configuration {
            io.out("Configuration: color \(config.color.width)x\(config.color.height)@\(config.color.fps), depth \(config.depth.width)x\(config.depth.height)@\(config.depth.fps), "
                + "preset \"\(config.preset)\", depth units \(config.depthUnits) m")
        }
        for (key, value) in manifest.notes.sorted(by: { $0.key < $1.key }) { io.out("Note: \(key) = \(value)") }
        let poses = Dictionary(grouping: manifest.frames, by: \.scanPose).mapValues(\.count)
        let order = ScanPose.allCases.map(\.rawValue)
        io.out("Frames: \(manifest.frames.count) ("
            + poses.sorted { (order.firstIndex(of: $0.key) ?? 99) < (order.firstIndex(of: $1.key) ?? 99) }
                .map { "\($0.key) \($0.value)" }.joined(separator: ", ") + ")")
        if let first = manifest.frames.first {
            let i = first.intrinsics
            io.out("Images: \(i.width)x\(i.height), fx \(format(i.fx, 1)) fy \(format(i.fy, 1)) cx \(format(i.ppx, 1)) cy \(format(i.ppy, 1)), "
                + "depth scale \(first.depthScale) m, masks: \(manifest.frames.filter { $0.mask != nil }.count)/\(manifest.frames.count)")
        }
        io.out("   id  pose        yaw   pitch   roll   dist(m)  valid  sharpness")
        for f in manifest.frames {
            io.out(String(f.id).leftPadded(5) + "  " + f.scanPose.padding(toLength: 9, withPad: " ", startingAt: 0)
                + format(f.headPose.yaw, 1).leftPadded(6) + format(f.headPose.pitch, 1).leftPadded(8)
                + format(f.headPose.roll, 1).leftPadded(7) + format(f.quality.faceDistanceM, 3).leftPadded(10)
                + format(f.quality.validDepthRatio * 100, 0).leftPadded(6) + "%" + format(f.quality.sharpness, 0).leftPadded(11))
        }
        if let truth {
            io.out("Ground truth: synthetic (seed \(truth.scene.seed), noise \(format(truth.scene.depthNoiseAtOneMeterM * 1000, 2)) mm at 1 m), "
                + "\(truth.frames.count) poses, mesh \(truth.mesh)")
        }
    }

    static func objInfo(_ url: URL, json: Bool, _ io: IFScanOutput) throws {
        let s: WavefrontOBJ.Statistics
        do {
            s = try WavefrontOBJ.statistics(url)
        } catch {
            throw Failure(code: .dataError, message: error.localizedDescription)
        }
        let size = s.boundsMax - s.boundsMin
        io.out("\(url.lastPathComponent): OBJ, \(s.vertices) vertices, \(s.triangles) triangles (\(s.polygons) faces)")
        io.out("Normals: \(s.normals), texture coordinates: \(s.textureCoordinates), vertex colors: \(s.vertexColors ? "yes" : "no")"
            + (s.materialLibraries.isEmpty ? "" : ", materials: \(s.materialLibraries.joined(separator: ", "))"))
        io.out("Bounds: \(vector(s.boundsMin)) ... \(vector(s.boundsMax)), size \(vector(size)) (file units; IntelFaceScan writes millimeters)")
    }

    static func plyInfo(_ url: URL, _ io: IFScanOutput) throws {
        guard let handle = try? FileHandle(forReadingFrom: url) else {
            throw Failure(code: .noInput, message: "cannot open \(url.path)")
        }
        defer { try? handle.close() }
        let head = (try? handle.read(upToCount: 65536)) ?? Data()
        guard let text = String(data: head.prefix(while: { _ in true }), encoding: .isoLatin1),
              text.hasPrefix("ply"), let end = text.range(of: "end_header") else {
            throw Failure(code: .dataError, message: "\(url.lastPathComponent) is not a PLY file")
        }
        let lines = text[..<end.lowerBound].split(whereSeparator: \.isNewline).map(String.init)
        io.out("\(url.lastPathComponent): PLY")
        for line in lines where line.hasPrefix("format") || line.hasPrefix("element") || line.hasPrefix("property") || line.hasPrefix("comment") {
            io.out("  \(line)")
        }
    }

    static func pngInfo(_ url: URL, _ io: IFScanOutput) throws {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil),
              let properties = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any] else {
            throw Failure(code: .dataError, message: "\(url.lastPathComponent) is not a readable image")
        }
        let width = properties[kCGImagePropertyPixelWidth] as? Int ?? 0
        let height = properties[kCGImagePropertyPixelHeight] as? Int ?? 0
        let depth = properties[kCGImagePropertyDepth] as? Int ?? 0
        let model = properties[kCGImagePropertyColorModel] as? String ?? "?"
        io.out("\(url.lastPathComponent): \(width)x\(height), \(depth)-bit \(model)")
    }

    static func recordingInfo(_ url: URL, helper: String?, _ io: IFScanOutput) async throws {
        let bytes = (try? FileManager.default.attributesOfItem(atPath: url.path)[.size] as? Int64) ?? 0
        io.out("\(url.lastPathComponent): \(url.pathExtension.lowercased() == "db3" ? "ROS 2 (sqlite3) bag" : "ROS 1 bag"), "
            + "\(format(Double(bytes) / 1_048_576, 1)) MiB")
        let helperURL: URL
        do {
            helperURL = try resolveHelper(helper)
        } catch let failure as Failure {
            io.err("note: \(failure.message); device and stream details need the helper")
            return
        }
        let source = PlaybackCameraSource(file: url, helperURL: helperURL, realTime: false)
        do {
            let device = try await source.connect()
            await source.shutdown()
            io.out("Device: \(device.name), serial \(device.serial), firmware \(device.firmware), depth scale \(device.depthScale) m")
            for p in device.profiles {
                io.out("  \(p.stream) \(p.index): \(p.format) \(p.width)x\(p.height) @ \(p.fps) fps")
            }
        } catch {
            await source.shutdown()
            io.err("note: cannot read the recording through the helper: \(error.localizedDescription)")
        }
    }

    // MARK: - Formatting

    static func format(_ value: Double, _ decimals: Int) -> String { String(format: "%.\(decimals)f", value) }
    static func format(_ value: Float, _ decimals: Int) -> String { format(Double(value), decimals) }
    static func vector(_ v: SIMD3<Float>) -> String { "(\(format(v.x, 2)), \(format(v.y, 2)), \(format(v.z, 2)))" }
}

/// Progress lines on stderr: on every stage change and every 5%.
final class ProgressPrinter: @unchecked Sendable {
    private let lock = NSLock()
    private let enabled: Bool
    private let io: IFScanOutput
    private var lastStage: ReconstructionProgress.Stage?
    private var lastFraction = -1.0

    init(enabled: Bool, io: IFScanOutput) {
        self.enabled = enabled
        self.io = io
    }

    func update(_ progress: ReconstructionProgress) {
        guard enabled else { return }
        let line: String? = lock.withLock {
            guard progress.stage != lastStage || progress.fraction >= lastFraction + 0.05 else { return nil }
            lastStage = progress.stage
            lastFraction = progress.fraction
            let percent = Int((min(max(progress.fraction, 0), 1) * 100).rounded())
            return "[\(String(percent).leftPadded(3))%] \(progress.stage.rawValue)" + (progress.message.isEmpty ? "" : ": \(progress.message)")
        }
        if let line { io.err(line) }
    }
}

extension String {
    func leftPadded(_ width: Int) -> String {
        count >= width ? self : String(repeating: " ", count: width - count) + self
    }
}
