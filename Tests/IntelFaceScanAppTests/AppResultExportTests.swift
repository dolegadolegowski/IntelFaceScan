// License: Apache 2.0. See LICENSE file in root directory.
//
// The result and export side of the app model: report timings, dataset destinations, saving (success and failure
// paths), the status message and its Show in Finder target, and a dataset opened as a source.

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing
@testable import IntelFaceScanApp

@Suite("App result and export")
struct AppResultExportTests {
    /// The reconstruction report's "total" is the sum of the stages, not another stage.
    @Test func reportTimings() {
        let t = ReportView.timings(["texturing": 3, "total": 7.5, "registration": 2, "fusion": 1, "segmentation": 1.5])
        #expect(t.total == 7.5)
        #expect(t.stages.map(\.key) == ["segmentation", "registration", "fusion", "texturing"])
        let untotaled = ReportView.timings(["fusion": 1, "cleanup": 0.5])
        #expect(untotaled.total == 1.5)
    }

    /// An existing session is not replaced under a name the save panel never showed.
    @Test func datasetDestination() throws {
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let ext = ScanDataset.fileExtension
        #expect(AppModel.datasetDestination(for: dir.appendingPathComponent("scan")).lastPathComponent == "scan.\(ext)")
        try FileManager.default.createDirectory(at: dir.appendingPathComponent("scan.\(ext)"), withIntermediateDirectories: false)
        #expect(AppModel.datasetDestination(for: dir.appendingPathComponent("scan")).lastPathComponent == "scan 2.\(ext)")
        let confirmed = dir.appendingPathComponent("scan.\(ext)")
        #expect(AppModel.datasetDestination(for: confirmed) == confirmed, "a name confirmed in the panel stays")
    }

    /// A small synthetic scan (keyframes with real images) and a one-triangle result.
    static func fixture() -> (capture: ScanCapture, result: ReconstructionResult) {
        var options = SyntheticScan.Options()
        options.width = 160
        options.height = 100
        options.framesPerPose = 1
        options.poses = [.front, .left]
        options.meshResolution = 32
        let capture = SyntheticScan.make(options).capture
        let mesh = ReconMesh(positions: [SIMD3(0, 0, 0), SIMD3(0.1, 0, 0), SIMD3(0, 0.1, 0)], indices: [0, 1, 2])
        let result = ReconstructionResult(mesh: mesh, texture: nil, cameraPoses: [],
                                          exportTransform: matrix_identity_float4x4, report: ReconstructionReport())
        return (capture, result)
    }

    @MainActor
    static func modelOnResultScreen(_ model: AppModel) -> ScanCapture {
        let (capture, result) = fixture()
        model.applyFixture(screen: .result, sourceKind: nil, device: nil, tracking: TrackingState(), scanUpdate: nil,
                           scanConfiguration: ScanConfiguration(), capture: capture, progress: nil, result: result,
                           stats: FrameStats())
        return capture
    }

    /// Saving reports the files and offers the OBJ in Finder; a later notice about something else (the opened
    /// dataset finished playing) no longer offers it. The old model kept the last saved file for every message.
    @MainActor
    @Test(.sourceTestLimit) func savedFileOnlyForItsOwnMessage() async throws {
        let (model, log) = AppTest.makeModel()
        defer { await AppTest.cleanUp(model, log) }
        let capture = Self.modelOnResultScreen(model)
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let obj = dir.appendingPathComponent("head.obj")
        await model.exportOBJ(to: obj)
        #expect(model.alert == nil, "export failed: \(model.alert?.message ?? "")")
        #expect(AppTest.fileSize(obj) > 0)
        #expect(model.statusMessage?.contains("head.obj") == true, "status \(model.statusMessage ?? "nil")")
        #expect(model.revealableURL == obj)
        #expect(!model.isWriting)

        // The dataset plays its frames once and ends.
        let dataset = dir.appendingPathComponent("session.\(ScanDataset.fileExtension)")
        try ScanDataset.write(capture, to: dataset)
        await model.openSource(.dataset(dataset))
        #expect(model.alert == nil, "opening the dataset failed: \(model.alert?.message ?? "")")
        #expect(model.screen == .preview)
        #expect(await AppTest.waitUntil(30) { model.statusMessage == S.playbackFinished }, "no end-of-playback notice")
        #expect(model.revealableURL == nil, "Show in Finder still offers \(model.revealableURL?.lastPathComponent ?? "")")

        // Its keyframes can be reconstructed as they are.
        #expect(await AppTest.waitUntil(30) { model.datasetCapture != nil }, "the dataset's keyframes were not loaded")
        #expect(model.datasetCapture?.keyframes.count == capture.keyframes.count)
        model.useDatasetCapture()
        #expect(model.screen == .scanComplete)
        #expect(model.capture?.keyframes.count == capture.keyframes.count)
        await model.closeSource()
        #expect(model.screen == .scanComplete, "closing the source must keep the scan summary")
    }

    /// A failed write is an alert, not a status message, and holds nothing for the termination.
    @MainActor
    @Test func failedSave() async throws {
        let (model, _) = AppTest.makeModel()
        _ = Self.modelOnResultScreen(model)
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let blocker = dir.appendingPathComponent("file")
        try Data("x".utf8).write(to: blocker)
        // A folder below a regular file cannot be created.
        await model.exportOBJ(to: blocker.appendingPathComponent("sub/head.obj"))
        let alert = try #require(model.alert, "no alert for a failed save")
        #expect(alert.title == S.saveFailed)
        #expect(!alert.message.isEmpty)
        #expect(model.statusMessage == nil)
        #expect(model.revealableURL == nil)
        #expect(!model.isWriting)
        #expect(!model.needsShutdown)
        #expect(model.screen == .result, "a failed save must keep the result")
        model.alert = nil

        // The session export writes a dataset folder (the extension is added) and offers it in Finder.
        let chosen = dir.appendingPathComponent("my scan")
        // The write waits at the gate until the flags are checked: one Task.yield() did not always let the export start
        // on the 3-core CI runner (CI run 41937143470), and polling missed this small write, done in milliseconds.
        let (gate, open) = AsyncStream<Void>.makeStream()
        model.writeGate = { for await _ in gate { break } }
        defer { model.writeGate = nil }
        let export = Task { await model.exportSession(to: chosen) }
        #expect(await AppTest.waitUntil(10) { model.isWriting }, "the save buttons do not wait for the running write")
        #expect(model.needsShutdown, "quitting would not wait for the running write")
        open.yield()
        open.finish()
        await export.value
        #expect(!model.isWriting)
        #expect(model.alert == nil, "session export failed: \(model.alert?.message ?? "")")
        let written = chosen.appendingPathExtension(ScanDataset.fileExtension)
        #expect(model.revealableURL == written)
        #expect(FileManager.default.fileExists(atPath: written.appendingPathComponent("manifest.json").path))
    }

    /// Reconstruction settings changed on the result screen do nothing to the model shown: the screen offers to
    /// reconstruct the scan again with them (before, they waited for a new scan). Display settings do not count.
    @MainActor
    @Test func reconstructionSettingsChangedAfterResult() async throws {
        let (model, _) = AppTest.makeModel()
        _ = Self.modelOnResultScreen(model)
        #expect(!model.resultSettingsChanged)
        model.settings.mirrored.toggle()
        model.settings.showLandmarks.toggle()
        #expect(!model.resultSettingsChanged, "display settings do not change the reconstruction")
        let voxel = model.settings.voxelSizeMM
        model.settings.voxelSizeMM = voxel + 0.25
        #expect(model.resultSettingsChanged)
        model.settings.voxelSizeMM = voxel
        #expect(!model.resultSettingsChanged)
        model.settings.method = model.settings.method == .tsdf ? .poisson : .tsdf
        #expect(model.resultSettingsChanged)

        // Reconstructing again uses the new settings (the two-frame fixture is too small and fails fast).
        model.reconstruct()
        #expect(model.screen == .reconstructing)
        #expect(!model.resultSettingsChanged)
        #expect(await AppTest.waitUntil(60) { model.screen != .reconstructing }, "the reconstruction did not end")
        #expect(model.screen == .scanComplete)
        #expect(model.alert?.title == S.reconstructionFailed)
        #expect(!model.resultSettingsChanged)
    }

    /// MeshExport's own refusal (an empty mesh) reads in the UI language like the reconstruction's errors.
    @MainActor
    @Test func exportErrorInPolish() async throws {
        let (model, _) = AppTest.makeModel()
        let (capture, _) = Self.fixture()
        let empty = ReconstructionResult(mesh: ReconMesh(), texture: nil, cameraPoses: [],
                                         exportTransform: matrix_identity_float4x4, report: ReconstructionReport())
        model.applyFixture(screen: .result, sourceKind: nil, device: nil, tracking: TrackingState(), scanUpdate: nil,
                           scanConfiguration: ScanConfiguration(), capture: capture, progress: nil, result: empty,
                           stats: FrameStats())
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        await AppLanguage.$override.withValue(.polish) {
            await model.exportOBJ(to: dir.appendingPathComponent("head.obj"))
        }
        let alert = try #require(model.alert, "an empty mesh was exported")
        #expect(alert.title == "Nie można zapisać")
        #expect(alert.message.hasPrefix("Etap „export” nie powiódł się:"), "\(alert.message)")
    }

    /// Two errors in a row are both shown: the second is appended to the alert on screen.
    @MainActor
    @Test func alertsAccumulate() {
        let (model, _) = AppTest.makeModel()
        model.present("First", message: "one")
        model.present("Second", message: "two")
        #expect(model.alert?.title == "First")
        #expect(model.alert?.message == "one\n\nSecond: two")
    }
}
