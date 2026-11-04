// License: Apache 2.0. See LICENSE file in root directory.
//
// Debug builds only: `IntelFaceScan --run-checks [--full] [name ...]` drives the real AppModel, FramePipeline and
// Metal renderer headlessly (no window, settings in memory) against the synthetic head and the helper's fake camera
// (`build/helper/IntelFaceScanHelper --fake`, found like the CameraKit tests find it). They cover the source
// lifecycle and the quit path, which have no unit test target (the app is an executable target). `--full` adds a
// complete guided scan of the synthetic head, its reconstruction and an OBJ export (minutes).

#if DEBUG
import CameraKit
import CoreGraphics
import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData

/// Forwards to a source and records its shutdown (optionally slowed down to widen race windows).
final class TrackedSource: CameraSource, @unchecked Sendable {
    let base: CameraSource
    let shutdownDelay: Double
    private let lock = NSLock()
    private var started: Date?
    private var finished: Date?
    private var connectedAt: Date?

    init(_ base: CameraSource, shutdownDelay: Double = 0) {
        self.base = base
        self.shutdownDelay = shutdownDelay
    }

    var shutdownStarted: Date? { lock.withLock { started } }
    var shutdownFinished: Date? { lock.withLock { finished } }
    var connectStarted: Date? { lock.withLock { connectedAt } }

    var kind: CameraSourceKind { base.kind }
    func connect() async throws -> DeviceInfo {
        lock.withLock { connectedAt = Date() }
        return try await base.connect()
    }
    func configure(_ configuration: CaptureConfiguration) async throws { try await base.configure(configuration) }
    func setFilters(_ filters: FilterConfiguration) async throws { try await base.setFilters(filters) }
    func start() async throws { try await base.start() }
    func stop() async throws { try await base.stop() }
    func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        try await base.setOption(sensor: sensor, name: name, value: value)
    }
    func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {
        try await base.setAutoExposureROI(rect, distanceMeters: distanceMeters)
    }
    func startRecording(to url: URL) async throws { try await base.startRecording(to: url) }
    func stopRecording() async throws -> Int64 { try await base.stopRecording() }
    func hardwareReset() async throws { try await base.hardwareReset() }
    func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> { base.frames(bufferingNewest: bufferingNewest) }
    func events() -> AsyncStream<CameraEvent> { base.events() }
    func shutdown() async {
        lock.withLock { if started == nil { started = Date() } }
        if shutdownDelay > 0 { try? await Task.sleep(for: .seconds(shutdownDelay)) }
        await base.shutdown()
        lock.withLock { if finished == nil { finished = Date() } }
    }
}

@MainActor
enum DeveloperChecks {
    struct Failure: Error, CustomStringConvertible {
        let description: String
        init(_ description: String) { self.description = description }
    }

    /// Sources created by a model's factory, in order.
    final class SourceLog {
        var sources: [TrackedSource] = []
    }

    static func run(arguments: [String]) async -> Int32 {
        setvbuf(stdout, nil, _IOLBF, 0)   // progress lines also when redirected to a file
        let full = arguments.contains("--full")
        let names = arguments.filter { !$0.hasPrefix("--") }
        var checks: [(String, () async throws -> Void)] = [
            ("reportTimings", reportTimings),
            ("datasetDestination", datasetDestination),
            ("syntheticPreview", syntheticPreview),
            ("switchWhileClosing", switchWhileClosing),
            ("quitWhileClosing", quitWhileClosing),
            ("quitWithMainQueueBlocked", quitWithMainQueueBlocked),
            ("fakeCameraRecordingOnClose", fakeCameraRecordingOnClose),
            ("fakeCameraQuitDuringRecording", fakeCameraQuitDuringRecording),
        ]
        if full { checks.append(("fullSyntheticScan", fullSyntheticScan)) }
        if !names.isEmpty { checks = checks.filter { names.contains($0.0) } }
        var failures = 0
        for (name, check) in checks {
            let start = Date()
            do {
                try await check()
                print(String(format: "PASS %@ (%.1f s)", name, Date().timeIntervalSince(start)))
            } catch {
                failures += 1
                print(String(format: "FAIL %@ (%.1f s): %@", name, Date().timeIntervalSince(start), "\(error)"))
            }
        }
        print("\(checks.count - failures) of \(checks.count) checks passed")
        return failures == 0 ? 0 : 1
    }

    // MARK: - Helpers

    static func expect(_ condition: Bool, _ message: @autoclosure () -> String) throws {
        if !condition { throw Failure(message()) }
    }

    static func waitUntil(_ timeout: Double, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while !condition() {
            if Date() > deadline { return false }
            try? await Task.sleep(for: .milliseconds(50))
        }
        return true
    }

    /// A model with in-memory settings whose sources are recorded; `.live` is the helper's fake camera.
    static func makeModel(shutdownDelay: Double = 0) -> (AppModel, SourceLog) {
        let log = SourceLog()
        let model = AppModel(defaults: nil) { request in
            let base: CameraSource
            switch request {
            case .live: base = FakeCameraSource()
            case .synthetic:
                var options = SyntheticCameraSource.defaultOptions
                options.width = 640
                options.height = 400
                base = SyntheticCameraSource(options: options)
            default: base = AppModel.makeSource(request)
            }
            let tracked = TrackedSource(base, shutdownDelay: shutdownDelay)
            log.sources.append(tracked)
            return tracked
        }
        return (model, log)
    }

    static func openAndWaitForFrames(_ model: AppModel, _ request: SourceRequest, timeout: Double = 30) async throws {
        await model.openSource(request)
        try expect(model.alert == nil, "opening the source failed: \(model.alert?.message ?? "")")
        try expect(model.screen == .preview, "screen \(model.screen) after opening, expected preview")
        try expect(await waitUntil(timeout) { model.hasFrames }, "no analyzed frame within \(timeout) s")
    }

    static var helperAvailable: Bool { CameraSources.locateHelper() != nil }

    static func temporaryDirectory() throws -> URL {
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("IntelFaceScanChecks-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
        return url
    }

    // MARK: - Checks

    /// The reconstruction report's "total" is the sum of the stages, not another stage.
    static func reportTimings() async throws {
        let t = ReportView.timings(["texturing": 3, "total": 7.5, "registration": 2, "fusion": 1, "segmentation": 1.5])
        try expect(t.total == 7.5, "total \(t.total), expected the reported 7.5")
        try expect(t.stages.map(\.key) == ["segmentation", "registration", "fusion", "texturing"],
                   "stage order \(t.stages.map(\.key))")
        let untotaled = ReportView.timings(["fusion": 1, "cleanup": 0.5])
        try expect(untotaled.total == 1.5, "sum without a total: \(untotaled.total)")
    }

    /// An existing session is not replaced under a name the save panel never showed.
    static func datasetDestination() async throws {
        let dir = try temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let ext = ScanDataset.fileExtension
        try expect(AppModel.datasetDestination(for: dir.appendingPathComponent("scan")).lastPathComponent == "scan.\(ext)",
                   "a free name keeps the chosen name")
        try FileManager.default.createDirectory(at: dir.appendingPathComponent("scan.\(ext)"), withIntermediateDirectories: false)
        let next = AppModel.datasetDestination(for: dir.appendingPathComponent("scan"))
        try expect(next.lastPathComponent == "scan 2.\(ext)", "existing session: got \(next.lastPathComponent)")
        let confirmed = dir.appendingPathComponent("scan.\(ext)")
        try expect(AppModel.datasetDestination(for: confirmed) == confirmed, "a name confirmed in the panel stays")
    }

    /// Frames reach the analysis; the pipeline idles while no screen shows the preview; closing shuts the source down.
    static func syntheticPreview() async throws {
        let (model, log) = makeModel()
        try await openAndWaitForFrames(model, .synthetic)
        try expect(model.isPipelineActive == true, "pipeline inactive on the preview")
        try expect(await waitUntil(20) { model.tracking.faceRect != nil }, "no face found on the synthetic head")
        model.startScan()
        try expect(model.screen == .scanning, "scan did not start")
        model.finishScan()
        try expect(model.screen == .scanComplete, "screen \(model.screen) after finishing")
        try expect(model.isPipelineActive == false, "pipeline still analyzing frames on the scan summary")
        let before = model.stats
        try? await Task.sleep(for: .milliseconds(400))
        model.backToPreview()
        try expect(model.isPipelineActive == true, "pipeline not resumed on the preview")
        _ = before
        await model.closeSource()
        try expect(model.screen == .welcome, "screen \(model.screen) after closing")
        try expect(log.sources.count == 1 && log.sources[0].shutdownFinished != nil, "source not shut down")
    }

    /// A second source request while the previous source is still shutting down neither orphans a source nor
    /// connects before the previous one is gone.
    static func switchWhileClosing() async throws {
        let (model, log) = makeModel(shutdownDelay: 1.0)
        try await openAndWaitForFrames(model, .synthetic)
        let switching = Task { await model.openSource(.synthetic) }
        await Task.yield()
        // The old code accepted this request too (isConnecting was false while the first one closed the source)
        // and both created a source.
        let third = Task { await model.openSource(.synthetic) }
        await switching.value
        await third.value
        try expect(log.sources.count == 2, "\(log.sources.count) sources created, expected 2")
        let first = log.sources[0], second = log.sources[1]
        try expect(first.shutdownFinished != nil, "the first source was not shut down")
        try expect(second.shutdownStarted == nil, "the current source was shut down")
        if let finished = first.shutdownFinished, let connected = second.connectStarted {
            try expect(connected >= finished, "the new source connected before the old one had shut down")
        } else {
            throw Failure("the new source never connected")
        }
        try expect(model.hasSource && model.screen == .preview, "no running source after the switch")
        await model.closeSource()
        await model.waitForClosingSources()
        try expect(log.sources.allSatisfy { $0.shutdownFinished != nil }, "a source was left running")
    }

    /// Quitting while a closed source is still shutting down waits for it (the old needsShutdown was false here and
    /// the app exited at once).
    static func quitWhileClosing() async throws {
        let (model, log) = makeModel(shutdownDelay: 1.0)
        try await openAndWaitForFrames(model, .synthetic)
        model.closeSourceAction()
        try expect(model.needsShutdown, "a closing source does not hold the termination")
        guard let work = model.beginTermination() else { throw Failure("nothing to wait for while a source closes") }
        try expect(await work.wait(cameraTimeout: 10), "termination wait timed out")
        try expect(log.sources[0].shutdownFinished != nil, "the source was not shut down before the reply")
        await model.openSource(.synthetic)
        try expect(log.sources.count == 1, "a source was opened while quitting")
    }

    /// applicationShouldTerminate can be called from a main-queue block; AppKit then holds the main queue until it
    /// gets the reply. The shutdown must finish without the main actor: here the main thread is blocked for real.
    static func quitWithMainQueueBlocked() async throws {
        let (model, log) = makeModel()
        let request: SourceRequest = helperAvailable ? .live : .synthetic
        try await openAndWaitForFrames(model, request)
        model.startScan()
        try? await Task.sleep(for: .milliseconds(300))
        let outcome: (finished: Bool, seconds: Double) = await withCheckedContinuation { continuation in
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    let start = Date()
                    let work = model.beginTermination()
                    print("  termination work: \(work.map { "\($0.camera.count) camera, \($0.writes.count) writes" } ?? "none")")
                    let done = DispatchSemaphore(value: 0)
                    Task.detached {
                        _ = await work?.wait(cameraTimeout: 20)
                        done.signal()
                    }
                    // Blocks the main thread like AppKit's modal wait inside a main-queue callout.
                    let finished = done.wait(timeout: .now() + 25) == .success
                    continuation.resume(returning: (finished, Date().timeIntervalSince(start)))
                }
            }
        }
        try expect(outcome.finished, "the shutdown needed the main queue (blocked for \(outcome.seconds) s)")
        try expect(log.sources.first?.shutdownFinished != nil, "the source was not shut down")
        try expect(outcome.seconds < 15, "the shutdown took \(outcome.seconds) s")
        print(String(format: "  shutdown of the %@ source with the main thread blocked: %.2f s",
                     helperAvailable ? "fake helper" : "synthetic", outcome.seconds))
    }

    /// Closing the source finishes a running recording and reports it.
    static func fakeCameraRecordingOnClose() async throws {
        guard helperAvailable else {
            print("  skipped: build/helper/IntelFaceScanHelper not found")
            return
        }
        let (model, log) = makeModel()
        let dir = try temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("rec.db3")
        try await openAndWaitForFrames(model, .live)
        await model.startRecording(to: file)
        try expect(model.isRecording, "recording did not start: \(model.alert?.message ?? "")")
        try? await Task.sleep(for: .seconds(1.5))
        await model.closeSource()
        try expect(await waitUntil(5) { model.statusMessage != nil || model.alert != nil }, "no report of the recording")
        try expect(model.alert == nil, "recording failed: \(model.alert?.message ?? "")")
        let size = (try? FileManager.default.attributesOfItem(atPath: file.path)[.size] as? Int) ?? 0
        try expect(size > 0, "the recording is empty")
        try expect(model.lastSavedURL == file, "Show in Finder does not point at the recording")
        try expect(log.sources[0].shutdownFinished != nil, "the helper was not shut down")
        print("  recording: \(size) bytes, status: \(model.statusMessage ?? "")")
    }

    /// Quitting during a recording delivers the file before the reply.
    static func fakeCameraQuitDuringRecording() async throws {
        guard helperAvailable else {
            print("  skipped: build/helper/IntelFaceScanHelper not found")
            return
        }
        let (model, log) = makeModel()
        let dir = try temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("quit.db3")
        try await openAndWaitForFrames(model, .live)
        await model.startRecording(to: file)
        try expect(model.isRecording, "recording did not start: \(model.alert?.message ?? "")")
        try? await Task.sleep(for: .seconds(1))
        guard let work = model.beginTermination() else { throw Failure("nothing to wait for during a recording") }
        try expect(await work.wait(cameraTimeout: 20), "termination wait timed out")
        let size = (try? FileManager.default.attributesOfItem(atPath: file.path)[.size] as? Int) ?? 0
        try expect(size > 0, "the recording is empty after quitting")
        try expect(log.sources[0].shutdownFinished != nil, "the helper was not shut down")
    }

    /// The app's live flow on the synthetic head: guided scan to the end, reconstruction, OBJ export in millimeters.
    static func fullSyntheticScan() async throws {
        let model = AppModel(defaults: nil)
        await model.openSource(.synthetic)
        try expect(await waitUntil(30) { model.hasFrames }, "no frames")
        try expect(await waitUntil(30) { model.tracking.faceRect != nil }, "no face")
        model.startScan()
        var lastPhase = ""
        var rejections: [String: Int] = [:]
        let finished = await waitUntil(240) {
            if let update = model.scanUpdate {
                let phase = "\(update.phase)"
                if phase != lastPhase {
                    print("  scan: \(phase), \(update.keyframeCount) keyframes")
                    lastPhase = phase
                }
                if let r = update.rejection { rejections["\(r)", default: 0] += 1 }
            }
            return model.screen != .scanning
        }
        if !finished {
            print("  rejections: \(rejections)")
            model.finishScan()
        }
        let keyframes = model.capture?.keyframes ?? []
        let poses = Set(keyframes.map(\.scanPose))
        print("  scan \(finished ? "completed" : "timed out"): \(keyframes.count) keyframes, poses \(poses.map(\.rawValue).sorted())")
        try expect(finished, "the guided scan did not complete in 240 s (rejections \(rejections))")
        try expect(model.screen == .scanComplete, "screen \(model.screen) after the scan")
        model.reconstruct()
        try expect(await waitUntil(1200) { model.screen != .reconstructing }, "reconstruction did not end in 20 min")
        try expect(model.screen == .result, "reconstruction failed: \(model.alert?.message ?? "")")
        let dir = try temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let obj = dir.appendingPathComponent("head.obj")
        await model.exportOBJ(to: obj)
        try expect(model.alert == nil, "export failed: \(model.alert?.message ?? "")")
        let text = try String(contentsOf: obj, encoding: .utf8)
        var lo = SIMD3<Float>(repeating: .infinity), hi = SIMD3<Float>(repeating: -.infinity)
        for line in text.split(separator: "\n") where line.hasPrefix("v ") {
            let p = line.split(separator: " ").dropFirst().prefix(3).compactMap { Float($0) }
            guard p.count == 3 else { continue }
            let v = SIMD3(p[0], p[1], p[2])
            lo = simd_min(lo, v); hi = simd_max(hi, v)
        }
        let size = hi - lo
        print(String(format: "  exported %@: %.0f × %.0f × %.0f mm", obj.lastPathComponent, size.x, size.y, size.z))
        try expect(size.y > 120 && size.y < 400 && size.x > 100 && size.x < 300, "not a head in millimeters: \(size)")
        if let report = model.result?.report {
            print(String(format: "  report: %d / %d keyframes, RMSE %.2f mm, %d triangles, total %.1f s", report.usedFrames,
                         report.keyframes, report.meanRegistrationRMSEmm, report.triangleCount,
                         ReportView.timings(report.durations).total))
        }
        await model.shutdownForTermination()
    }
}
#endif
