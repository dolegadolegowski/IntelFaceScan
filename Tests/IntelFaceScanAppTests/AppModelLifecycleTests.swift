// License: Apache 2.0. See LICENSE file in root directory.
//
// The real AppModel, FramePipeline and Metal renderer, headless (no window, settings in memory), against the
// synthetic head and the helper's fake camera: source switching, closing, the quit path with the main queue
// blocked, recordings finished on close and on quit. `IFS_APP_FULL_SCAN=1` adds the complete guided scan of the
// synthetic head through the app model, its reconstruction and an OBJ export (minutes).

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing
@testable import IntelFaceScanApp

@MainActor
@Suite("App model lifecycle", .serialized)
struct AppModelLifecycleTests {
    /// Frames reach the analysis; the pipeline idles while no screen shows the preview; closing shuts the source down.
    @Test func syntheticPreview() async throws {
        let (model, log) = AppTest.makeModel()
        try await AppTest.openAndWaitForFrames(model, .synthetic)
        #expect(model.isPipelineActive == true, "pipeline inactive on the preview")
        #expect(await AppTest.waitUntil(60) { model.tracking.faceRect != nil }, "no face found on the synthetic head")
        model.startScan()
        #expect(model.screen == .scanning, "scan did not start")
        model.finishScan()
        #expect(model.screen == .scanComplete, "screen \(model.screen) after finishing")
        #expect(model.isPipelineActive == false, "pipeline still analyzing frames on the scan summary")
        model.backToPreview()
        #expect(model.screen == .preview)
        #expect(model.isPipelineActive == true, "pipeline not resumed on the preview")
        await model.closeSource()
        #expect(model.screen == .welcome, "screen \(model.screen) after closing")
        #expect(log.sources.count == 1 && log.sources[0].shutdownFinished != nil, "source not shut down")
        #expect(!model.needsShutdown)
    }

    /// A second source request while the previous source is still shutting down neither orphans a source nor
    /// connects before the previous one is gone.
    @Test func switchWhileClosing() async throws {
        let (model, log) = AppTest.makeModel(shutdownDelay: 1.0)
        try await AppTest.openAndWaitForFrames(model, .synthetic)
        let switching = Task { await model.openSource(.synthetic) }
        await Task.yield()
        // An older model accepted this request too (isConnecting was false while the first one closed the source)
        // and both created a source.
        let third = Task { await model.openSource(.synthetic) }
        await switching.value
        await third.value
        try #require(log.sources.count == 2, "\(log.sources.count) sources created, expected 2")
        let first = log.sources[0], second = log.sources[1]
        #expect(first.shutdownFinished != nil, "the first source was not shut down")
        #expect(second.shutdownStarted == nil, "the current source was shut down")
        let finished = try #require(first.shutdownFinished)
        let connected = try #require(second.connectStarted, "the new source never connected")
        // (Not `>=`: Date's <= and >= are new in macOS 27.2, and the CI runner's 27.0.1 then cannot load the bundle.)
        #expect(connected.timeIntervalSince(finished) >= 0, "the new source connected before the old one had shut down")
        #expect(model.hasSource && model.screen == .preview, "no running source after the switch")
        await model.closeSource()
        await model.waitForClosingSources()
        #expect(log.sources.allSatisfy { $0.shutdownFinished != nil }, "a source was left running")
    }

    /// "Keep neck and shoulders" switched on after a scan that cut them off cannot bring them back (the keyframe
    /// masks end at the chin): the sidebar says so until the next scan.
    @Test func shouldersSwitchedOnAfterScan() async throws {
        let (model, _) = AppTest.makeModel()
        try await AppTest.openAndWaitForFrames(model, .synthetic)
        model.startScan()
        model.finishScan()
        #expect(model.screen == .scanComplete)
        #expect(model.captureKeptShoulders == false)
        #expect(!model.shouldersMissingFromCapture)
        model.settings.keepShoulders = true
        #expect(model.shouldersMissingFromCapture, "the setting looks applied to a scan without shoulders")
        model.settings.keepShoulders = false
        #expect(!model.shouldersMissingFromCapture, "switching it off does apply (the reconstruction cuts the neck)")
        model.settings.keepShoulders = true
        model.backToPreview()
        model.startScan()
        #expect(!model.shouldersMissingFromCapture, "no hint while scanning")
        model.finishScan()
        #expect(model.captureKeptShoulders == true)
        #expect(!model.shouldersMissingFromCapture)
        await model.closeSource()
    }

    /// No source can be opened while a scan runs (it would end the scan) or while a reconstruction runs.
    @Test func noSwitchDuringScan() async throws {
        let (model, log) = AppTest.makeModel()
        try await AppTest.openAndWaitForFrames(model, .synthetic)
        #expect(model.canOpenSource)
        model.startScan()
        #expect(model.screen == .scanning)
        #expect(!model.canOpenSource)
        model.cancelScan()
        #expect(model.screen == .preview)
        #expect(model.canOpenSource)
        await model.closeSource()
        #expect(log.sources.count == 1)
    }

    /// Quitting while a closed source is still shutting down waits for it (an older needsShutdown was false here and
    /// the app exited at once); no source opens afterwards.
    @Test func quitWhileClosing() async throws {
        let (model, log) = AppTest.makeModel(shutdownDelay: 1.0)
        try await AppTest.openAndWaitForFrames(model, .synthetic)
        model.closeSourceAction()
        #expect(model.needsShutdown, "a closing source does not hold the termination")
        let work = try #require(model.beginTermination(), "nothing to wait for while a source closes")
        #expect(await work.wait(cameraTimeout: 10), "termination wait timed out")
        #expect(log.sources[0].shutdownFinished != nil, "the source was not shut down before the reply")
        await model.openSource(.synthetic)
        #expect(log.sources.count == 1, "a source was opened while quitting")
        #expect(!model.canOpenSource)
    }

    /// applicationShouldTerminate can be called from a main-queue block; AppKit then holds the main queue until it
    /// gets the reply. The shutdown must finish without the main actor: here the main thread is blocked for real.
    @Test func quitWithMainQueueBlocked() async throws {
        let (model, log) = AppTest.makeModel()
        let request: SourceRequest = AppTest.helperAvailable ? .live : .synthetic
        try await AppTest.openAndWaitForFrames(model, request)
        model.startScan()
        try? await Task.sleep(for: .milliseconds(300))
        let outcome: (finished: Bool, seconds: Double, hadWork: Bool) = await withCheckedContinuation { continuation in
            DispatchQueue.main.async {
                MainActor.assumeIsolated {
                    let start = Date()
                    let work = model.beginTermination()
                    let done = DispatchSemaphore(value: 0)
                    Task.detached {
                        _ = await work?.wait(cameraTimeout: 20)
                        done.signal()
                    }
                    // Blocks the main thread like AppKit's modal wait inside a main-queue callout.
                    let finished = done.wait(timeout: .now() + 25) == .success
                    continuation.resume(returning: (finished, Date().timeIntervalSince(start), work != nil))
                }
            }
        }
        #expect(outcome.hadWork, "a running source left nothing to wait for")
        #expect(outcome.finished, "the shutdown needed the main queue (blocked for \(outcome.seconds) s)")
        #expect(log.sources.first?.shutdownFinished != nil, "the source was not shut down")
        #expect(outcome.seconds < 15, "the shutdown took \(outcome.seconds) s")
    }

    /// Closing the source finishes a running recording and reports it; Show in Finder points at the file.
    @Test(.enabled(if: AppTest.helperAvailable, AppTest.helperSkipReason))
    func fakeCameraRecordingOnClose() async throws {
        let (model, log) = AppTest.makeModel()
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("rec.db3")
        try await AppTest.openAndWaitForFrames(model, .live)
        #expect(model.isLive)
        await model.startRecording(to: file)
        try #require(model.isRecording, "recording did not start: \(model.alert?.message ?? "")")
        #expect(model.depthSettingsLocked, "the helper refuses stream changes during a recording")
        try? await Task.sleep(for: .seconds(1.5))
        await model.closeSource()
        #expect(!model.depthSettingsLocked)
        #expect(await AppTest.waitUntil(5) { model.statusMessage != nil || model.alert != nil }, "no report of the recording")
        #expect(model.alert == nil, "recording failed: \(model.alert?.message ?? "")")
        #expect(AppTest.fileSize(file) > 0, "the recording is empty")
        #expect(model.revealableURL == file, "Show in Finder does not point at the recording")
        #expect(log.sources[0].shutdownFinished != nil, "the helper was not shut down")
    }

    /// Quitting during a recording delivers the file before the reply.
    @Test(.enabled(if: AppTest.helperAvailable, AppTest.helperSkipReason))
    func fakeCameraQuitDuringRecording() async throws {
        let (model, log) = AppTest.makeModel()
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("quit.db3")
        try await AppTest.openAndWaitForFrames(model, .live)
        await model.startRecording(to: file)
        try #require(model.isRecording, "recording did not start: \(model.alert?.message ?? "")")
        try? await Task.sleep(for: .seconds(1))
        let work = try #require(model.beginTermination(), "nothing to wait for during a recording")
        #expect(await work.wait(cameraTimeout: 20), "termination wait timed out")
        #expect(AppTest.fileSize(file) > 0, "the recording is empty after quitting")
        #expect(log.sources[0].shutdownFinished != nil, "the helper was not shut down")
    }

    /// The app's live flow on the synthetic head: guided scan to the end, reconstruction, OBJ export in millimeters.
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_APP_FULL_SCAN"] == "1",
                   "the full guided scan takes minutes: set IFS_APP_FULL_SCAN=1"),
          .timeLimit(.minutes(30)))
    func fullSyntheticScan() async throws {
        let model = AppModel(defaults: nil)
        await model.openSource(.synthetic)
        try #require(await AppTest.waitUntil(30) { model.hasFrames }, "no frames")
        try #require(await AppTest.waitUntil(30) { model.tracking.faceRect != nil }, "no face")
        model.startScan()
        var rejections: [String: Int] = [:]
        let finished = await AppTest.waitUntil(240) {
            if let r = model.scanUpdate?.rejection { rejections["\(r)", default: 0] += 1 }
            return model.screen != .scanning
        }
        if !finished { model.finishScan() }
        let keyframes = model.capture?.keyframes ?? []
        try #require(finished, "the guided scan did not complete in 240 s (\(keyframes.count) keyframes, rejections \(rejections))")
        try #require(model.screen == .scanComplete, "screen \(model.screen) after the scan")
        model.reconstruct()
        try #require(await AppTest.waitUntil(1200) { model.screen != .reconstructing }, "reconstruction did not end in 20 min")
        try #require(model.screen == .result, "reconstruction failed: \(model.alert?.message ?? "")")
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let obj = dir.appendingPathComponent("head.obj")
        await model.exportOBJ(to: obj)
        #expect(model.alert == nil, "export failed: \(model.alert?.message ?? "")")
        let text = try String(contentsOf: obj, encoding: .utf8)
        var lo = SIMD3<Float>(repeating: .infinity), hi = SIMD3<Float>(repeating: -.infinity)
        for line in text.split(separator: "\n") where line.hasPrefix("v ") {
            let p = line.split(separator: " ").dropFirst().prefix(3).compactMap { Float($0) }
            guard p.count == 3 else { continue }
            let v = SIMD3(p[0], p[1], p[2])
            lo = simd_min(lo, v); hi = simd_max(hi, v)
        }
        let size = hi - lo
        #expect(size.y > 120 && size.y < 400 && size.x > 100 && size.x < 300, "not a head in millimeters: \(size)")
        await model.shutdownForTermination()
    }
}
