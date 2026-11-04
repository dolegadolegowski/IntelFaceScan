// License: Apache 2.0. See LICENSE file in root directory.
//
// Moves frames from the camera source to the Metal preview and to the face analysis (Vision + scan state
// machine). Both consumers keep only the newest frame: a slow analysis never delays the preview, and neither
// builds a backlog. Results are published to the main actor, coalesced to the newest one; the frame tasks never
// wait for the main actor, so stop() finishes even while the main queue cannot run (see AppModel termination).

import CameraKit
import CoreGraphics
import Foundation
import QuartzCore
import ScanKit

struct FrameStats: Equatable {
    var displayFPS: Double = 0
    var analysisFPS: Double = 0
}

/// What the analysis of one frame produced, delivered on the main actor.
struct AnalysisResult: Sendable {
    var update: ScanUpdate
    var imageSize: CGSize
    var stats: FrameStats
    /// The controller that produced the update (scan updates of a replaced controller are ignored).
    var controllerID: ObjectIdentifier
}

/// Frames per second over the last second.
struct RateCounter {
    private var times: [CFTimeInterval] = []

    mutating func tick(_ now: CFTimeInterval = CACurrentMediaTime()) {
        times.append(now)
        times.removeAll { now - $0 > 1 }
    }

    func rate(_ now: CFTimeInterval = CACurrentMediaTime()) -> Double {
        guard let first = times.first, times.count > 1 else { return 0 }
        let span = max(now - first, 1e-3)
        return now - first > 1.5 ? 0 : Double(times.count - 1) / span
    }
}

final class FramePipeline: @unchecked Sendable {
    private let source: CameraSource
    private let renderer: PreviewRenderer?
    private let lock = NSLock()
    private var controller: ScanController
    private var tasks: [Task<Void, Never>] = []
    private var displayRate = RateCounter()
    private var analysisRate = RateCounter()
    /// false while no screen shows the preview (scan summary, reconstruction, result): frames are dropped instead
    /// of uploaded and analyzed, so the reconstruction does not compete with Vision for the CPU.
    private var active = true
    private var stopped = false
    private var pendingResult: AnalysisResult?
    private var deliveryScheduled = false

    init(source: CameraSource, renderer: PreviewRenderer?, controller: ScanController) {
        self.source = source
        self.renderer = renderer
        self.controller = controller
    }

    /// The controller that analyzes the next frames (idle for the preview, started for a scan).
    func setController(_ controller: ScanController) {
        lock.withLock { self.controller = controller }
    }

    func setActive(_ active: Bool) {
        lock.withLock { self.active = active }
    }

    var isActive: Bool { lock.withLock { active } }

    func start(onResult: @escaping @MainActor @Sendable (AnalysisResult) -> Void,
               onEnd: @escaping @MainActor @Sendable () -> Void) {
        let (analysisFrames, analysisInput) = AsyncStream.makeStream(of: CameraFrame.self,
                                                                     bufferingPolicy: .bufferingNewest(1))
        let frames = source.frames(bufferingNewest: 2)
        let distribute = Task.detached(priority: .userInitiated) { [weak self, renderer] in
            for await frame in frames {
                guard let self, !Task.isCancelled else { break }
                let use = self.lock.withLock { () -> Bool in
                    guard self.active, !self.stopped else { return false }
                    self.displayRate.tick()
                    return true
                }
                guard use else { continue }
                renderer?.submit(frame)
                analysisInput.yield(frame)
            }
            analysisInput.finish()
            if !Task.isCancelled { Task { @MainActor in onEnd() } }
        }
        let analyze = Task.detached(priority: .userInitiated) { [weak self] in
            for await frame in analysisFrames {
                guard let self, !Task.isCancelled else { break }
                let controller = self.lock.withLock { self.controller }
                var update = await controller.process(frame)
                // Masks are megabytes and the UI does not draw them.
                update.observation?.personMask = nil
                let stats = self.lock.withLock { () -> FrameStats in
                    self.analysisRate.tick()
                    return FrameStats(displayFPS: self.displayRate.rate(), analysisFPS: self.analysisRate.rate())
                }
                let result = AnalysisResult(update: update,
                                            imageSize: CGSize(width: frame.color.width, height: frame.color.height),
                                            stats: stats, controllerID: ObjectIdentifier(controller))
                self.deliver(result, to: onResult)
            }
        }
        lock.withLock { tasks = [distribute, analyze] }
    }

    /// Hands the newest result to the main actor: at most one hop is pending, a newer result replaces an
    /// undelivered one (the scan state lives in the controller, an update is only what the UI shows).
    private func deliver(_ result: AnalysisResult, to onResult: @escaping @MainActor @Sendable (AnalysisResult) -> Void) {
        let schedule = lock.withLock { () -> Bool in
            guard !stopped else { return false }
            pendingResult = result
            if deliveryScheduled { return false }
            deliveryScheduled = true
            return true
        }
        guard schedule else { return }
        Task { @MainActor [weak self] in
            guard let self else { return }
            let latest = self.lock.withLock { () -> AnalysisResult? in
                self.deliveryScheduled = false
                defer { self.pendingResult = nil }
                return self.stopped ? nil : self.pendingResult
            }
            if let latest { onResult(latest) }
        }
    }

    /// Stops both consumers without waiting; no result is delivered afterwards.
    func cancel() {
        let running = lock.withLock { () -> [Task<Void, Never>] in
            stopped = true
            pendingResult = nil
            return tasks
        }
        for task in running { task.cancel() }
    }

    /// Stops both consumers and waits for them (never for the main actor).
    func stop() async {
        cancel()
        let running = lock.withLock { tasks }
        for task in running { await task.value }
    }
}
