// License: Apache 2.0. See LICENSE file in root directory.
//
// Moves frames from the camera source to the Metal preview and to the face analysis (Vision + scan state
// machine). Both consumers keep only the newest frame: a slow analysis never delays the preview, and neither
// builds a backlog. Results are published to the main actor.

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

    init(source: CameraSource, renderer: PreviewRenderer?, controller: ScanController) {
        self.source = source
        self.renderer = renderer
        self.controller = controller
    }

    /// The controller that analyzes the next frames (idle for the preview, started for a scan).
    func setController(_ controller: ScanController) {
        lock.withLock { self.controller = controller }
    }

    func start(onResult: @escaping @MainActor @Sendable (AnalysisResult) -> Void,
               onEnd: @escaping @MainActor @Sendable () -> Void) {
        let (analysisFrames, analysisInput) = AsyncStream.makeStream(of: CameraFrame.self,
                                                                     bufferingPolicy: .bufferingNewest(1))
        let frames = source.frames(bufferingNewest: 2)
        let distribute = Task.detached(priority: .userInitiated) { [weak self, renderer] in
            for await frame in frames {
                if Task.isCancelled { break }
                renderer?.submit(frame)
                self?.lock.withLock { self?.displayRate.tick() }
                analysisInput.yield(frame)
            }
            analysisInput.finish()
            if !Task.isCancelled { await onEnd() }
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
                await onResult(result)
            }
        }
        lock.withLock { tasks = [distribute, analyze] }
    }

    /// Stops both consumers and waits for them.
    func stop() async {
        let running = lock.withLock { () -> [Task<Void, Never>] in
            let t = tasks
            tasks = []
            return t
        }
        for task in running { task.cancel() }
        for task in running { await task.value }
    }
}
