// License: Apache 2.0. See LICENSE file in root directory.
//
// Runs the guided scan offline over a .db3/.bag recording: the helper plays the file (as the user, no password,
// every frame, no real-time pacing), the frames go through the same post-processing as live, and ScanController
// (ScanConfiguration.offline) accepts keyframes exactly as it would in the app.

import CameraKit
import Foundation
import ScanKit

public enum RecordingScan {
    public struct Result: Sendable {
        public var capture: ScanCapture
        public var device: DeviceInfo
        public var framesProcessed: Int
        /// true when the guided scan reached .finished (all poses captured) before the recording ended.
        public var completed: Bool
        public var failure: String?
    }

    public static func run(recording: URL, helper: URL, configuration: ScanConfiguration,
                           log: @escaping @Sendable (String) -> Void) async throws -> Result {
        let source = PlaybackCameraSource(file: recording, helperURL: helper, realTime: false)
        let device: DeviceInfo
        do {
            device = try await source.connect()
        } catch {
            await source.shutdown()
            throw error
        }

        // Keep the recording's own stream profiles; the rest (preset-independent filters, alignment) is the
        // face-scan configuration, like a live scan.
        var capture = CaptureConfiguration.faceScanD455
        if let color = device.profiles.first(where: { $0.stream == "color" && $0.width > 0 }) {
            capture.color = StreamSettings(width: color.width, height: color.height, fps: color.fps)
        }
        if let depth = device.profiles.first(where: { $0.stream == "depth" && $0.width > 0 }) {
            capture.depth = StreamSettings(width: depth.width, height: depth.height, fps: depth.fps)
        }
        do {
            try await source.configure(capture)
        } catch {
            log("warning: the recording does not accept the face-scan configuration (\(error.localizedDescription)); using its own settings")
        }

        let controller = ScanController(configuration: configuration)
        controller.start(device: device, captureConfiguration: capture)
        let monitor = Monitor()
        let frames = source.frames(bufferingNewest: 16)
        let events = source.events()
        do {
            try await source.start()
        } catch {
            await source.shutdown()
            throw error
        }

        await withTaskGroup(of: Void.self) { group in
            group.addTask {
                for await frame in frames {
                    monitor.begin()
                    let update = await controller.process(frame)
                    monitor.end()
                    if update.phase == .finished {
                        monitor.complete()
                        return
                    }
                    if case .failed(let reason) = update.phase {
                        monitor.fail(reason)
                        return
                    }
                }
            }
            group.addTask {
                for await event in events {
                    switch event {
                    case .playbackFinished, .helperExited, .disconnected:
                        // Frames already received are still being processed: wait until the consumer has been
                        // idle (waiting for frames) for a second before ending the scan.
                        while !Task.isCancelled && !monitor.idle(for: 1.0) {
                            try? await Task.sleep(nanoseconds: 100_000_000)
                        }
                        return
                    case .error(let message):
                        log("camera: \(message)")
                    case .log(let level, let message) where level == .warning || level == .error:
                        log("helper \(level.rawValue): \(message)")
                    default:
                        break
                    }
                }
            }
            await group.next()
            group.cancelAll()
        }

        try? await source.stop()
        await source.shutdown()
        var result = controller.finish()
        result.notes["source"] = recording.lastPathComponent
        if result.device == nil { result.device = device }
        if result.configuration == nil { result.configuration = capture }
        let state = monitor.state
        return Result(capture: result, device: device, framesProcessed: state.frames, completed: state.completed,
                      failure: state.failure)
    }

    private final class Monitor: @unchecked Sendable {
        struct State { var frames = 0; var busy = false; var lastActivity = ProcessInfo.processInfo.systemUptime
                       var completed = false; var failure: String? }
        private let lock = NSLock()
        private var current = State()

        var state: State { lock.withLock { current } }
        func begin() { lock.withLock { current.busy = true; current.lastActivity = ProcessInfo.processInfo.systemUptime } }
        func end() {
            lock.withLock {
                current.busy = false
                current.frames += 1
                current.lastActivity = ProcessInfo.processInfo.systemUptime
            }
        }
        func complete() { lock.withLock { current.completed = true } }
        func fail(_ reason: String) { lock.withLock { current.failure = reason } }
        func idle(for seconds: Double) -> Bool {
            lock.withLock { !current.busy && ProcessInfo.processInfo.systemUptime - current.lastActivity >= seconds }
        }
    }
}
