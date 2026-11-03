// License: Apache 2.0. See LICENSE file in root directory.
//
// Replays the frames of a ScanDataset folder as a CameraSource, in order.
//
// Frames are pulled by each consumer (AsyncStream(unfolding:)), which gives the two behaviors a recording source
// needs without a producer task: offline (realTime = false) every frame is delivered as fast as the consumer reads
// it, nothing is dropped; real time, frames are released at their recorded time offsets (gaps capped at
// maxFrameGapSeconds, since keyframes of a scan can be seconds apart) and a slow consumer skips to the newest
// `bufferingNewest` frames, like a live camera. Every start() replays from the first frame. Without `loop`, a
// consumer's stream finishes after the last frame and `.playbackFinished` is sent.

import CameraKit
import CoreGraphics
import Foundation

public final class DatasetCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind
    public let directory: URL
    public let realTime: Bool
    public let loop: Bool
    public let maxFrameGapSeconds: Double

    private let lock = NSLock()
    private var capture: ScanCapture?
    private var offsets: [Double] = []      // playback time of each frame (seconds from the first)
    private var running = false
    private var isShutdown = false
    private var generation = 0
    private var startTime: Double = 0
    private var finishedGeneration = -1
    private var configuration: CaptureConfiguration?
    private var eventContinuations: [UUID: AsyncStream<CameraEvent>.Continuation] = [:]

    public init(directory: URL, realTime: Bool = true, loop: Bool = false, maxFrameGapSeconds: Double = 2) {
        self.directory = directory
        self.kind = .dataset(directory)
        self.realTime = realTime
        self.loop = loop
        self.maxFrameGapSeconds = max(0, maxFrameGapSeconds)
    }

    /// The loaded dataset (after connect()).
    public var loadedCapture: ScanCapture? { lock.withLock { capture } }

    public func connect() async throws -> DeviceInfo {
        if lock.withLock({ isShutdown }) { throw CameraError.notConnected }
        let directory = self.directory
        let loaded = try await Task.detached(priority: .userInitiated) { try ScanDataset.read(from: directory) }.value
        guard !loaded.keyframes.isEmpty else {
            throw CameraError.commandFailed("The dataset \(directory.lastPathComponent) has no frames.")
        }
        var times: [Double] = [0]
        let frames = loaded.keyframes.map(\.frame)
        for i in 1 ..< max(frames.count, 1) {
            var gap = (frames[i].timestampMs - frames[i - 1].timestampMs) / 1000
            if !(gap > 0) { gap = 1.0 / 30 }   // missing or non-increasing timestamps: 30 fps
            times.append(times[i - 1] + min(gap, maxFrameGapSeconds))
        }
        let first = frames[0].color
        let device = loaded.device ?? DatasetCameraSource.placeholderDevice(width: first.width, height: first.height)
        lock.withLock {
            capture = loaded
            offsets = times
            configuration = loaded.configuration
        }
        emit(.connected(device))
        return device
    }

    public func configure(_ configuration: CaptureConfiguration) async throws {
        // Recorded frames are already filtered and aligned; the configuration is only remembered.
        lock.withLock { self.configuration = configuration }
    }

    public func setFilters(_ filters: FilterConfiguration) async throws {}

    public func start() async throws {
        let started: Bool = try lock.withLock {
            if isShutdown || capture == nil { throw CameraError.notConnected }
            if running { return false }
            running = true
            generation += 1
            startTime = ProcessInfo.processInfo.systemUptime
            return true
        }
        if started { emit(.streamingStarted) }
    }

    public func stop() async throws {
        let stopped: Bool = lock.withLock {
            defer { running = false }
            return running
        }
        if stopped { emit(.streamingStopped) }
    }

    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        throw CameraError.unsupported("options of a recorded dataset")
    }
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording a dataset") }
    public func stopRecording() async throws -> Int64 { throw CameraError.unsupported("recording a dataset") }
    public func hardwareReset() async throws {}

    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        let cursor = Cursor()
        let limit = max(1, bufferingNewest)
        return AsyncStream(unfolding: { [weak self] in
            guard let self else { return nil }
            return await self.next(cursor, limit: limit)
        })
    }

    public func events() -> AsyncStream<CameraEvent> {
        let id = UUID()
        return AsyncStream(bufferingPolicy: .unbounded) { continuation in
            let closed: Bool = lock.withLock {
                if isShutdown { return true }
                eventContinuations[id] = continuation
                return false
            }
            if closed { continuation.finish(); return }
            continuation.onTermination = { [weak self] _ in
                self?.lock.withLock { _ = self?.eventContinuations.removeValue(forKey: id) }
            }
        }
    }

    public func shutdown() async {
        let continuations: [AsyncStream<CameraEvent>.Continuation] = lock.withLock {
            if isShutdown { return [] }
            isShutdown = true
            running = false
            defer { eventContinuations.removeAll() }
            return Array(eventContinuations.values)
        }
        for c in continuations {
            c.yield(.streamingStopped)
            c.finish()
        }
    }

    // MARK: Replay

    private final class Cursor: @unchecked Sendable {
        var generation = -1
        var frame = 0      // global frame number (cycles * count + index)
    }

    private func next(_ cursor: Cursor, limit: Int) async -> CameraFrame? {
        while true {
            if Task.isCancelled { return nil }
            let state = lock.withLock { (isShutdown, running, generation, startTime, capture?.keyframes, offsets) }
            let (shutdown, isRunning, currentGeneration, start, keyframes, times) = state
            if shutdown { return nil }
            guard isRunning, let keyframes, !keyframes.isEmpty else {
                if !(await pause(0.01)) { return nil }
                continue
            }
            if cursor.generation != currentGeneration {
                cursor.generation = currentGeneration
                cursor.frame = 0
            }
            let count = keyframes.count
            if realTime {
                let cycle = (times.last ?? 0) + 1.0 / 30
                let elapsed = ProcessInfo.processInfo.systemUptime - start
                // The newest frame that is due, then skip what a live camera would have dropped.
                let round = loop ? Int(elapsed / cycle) : 0
                let latest = round * count + lastIndex(times, atOrBefore: elapsed - Double(round) * cycle)
                cursor.frame = max(cursor.frame, latest - limit + 1)
                if !loop && cursor.frame >= count { return finish(currentGeneration) }
                let due = Double(cursor.frame / count) * cycle + times[cursor.frame % count]
                if due > elapsed {
                    if !(await pause(min(due - elapsed, 0.05))) { return nil }
                    continue
                }
            } else if cursor.frame >= count {
                if !loop { return finish(currentGeneration) }
            }
            let frame = keyframes[cursor.frame % count].frame
            cursor.frame += 1
            return frame
        }
    }

    private func finish(_ generation: Int) -> CameraFrame? {
        let first: Bool = lock.withLock {
            defer { finishedGeneration = max(finishedGeneration, generation) }
            return finishedGeneration < generation
        }
        if first { emit(.playbackFinished) }
        return nil
    }

    /// Last index whose offset is <= t (-1 if none).
    private func lastIndex(_ times: [Double], atOrBefore t: Double) -> Int {
        var lo = 0, hi = times.count   // first index with times[i] > t
        while lo < hi {
            let mid = (lo + hi) / 2
            if times[mid] <= t { lo = mid + 1 } else { hi = mid }
        }
        return lo - 1
    }

    /// Sleeps; false when the waiting task was cancelled.
    private func pause(_ seconds: Double) async -> Bool {
        do {
            try await Task.sleep(nanoseconds: UInt64(max(seconds, 0.001) * 1e9))
            return true
        } catch {
            return false
        }
    }

    private func emit(_ event: CameraEvent) {
        let continuations = lock.withLock { Array(eventContinuations.values) }
        for c in continuations { c.yield(event) }
    }

    /// DeviceInfo for datasets saved without one. DeviceInfo has no public memberwise initializer, so it is decoded
    /// from its protocol JSON (docs/PROTOCOL.md, QUERY_DEVICE).
    static func placeholderDevice(width: Int, height: Int) -> DeviceInfo {
        let json: [String: Any] = [
            "name": "IntelFaceScan dataset", "serial": "", "firmware": "", "usb_type": "", "product_id": "",
            "product_line": "", "advanced_mode": false, "depth_scale": 0.0001, "imu": false, "source": "playback",
            "presets": [String](), "sensors": [[String: Any]](),
            "profiles": [
                ["stream": "depth", "index": 0, "format": "z16", "width": width, "height": height, "fps": 30],
                ["stream": "color", "index": 0, "format": "rgb8", "width": width, "height": height, "fps": 30],
            ],
        ]
        do {
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            return try decoder.decode(DeviceInfo.self, from: JSONSerialization.data(withJSONObject: json))
        } catch {
            preconditionFailure("placeholder DeviceInfo JSON does not match CameraTypes.swift: \(error)")
        }
    }
}
