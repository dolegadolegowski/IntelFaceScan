// License: Apache 2.0. See LICENSE file in root directory.
//
// A CameraSource streaming the rendered synthetic head at the configured frame rate (default 30 fps), turning
// through the scan poses in a loop: ease from one pose target to the next, hold it with a slight sway, continue.
// For UI development and tests without a camera. Frames are rendered on demand at the configured color
// resolution; the animation follows the wall clock, so a slow (debug) build drops frames instead of slowing down.

import CameraKit
import CoreGraphics
import Foundation
import ScanKit
import simd

public final class SyntheticCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind = .synthetic
    public let options: SyntheticScan.Options
    /// Seconds to turn from one pose to the next, and to hold each pose.
    public let transitionSeconds: Double
    public let holdSeconds: Double

    private let lock = NSLock()
    private var scene: SyntheticScene?
    private var fps = 30
    private var filters = FilterConfiguration.faceScan
    private var frameContinuations: [UUID: AsyncStream<CameraFrame>.Continuation] = [:]
    private var eventContinuations: [UUID: AsyncStream<CameraEvent>.Continuation] = [:]
    private var streamTask: Task<Void, Never>?
    private var isShutdown = false
    private var sequence: UInt64 = 0
    private let clockOrigin = ProcessInfo.processInfo.systemUptime

    public init(options: SyntheticScan.Options = SyntheticScan.Options(), transitionSeconds: Double = 1.2,
                holdSeconds: Double = 1.5) {
        self.options = options
        self.transitionSeconds = max(0.05, transitionSeconds)
        self.holdSeconds = max(0, holdSeconds)
    }

    public func connect() async throws -> DeviceInfo {
        if lock.withLock({ isShutdown }) { throw CameraError.notConnected }
        let current = lock.withLock { scene }
        let ready: SyntheticScene
        if let current {
            ready = current
        } else {
            let options = self.options
            // Tessellating the head takes ~0.1 s the first time; keep it off the caller's executor.
            ready = await Task.detached(priority: .userInitiated) { SyntheticScene(options: options) }.value
            lock.withLock { if scene == nil { scene = ready } }
        }
        let info = SyntheticScene.deviceInfo(width: ready.options.width, height: ready.options.height, fps: lock.withLock { fps })
        emit(.connected(info))
        return info
    }

    /// Applies the color resolution and frame rate (the synthetic depth is always aligned to color).
    public func configure(_ configuration: CaptureConfiguration) async throws {
        let c = configuration.color
        guard c.width >= 16, c.height >= 16, c.width <= 4096, c.height <= 4096, c.fps >= 1, c.fps <= 120 else {
            throw CameraError.commandFailed("Unsupported synthetic stream \(c.width)x\(c.height) @ \(c.fps) fps")
        }
        let current = lock.withLock { scene }
        if current?.options.width != c.width || current?.options.height != c.height {
            var options = self.options
            options.width = c.width
            options.height = c.height
            let rebuilt = await Task.detached(priority: .userInitiated) { SyntheticScene(options: options) }.value
            lock.withLock { scene = rebuilt }
        }
        lock.withLock {
            fps = c.fps
            filters = configuration.filters
        }
    }

    public func setFilters(_ filters: FilterConfiguration) async throws {
        lock.withLock { self.filters = filters }
    }

    public func start() async throws {
        let launch: Bool = try lock.withLock {
            if isShutdown || scene == nil { throw CameraError.notConnected }
            return streamTask == nil
        }
        guard launch else { return }
        let task = Task.detached(priority: .userInitiated) { [weak self] () -> Void in
            guard let self else { return }
            await self.run()
        }
        lock.withLock { streamTask = task }
        emit(.streamingStarted)
    }

    public func stop() async throws {
        let task: Task<Void, Never>? = lock.withLock {
            defer { streamTask = nil }
            return streamTask
        }
        guard let task else { return }
        task.cancel()
        await task.value   // no frame is delivered after stop() returns
        emit(.streamingStopped)
    }

    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        emit(.log(.debug, "synthetic camera: \(sensor.rawValue) option \"\(name)\" = \(value) has no effect"))
    }

    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording a synthetic source") }
    public func stopRecording() async throws -> Int64 { throw CameraError.unsupported("recording a synthetic source") }
    public func hardwareReset() async throws {
        emit(.streamRestarted("synthetic camera reset"))
    }

    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        let id = UUID()
        return AsyncStream(bufferingPolicy: .bufferingNewest(max(1, bufferingNewest))) { continuation in
            let closed: Bool = lock.withLock {
                if isShutdown { return true }
                frameContinuations[id] = continuation
                return false
            }
            if closed { continuation.finish(); return }
            continuation.onTermination = { [weak self] _ in
                self?.lock.withLock { _ = self?.frameContinuations.removeValue(forKey: id) }
            }
        }
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
        if lock.withLock({ isShutdown }) { return }
        try? await stop()
        let (frames, events): ([AsyncStream<CameraFrame>.Continuation], [AsyncStream<CameraEvent>.Continuation]) = lock.withLock {
            isShutdown = true
            defer { frameContinuations.removeAll(); eventContinuations.removeAll() }
            return (Array(frameContinuations.values), Array(eventContinuations.values))
        }
        frames.forEach { $0.finish() }
        events.forEach { $0.finish() }
    }

    // MARK: Animation

    /// Head orientation and translation at animation time t (seconds); the sequence loops.
    public func headPose(at t: Double) -> (pose: HeadPose, translation: SIMD3<Float>) {
        let poses = options.poses.isEmpty ? [ScanPose.front] : options.poses
        let segment = transitionSeconds + holdSeconds
        let cycle = segment * Double(poses.count)
        let local = t.truncatingRemainder(dividingBy: cycle)
        let index = min(Int(local / segment), poses.count - 1)
        let within = local - Double(index) * segment
        let from = poses[(index + poses.count - 1) % poses.count].target, to = poses[index].target
        let blend = Float(min(within / transitionSeconds, 1))
        let eased = blend * blend * (3 - 2 * blend)
        let sway = Float(sin(t * 1.7)), nod = Float(sin(t * 1.3 + 0.5))
        let pose = HeadPose(yaw: from.yaw + (to.yaw - from.yaw) * eased + 1.5 * sway,
                            pitch: from.pitch + (to.pitch - from.pitch) * eased + 1.0 * nod,
                            roll: 0.8 * Float(sin(t * 0.9)))
        return (pose, SIMD3(0.001 * sway, 0.001 * nod, 0.0015 * Float(sin(t * 0.7))))
    }

    private func run() async {
        var next = ProcessInfo.processInfo.systemUptime
        while !Task.isCancelled {
            let (scene, fps, filters, sequence) = lock.withLock { () -> (SyntheticScene?, Int, FilterConfiguration, UInt64) in
                self.sequence += 1
                return (self.scene, self.fps, self.filters, self.sequence)
            }
            guard let scene else { return }
            let now = ProcessInfo.processInfo.systemUptime
            let animationTime = now - clockOrigin
            let (pose, translation) = headPose(at: animationTime)
            let timestamp = Date().timeIntervalSince1970 * 1000
            var view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose, translation: translation),
                                    frameIndex: sequence, sequence: sequence, timestampMs: timestamp)
            if filters.threshold.enabled {
                SyntheticCameraSource.applyThreshold(&view.frame.alignedDepth, min: filters.threshold.minM,
                                                     max: filters.threshold.maxM)
            }
            if Task.isCancelled { return }
            let continuations = lock.withLock { Array(frameContinuations.values) }
            for c in continuations { c.yield(view.frame) }

            // Fixed-rate schedule; when rendering falls behind, skip ahead instead of bursting.
            let period = 1.0 / Double(max(fps, 1))
            next += period
            let after = ProcessInfo.processInfo.systemUptime
            if next < after { next = after }
            do {
                try await Task.sleep(nanoseconds: UInt64((next - after) * 1e9))
            } catch {
                return
            }
        }
    }

    /// The helper's threshold filter: depth outside [min, max] meters becomes invalid.
    static func applyThreshold(_ depth: inout DepthImage, min: Float, max: Float) {
        let lo = UInt16(clamping: Int((min / depth.depthScale).rounded(.up)))
        let hi = UInt16(clamping: Int((max / depth.depthScale).rounded(.down)))
        depth.pixels.withUnsafeMutableBytes { raw in
            let values = raw.bindMemory(to: UInt16.self)
            for i in 0 ..< values.count {
                let v = UInt16(littleEndian: values[i])
                if v != 0 && (v < lo || v > hi) { values[i] = 0 }
            }
        }
    }

    private func emit(_ event: CameraEvent) {
        let continuations = lock.withLock { Array(eventContinuations.values) }
        for c in continuations { c.yield(event) }
    }
}
