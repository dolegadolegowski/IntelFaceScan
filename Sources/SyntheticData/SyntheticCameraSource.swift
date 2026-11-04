// License: Apache 2.0. See LICENSE file in root directory.
//
// A CameraSource streaming the rendered synthetic head at the configured frame rate (default 30 fps), turning
// through the scan poses in a loop: ease from one pose target to the next, hold it with a slight sway, turn a bit
// further and hold again (like a user who keeps turning while the guidance says so), continue with the next pose.
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
    /// Seconds to turn from one pose to the next, and to hold each stop of a pose.
    public let transitionSeconds: Double
    public let holdSeconds: Double
    /// The stops of every pose but the front: the target scaled by these factors, in this order (default: left at
    /// 45, 49.5 and 54 degrees, down at -25, -27.5 and -30). Vision reads the synthetic head's large turns short and
    /// not monotonically (at 1280x800: yaw +45 -> ~30, +50 -> ~39, +55 -> ~53; pitch -25 -> ~-23 but -30 -> ~-12;
    /// GuidanceSyntheticVisionStudy): a head that stopped exactly at the targets rarely completed "left" and was slow
    /// on "down", and turning much further does not help "down". A real user keeps turning while asked to.
    public let amplitudes: [Float]
    /// Seconds to turn from one stop of a pose to the next (a few degrees: below the guided scan's motion gate).
    public let stepSeconds: Double
    /// Animation stops in order: head yaw/pitch (degrees) and the time to ease there from the previous stop.
    private let stops: [(yaw: Float, pitch: Float, transition: Double)]

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
    /// Frames are rendered here, off the Swift cooperative pool and below the priority of the face analysis. A debug
    /// build renders 1280x800 slower than 30 fps, so the loop renders back to back: on a pool thread at
    /// userInitiated, with its concurrentPerform bands, it took the threads that Vision's own work needs (see
    /// ScanController.processingQueue), and on a 3-core machine no frame was analyzed any more.
    private let renderQueue = DispatchQueue(label: "IntelFaceScan.SyntheticCameraSource.render", qos: .utility)

    /// Defaults for UI work: stronger shading than the reconstruction datasets. Apple Vision needs the shading cues:
    /// with lighting 0.25 it misses the face in the down and +-45 degree poses and person segmentation fails; at 0.6
    /// every scan pose is detected with the right yaw sign (measured with ScanKit's HeadTracker). What Vision reads
    /// depends on the resolution: at 640x400 yaw +45 reads ~46 and -45 ~-52, and person segmentation covers the face in
    /// nearly every frame; at 1280x800 +45 reads ~30 and -45 ~-50, and segmentation covers the frontal face in only
    /// about half of the frames (HeadMaskBuilder then falls back to the face-rectangle ellipse).
    public static var defaultOptions: SyntheticScan.Options {
        var options = SyntheticScan.Options()
        options.lighting = 0.6
        return options
    }

    public init(options: SyntheticScan.Options = SyntheticCameraSource.defaultOptions, transitionSeconds: Double = 1.2,
                holdSeconds: Double = 1.5, amplitudes: [Float] = [1, 1.1, 1.2], stepSeconds: Double = 0.5) {
        self.options = options
        self.transitionSeconds = max(0.05, transitionSeconds)
        self.holdSeconds = max(0, holdSeconds)
        self.amplitudes = amplitudes.isEmpty ? [1] : amplitudes
        self.stepSeconds = max(0.05, stepSeconds)
        var stops: [(yaw: Float, pitch: Float, transition: Double)] = []
        for pose in options.poses.isEmpty ? [ScanPose.front] : options.poses {
            let target = pose.target
            let factors: [Float] = target.yaw == 0 && target.pitch == 0 ? [1] : self.amplitudes
            for (k, factor) in factors.enumerated() {
                stops.append((target.yaw * factor, target.pitch * factor, k == 0 ? self.transitionSeconds : self.stepSeconds))
            }
        }
        self.stops = stops
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

    /// Seconds of one pass through every pose and its stops.
    public var cycleSeconds: Double { stops.reduce(0) { $0 + $1.transition + holdSeconds } }

    /// Head orientation and translation at animation time t (seconds); the sequence loops.
    public func headPose(at t: Double) -> (pose: HeadPose, translation: SIMD3<Float>) {
        var local = max(0, t).truncatingRemainder(dividingBy: max(1e-6, cycleSeconds))
        var index = 0
        while index < stops.count - 1, local >= stops[index].transition + holdSeconds {
            local -= stops[index].transition + holdSeconds
            index += 1
        }
        let from = stops[(index + stops.count - 1) % stops.count], to = stops[index]
        let blend = Float(min(local / to.transition, 1))
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
            let frame = await withCheckedContinuation { continuation in
                renderQueue.async {
                    continuation.resume(returning: self.renderFrame(scene, filters: filters, sequence: sequence))
                }
            }
            if Task.isCancelled { return }
            let continuations = lock.withLock { Array(frameContinuations.values) }
            for c in continuations { c.yield(frame) }

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

    /// The frame at the current wall-clock time (renderQueue).
    private func renderFrame(_ scene: SyntheticScene, filters: FilterConfiguration, sequence: UInt64) -> CameraFrame {
        let animationTime = ProcessInfo.processInfo.systemUptime - clockOrigin
        let (pose, translation) = headPose(at: animationTime)
        let timestamp = Date().timeIntervalSince1970 * 1000
        var view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose, translation: translation),
                                frameIndex: sequence, sequence: sequence, timestampMs: timestamp)
        if filters.threshold.enabled {
            SyntheticCameraSource.applyThreshold(&view.frame.alignedDepth, min: filters.threshold.minM,
                                                 max: filters.threshold.maxM)
        }
        return view.frame
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
