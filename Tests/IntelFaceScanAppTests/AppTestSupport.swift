// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared pieces of the app tests: a source wrapper that records its shutdown, a model with in-memory settings whose
// sources are recorded, the helper's fake camera, waiting and temporary folders.

import CameraKit
import Foundation
import SyntheticData
import Testing
@testable import IntelFaceScanApp

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

/// Sources created by a model's factory, in order.
@MainActor
final class SourceLog {
    var sources: [TrackedSource] = []
}

enum AppTest {
    /// $IFS_HELPER_PATH or build/helper/IntelFaceScanHelper of this checkout; the fake-camera tests are skipped
    /// without it (the synthetic head stands in where it can).
    static let helperURL: URL? = {
        if let path = ProcessInfo.processInfo.environment["IFS_HELPER_PATH"], !path.isEmpty,
           FileManager.default.isExecutableFile(atPath: path) {
            return URL(fileURLWithPath: path)
        }
        let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent()
            .deletingLastPathComponent()
        let built = root.appendingPathComponent("build/helper/IntelFaceScanHelper")
        return FileManager.default.isExecutableFile(atPath: built.path) ? built : nil
    }()

    static let helperAvailable = helperURL != nil
    static let helperSkipReason: Comment =
        "IntelFaceScanHelper not built: run the helper CMake build (build/helper) or set IFS_HELPER_PATH"

    /// A model with in-memory settings whose sources are recorded: `.live` is the helper's fake camera, `.synthetic`
    /// the synthetic head at 640×400.
    @MainActor
    static func makeModel(shutdownDelay: Double = 0) -> (AppModel, SourceLog) {
        let log = SourceLog()
        let model = AppModel(defaults: nil) { request in
            let base: CameraSource
            switch request {
            case .live:
                base = helperURL.map { FakeCameraSource(helperURL: $0) } ?? FakeCameraSource()
            case .synthetic:
                var options = SyntheticCameraSource.defaultOptions
                options.width = 640
                options.height = 400
                base = SyntheticCameraSource(options: options)
            default:
                base = AppModel.makeSource(request)
            }
            let tracked = TrackedSource(base, shutdownDelay: shutdownDelay)
            log.sources.append(tracked)
            return tracked
        }
        return (model, log)
    }

    /// Polls `condition` every 50 ms; false when it is still false after `timeout` seconds.
    @MainActor
    static func waitUntil(_ timeout: Double, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while !condition() {
            if Date() > deadline { return false }
            try? await Task.sleep(for: .milliseconds(50))
        }
        return true
    }

    /// Opens a source and waits for the first analyzed frame.
    @MainActor
    static func openAndWaitForFrames(_ model: AppModel, _ request: SourceRequest, timeout: Double = 60) async throws {
        await model.openSource(request)
        try #require(model.alert == nil, "opening the source failed: \(model.alert?.message ?? "")")
        try #require(model.screen == .preview, "screen \(model.screen) after opening, expected preview")
        try #require(await waitUntil(timeout) { model.hasFrames }, "no analyzed frame within \(timeout) s")
    }

    static func temporaryDirectory() throws -> URL {
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("IntelFaceScanAppTests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
        return url
    }

    static func fileSize(_ url: URL) -> Int {
        (try? FileManager.default.attributesOfItem(atPath: url.path)[.size] as? Int) ?? 0
    }

    /// The value of `body` in both UI languages.
    static func inBothLanguages<T>(_ body: () -> T) -> (en: T, pl: T) {
        (AppLanguage.$override.withValue(.english, operation: body),
         AppLanguage.$override.withValue(.polish, operation: body))
    }
}
