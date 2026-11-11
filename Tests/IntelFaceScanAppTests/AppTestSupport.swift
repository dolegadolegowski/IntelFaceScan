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

extension Trait where Self == TimeLimitTrait {
    /// For the tests that open a source. Their waits are bounded well below it (AppTest.openAndWaitForFrames,
    /// waitUntil, cleanUp); the limit cancels a test whose wait is not. Per test, not on the suite: the shortest
    /// limit wins, and the opt-in full scan needs 30 minutes.
    static var sourceTestLimit: Self { .timeLimit(.minutes(5)) }
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

    /// A camera service that is installed and ready, with no files behind it: a `.live` request goes straight to the
    /// source factory, and nothing probes, installs or prompts.
    static let readyCameraService = CameraService(
        kit: CameraService.Kit(daemon: URL(fileURLWithPath: "/nonexistent/realsense-usbd"),
                               manifest: URL(fileURLWithPath: "/nonexistent/service.plist"),
                               installScript: URL(fileURLWithPath: "/nonexistent/install-camera-access.sh")),
        state: { _ in .ready(serviceVersion: 2) },
        install: { _, _ in Issue.record("a test ran the camera service installer") },
        installedServiceVersion: { 2 },
        openLoginItemsSettings: { Issue.record("a test opened System Settings") })

    /// A model with in-memory settings whose sources are recorded: `.live` is the helper's fake camera, `.synthetic`
    /// the synthetic head at 640×400. The camera service is a stand-in that is ready.
    @MainActor
    static func makeModel(shutdownDelay: Double = 0) -> (AppModel, SourceLog) {
        let log = SourceLog()
        let model = AppModel(defaults: nil, cameraService: readyCameraService) { request, _ in
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
                base = AppModel.makeSource(request, liveAccess: .service)
            }
            let tracked = TrackedSource(base, shutdownDelay: shutdownDelay)
            log.sources.append(tracked)
            return tracked
        }
        return (model, log)
    }

    /// Polls `condition` every 50 ms; false when it is still false after `timeout` seconds or when the test was
    /// cancelled (its time limit ran out).
    @MainActor
    static func waitUntil(_ timeout: Double, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while !condition() {
            if Date() > deadline || Task.isCancelled { return false }
            try? await Task.sleep(for: .milliseconds(50))
        }
        return true
    }

    /// The timers of wait(for:timeout:). A queue of its own always gets a thread; a global dispatch queue gets none
    /// while as many threads of the Swift cooperative pool as the Mac has cores are in use.
    private static let timeoutQueue = DispatchQueue(label: "IntelFaceScanAppTests.timeouts")

    /// Waits for `task` at most `timeout` seconds; false when it did not end in time (it keeps running). The timeout
    /// is a dispatch timer on a queue of its own: it fires even when no thread of the Swift cooperative pool is free.
    static func wait(for task: Task<Void, Never>, timeout: Double) async -> Bool {
        await withCheckedContinuation { (continuation: CheckedContinuation<Bool, Never>) in
            let once = OnceFlag()
            Task {
                await task.value
                if once.claim() { continuation.resume(returning: true) }
            }
            // Not a global dispatch queue: its blocks do not start while the Swift cooperative pool is full,
            // which is when this timeout is needed.
            timeoutQueue.asyncAfter(deadline: .now() + timeout) {
                if once.claim() { continuation.resume(returning: false) }
            }
        }
    }

    /// Opens a source and waits for the first analyzed frame. Every step is bounded.
    @MainActor
    static func openAndWaitForFrames(_ model: AppModel, _ request: SourceRequest, timeout: Double = 60) async throws {
        let opening = Task { await model.openSource(request) }
        try #require(await wait(for: opening, timeout: timeout), "opening the source did not end within \(timeout) s")
        try #require(model.alert == nil, "opening the source failed: \(model.alert?.message ?? "")")
        try #require(model.screen == .preview, "screen \(model.screen) after opening, expected preview")
        try #require(await waitUntil(timeout) { model.hasFrames }, "no analyzed frame within \(timeout) s")
    }

    /// Closes the model's source and shuts down every source its factory made: call it on every exit path of a test
    /// that opens a source (`defer { await AppTest.cleanUp(model, log) }`). A test that stops at a failed #require
    /// would otherwise leave its camera source and frame pipeline running into the following tests. Bounded: a
    /// shutdown that does not end within `timeout` seconds is recorded instead of holding the test run.
    @MainActor
    static func cleanUp(_ model: AppModel, _ log: SourceLog, timeout: Double = 30,
                        sourceLocation: SourceLocation = #_sourceLocation) async {
        let cleanup = Task {
            await model.closeSource()
            await model.waitForClosingSources()
            for source in log.sources where source.shutdownFinished == nil { await source.shutdown() }
        }
        let ended = await wait(for: cleanup, timeout: timeout)
        #expect(ended, "the sources did not shut down within \(timeout) s", sourceLocation: sourceLocation)
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
