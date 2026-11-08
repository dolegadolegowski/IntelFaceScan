// License: Apache 2.0. See LICENSE file in root directory.
//
// The live RealSense camera through the privileged helper (docs/HELPER.md). connect() shows the macOS administrator
// prompt once per helper; shutdown() makes the helper stop the streams, hand the camera back to macOS (it works as a
// webcam again) and exit.
//
// The prompt comes from AppleScript's "do shell script ... with administrator privileges". In an AppKit application
// (IntelFaceScan.app) it runs in-process and the prompt is the app's; elsewhere (a command line tool) it runs through
// /usr/bin/osascript, because an in-process "do shell script" can hang without an AppKit event loop
// (HelperLauncher). An AppKit application whose main run loop is not running (NSApplication created, never run) is
// not supported.

import CoreGraphics
import Foundation

public final class LiveCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind = .live
    public let helperURL: URL
    private let session: HelperCameraSession

    public init(helperURL: URL = CameraSources.bundledHelperURL) {
        self.helperURL = helperURL
        session = HelperCameraSession(mode: .live, helperURL: helperURL)
    }

    /// Starts the helper (administrator prompt; CameraError.authorizationCancelled when the user cancels it) and
    /// returns the device. When the helper runs but no camera is connected the error comes from the helper, and a later
    /// connect() asks again without a new prompt.
    public func connect() async throws -> DeviceInfo { try await session.connect() }
    public func configure(_ configuration: CaptureConfiguration) async throws { try await session.configure(configuration) }
    public func setFilters(_ filters: FilterConfiguration) async throws { try await session.setFilters(filters) }
    public func start() async throws { try await session.start() }
    public func stop() async throws { try await session.stop() }
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        try await session.setOption(sensor: sensor, name: name, value: value)
    }
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {
        try await session.setAutoExposureROI(rect, distanceMeters: distanceMeters)
    }
    /// `url` is created (truncated) by the app; ".bag" records a rosbag, anything else ".db3".
    public func startRecording(to url: URL) async throws { try await session.startRecording(to: url) }
    public func stopRecording() async throws -> Int64 { try await session.stopRecording() }
    public func hardwareReset() async throws { try await session.hardwareReset() }
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        session.frames.subscribe(bufferingNewest: bufferingNewest)
    }
    public func events() -> AsyncStream<CameraEvent> { session.events.subscribe(bufferingNewest: 256) }

    /// Asks the helper to release the camera and exit; waits for it (at most `timeout`, which covers handing the
    /// camera back to macOS). Safe from applicationShouldTerminate: nothing here runs on the main thread.
    public func shutdown() async { await session.shutdown(timeout: 20) }
    public func shutdown(timeout: TimeInterval) async { await session.shutdown(timeout: timeout) }
    /// Blocking variant for code that cannot await (e.g. applicationWillTerminate). Never call it from a task that
    /// the helper session itself needs; it only waits on its own threads.
    public func shutdownAndWait(timeout: TimeInterval = 20) { session.shutdownBlocking(timeout: timeout) }

    public var isConnected: Bool { session.isConnected }
    /// What the running helper reported when it connected.
    public var helperHello: HelperHello? { session.hello }

    /// Recovery: re-enumerates RealSense cameras that a crashed helper left detached from macOS
    /// (`IntelFaceScanHelper --release -w`, administrator prompt). Returns the tool's lines, e.g.
    /// "RealSense device re-enumerated: ok (0x00000000)" and "macOS drivers attached", or "No RealSense device found".
    public static func releaseCameraToSystem(helperURL: URL = CameraSources.bundledHelperURL) async throws -> String {
        let helper = FileManager.default.isExecutableFile(atPath: helperURL.path)
            ? helperURL : (CameraSources.locateHelper() ?? helperURL)
        return try await releaseCamera(helper: helper) { helper, arguments in
            HelperLauncher.launchPrivileged(helper: helper, arguments: arguments, captureOutput: true)
        }
    }

    /// releaseCameraToSystem() with the launch passed in (tests run it without the administrator clause).
    static func releaseCamera(helper: URL, launch: (URL, [String]) -> HelperProcessHandle) async throws -> String {
        try HelperLauncher.checkExecutable(helper)
        let handle = launch(helper, ["--release", "-w"])
        return try await HelperCameraSession.blocking {
            // The prompt waits for the user; the release itself takes seconds.
            guard handle.waitForExit(timeout: 3600) else {
                throw CameraError.timeout("releasing the camera")
            }
            if let error = handle.launchError { throw error }
            if let status = handle.exitStatus, status != 0 {
                throw CameraError.commandFailed("Releasing the camera failed (status \(status)): \(handle.diagnostics)")
            }
            return handle.diagnostics
        }
    }
}
