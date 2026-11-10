// License: Apache 2.0. See LICENSE file in root directory.
//
// The live RealSense camera through the camera helper, in one of two ways (docs/HELPER.md, docs/CAMERA-SERVICE.md):
//
// * Through the shared RealSense camera service (realsense-usbd): the helper runs as the user, started directly, and
//   its librealsense reaches the camera through the service. No password. shutdown() makes the helper stop the
//   streams and exit; the service hands the camera back to macOS a few seconds after the last program let go of it.
// * With the administrator password, for one session: the helper runs as root, takes the USB device over itself and,
//   at shutdown(), hands the camera back to macOS (it works as a webcam again) and exits. connect() shows the macOS
//   administrator prompt once per helper.
//
// The prompt comes from AppleScript's "do shell script ... with administrator privileges". In an AppKit application
// (IntelFaceScan.app) it runs in-process and the prompt is the app's; elsewhere (a command line tool) it runs through
// /usr/bin/osascript, because an in-process "do shell script" can hang without an AppKit event loop
// (HelperLauncher). An AppKit application whose main run loop is not running (NSApplication created, never run) is
// not supported.

import CoreGraphics
import Foundation

public final class LiveCameraSource: CameraSource, @unchecked Sendable {
    /// How the helper reaches the camera.
    public enum Access: Sendable, Equatable {
        /// Through the camera service when it is ready. When it is not, connect() starts nothing and throws
        /// `CameraError.cameraServiceNotReady` with what the state check found: the caller decides (install the
        /// service, ask the user, or open the camera again with `.administrator`). Never shows a prompt.
        case automatic
        /// Through the camera service, without checking its state first (the caller did): the helper runs as the
        /// user. Never shows a prompt.
        case service
        /// The helper as root after the administrator prompt, for this session. `skipUSBRelease`: the helper does not
        /// re-enumerate the camera when the session ends (`--no-usb-release`); pass true when the camera service is
        /// alive and serves root (CameraService.Advice.administratorAccess says which it is for a state).
        case administrator(skipUSBRelease: Bool)
    }

    public let kind: CameraSourceKind = .live
    public let helperURL: URL
    public let access: Access
    private let cameraService: CameraService
    private let session: HelperCameraSession

    /// - Parameters:
    ///   - access: see `Access`. The default never asks for a password.
    ///   - cameraService: asked by `.automatic` only.
    public init(helperURL: URL = CameraSources.bundledHelperURL, access: Access = .automatic,
                cameraService: CameraService = .standard) {
        self.helperURL = helperURL
        self.access = access
        self.cameraService = cameraService
        // .automatic only ever starts the helper as the user.
        session = HelperCameraSession(mode: .live, helperURL: helperURL,
                                      liveLaunch: Self.launch(for: access) ?? .asUser)
    }

    /// The launch of an access that needs no state check; nil for `.automatic`.
    static func launch(for access: Access) -> LiveLaunch? {
        switch access {
        case .automatic: return nil
        case .service: return .asUser
        case .administrator(let skipUSBRelease): return .administrator(skipUSBRelease: skipUSBRelease)
        }
    }

    /// Starts the helper and returns the device. `.administrator` shows the administrator prompt
    /// (CameraError.authorizationCancelled when the user cancels it); `.automatic` checks the camera service first
    /// and throws CameraError.cameraServiceNotReady when it cannot be used. When the helper runs but no camera is
    /// connected the error comes from the helper, and a later connect() asks again without a new check or prompt.
    /// A shutdown() ends a connect that is still checking the camera service (CameraError.notConnected): no helper
    /// starts after it.
    public func connect() async throws -> DeviceInfo {
        guard access == .automatic && !session.isConnected else { return try await session.connect() }
        // The helper first: a missing helper is the error to report, whatever the service says.
        _ = try session.resolvedHelperURL()
        // The session stops a connect that a shutdown overtakes, but only from session.connect() on. The check runs
        // before that, so the session is told which shutdowns this connect has seen.
        let shutdowns = session.shutdownCount
        let availability = await cameraService.check()
        guard availability.isReady else { throw CameraError.cameraServiceNotReady(availability) }
        return try await session.connect(ifNoShutdownSince: shutdowns)
    }
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

    /// Asks the helper to release the camera and exit; waits for it (at most `timeout`, which covers the root helper
    /// handing the camera back to macOS). Safe from applicationShouldTerminate: nothing here runs on the main thread.
    public func shutdown() async { await session.shutdown(timeout: 20) }
    public func shutdown(timeout: TimeInterval) async { await session.shutdown(timeout: timeout) }
    /// Blocking variant for code that cannot await (e.g. applicationWillTerminate). Never call it from a task that
    /// the helper session itself needs; it only waits on its own threads.
    public func shutdownAndWait(timeout: TimeInterval = 20) { session.shutdownBlocking(timeout: timeout) }

    public var isConnected: Bool { session.isConnected }
    /// What the running helper reported when it connected (`euid` 0: the administrator path).
    public var helperHello: HelperHello? { session.hello }

    /// For tests: the session behind the source.
    var helperSession: HelperCameraSession { session }

    // MARK: Command line tools

    /// The access for a program without a user interface, which never installs the camera service and shows no
    /// dialog: the service when it is ready; otherwise the administrator path as before the service existed, with
    /// one line for stderr (`note`) that says why and how to install the service. Where the administrator path must
    /// not be used (camera use disabled on this Mac, or the service is full) there is no access, only the line.
    public static func commandLineAccess(for availability: CameraService.Availability,
                                         ensureScript: URL? = nil) -> (access: Access?, note: String?) {
        let advice = CameraService.advice(for: availability)
        if advice.step == .useService { return (.service, nil) }
        let reason = CameraService.describe(availability)
        guard let administrator = advice.administratorAccess else {
            return (nil, "The camera cannot be used: \(reason).")
        }
        let script = ensureScript?.path
            ?? "IntelFaceScan.app/Contents/Resources/realsense-usbd/ensure-camera-access.sh"
        let fix: String
        switch advice.step {
        case .install, .useAdministrator:
            fix = "to use the camera without a password, open IntelFaceScan.app and choose Live camera once, or run "
                + HelperLauncher.shellQuote(script)
        case .explain where advice.offersLoginItemsSettings:
            fix = "an administrator can switch it on there"
        default:
            fix = "log in at this Mac's screen to use the camera without a password"
        }
        return (administrator, "note: \(reason); asking for the administrator password this time (\(fix)).")
    }

    /// commandLineAccess(for:) with the state check (blocking, a few seconds at most; no prompt, no installation).
    /// `note` gets the line for stderr, when there is one. Throws CameraError.cameraServiceNotReady where the camera
    /// must not be opened with the administrator password either.
    public static func commandLineAccess(cameraService: CameraService = .standard,
                                         note: (String) -> Void) throws -> Access {
        let availability = cameraService.checkBlocking()
        let decision = commandLineAccess(for: availability, ensureScript: cameraService.ensureScript)
        if let line = decision.note { note(line) }
        guard let access = decision.access else { throw CameraError.cameraServiceNotReady(availability) }
        return access
    }

    // MARK: Recovery

    /// Recovery after the administrator path: re-enumerates RealSense cameras that a crashed root helper left
    /// detached from macOS (`IntelFaceScanHelper --release -w`, administrator prompt). Returns the tool's lines, e.g.
    /// "RealSense device re-enumerated: ok (0x00000000)" and "macOS drivers attached", or "No RealSense device found".
    /// Not needed after a session through the camera service: the service hands the camera back itself, also when
    /// the helper crashed.
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
