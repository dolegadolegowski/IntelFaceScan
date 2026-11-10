// License: Apache 2.0. See LICENSE file in root directory.
//
// The shared RealSense camera service (realsense-usbd, docs/CAMERA-SERVICE.md) as IntelFaceScan uses it: where the
// service files of this process are, whether the service can be used right now, installing it, and what to do about
// each state. The checks and the installation are the vendored client's (RealSenseCameraAccess, compiled unchanged
// from third_party/realsense-usbd/client); this file adds the kit location for development runs, closures that tests
// replace, and the decision table the app and command line tools share.

import Darwin
import Foundation
import RealSenseCameraClient

/// The camera service for one process: its files, its state, its installation.
///
/// `CameraService.standard` is the real one. Tests build their own with closures in place of the probe and the
/// installer, so nothing in a test runs `realsense-usbd --probe` or shows an administrator prompt.
public struct CameraService: Sendable {
    public typealias Kit = RealSenseCameraAccess.Kit
    public typealias State = RealSenseCameraAccess.State
    public typealias InstallError = RealSenseCameraAccess.InstallError

    /// What the state check found.
    public enum Availability: Sendable, Equatable {
        /// This process carries no camera service files (a development run without the app bundle, or a bundle built
        /// without them): the service can be neither asked nor installed from here.
        case noKit
        case state(State)

        public var isReady: Bool { state?.isReady == true }

        public var state: State? {
            if case .state(let state) = self { return state }
            return nil
        }
    }

    /// The service files of this process (daemon, manifest, installer); nil when it has none.
    public var kit: Kit?
    private var probeState: @Sendable (Kit) -> State
    private var runInstaller: @Sendable (Kit, String) throws -> Void
    private var readInstalledVersion: @Sendable () -> Int?
    private var openSettings: @Sendable () -> Void

    /// - Parameters:
    ///   - kit: the service files; nil for none.
    ///   - state: the state check (blocking); default `RealSenseCameraAccess.state(kit:)`.
    ///   - install: the installation (blocking, shows the administrator prompt with the given text); default
    ///     `RealSenseCameraAccess.install(kit:prompt:)`.
    ///   - installedServiceVersion: default `RealSenseCameraAccess.installedServiceVersion()`.
    ///   - openLoginItemsSettings: default `RealSenseCameraAccess.openLoginItemsSettings()`.
    public init(kit: Kit?,
                state: @escaping @Sendable (Kit) -> State = { RealSenseCameraAccess.state(kit: $0) },
                install: @escaping @Sendable (Kit, String) throws -> Void = {
                    try RealSenseCameraAccess.install(kit: $0, prompt: $1)
                },
                installedServiceVersion: @escaping @Sendable () -> Int? = {
                    RealSenseCameraAccess.installedServiceVersion()
                },
                openLoginItemsSettings: @escaping @Sendable () -> Void = {
                    RealSenseCameraAccess.openLoginItemsSettings()
                }) {
        self.kit = kit
        probeState = state
        runInstaller = install
        readInstalledVersion = installedServiceVersion
        openSettings = openLoginItemsSettings
    }

    /// The real camera service with the files of this process (CameraServiceKit.locate()).
    public static let standard = CameraService(kit: CameraServiceKit.locate())

    // MARK: State

    /// Whether the camera service can be used right now, and if not, why. Blocks for the probe: normally well under
    /// a second, several seconds when the service is installed and does not answer. Never touches USB, never asks
    /// for anything.
    public func checkBlocking() -> Availability {
        guard let kit else { return .noKit }
        return .state(probeState(kit))
    }

    /// checkBlocking() on a thread of its own, so neither the main thread nor the Swift cooperative pool waits.
    public func check() async -> Availability {
        await Self.onOwnThread { checkBlocking() }
    }

    /// ServiceVersion of the service installed on this Mac; nil when none is, or one of the first generation.
    public var installedServiceVersion: Int? { readInstalledVersion() }

    /// ServiceVersion of the service this process carries; nil without a kit.
    public var bundledServiceVersion: Int? { kit?.serviceVersion }

    /// Opens System Settings > General > Login Items & Extensions, where an administrator switches the service on.
    public func openLoginItemsSettings() { openSettings() }

    // MARK: Installation

    /// Installs (or updates, or repairs) the camera service from this process's files. Shows the macOS administrator
    /// prompt with `prompt` and blocks until the installer ended; throws InstallError (.cancelled, .untrusted,
    /// .failed). The installer never replaces a newer service.
    public func installBlocking(prompt: String) throws {
        guard let kit else {
            throw InstallError.failed("This copy of IntelFaceScan carries no camera service files.")
        }
        try runInstaller(kit, prompt)
    }

    /// installBlocking(prompt:) off the caller's thread (the prompt waits for the user): on the queue every
    /// administrator prompt of IntelFaceScan runs on, so it never shows together with the root helper's prompt, and
    /// waits for a root helper that is still running.
    public func install(prompt: String) async throws {
        let result: Result<Void, Error> = await withCheckedContinuation { continuation in
            HelperLauncher.authorizationQueue.async {
                continuation.resume(returning: Result { try installBlocking(prompt: prompt) })
            }
        }
        try result.get()
    }

    /// `ensure-camera-access.sh` next to this process's installer: what a person runs in a terminal to install the
    /// service without opening the app. nil when the kit has none.
    public var ensureScript: URL? {
        guard let script = kit?.installScript.deletingLastPathComponent()
            .appendingPathComponent("ensure-camera-access.sh"),
            FileManager.default.fileExists(atPath: script.path) else { return nil }
        return script
    }

    private static func onOwnThread<T: Sendable>(_ work: @escaping @Sendable () -> T) async -> T {
        await withCheckedContinuation { (continuation: CheckedContinuation<T, Never>) in
            let thread = Thread { continuation.resume(returning: work()) }
            thread.name = "IntelFaceScan camera service"
            thread.start()
        }
    }
}

// MARK: - What to do about a state

extension CameraService {
    /// What an application does about the result of the state check (docs/CAMERA-SERVICE.md, "At the start of a live
    /// session"). The app follows `step`; a command line tool, which never installs and shows no dialog, uses
    /// `administratorAccess` (LiveCameraSource.commandLineAccess).
    public struct Advice: Sendable, Equatable {
        public enum Step: Sendable, Equatable {
            /// The service answers and admits this user: start the helper as the user. No dialog, no password.
            case useService
            /// Installing is the fix (not installed, too old, not answering): run the installer - an administrator's
            /// name and password, once - then check again.
            case install
            /// Nothing to ask or install from (no kit): the helper as root, as before the camera service existed.
            case useAdministrator
            /// Installing does not help: say why, offer what the other two fields allow.
            case explain
        }

        public var step: Step
        /// The one-time administrator path for this state; nil where it must not be offered: camera use is disabled
        /// on this Mac (the root helper would get around the ban), or the service is full (the root helper would
        /// fight over the cameras the service holds). `.administrator(skipUSBRelease: true)` where the service is
        /// alive and serves root: it then owns the hand-back of the camera, and the helper must not re-enumerate USB.
        public var administratorAccess: LiveCameraSource.Access?
        /// The fix is in System Settings > General > Login Items & Extensions.
        public var offersLoginItemsSettings: Bool

        public init(step: Step, administratorAccess: LiveCameraSource.Access?, offersLoginItemsSettings: Bool = false) {
            self.step = step
            self.administratorAccess = administratorAccess
            self.offersLoginItemsSettings = offersLoginItemsSettings
        }
    }

    public static func advice(for availability: Availability) -> Advice {
        let administrator = LiveCameraSource.Access.administrator(skipUSBRelease: false)
        switch availability {
        case .noKit:
            return Advice(step: .useAdministrator, administratorAccess: administrator)
        case .state(let state):
            switch state {
            case .ready:
                return Advice(step: .useService, administratorAccess: nil)
            case .notInstalled, .needsUpdate, .notAnswering:
                return Advice(step: .install, administratorAccess: administrator)
            case .switchedOff:
                return Advice(step: .explain, administratorAccess: administrator, offersLoginItemsSettings: true)
            case .notConsoleUser:
                // The service runs and admits root: the root helper's librealsense goes through it too.
                return Advice(step: .explain, administratorAccess: .administrator(skipUSBRelease: true))
            case .cameraDisabled, .tooManyClients:
                return Advice(step: .explain, administratorAccess: nil)
            }
        }
    }

    /// One English sentence about an availability, for logs, command line tools and CameraError (the app has its own
    /// texts in both languages).
    public static func describe(_ availability: Availability) -> String {
        switch availability {
        case .noKit:
            return "this copy of IntelFaceScan carries no camera service files"
        case .state(let state):
            switch state {
            case .ready(let version):
                return "the RealSense camera service is ready" + (version.map { " (version \($0))" } ?? "")
            case .notInstalled:
                return "the RealSense camera service is not installed"
            case .switchedOff:
                return "the RealSense camera service is switched off in System Settings > General > "
                    + "Login Items & Extensions"
            case .needsUpdate:
                return "the installed RealSense camera service is too old for this version of IntelFaceScan"
            case .notAnswering(let detail):
                return "the RealSense camera service is installed but does not answer"
                    + (detail.isEmpty ? "" : " (\(detail))")
            case .notConsoleUser:
                return "the RealSense camera service only serves the user logged in at this Mac's screen"
            case .cameraDisabled:
                return "camera use is disabled on this Mac by its administrator"
            case .tooManyClients:
                return "too many programs are using the RealSense camera service"
            }
        }
    }
}

// MARK: - Where the service files are

/// The camera service files for this process (daemon `realsense-usbd`, `service.plist`, `install-camera-access.sh`).
public enum CameraServiceKit {
    public typealias Kit = RealSenseCameraAccess.Kit

    /// Development: a folder that holds the three files side by side, or an application bundle that carries them.
    public static let environmentVariable = "IFS_CAMERA_SERVICE_KIT"

    /// The kit of the app bundle; otherwise, for runs without a bundle of their own: `$IFS_CAMERA_SERVICE_KIT`, the
    /// bundle the executable sits in (ifscan in Contents/MacOS), or `build/IntelFaceScan.app` in a parent folder of
    /// the executable (swift run / swift test from the repository, after scripts/build.sh). nil when there is none:
    /// the live camera then only works with the administrator password.
    public static func locate(environment: [String: String] = ProcessInfo.processInfo.environment,
                              bundle: URL = Bundle.main.bundleURL,
                              executable: URL? = Bundle.main.executableURL) -> Kit? {
        if let kit = Kit.inBundle(bundle) { return kit }
        if let path = environment[environmentVariable], !path.isEmpty,
           let kit = inFolder(URL(fileURLWithPath: path, isDirectory: true)) {
            return kit
        }
        guard let executable = executable?.resolvingSymlinksInPath() else { return nil }
        let folder = executable.deletingLastPathComponent()
        if folder.lastPathComponent == "MacOS", folder.deletingLastPathComponent().lastPathComponent == "Contents",
           let kit = Kit.inBundle(folder.deletingLastPathComponent().deletingLastPathComponent()) {
            return kit
        }
        return searchBuildFolders(from: folder)
    }

    /// `folder` as an application bundle (it has `Contents`), or as a flat folder with the three files.
    static func inFolder(_ folder: URL) -> Kit? {
        if let kit = Kit.inBundle(folder) { return kit }
        // One path for the trust check, the probe and the installer, as Kit.inBundle does.
        guard let resolved = realpath(folder.path, nil) else { return nil }
        defer { free(resolved) }
        let root = URL(fileURLWithPath: String(cString: resolved), isDirectory: true)
        let kit = Kit(daemon: root.appendingPathComponent("realsense-usbd"),
                      manifest: root.appendingPathComponent("service.plist"),
                      installScript: root.appendingPathComponent("install-camera-access.sh"))
        return kit.isComplete ? kit : nil
    }

    /// The kit of `build/IntelFaceScan.app` in `folder` or one of its 7 parents. Like the helper search
    /// (HelperLocation.searchBuildFolders) this can climb out of the repository into folders other users can write
    /// to, and the kit's daemon is run for the probe: a candidate is only taken when nobody but root and this user
    /// can replace any of its three files (HelperLocation.problem). The installer checks its own, stricter rule
    /// again before anything runs as root (RealSenseCameraAccess.untrustedReason).
    static func searchBuildFolders(from folder: URL, uid: uid_t = getuid()) -> Kit? {
        var dir = folder
        for _ in 0..<8 {
            if let kit = Kit.inBundle(dir.appendingPathComponent("build/IntelFaceScan.app")),
               problem(kit, uid: uid) == nil {
                return kit
            }
            let parent = dir.deletingLastPathComponent()
            if parent.path == dir.path { break }
            dir = parent
        }
        return nil
    }

    /// Why a kit found by searching is not taken: the first of its files that somebody else could replace.
    static func problem(_ kit: Kit, uid: uid_t = getuid()) -> String? {
        for file in [kit.daemon, kit.manifest, kit.installScript] {
            if let why = HelperLocation.problem(file.path, uid: uid) { return why }
        }
        return nil
    }
}
