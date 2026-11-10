// License: Apache 2.0. See LICENSE file in root directory.
//
// The shared RealSense camera service as CameraKit uses it (docs/CAMERA-SERVICE.md): what to do about every state,
// the command line policy, the stand-in closures, where the service files are taken from, and the installer's
// command line run through osascript and the shell without the administrator clause. Nothing here runs the real
// probe against launchd, installs anything or shows a prompt.

import Darwin
import Foundation
import RealSenseCameraClient
import Testing
@testable import CameraKit

/// A folder laid out like the camera service files of an application, with stand-ins for the three files.
private enum KitFixture {
    /// `<root>/<name>.app/Contents/{<daemonFolder>/realsense-usbd, Resources/realsense-usbd/{service.plist, ...}}`.
    @discardableResult
    static func makeBundle(_ bundle: URL, daemonFolder: String = "Helpers", serviceVersion: Int = 2,
                           installer: String = "#!/bin/bash\nexit 0\n") throws -> CameraService.Kit {
        let fm = FileManager.default
        let daemonDir = bundle.appendingPathComponent("Contents").appendingPathComponent(daemonFolder)
        let resources = bundle.appendingPathComponent("Contents/Resources/realsense-usbd")
        try fm.createDirectory(at: daemonDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: resources, withIntermediateDirectories: true)
        try writeFiles(daemon: daemonDir.appendingPathComponent("realsense-usbd"),
                       manifest: resources.appendingPathComponent("service.plist"),
                       installScript: resources.appendingPathComponent("install-camera-access.sh"),
                       serviceVersion: serviceVersion, installer: installer)
        return try #require(CameraService.Kit.inBundle(bundle))
    }

    static func writeFiles(daemon: URL, manifest: URL, installScript: URL, serviceVersion: Int = 2,
                           installer: String = "#!/bin/bash\nexit 0\n") throws {
        try "#!/bin/sh\necho 'unreachable: stand-in'\nexit 2\n".write(to: daemon, atomically: true, encoding: .utf8)
        #expect(chmod(daemon.path, 0o755) == 0)
        let plist: [String: Any] = ["Version": 2, "ServiceVersion": serviceVersion, "ProtocolMin": 1, "ProtocolMax": 1,
                                    "DaemonCDHash": String(repeating: "0", count: 40)]
        try PropertyListSerialization.data(fromPropertyList: plist, format: .xml, options: 0).write(to: manifest)
        try installer.write(to: installScript, atomically: true, encoding: .utf8)
        #expect(chmod(installScript.path, 0o755) == 0)
    }

    static func real(_ url: URL) -> String? {
        realpath(url.path, nil).map { p in defer { free(p) }; return String(cString: p) }
    }
}

@Suite("Camera service decisions")
struct CameraServiceDecisionTests {
    typealias Availability = CameraService.Availability
    typealias Advice = CameraService.Advice

    static let everyState: [CameraService.State] = [
        .ready(serviceVersion: 2), .ready(serviceVersion: nil), .notInstalled, .switchedOff, .needsUpdate,
        .notAnswering("unreachable: the camera service is not installed, switched off, or does not admit this program"),
        .notConsoleUser, .cameraDisabled, .tooManyClients,
    ]

    @Test func adviceForEveryState() {
        let administrator = LiveCameraSource.Access.administrator(skipUSBRelease: false)
        let expected: [(Availability, Advice)] = [
            // Ready: as the user, nothing else to offer.
            (.state(.ready(serviceVersion: 2)), Advice(step: .useService, administratorAccess: nil)),
            (.state(.ready(serviceVersion: nil)), Advice(step: .useService, administratorAccess: nil)),
            // Installing is the fix; the administrator path stays for a cancelled or failed installation.
            (.state(.notInstalled), Advice(step: .install, administratorAccess: administrator)),
            (.state(.needsUpdate), Advice(step: .install, administratorAccess: administrator)),
            (.state(.notAnswering("x")), Advice(step: .install, administratorAccess: administrator)),
            // Switched off by an administrator: never reinstalled over their choice.
            (.state(.switchedOff), Advice(step: .explain, administratorAccess: administrator,
                                          offersLoginItemsSettings: true)),
            // The service runs and serves root: the root helper goes through it and must not re-enumerate USB.
            (.state(.notConsoleUser), Advice(step: .explain,
                                             administratorAccess: .administrator(skipUSBRelease: true))),
            // No way around a camera ban, and no fight with the service over its cameras.
            (.state(.cameraDisabled), Advice(step: .explain, administratorAccess: nil)),
            (.state(.tooManyClients), Advice(step: .explain, administratorAccess: nil)),
            // Nothing to ask or install from: as before the camera service existed.
            (.noKit, Advice(step: .useAdministrator, administratorAccess: administrator)),
        ]
        for (availability, advice) in expected {
            #expect(CameraService.advice(for: availability) == advice, "\(availability)")
        }
        // The table above is complete, and installing is advised exactly where the client says it helps.
        #expect(Set(expected.compactMap { $0.0.state }.map { "\($0)" }).count == 9)
        for state in Self.everyState {
            let advice = CameraService.advice(for: .state(state))
            #expect((advice.step == .install) == state.installationHelps, "\(state)")
            #expect((advice.step == .useService) == state.isReady, "\(state)")
            #expect(Availability.state(state).isReady == state.isReady)
            // --no-usb-release only where the service is alive and admits root.
            #expect((advice.administratorAccess == .administrator(skipUSBRelease: true)) == (state == .notConsoleUser))
            // The service is never offered together with a password, nor a ban together with a way around it.
            if state.isReady || state == .cameraDisabled || state == .tooManyClients {
                #expect(advice.administratorAccess == nil, "\(state)")
            }
            #expect(advice.administratorAccess != .automatic && advice.administratorAccess != .service)
        }
        #expect(!Availability.noKit.isReady && Availability.noKit.state == nil)
    }

    /// What IntelFaceScan relies on in the client's own decision (RealSenseCameraAccess.state(probe:...)).
    @Test func stateFromTheProbe() {
        typealias Client = RealSenseCameraAccess
        func state(_ code: Int32, _ line: String = "", installed: Bool = true, off: Bool = false,
                   current: Bool = true) -> Client.State {
            Client.state(probe: Client.Probe(code: code, line: line), installed: installed, switchedOff: off,
                         currentGeneration: current)
        }
        #expect(state(0, "ok 2") == .ready(serviceVersion: 2))
        #expect(state(0, "ok 1", current: false) == .ready(serviceVersion: 1))
        #expect(state(0, "ok") == .ready(serviceVersion: nil))
        // The service's own verdicts win over what is on disk.
        #expect(state(3, installed: false) == .notConsoleUser)
        #expect(state(4, off: true) == .cameraDisabled)
        #expect(state(6) == .tooManyClients)
        // Not reachable: why, from what is installed.
        #expect(state(2, installed: false) == .notInstalled)
        #expect(state(2, installed: false, off: true, current: false) == .notInstalled)
        #expect(state(2, off: true) == .switchedOff)
        #expect(state(5, off: true) == .switchedOff)
        #expect(state(5) == .needsUpdate)
        #expect(state(2, current: false) == .needsUpdate, "the first generation admitted one build only")
        #expect(state(2, "unreachable: x") == .notAnswering("unreachable: x"))
        #expect(state(77, "odd") == .notAnswering("odd"))
        #expect(Client.Probe(code: 0, line: "ok 12").serviceVersion == 12)
        #expect(Client.Probe(code: 2, line: "ok 12").serviceVersion == nil)
    }

    @Test func commandLineNeverInstalls() throws {
        let script = URL(fileURLWithPath: "/Applications/Intel Face's Scan.app/Contents/Resources/realsense-usbd/"
                         + "ensure-camera-access.sh")
        // Ready: the service, silently.
        let ready = LiveCameraSource.commandLineAccess(for: .state(.ready(serviceVersion: 2)), ensureScript: script)
        #expect(ready.access == .service && ready.note == nil)

        // Installing would help: the administrator path as before, and one line that says how to install.
        for state in [CameraService.State.notInstalled, .needsUpdate, .notAnswering("no answer")] {
            let decision = LiveCameraSource.commandLineAccess(for: .state(state), ensureScript: script)
            #expect(decision.access == .administrator(skipUSBRelease: false), "\(state)")
            let note = try #require(decision.note)
            #expect(!note.contains("\n"), "one line: \(note)")
            #expect(note.hasPrefix("note: ") && note.contains("administrator password this time"), "\(note)")
            #expect(note.contains("open IntelFaceScan.app"), "\(note)")
            // Quoted for pasting into a shell.
            #expect(note.contains(#"'/Applications/Intel Face'\''s Scan.app/Contents/Resources/realsense-usbd/"#
                                  + "ensure-camera-access.sh'"), "\(note)")
        }
        #expect(LiveCameraSource.commandLineAccess(for: .state(.notInstalled)).note?
            .contains("'IntelFaceScan.app/Contents/Resources/realsense-usbd/ensure-camera-access.sh'") == true)
        let noKit = LiveCameraSource.commandLineAccess(for: .noKit)
        #expect(noKit.access == .administrator(skipUSBRelease: false))
        #expect(noKit.note?.contains("carries no camera service files") == true)

        // Installing would not help: the line says what would.
        let off = LiveCameraSource.commandLineAccess(for: .state(.switchedOff), ensureScript: script)
        #expect(off.access == .administrator(skipUSBRelease: false))
        #expect(off.note?.contains("Login Items & Extensions") == true)
        #expect(off.note?.contains("ensure-camera-access.sh") == false)
        let remote = LiveCameraSource.commandLineAccess(for: .state(.notConsoleUser), ensureScript: script)
        #expect(remote.access == .administrator(skipUSBRelease: true))
        #expect(remote.note?.contains("ensure-camera-access.sh") == false)

        // No administrator path either.
        for state in [CameraService.State.cameraDisabled, .tooManyClients] {
            let decision = LiveCameraSource.commandLineAccess(for: .state(state), ensureScript: script)
            #expect(decision.access == nil, "\(state)")
            #expect(decision.note?.hasPrefix("The camera cannot be used: ") == true)
        }
    }

    @Test func commandLineAccessChecksOnceAndReportsTheLine() throws {
        let kit = CameraService.Kit(daemon: URL(fileURLWithPath: "/nonexistent/realsense-usbd"),
                                    manifest: URL(fileURLWithPath: "/nonexistent/service.plist"),
                                    installScript: URL(fileURLWithPath: "/nonexistent/install-camera-access.sh"))
        let installs = PromptRecord(), checks = PromptRecord()
        func service(_ state: CameraService.State) -> CameraService {
            CameraService(kit: kit, state: { _ in checks.note(); return state }, install: { _, _ in installs.note() })
        }
        var lines: [String] = []
        #expect(try LiveCameraSource.commandLineAccess(cameraService: service(.ready(serviceVersion: 2))) {
            lines.append($0)
        } == .service)
        #expect(lines.isEmpty)
        #expect(try LiveCameraSource.commandLineAccess(cameraService: service(.notInstalled)) { lines.append($0) }
                == .administrator(skipUSBRelease: false))
        #expect(lines.count == 1 && lines[0].contains("not installed"))
        #expect(throws: CameraError.cameraServiceNotReady(.state(.cameraDisabled))) {
            _ = try LiveCameraSource.commandLineAccess(cameraService: service(.cameraDisabled)) { lines.append($0) }
        }
        #expect(lines.count == 2 && lines[1].contains("disabled on this Mac"))
        #expect(checks.count == 3)
        #expect(installs.count == 0, "a command line tool installed the camera service")
    }

    @Test func descriptions() {
        var seen = Set<String>()
        for availability in [Availability.noKit] + Self.everyState.map(Availability.state) {
            let text = CameraService.describe(availability)
            #expect(!text.isEmpty && !text.hasSuffix("."), "\(availability): the end of a sentence, no period")
            seen.insert(text)
        }
        #expect(seen.count == 10, "every state reads differently")
        #expect(CameraService.describe(.state(.ready(serviceVersion: 2))).hasSuffix("(version 2)"))
        #expect(CameraService.describe(.state(.notAnswering("probe line"))).contains("probe line"))
        #expect(CameraError.cameraServiceNotReady(.state(.switchedOff)).errorDescription
                == "The RealSense camera service cannot be used: the RealSense camera service is switched off in "
                + "System Settings > General > Login Items & Extensions.")
    }
}

@Suite("Camera service stand-ins and files")
struct CameraServiceFilesTests {
    @Test func closuresReplaceTheProbeAndTheInstaller() async throws {
        let kit = CameraService.Kit(daemon: URL(fileURLWithPath: "/nonexistent/realsense-usbd"),
                                    manifest: URL(fileURLWithPath: "/nonexistent/service.plist"),
                                    installScript: URL(fileURLWithPath: "/nonexistent/install-camera-access.sh"))
        final class Record: @unchecked Sendable {
            let lock = NSLock()
            var kits: [CameraService.Kit] = []
            var prompts: [String] = []
            var threads: [Bool] = []
            var settings = 0
        }
        let record = Record()
        let service = CameraService(
            kit: kit,
            state: { kit in
                record.lock.withLock { record.kits.append(kit); record.threads.append(Thread.isMainThread) }
                return .switchedOff
            },
            install: { kit, prompt in
                record.lock.withLock { record.kits.append(kit); record.prompts.append(prompt) }
                if prompt == "cancel" { throw CameraService.InstallError.cancelled }
                if prompt == "fail" { throw CameraService.InstallError.failed("installer: no") }
                if prompt == "trust" { throw CameraService.InstallError.untrusted("not root's") }
            },
            installedServiceVersion: { 7 },
            openLoginItemsSettings: { record.lock.withLock { record.settings += 1 } })

        #expect(service.checkBlocking() == .state(.switchedOff))
        #expect(await service.check() == .state(.switchedOff))
        #expect(record.kits == [kit, kit])
        #expect(record.threads.last == false, "the asynchronous check ran on the main thread")
        try await service.install(prompt: "Install once")
        #expect(record.prompts == ["Install once"])
        await #expect(throws: CameraService.InstallError.cancelled) { try await service.install(prompt: "cancel") }
        await #expect(throws: CameraService.InstallError.failed("installer: no")) {
            try await service.install(prompt: "fail")
        }
        #expect(throws: CameraService.InstallError.untrusted("not root's")) {
            try service.installBlocking(prompt: "trust")
        }
        #expect(service.installedServiceVersion == 7)
        service.openLoginItemsSettings()
        #expect(record.settings == 1)

        // Without service files nothing is asked and nothing can be installed.
        let bare = CameraService(kit: nil, state: { _ in
            Issue.record("the probe ran without a kit")
            return .ready(serviceVersion: 2)
        }, install: { _, _ in Issue.record("the installer ran without a kit") })
        #expect(bare.checkBlocking() == .noKit)
        #expect(await bare.check() == .noKit)
        #expect(bare.bundledServiceVersion == nil && bare.ensureScript == nil)
        #expect(throws: CameraService.InstallError.self) { try bare.installBlocking(prompt: "x") }
    }

    @Test func kitInABundleAndInAFolder() throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        // IntelFaceScan keeps the daemon in Contents/Helpers; the viewer applications in Contents/MacOS.
        let app = dir.appendingPathComponent("IntelFaceScan.app")
        let kit = try KitFixture.makeBundle(app, daemonFolder: "Helpers", serviceVersion: 5)
        let root = try #require(KitFixture.real(app))
        #expect(kit.daemon.path == root + "/Contents/Helpers/realsense-usbd")
        #expect(kit.manifest.path == root + "/Contents/Resources/realsense-usbd/service.plist")
        #expect(kit.installScript.path == root + "/Contents/Resources/realsense-usbd/install-camera-access.sh")
        #expect(kit.isComplete && kit.serviceVersion == 5)
        let viewer = try KitFixture.makeBundle(dir.appendingPathComponent("Viewer.app"), daemonFolder: "MacOS")
        #expect(viewer.daemon.path.hasSuffix("/Viewer.app/Contents/MacOS/realsense-usbd"))

        // The service: its version, and the script a person can run, when the bundle has one.
        let service = CameraService(kit: kit, state: { _ in .notInstalled })
        #expect(service.bundledServiceVersion == 5)
        #expect(service.ensureScript == nil)
        let ensure = kit.installScript.deletingLastPathComponent().appendingPathComponent("ensure-camera-access.sh")
        try "#!/bin/bash\n".write(to: ensure, atomically: true, encoding: .utf8)
        #expect(service.ensureScript == ensure)

        // $IFS_CAMERA_SERVICE_KIT: an application bundle, or a folder with the three files side by side.
        #expect(CameraServiceKit.inFolder(app) == kit)
        let flat = dir.appendingPathComponent("flat kit")
        try FileManager.default.createDirectory(at: flat, withIntermediateDirectories: true)
        #expect(CameraServiceKit.inFolder(flat) == nil, "an empty folder is no kit")
        try KitFixture.writeFiles(daemon: flat.appendingPathComponent("realsense-usbd"),
                                  manifest: flat.appendingPathComponent("service.plist"),
                                  installScript: flat.appendingPathComponent("install-camera-access.sh"))
        let flatKit = try #require(CameraServiceKit.inFolder(flat))
        let flatRoot = try #require(KitFixture.real(flat))
        #expect(flatKit.daemon.path == flatRoot + "/realsense-usbd")
        #expect(flatKit.manifest.path == flatRoot + "/service.plist")
        #expect(flatKit.installScript.path == flatRoot + "/install-camera-access.sh")
        // The installer gets the one folder once.
        #expect(RealSenseCameraAccess.installCommand(kit: flatKit).components(separatedBy: "--source").count == 2)

        // A kit with a file missing, or a daemon that cannot be run, is none.
        try FileManager.default.removeItem(at: flat.appendingPathComponent("service.plist"))
        #expect(CameraServiceKit.inFolder(flat) == nil)
        #expect(chmod(kit.daemon.path, 0o644) == 0)
        #expect(CameraService.Kit.inBundle(app) == nil)
        #expect(CameraServiceKit.inFolder(dir.appendingPathComponent("missing")) == nil)
    }

    @Test func locateFollowsTheDocumentedOrder() throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let fm = FileManager.default
        let own = try KitFixture.makeBundle(dir.appendingPathComponent("Own.app"), serviceVersion: 3)
        let fromEnvironment = try KitFixture.makeBundle(dir.appendingPathComponent("Env.app"), serviceVersion: 4)
        let environment = [CameraServiceKit.environmentVariable: dir.appendingPathComponent("Env.app").path]
        let nowhere = URL(fileURLWithPath: "/nonexistent/tool")
        let noBundle = dir.appendingPathComponent("no bundle")

        // 1. The bundle of the process, whatever the environment says.
        #expect(CameraServiceKit.locate(environment: environment, bundle: dir.appendingPathComponent("Own.app"),
                                        executable: nowhere) == own)
        // 2. The environment variable when the process has no bundle with a kit.
        #expect(CameraServiceKit.locate(environment: environment, bundle: noBundle, executable: nowhere)
                == fromEnvironment)
        // A variable that names no kit is passed over.
        #expect(CameraServiceKit.locate(environment: [CameraServiceKit.environmentVariable: noBundle.path],
                                        bundle: noBundle, executable: nowhere) == nil)
        // 3. The bundle the executable sits in (ifscan in Contents/MacOS).
        let macOS = dir.appendingPathComponent("Own.app/Contents/MacOS")
        try fm.createDirectory(at: macOS, withIntermediateDirectories: true)
        let tool = macOS.appendingPathComponent("ifscan")
        try fm.copyItem(atPath: "/bin/echo", toPath: tool.path)
        #expect(CameraServiceKit.locate(environment: [:], bundle: macOS, executable: tool) == own)
        // 4. build/IntelFaceScan.app above the executable (swift run, swift test).
        let repo = dir.appendingPathComponent("repo")
        let debug = repo.appendingPathComponent(".build/arm64-apple-macosx/debug")
        try fm.createDirectory(at: debug, withIntermediateDirectories: true)
        let built = try KitFixture.makeBundle(repo.appendingPathComponent("build/IntelFaceScan.app"), serviceVersion: 6)
        #expect(CameraServiceKit.locate(environment: [:], bundle: debug,
                                        executable: debug.appendingPathComponent("ifscan")) == built)
        // Nothing anywhere: the live camera then only works with the administrator password.
        let lonely = dir.appendingPathComponent("lonely/bin")
        try fm.createDirectory(at: lonely, withIntermediateDirectories: true)
        #expect(CameraServiceKit.searchBuildFolders(from: lonely) == nil)
        #expect(CameraServiceKit.locate(environment: [:], bundle: noBundle, executable: nil) == nil)
    }

    @Test func searchSkipsKitsOthersCanReplace() throws {
        // The same rule as for the helper (HelperSecurityLaunchTests.searchSkipsHelpersOthersCanReplace): the search
        // climbs out of the checkout, and what it finds is run (the probe) and offered to the installer.
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let shared = dir.appendingPathComponent("shared")
        let start = shared.appendingPathComponent("repo/.build/debug")
        try FileManager.default.createDirectory(at: start, withIntermediateDirectories: true)
        let planted = try KitFixture.makeBundle(shared.appendingPathComponent("build/IntelFaceScan.app"))

        #expect(CameraServiceKit.problem(planted) == nil)
        #expect(CameraServiceKit.searchBuildFolders(from: start) == planted)

        // A world-writable folder without the sticky bit: anyone could have put the kit there or swap it later.
        #expect(chmod(shared.path, 0o777) == 0)
        #expect(CameraServiceKit.problem(planted)?.contains("writable by everyone") == true)
        #expect(CameraServiceKit.searchBuildFolders(from: start) == nil)
        // Sticky (like /Users/Shared): other users cannot touch entries they do not own.
        #expect(chmod(shared.path, 0o1777) == 0)
        #expect(CameraServiceKit.searchBuildFolders(from: start) == planted)
        #expect(chmod(shared.path, 0o755) == 0)

        // Any one of the three files that somebody else may rewrite spoils the kit.
        for file in [planted.daemon, planted.manifest, planted.installScript] {
            let mode = file == planted.manifest ? mode_t(0o644) : mode_t(0o755)
            #expect(chmod(file.path, mode | 0o002) == 0)
            #expect(CameraServiceKit.problem(planted)?.contains(file.lastPathComponent) == true)
            #expect(CameraServiceKit.searchBuildFolders(from: start) == nil, "\(file.lastPathComponent)")
            #expect(chmod(file.path, mode) == 0)
        }
        // Owned by somebody else (seen as another uid).
        #expect(CameraServiceKit.problem(planted, uid: getuid() + 1)?.contains("another user") == true)
        #expect(CameraServiceKit.searchBuildFolders(from: start, uid: getuid() + 1) == nil)
        #expect(CameraServiceKit.searchBuildFolders(from: start) == planted)

        // An incomplete bundle further down does not hide a complete one further up.
        let inner = shared.appendingPathComponent("repo/build/IntelFaceScan.app/Contents/Helpers")
        try FileManager.default.createDirectory(at: inner, withIntermediateDirectories: true)
        #expect(CameraServiceKit.searchBuildFolders(from: start) == planted)
    }

    @Test func installCommandSurvivesAppleScriptAndShell() throws {
        // The installer's command line as the client builds it for IntelFaceScan's bundle layout, run through
        // osascript and the shell without the administrator clause: the stand-in installer must get the two source
        // folders byte for byte, whatever the bundle's path contains, and a clean environment.
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let odd = dir.appendingPathComponent("it's \"odd\" \\ $HOME `id` ; | é 🎥")
        let output = dir.appendingPathComponent("argv")
        // bash -p ignores the environment's functions and BASH_ENV; the stand-in records what it was given.
        let installer = """
            #!/bin/bash
            {
                printf '%s\\0' "$@"
                printf 'PATH=%s\\0' "$PATH"
                printf 'vars=%s\\0' "$(/usr/bin/env | /usr/bin/wc -l | /usr/bin/tr -d ' ')"
            } > '\(output.path)'
            echo "installed from $2"

            """
        let kit = try KitFixture.makeBundle(odd.appendingPathComponent("Intel Face Scan.app"), installer: installer)
        let helpers = kit.daemon.deletingLastPathComponent().path
        let resources = kit.manifest.deletingLastPathComponent().path

        let command = RealSenseCameraAccess.installCommand(kit: kit)
        #expect(command.hasPrefix("/usr/bin/env -i PATH=/usr/bin:/bin:/usr/sbin:/sbin /bin/bash -p '"))
        #expect(!command.contains("--allow-downgrade"), "the app never replaces a newer service")
        let source = RealSenseCameraAccess.appleScriptSource(command: command, prompt: "unused", privileged: false)
        #expect(!source.contains("administrator privileges"))
        // The real thing differs by the clause only.
        #expect(RealSenseCameraAccess.appleScriptSource(command: command, prompt: "Why")
                == source + " with administrator privileges with prompt \"Why\"")

        let result = RealSenseCameraAccess.runAppleScript(source, inProcess: false)
        #expect(result == .success("installed from \(helpers)"), "\(result)")
        var received = try Data(contentsOf: output).split(separator: 0, omittingEmptySubsequences: false).map {
            String(decoding: $0, as: UTF8.self)
        }
        #expect(received.last == "")
        received.removeLast()
        #expect(Array(received.prefix(4)) == ["--source", helpers, "--source", resources])
        #expect(received.count == 6 && received[4] == "PATH=/usr/bin:/bin:/usr/sbin:/sbin")
        // env -i: PATH, plus what bash itself sets (PWD, SHLVL, _); nothing of the test runner's environment.
        let variables = try #require(Int(received[5].dropFirst("vars=".count)))
        #expect(variables <= 5, "\(variables) environment variables reached the installer")

        // A failing installer: its message and status come back (what InstallError.failed carries).
        let failing = try KitFixture.makeBundle(dir.appendingPathComponent("Failing.app"),
                                                installer: "#!/bin/bash\necho 'camera access: no' >&2\nexit 1\n")
        let failure = RealSenseCameraAccess.runAppleScript(
            RealSenseCameraAccess.appleScriptSource(command: RealSenseCameraAccess.installCommand(kit: failing),
                                                    prompt: "unused", privileged: false), inProcess: false)
        #expect(failure == .failure(number: 1, message: "camera access: no"), "\(failure)")
        // A cancelled prompt is AppleScript's error -128.
        #expect(RealSenseCameraAccess.runAppleScript("error \"User canceled.\" number -128", inProcess: false)
                == .failure(number: -128, message: "User canceled."))
    }
}
