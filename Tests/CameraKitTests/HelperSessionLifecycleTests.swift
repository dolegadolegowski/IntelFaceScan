// License: Apache 2.0. See LICENSE file in root directory.
//
// HelperCameraSession with a stand-in helper compiled at test time (needs /usr/bin/cc, otherwise skipped): a helper
// that says HELLO and is gone right away (as one that crashes while opening the device) must not leave a dead
// session behind that every later connect() would reuse. And the two ways to start the live helper: as the user
// through the camera service (started directly, uid and pid of the peer checked, no prompt) and as root after the
// administrator prompt (whose launcher the tests replace: nothing here may prompt).

import Darwin
import Foundation
import RealSenseCameraClient
import Testing
@testable import CameraKit

private enum StubHelper {
    static let compiler = "/usr/bin/cc"
    static var available: Bool { FileManager.default.isExecutableFile(atPath: compiler) }

    /// Appends its --socket path to <its own path>.sockets, connects, sends HELLO (mode live, fake with --fake or
    /// playback with --playback) and exits.
    static let source = #"""
    #include <stdint.h>
    #include <stdio.h>
    #include <string.h>
    #include <sys/socket.h>
    #include <sys/un.h>
    #include <unistd.h>
    int main(int argc, char **argv) {
        const char *path = 0, *mode = "live";
        for (int i = 1; i < argc; ++i) {
            if (!strcmp(argv[i], "--socket") && i + 1 < argc) path = argv[++i];
            else if (!strcmp(argv[i], "--playback")) mode = "playback";
            else if (!strcmp(argv[i], "--fake")) mode = "fake";
        }
        if (!path) return 64;
        char log[1024];
        snprintf(log, sizeof log, "%s.sockets", argv[0]);
        FILE *f = fopen(log, "a");
        if (f) { fprintf(f, "%s\n", path); fclose(f); }
        int s = socket(AF_UNIX, SOCK_STREAM, 0);
        struct sockaddr_un a;
        memset(&a, 0, sizeof a);
        a.sun_family = AF_UNIX;
        strncpy(a.sun_path, path, sizeof a.sun_path - 1);
        if (connect(s, (struct sockaddr *)&a, sizeof a) != 0) return 69;
        usleep(200000);  /* the app checks the peer's executable first */
        char json[256];
        int n = snprintf(json, sizeof json, "{\"protocol\":1,\"helper_version\":\"stub\",\"sdk_version\":\"0\","
                         "\"pid\":%d,\"euid\":%d,\"mode\":\"%s\"}", (int)getpid(), (int)geteuid(), mode);
        struct __attribute__((packed)) { uint32_t magic; uint16_t version, type; uint32_t json_bytes, flags;
                                         uint64_t payload; } h = { 0x31534649u, 1, 1, (uint32_t)n, 0, 0 };
        if (write(s, &h, sizeof h) != (ssize_t)sizeof h || write(s, json, n) != n) return 70;
        return 0;
    }
    """#

    static func build(in dir: URL) throws -> URL {
        let file = dir.appendingPathComponent("stub-helper.c")
        let binary = dir.appendingPathComponent("stub-helper")
        try source.write(to: file, atomically: true, encoding: .utf8)
        let process = Process()
        process.executableURL = URL(fileURLWithPath: compiler)
        process.arguments = ["-o", binary.path, file.path]
        try process.run()
        process.waitUntilExit()
        guard process.terminationStatus == 0 else { throw CameraError.launchFailed("cc failed") }
        return binary
    }
}

@Suite("Helper session lifecycle", .serialized, .enabled(if: StubHelper.available, "no C compiler"),
       .timeLimit(.minutes(1)))
struct HelperSessionLifecycleTests {
    private func eventually(_ seconds: Double = 2, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: seconds)
        while !condition() && Date() < deadline { try? await Task.sleep(for: .milliseconds(20)) }
        return condition()
    }

    @Test func helperGoneRightAfterHelloLeavesNoSession() async throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let stub = try StubHelper.build(in: dir)
        let source = FakeCameraSource(helperURL: stub)
        for attempt in 1...3 {
            do {
                _ = try await source.connect()
                Issue.record("attempt \(attempt): a helper that is gone cannot answer QUERY_DEVICE")
            } catch let error as CameraError {
                // Not "the camera is not connected": the helper connected and then went away, and says how.
                #expect(error == .launchFailed("the camera helper exited with status 0"), "attempt \(attempt): \(error)")
            }
            #expect(await eventually { !source.isConnected }, "attempt \(attempt): the dead helper is still the session")
            #expect(source.helperHello == nil, "attempt \(attempt)")
            // The socket and its private directory are cleaned up, whatever came first: the HELLO, the end of the
            // connection or the end of the process.
            let sockets = (try? String(contentsOf: URL(fileURLWithPath: stub.path + ".sockets"), encoding: .utf8)) ?? ""
            let lines = sockets.split(separator: "\n").map(String.init)
            #expect(lines.count == attempt)
            if let last = lines.last {
                let directory = (last as NSString).deletingLastPathComponent
                #expect(await eventually { !FileManager.default.fileExists(atPath: directory) },
                        "attempt \(attempt): \(directory) was left behind")
            }
        }
        await source.shutdown()
    }
}

/// What a test's launch override started.
private final class LaunchRecord: @unchecked Sendable {
    private let lock = NSLock()
    private var started: HelperProcessHandle?
    private var other: HelperProcessHandle?
    private var args: [String] = []

    var arguments: [String] { lock.withLock { args } }

    func record(_ handle: HelperProcessHandle, arguments: [String], impostor: HelperProcessHandle? = nil) {
        lock.withLock {
            started = handle
            args = arguments
            other = impostor
        }
    }

    var handle: HelperProcessHandle? { lock.withLock { started } }
    var impostor: HelperProcessHandle? { lock.withLock { other } }
    var socketDirectory: String? {
        lock.withLock {
            guard let i = args.firstIndex(of: "--socket"), i + 1 < args.count else { return nil }
            return (args[i + 1] as NSString).deletingLastPathComponent
        }
    }

    func cleanUp() {
        for handle in [handle, impostor].compactMap({ $0 }) where !handle.hasExited {
            handle.terminate()
            handle.waitForExit(timeout: 3)
        }
    }
}

@Suite("Helper session launch checks", .serialized, .timeLimit(.minutes(1)))
struct HelperSessionLaunchTests {
    private func eventually(_ seconds: Double = 5, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: seconds)
        while !condition() && Date() < deadline { try? await Task.sleep(for: .milliseconds(20)) }
        return condition()
    }

    @Test func closingWhileTheLivePromptShowsDoesNotWait() async throws {
        // The live helper's handle ends only when the in-process AppleScript returns, i.e. after the administrator
        // prompt. A stand-in that never connects and never exits plays the prompt that is still showing (no real
        // prompt in tests). Closing the source used to wait ~15 s for it; the socket is removed instead, so a helper
        // started after all could not connect and would exit by itself (or with the app). Timed on a thread of its own:
        // the shutdown needs no thread of the Swift cooperative pool, which the blocking tests of this target running
        // alongside can hold for seconds on a 3-core runner (awaiting shutdown(timeout:) measured 14 s there in CI run
        // 41873434467, while the shutdown still awaited the connect task).
        let sleep = URL(fileURLWithPath: "/bin/sleep")
        let session = HelperCameraSession(mode: .live, helperURL: sleep,
                                          liveLaunch: .administrator(skipUSBRelease: false))
        let launched = LaunchRecord()
        defer { launched.cleanUp() }
        session.launchOverride = { _, args in
            let handle = try HelperLauncher.launchAsUser(helper: sleep, arguments: ["30"])
            launched.record(handle, arguments: args)
            return handle
        }
        let attempt = Task { try await session.connect() }
        #expect(await eventually { launched.handle != nil })
        let elapsed: Double = await withCheckedContinuation { continuation in
            Thread {
                let t0 = Date()
                session.shutdownBlocking(timeout: 20)
                continuation.resume(returning: Date().timeIntervalSince(t0))
            }.start()
        }
        #expect(elapsed < 5, "closing waited \(elapsed) s for a helper that never connected")
        do {
            _ = try await attempt.value
            Issue.record("connect() succeeded without a helper")
        } catch {}
        let directory = try #require(launched.socketDirectory)
        #expect(!FileManager.default.fileExists(atPath: directory), "\(directory) was left behind")
        // The root helper cannot be signalled by the app: the stand-in was left alone.
        #expect(launched.handle?.hasExited == false)
    }

    @Test(.enabled(if: TestHelper.available, TestHelper.skipReason))
    func helperTheAppDidNotStartIsRejected() async throws {
        // A process of the same user that connects first and then executes the real helper passes the uid and
        // executable checks: here the real helper is started beside the process the session believes it started.
        let helper = try #require(TestHelper.url)
        let session = HelperCameraSession(mode: .fake(colorRowPadding: 0), helperURL: helper)
        let launched = LaunchRecord()
        defer { launched.cleanUp() }
        session.launchOverride = { helper, args in
            let impostor = try HelperLauncher.launchAsUser(helper: helper, arguments: args)
            let handle = try HelperLauncher.launchAsUser(helper: URL(fileURLWithPath: "/bin/sleep"), arguments: ["30"])
            launched.record(handle, arguments: args, impostor: impostor)
            return handle
        }
        do {
            _ = try await session.connect()
            Issue.record("a helper the app did not start was accepted")
        } catch let error as CameraError {
            guard case .connectionFailed(let message) = error else {
                Issue.record("unexpected error: \(error)")
                return
            }
            #expect(message.contains("not the camera helper the app started"), "\(message)")
        }
        #expect(!session.isConnected)
        // The rejected helper sees its connection closed and ends; the process that never connected is stopped.
        #expect(launched.impostor?.waitForExit(timeout: 5) == true)
        #expect(launched.impostor?.exitStatus == 0)
        #expect(launched.handle?.waitForExit(timeout: 5) == true)
        await session.shutdown(timeout: 5)
    }
}

/// The two ways to start the live helper (docs/HELPER.md, "Two ways to run the live camera").
@Suite("Helper live launch paths", .serialized, .timeLimit(.minutes(1)))
struct HelperLiveLaunchTests {
    private func eventually(_ seconds: Double = 5, _ condition: () -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: seconds)
        while !condition() && Date() < deadline { try? await Task.sleep(for: .milliseconds(20)) }
        return condition()
    }

    @Test func launchPlans() {
        let socket = "/private/var/folders/x/T/IntelFaceScan.abc/camera.sock"
        func plan(_ mode: HelperCameraSession.Mode, _ launch: LiveLaunch) -> HelperLaunchPlan {
            .make(mode: mode, liveLaunch: launch, socket: socket, uid: 501, gid: 20, parentPID: 4242)
        }

        // The camera service path: the helper as the user, started directly; the peer must be this user and the
        // very process the app started; no prompt to wait for.
        let service = plan(.live, .asUser)
        #expect(service.arguments == ["--socket", socket, "--uid", "501", "--parent-pid", "4242"])
        #expect(!service.privileged)
        #expect(service.expectedPeerUID == 501)
        #expect(service.verifiesPeerPID)
        #expect(service.connectTimeout == 20)

        // The administrator path: as before the camera service existed. The peer is root; its pid is unknown.
        let administrator = plan(.live, .administrator(skipUSBRelease: false))
        #expect(administrator.arguments == ["--socket", socket, "--uid", "501", "--gid", "20", "--parent-pid", "4242"])
        #expect(administrator.privileged)
        #expect(administrator.expectedPeerUID == 0)
        #expect(!administrator.verifiesPeerPID)
        #expect(administrator.connectTimeout == 600, "the administrator prompt waits for the user")

        // The service is alive and serves root: the root helper must not re-enumerate the camera at the end.
        let noRelease = plan(.live, .administrator(skipUSBRelease: true))
        #expect(noRelease.arguments == administrator.arguments + ["--no-usb-release"])
        #expect(noRelease.privileged && noRelease.expectedPeerUID == 0 && !noRelease.verifiesPeerPID)

        // Playback and the fake camera never run as root and never get the live-only flag, whatever the launch says.
        for launch in [LiveLaunch.asUser, .administrator(skipUSBRelease: true)] {
            let playback = plan(.playback(file: URL(fileURLWithPath: "/tmp/a b.db3"), realTime: false), launch)
            #expect(playback.arguments == ["--socket", socket, "--uid", "501", "--parent-pid", "4242",
                                           "--playback", "/tmp/a b.db3", "--no-real-time"])
            let fake = plan(.fake(colorRowPadding: 3), launch)
            #expect(fake.arguments == ["--socket", socket, "--uid", "501", "--parent-pid", "4242", "--fake",
                                       "--fake-row-padding", "3"])
            for user in [playback, fake] {
                #expect(!user.privileged && user.expectedPeerUID == 501 && user.verifiesPeerPID)
                #expect(user.connectTimeout == 20)
            }
        }
    }

    @Test func accessChoosesTheLaunch() {
        #expect(LiveCameraSource.launch(for: .service) == .asUser)
        #expect(LiveCameraSource.launch(for: .administrator(skipUSBRelease: false))
                == .administrator(skipUSBRelease: false))
        #expect(LiveCameraSource.launch(for: .administrator(skipUSBRelease: true))
                == .administrator(skipUSBRelease: true))
        #expect(LiveCameraSource.launch(for: .automatic) == nil, "automatic asks the camera service first")
        // Whatever the service says, an automatic source can only ever start the helper as the user.
        let source = LiveCameraSource(helperURL: URL(fileURLWithPath: "/nonexistent/helper"), access: .automatic,
                                      cameraService: CameraService(kit: nil))
        #expect(source.helperSession.liveLaunch == .asUser)
    }

    @Test(.enabled(if: StubHelper.available, "no C compiler"))
    func servicePathStartsTheHelperAsTheUser() async throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let stub = try StubHelper.build(in: dir)
        let source = LiveCameraSource(helperURL: stub, access: .service, cameraService: CameraService(kit: nil))
        let prompted = PromptRecord()
        source.helperSession.privilegedLauncher = prompted.launcher
        do {
            _ = try await source.connect()
            Issue.record("a helper that is gone cannot answer QUERY_DEVICE")
        } catch let error as CameraError {
            // The stand-in was started directly, passed the peer checks (this user, the pid the app started), said
            // HELLO in live mode and left: the session reports its exit, not a refused peer.
            #expect(error == .launchFailed("the camera helper exited with status 0"), "\(error)")
        }
        #expect(prompted.count == 0, "the camera service path asked for the administrator password")
        #expect(source.helperSession.liveLaunch == .asUser)
        await source.shutdown()
    }

    @Test(.enabled(if: StubHelper.available, "no C compiler"))
    func servicePathRejectsAHelperTheAppDidNotStart() async throws {
        // As for playback and the fake camera: a process of the same user that connects first passes the uid and
        // executable checks, so the pid must be the one the app started.
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let stub = try StubHelper.build(in: dir)
        let session = HelperCameraSession(mode: .live, helperURL: stub, liveLaunch: .asUser)
        let launched = LaunchRecord()
        defer { launched.cleanUp() }
        session.launchOverride = { helper, args in
            let impostor = try HelperLauncher.launchAsUser(helper: helper, arguments: args)
            let handle = try HelperLauncher.launchAsUser(helper: URL(fileURLWithPath: "/bin/sleep"), arguments: ["30"])
            launched.record(handle, arguments: args, impostor: impostor)
            return handle
        }
        do {
            _ = try await session.connect()
            Issue.record("a helper the app did not start was accepted")
        } catch let error as CameraError {
            guard case .connectionFailed(let message) = error else {
                Issue.record("unexpected error: \(error)")
                return
            }
            #expect(message.contains("not the camera helper the app started"), "\(message)")
        }
        #expect(!launched.arguments.contains("--gid") && !launched.arguments.contains("--no-usb-release"))
        #expect(!session.isConnected)
        // A helper running as the user can be signalled: the process that never connected is stopped.
        #expect(launched.handle?.waitForExit(timeout: 5) == true)
        await session.shutdown(timeout: 5)
    }

    @Test(.enabled(if: StubHelper.available, "no C compiler"))
    func administratorPathExpectsRoot() async throws {
        // The administrator launcher is replaced (no prompt in tests) by one that starts the stand-in as this user:
        // it gets the root helper's arguments, and what connects is refused because it does not run as root.
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let stub = try StubHelper.build(in: dir)
        let source = LiveCameraSource(helperURL: stub, access: .administrator(skipUSBRelease: true),
                                      cameraService: CameraService(kit: nil))
        let launched = LaunchRecord()
        defer { launched.cleanUp() }
        source.helperSession.privilegedLauncher = { helper, args in
            let impostor = try? HelperLauncher.launchAsUser(helper: helper, arguments: args)
            // Plays the root shell of the administrator launch: no pid of the helper, alive until the session ends.
            let handle = (try? HelperLauncher.launchAsUser(helper: URL(fileURLWithPath: "/bin/sleep"),
                                                           arguments: ["30"])) ?? HelperProcessHandle()
            launched.record(handle, arguments: args, impostor: impostor)
            return handle
        }
        do {
            _ = try await source.connect()
            Issue.record("a helper that does not run as root was accepted on the administrator path")
        } catch let error as CameraError {
            #expect(error == .connectionFailed("the connected process runs as uid \(getuid()), expected 0"), "\(error)")
        }
        let args = launched.arguments
        #expect(args.count == 9 && args[0] == "--socket")
        #expect(Array(args.dropFirst(2)) == ["--uid", String(getuid()), "--gid", String(getgid()), "--parent-pid",
                                             String(getpid()), "--no-usb-release"])
        #expect(source.helperSession.liveLaunch == .administrator(skipUSBRelease: true))
        await source.shutdown(timeout: 5)
    }

    /// The real helper on the camera service path, as far as a Mac without the service gets: started as the user, it
    /// passes the app's peer checks and says HELLO in live mode; with no service its librealsense finds no camera
    /// it could open, which the helper reports itself and survives. Skipped where the service is installed: a
    /// connected camera would then really be opened.
    @Test(.enabled(if: TestHelper.available, TestHelper.skipReason),
          .enabled(if: !FileManager.default.fileExists(atPath: RealSenseCameraAccess.installedDaemonPlist),
                   "the RealSense camera service is installed on this Mac: the helper could open a real camera"))
    func realHelperRunsLiveAsTheUser() async throws {
        let helper = try #require(TestHelper.url)
        let source = LiveCameraSource(helperURL: helper, access: .service, cameraService: CameraService(kit: nil))
        let prompted = PromptRecord()
        source.helperSession.privilegedLauncher = prompted.launcher
        do {
            let device = try await source.connect()
            Issue.record("a camera opened without root and without the camera service: \(device.name)")
        } catch let error as CameraError {
            // The helper's own answer to QUERY_DEVICE, not a refused peer or a helper that would not start.
            guard case .commandFailed(let message) = error else {
                Issue.record("unexpected error: \(error)")
                await source.shutdown()
                return
            }
            #expect(!message.isEmpty && !message.contains("sudo"), "\(message)")
        }
        #expect(prompted.count == 0)
        // The helper stays for the next attempt (the camera may be plugged in later), running as this user.
        #expect(source.isConnected)
        let hello = try #require(source.helperHello)
        #expect(hello.mode == "live" && hello.euid == getuid(), "\(hello)")
        let t0 = Date()
        await source.shutdown()
        #expect(!source.isConnected)
        // No USB re-enumeration to wait for (the root helper waits up to 5 s for the macOS drivers).
        #expect(Date().timeIntervalSince(t0) < 10, "the shutdown took \(Date().timeIntervalSince(t0)) s")
    }

    @Test func closingAServiceHelperThatNeverConnectedStopsIt() async throws {
        // Unlike the root helper behind the administrator prompt, a helper started as the user can be signalled.
        let sleep = URL(fileURLWithPath: "/bin/sleep")
        let session = HelperCameraSession(mode: .live, helperURL: sleep, liveLaunch: .asUser)
        let launched = LaunchRecord()
        defer { launched.cleanUp() }
        session.launchOverride = { _, args in
            let handle = try HelperLauncher.launchAsUser(helper: sleep, arguments: ["30"])
            launched.record(handle, arguments: args)
            return handle
        }
        let attempt = Task { try await session.connect() }
        #expect(await eventually { launched.handle != nil })
        let elapsed: Double = await withCheckedContinuation { continuation in
            Thread {
                let t0 = Date()
                session.shutdownBlocking(timeout: 20)
                continuation.resume(returning: Date().timeIntervalSince(t0))
            }.start()
        }
        #expect(elapsed < 8, "closing waited \(elapsed) s for a helper that never connected")
        do {
            _ = try await attempt.value
            Issue.record("connect() succeeded without a helper")
        } catch {}
        #expect(await eventually { launched.handle?.hasExited == true }, "the user helper was left running")
    }

    @Test(.enabled(if: StubHelper.available, "no C compiler"))
    func automaticAccessAsksTheCameraServiceFirst() async throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let stub = try StubHelper.build(in: dir)
        let kit = CameraService.Kit(daemon: dir.appendingPathComponent("realsense-usbd"),
                                    manifest: dir.appendingPathComponent("service.plist"),
                                    installScript: dir.appendingPathComponent("install-camera-access.sh"))
        let sockets = URL(fileURLWithPath: stub.path + ".sockets")

        // Not usable: nothing is started (the stand-in would have logged its socket), nothing is installed, and the
        // error carries what the check found.
        for state in [CameraService.State.notInstalled, .switchedOff, .needsUpdate, .notAnswering("no answer"),
                      .notConsoleUser, .cameraDisabled, .tooManyClients] {
            let installs = PromptRecord()
            let service = CameraService(kit: kit, state: { _ in state }, install: { _, _ in installs.note() })
            let source = LiveCameraSource(helperURL: stub, access: .automatic, cameraService: service)
            let prompted = PromptRecord()
            source.helperSession.privilegedLauncher = prompted.launcher
            await #expect(throws: CameraError.cameraServiceNotReady(.state(state))) { _ = try await source.connect() }
            #expect(prompted.count == 0 && installs.count == 0, "\(state)")
            #expect(!FileManager.default.fileExists(atPath: sockets.path), "\(state): a helper was started")
            await source.shutdown()
        }
        // No service files at all: the same, with that reason.
        let bare = LiveCameraSource(helperURL: stub, access: .automatic, cameraService: CameraService(kit: nil))
        await #expect(throws: CameraError.cameraServiceNotReady(.noKit)) { _ = try await bare.connect() }
        #expect(!FileManager.default.fileExists(atPath: sockets.path))

        // A missing helper is reported before the service is asked.
        let checks = PromptRecord()
        let asked = CameraService(kit: kit, state: { _ in checks.note(); return .ready(serviceVersion: 2) })
        let missing = LiveCameraSource(helperURL: dir.appendingPathComponent("none"), access: .automatic,
                                       cameraService: asked)
        await #expect(throws: CameraError.helperNotFound(dir.appendingPathComponent("none").path)) {
            _ = try await missing.connect()
        }
        #expect(checks.count == 0)

        // Ready: the helper is started as the user.
        let ready = LiveCameraSource(helperURL: stub, access: .automatic, cameraService: asked)
        let prompted = PromptRecord()
        ready.helperSession.privilegedLauncher = prompted.launcher
        await #expect(throws: CameraError.launchFailed("the camera helper exited with status 0")) {
            _ = try await ready.connect()
        }
        #expect(checks.count == 1)
        #expect(prompted.count == 0)
        #expect(FileManager.default.fileExists(atPath: sockets.path), "the helper was not started")
        await ready.shutdown()
    }

    @Test func shutdownDuringTheServiceCheckEndsAnAutomaticConnect() async throws {
        // The session stops a connect that a shutdown overtakes only from session.connect() on, and the check of the
        // camera service runs before that: a helper started after the shutdown would have nobody left to stop it.
        let sleep = URL(fileURLWithPath: "/bin/sleep")
        let kit = CameraService.Kit(daemon: URL(fileURLWithPath: "/nonexistent/realsense-usbd"),
                                    manifest: URL(fileURLWithPath: "/nonexistent/service.plist"),
                                    installScript: URL(fileURLWithPath: "/nonexistent/install-camera-access.sh"))
        let checks = PromptRecord(), launches = PromptRecord()
        let gate = DispatchSemaphore(value: 0)
        let service = CameraService(kit: kit, state: { _ in
            checks.note()
            // The first check takes its time, as when launchd starts the service on demand.
            if checks.count == 1 { gate.wait() }
            return .ready(serviceVersion: 2)
        })
        let source = LiveCameraSource(helperURL: sleep, access: .automatic, cameraService: service)
        // In place of the helper: the launch is counted and ends at once, like a cancelled prompt.
        source.helperSession.launchOverride = launches.launcher
        defer { gate.signal() }
        let attempt = Task { try await source.connect() }
        try #require(await eventually { checks.count == 1 }, "the camera service was not asked")
        await source.shutdown()
        gate.signal()
        await #expect(throws: CameraError.notConnected) { _ = try await attempt.value }
        #expect(launches.count == 0, "a helper was started after the shutdown")
        #expect(!source.isConnected)

        // The source can be used again: the next connect asks the service and starts the helper.
        await #expect(throws: CameraError.authorizationCancelled) { _ = try await source.connect() }
        #expect(checks.count == 2 && launches.count == 1)
        await source.shutdown()
    }
}

/// Counts what must not happen in a test: an administrator launch, an installation.
final class PromptRecord: @unchecked Sendable {
    private let lock = NSLock()
    private var calls = 0

    var count: Int { lock.withLock { calls } }
    func note() { lock.withLock { calls += 1 } }

    /// Stands in for HelperLauncher.launchPrivileged: records the call and reports a cancelled prompt.
    var launcher: (URL, [String]) -> HelperProcessHandle {
        { _, _ in
            self.note()
            let handle = HelperProcessHandle()
            handle.markExited(status: -1, error: .authorizationCancelled)
            return handle
        }
    }
}
