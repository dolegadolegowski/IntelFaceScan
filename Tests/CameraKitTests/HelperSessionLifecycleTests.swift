// License: Apache 2.0. See LICENSE file in root directory.
//
// HelperCameraSession with a stand-in helper compiled at test time (needs /usr/bin/cc, otherwise skipped): a helper
// that says HELLO and is gone right away (as one that crashes while opening the device) must not leave a dead
// session behind that every later connect() would reuse.

import Darwin
import Foundation
import Testing
@testable import CameraKit

private enum StubHelper {
    static let compiler = "/usr/bin/cc"
    static var available: Bool { FileManager.default.isExecutableFile(atPath: compiler) }

    /// Appends its --socket path to <its own path>.sockets, connects, sends HELLO (mode fake, or playback with
    /// --playback) and exits.
    static let source = #"""
    #include <stdint.h>
    #include <stdio.h>
    #include <string.h>
    #include <sys/socket.h>
    #include <sys/un.h>
    #include <unistd.h>
    int main(int argc, char **argv) {
        const char *path = 0, *mode = "fake";
        for (int i = 1; i < argc; ++i) {
            if (!strcmp(argv[i], "--socket") && i + 1 < argc) path = argv[++i];
            else if (!strcmp(argv[i], "--playback")) mode = "playback";
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
        let session = HelperCameraSession(mode: .live, helperURL: sleep)
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
