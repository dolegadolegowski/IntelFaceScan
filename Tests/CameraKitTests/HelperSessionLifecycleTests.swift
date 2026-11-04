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
