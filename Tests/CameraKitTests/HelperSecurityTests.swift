// License: Apache 2.0. See LICENSE file in root directory.
//
// The privileged boundary under attack (docs/HELPER.md, threat model): the real helper in --fake mode driven by a
// hostile peer, the inherited state it starts with, the launch quoting through AppleScript and sh, and where the
// helper may be taken from.

import Darwin
import Foundation
import Testing
@testable import CameraKit

@Suite("Helper security (hostile peer)", .serialized, .enabled(if: TestHelper.available, TestHelper.skipReason),
       .timeLimit(.minutes(2)))
struct HelperSecurityPeerTests {
    private var helper: URL { TestHelper.url ?? URL(fileURLWithPath: "/nonexistent") }

    private func nested(_ depth: Int) -> Data {
        // {"request_id":1,"x":[[...]]}: the object itself is level 1.
        Data(("{\"request_id\":1,\"x\":" + String(repeating: "[", count: depth - 1)
              + String(repeating: "]", count: depth - 1) + "}").utf8)
    }

    private func errorText(_ message: WireMessage?) throws -> String? {
        guard let message else { return nil }
        let object = try JSONSerialization.jsonObject(with: message.json) as? [String: Any]
        return object?["error"] as? String
    }

    @Test func deeplyNestedBodiesCloseTheConnectionInsteadOfCrashing() throws {
        // ~100000 levels in 200 KB (under the 256 KiB limit): copying such a value recursed once per level and
        // overflowed the stack of the root helper (SIGSEGV, camera left captured).
        let client = try RawHelperClient(helper: helper)
        try client.io.writeMessage(type: HelperMessageType.configure.rawValue, json: nested(100_000))
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 10))
        #expect(client.process.exitStatus == 0, "the helper must end its session, not crash")
    }

    @Test func nestingUpToTheLimitIsAnsweredAndBracketsInStringsDoNotCount() throws {
        let client = try RawHelperClient(helper: helper)
        // 32 levels: a valid message, so the command is answered (and refused for its unknown field).
        try client.io.writeMessage(type: HelperMessageType.queryDevice.rawValue, json: nested(32))
        #expect(try errorText(client.read(until: .reply)) == "unknown field 'x'")
        // Brackets and escaped quotes inside a string are text.
        let text = "{\"request_id\":2,\"x\":\"\\\"" + String(repeating: "[{", count: 1000) + "\"}"
        try client.io.writeMessage(type: HelperMessageType.queryDevice.rawValue, json: Data(text.utf8))
        #expect(try errorText(client.read(until: .reply)) == "unknown field 'x'")
        // 33 levels: a protocol violation.
        try client.io.writeMessage(type: HelperMessageType.queryDevice.rawValue, json: nested(33))
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 10) && client.process.exitStatus == 0)
    }

    private func usage(_ pid: pid_t) -> rusage_info_v4? {
        var info = rusage_info_v4()
        let rc = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: rusage_info_t?.self, capacity: 1) { proc_pid_rusage(pid, RUSAGE_INFO_V4, $0) }
        }
        return rc == 0 ? info : nil
    }

    /// Threads of a process that are on a CPU right now (nil once it is gone).
    private func runningThreads(_ pid: pid_t) -> Int? {
        var handles = [UInt64](repeating: 0, count: 256)
        let bytes = handles.withUnsafeMutableBytes { raw in
            proc_pidinfo(pid, PROC_PIDLISTTHREADS, 0, raw.baseAddress, Int32(raw.count))
        }
        guard bytes > 0 else { return nil }
        var running = 0
        for handle in handles.prefix(Int(bytes) / MemoryLayout<UInt64>.size) {
            var info = proc_threadinfo()
            let size = Int32(MemoryLayout<proc_threadinfo>.size)
            if proc_pidinfo(pid, PROC_PIDTHREADINFO, handle, &info, size) == size,
               info.pth_run_state == TH_STATE_RUNNING {
                running += 1
            }
        }
        return running
    }

    /// Waits until no thread of the helper ran for a few samples in a row: it ran every command sent so far (bounded
    /// wait). Its CPU time is no indicator: it only advances when a running thread is switched out.
    private func waitUntilIdle(_ pid: pid_t) {
        var quiet = 0
        for _ in 0..<2000 where quiet < 3 {
            guard let running = runningThreads(pid) else { return }
            quiet = running == 0 ? quiet + 1 : 0
            Thread.sleep(forTimeInterval: 0.001)
        }
    }

    /// Sends QUERY_DEVICE commands in batches the helper runs before the next one arrives (a burst beyond 1024 queued
    /// commands ends the session for another reason) and never reads a reply.
    /// Returns the number sent until the helper closed the connection, and its peak footprint meanwhile.
    private func flood(_ client: RawHelperClient, _ body: [String: Any], upTo limit: Int) -> (sent: Int, peak: UInt64) {
        let pid = client.hello.pid
        var peak = usage(pid)?.ri_lifetime_max_phys_footprint ?? 0
        var sent = 0
        while sent < limit {
            do {
                try client.send(HelperMessageType.queryDevice.rawValue, body)
            } catch {
                break  // the helper closed the connection
            }
            sent += 1
            if sent % 512 == 0 {
                waitUntilIdle(pid)
                guard let info = usage(pid) else { break }
                peak = max(peak, info.ri_lifetime_max_phys_footprint)
            }
        }
        return (sent, peak)
    }

    @Test func floodingCommandsWithoutReadingEndsTheSession() throws {
        // A peer that sends commands and never reads the replies: they piled up in the (root) helper without a bound,
        // ~4 KB per QUERY_DEVICE here (12 MB -> 131 MB after 20000 commands with a camera's longer replies). Now the
        // session ends once 4096 replies, events and logs or 16 MiB wait behind the socket buffer ("the app stopped
        // reading"), through the normal shutdown.
        let client = try RawHelperClient(helper: helper)
        let (sent, peak) = flood(client, [:], upTo: 40_000)
        #expect(client.process.waitForExit(timeout: 10), "the helper kept its session after \(sent) unread replies")
        #expect(client.process.exitStatus == 0, "the session must end through the normal shutdown")
        // About 30 MB (16 MiB of replies plus the helper itself); unbounded it was past 240 MB after 40000 commands.
        #expect(peak < 64 * 1024 * 1024, "helper footprint peaked at \(peak >> 20) MiB after \(sent) commands")
    }

    @Test func manyUnreadRepliesEndTheSessionBeforeTheirBytesDo() throws {
        // The count bound: 2 KB error replies ("unknown field 'xx...'") reach 4096 queued messages at 8 MiB. About 2000
        // of them fit into the socket buffer first.
        let client = try RawHelperClient(helper: helper)
        let (sent, _) = flood(client, [String(repeating: "x", count: 2000): 1], upTo: 40_000)
        #expect(client.process.waitForExit(timeout: 10), "the helper kept its session after \(sent) unread replies")
        #expect(client.process.exitStatus == 0)
        #expect(sent < 20_000, "the session should end after about 4096 queued replies, not \(sent)")
    }

    @Test func setIDRecordingDestinationIsRefused() throws {
        // Root writing through the descriptor would keep the set-id bits the kernel clears for any other writer.
        let client = try RawHelperClient(helper: helper)
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let path = dir.appendingPathComponent("setid.db3").path
        #expect(FileManager.default.createFile(atPath: path, contents: Data()))
        #expect(chmod(path, 0o4644) == 0)
        var st = stat()
        #expect(stat(path, &st) == 0 && st.st_mode & S_ISUID != 0)
        let fd = open(path, O_WRONLY)
        defer { close(fd) }
        let id = try client.send(HelperMessageType.recordStart.rawValue, ["format": "db3"], descriptor: fd)
        #expect((try client.reply(to: id)?["error"] as? String)?.contains("set-user-ID") == true)
    }

    @Test func staleRecordingSpoolsAreRemoved() throws {
        // A helper killed in the middle of a recording leaves its spool in a directory only its user (root, live) can
        // enter. The fake helper spools in this user's Darwin temporary directory, so the same rule is visible here.
        var buffer = [CChar](repeating: 0, count: Int(PATH_MAX))
        try #require(confstr(_CS_DARWIN_USER_TEMP_DIR, &buffer, buffer.count) > 0)
        let base = try #require(realpath(String(cString: buffer), nil).map { p in defer { free(p) }; return String(cString: p) })
        let tag = UUID().uuidString
        let old = base + "/ifs-recording.sectest-old-" + tag
        let fresh = base + "/ifs-recording.sectest-fresh-" + tag
        let foreign = base + "/other.sectest-" + tag  // not a spool name: never touched
        for dir in [old, fresh, foreign] {
            #expect(mkdir(dir, 0o700) == 0)
            #expect(FileManager.default.createFile(atPath: dir + "/recording.db3", contents: Data([1, 2, 3])))
        }
        defer { for dir in [old, fresh, foreign] { try? FileManager.default.removeItem(atPath: dir) } }
        let twoDaysAgo = Date().addingTimeInterval(-2 * 86400)
        for dir in [old, foreign] {
            try FileManager.default.setAttributes([.modificationDate: twoDaysAgo], ofItemAtPath: dir)
        }

        let client = try RawHelperClient(helper: helper)
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let fd = open(dir.appendingPathComponent("out.db3").path, O_WRONLY | O_CREAT | O_TRUNC, 0o644)
        defer { close(fd) }
        let start = try client.send(HelperMessageType.recordStart.rawValue, ["format": "db3"], descriptor: fd)
        #expect(try client.reply(to: start)?["ok"] as? Bool == true)
        let stop = try client.send(HelperMessageType.recordStop.rawValue)
        #expect(try client.reply(to: stop)?["ok"] as? Bool == true)

        #expect(!FileManager.default.fileExists(atPath: old), "a spool untouched for two days is removed")
        #expect(FileManager.default.fileExists(atPath: fresh + "/recording.db3"), "a recent spool is kept")
        #expect(FileManager.default.fileExists(atPath: foreign + "/recording.db3"), "other names are not touched")
    }

    @Test func overflowingRegionIsRefused() throws {
        let client = try RawHelperClient(helper: helper)
        let id = try client.send(HelperMessageType.setROI.rawValue,
                                 ["x": 2_147_483_000, "y": 0, "width": 2000, "height": 10])
        #expect(try client.reply(to: id)?["ok"] as? Bool == false)
    }

    @Test func inheritedDescriptorAboveTheSoftLimitIsClosed() throws {
        // The helper must not keep descriptors from whoever started it, also above a soft RLIMIT_NOFILE the parent
        // lowered after opening them (a sweep up to the limit misses those). Observable: the write end of a pipe
        // moved to descriptor 1000 reaches EOF on our side while the helper still runs.
        let server = try HelperSocketServer()
        defer { server.close() }
        var pipeFDs: [Int32] = [-1, -1]
        #expect(pipe(&pipeFDs) == 0)
        let readEnd = pipeFDs[0]
        defer { close(readEnd) }

        let script = "ulimit -Sn 4096 && exec 1000>&3 3>&- && ulimit -Sn 256 && exec \"$0\" \"$@\""
        let arguments = ["/bin/sh", "-c", script, helper.path, "--socket", server.socketPath, "--uid", String(getuid()),
                         "--parent-pid", String(getpid()), "--fake"]
        var actions: posix_spawn_file_actions_t?
        posix_spawn_file_actions_init(&actions)
        defer { posix_spawn_file_actions_destroy(&actions) }
        posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0)
        posix_spawn_file_actions_addopen(&actions, 1, "/dev/null", O_WRONLY, 0)
        posix_spawn_file_actions_addopen(&actions, 2, "/dev/null", O_WRONLY, 0)
        posix_spawn_file_actions_adddup2(&actions, pipeFDs[1], 3)
        var attributes: posix_spawnattr_t?
        posix_spawnattr_init(&attributes)
        defer { posix_spawnattr_destroy(&attributes) }
        posix_spawnattr_setflags(&attributes, Int16(POSIX_SPAWN_CLOEXEC_DEFAULT))
        let argv: [UnsafeMutablePointer<CChar>?] = arguments.map { strdup($0) } + [nil]
        defer { argv.forEach { free($0) } }
        var envp: [UnsafeMutablePointer<CChar>?] = [nil]
        var pid: pid_t = 0
        let spawned = posix_spawn(&pid, "/bin/sh", &actions, &attributes, argv, &envp)
        close(pipeFDs[1])
        try #require(spawned == 0, "posix_spawn: \(errnoDescription(spawned))")
        defer {
            // Reaped here: the session ends when our end of the socket closes (server and fd closed before this).
            var status: Int32 = 0
            for _ in 0..<100 where waitpid(pid, &status, WNOHANG) == 0 { usleep(50_000) }
            if waitpid(pid, &status, WNOHANG) == 0 { kill(pid, SIGKILL); waitpid(pid, &status, 0) }
        }

        let fd = try server.accept(timeout: 10)
        let io = HelperSocketIO(fd: fd)
        defer { io.close() }
        var tv = timeval(tv_sec: 10, tv_usec: 0)
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        let hello = try #require(try io.readMessage())
        #expect(hello.type == HelperMessageType.hello.rawValue)

        var poller = pollfd(fd: readEnd, events: Int16(POLLIN), revents: 0)
        let ready = poll(&poller, 1, 5000)
        var byte: UInt8 = 0
        let eof = ready == 1 && read(readEnd, &byte, 1) == 0
        #expect(eof, "descriptor 1000 is still open in the helper")
        io.shutdown()
    }
}

@Suite("Helper security (launch and location)")
struct HelperSecurityLaunchTests {
    @Test func quotingSurvivesAppleScriptAndShell() throws {
        // The exact privileged command, run through NSAppleScript and /bin/sh without the administrator clause:
        // every argument must arrive byte for byte, whatever quotes, newlines or shell syntax it contains.
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let odd = dir.appendingPathComponent("it's \"odd\" \\ $HOME `id` ; |\nnew line é 🎥")
        try FileManager.default.createDirectory(at: odd, withIntermediateDirectories: true)
        let shell = odd.appendingPathComponent("sh")
        try FileManager.default.createSymbolicLink(atPath: shell.path, withDestinationPath: "/bin/sh")
        let output = dir.appendingPathComponent("argv")
        let tricky = ["--socket", odd.appendingPathComponent("camera.sock").path, "'", "\"", "\\", "\\\"", "$(id)",
                      "`id`", "a\nb", "a\rb", "tab\there", "-n", "", "* ?", "é 🎥", "\\n", "}{"]
        let script = "printf '%s\\0' \"$@\" > \"$0\""
        let command = HelperLauncher.privilegedShellCommand(helper: shell.path,
                                                            arguments: ["-c", script, output.path] + tricky)
        let source = HelperLauncher.appleScriptSource(command: command, prompt: "unused", privileged: false)
        // osascript, not NSAppleScript: in a process without an AppKit event loop (the test runner) an in-process
        // "do shell script" waits for an Apple event nobody dispatches. The app has the loop; the source is the same.
        let osascript = Process()
        osascript.executableURL = URL(fileURLWithPath: "/usr/bin/osascript")
        osascript.arguments = ["-e", source]
        let errors = Pipe()
        osascript.standardError = errors
        osascript.standardOutput = FileHandle.nullDevice
        let finished = DispatchSemaphore(value: 0)
        osascript.terminationHandler = { _ in finished.signal() }
        try osascript.run()
        if finished.wait(timeout: .now() + 60) == .timedOut {
            osascript.terminate()
            Issue.record("osascript did not finish")
            return
        }
        let stderr = String(decoding: errors.fileHandleForReading.readDataToEndOfFile(), as: UTF8.self)
        #expect(osascript.terminationStatus == 0, "\(stderr)")
        let data = try Data(contentsOf: output)
        var received = data.split(separator: 0, omittingEmptySubsequences: false).map {
            String(decoding: $0, as: UTF8.self)
        }
        #expect(received.last == "")  // the terminator of the last argument
        received.removeLast()
        #expect(received == tricky)
    }

    @Test func searchSkipsHelpersOthersCanReplace() throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let shared = dir.appendingPathComponent("shared")
        let start = shared.appendingPathComponent("repo/.build/debug")
        let planted = shared.appendingPathComponent("build/helper/IntelFaceScanHelper")
        try FileManager.default.createDirectory(at: start, withIntermediateDirectories: true)
        try FileManager.default.createDirectory(at: planted.deletingLastPathComponent(),
                                                withIntermediateDirectories: true)
        try FileManager.default.copyItem(atPath: "/bin/echo", toPath: planted.path)
        func real(_ path: String) -> String? { realpath(path, nil).map { p in defer { free(p) }; return String(cString: p) } }
        let resolved = try #require(real(planted.path))

        // A world-writable folder without the sticky bit: anyone could have put the binary there or swap it later.
        #expect(chmod(shared.path, 0o777) == 0)
        #expect(HelperLocation.problem(planted.path)?.contains("writable by everyone") == true)
        #expect(HelperLocation.searchBuildFolders(from: start) == nil)

        // Sticky (like /Users/Shared and /private/tmp): other users cannot touch entries they do not own.
        #expect(chmod(shared.path, 0o1777) == 0)
        #expect(HelperLocation.problem(planted.path) == nil)
        #expect(HelperLocation.searchBuildFolders(from: start).flatMap { real($0.path) } == resolved)

        // Writable by a group of ordinary users (staff), or a binary anyone may rewrite.
        #expect(chmod(shared.path, 0o755) == 0 && chown(shared.path, getuid(), 20) == 0)
        #expect(chmod(shared.path, 0o775) == 0)
        #expect(HelperLocation.problem(planted.path)?.contains("writable by group 20") == true)
        #expect(chmod(shared.path, 0o755) == 0 && chmod(planted.path, 0o757) == 0)
        #expect(HelperLocation.problem(planted.path)?.contains("writable by everyone") == true)
        #expect(chmod(planted.path, 0o755) == 0)

        // Owned by somebody else (seen as another uid).
        #expect(HelperLocation.problem(planted.path, uid: getuid() + 1)?.contains("another user") == true)
        #expect(HelperLocation.searchBuildFolders(from: start, uid: getuid() + 1) == nil)

        // The repository's own build folder stays usable.
        #expect(HelperLocation.problem(planted.path) == nil)
        if let built = TestHelper.url { #expect(HelperLocation.problem(built.path) == nil) }
    }
}
