// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared pieces of the helper tests: locating the helper binary, timeouts, event recording and a raw protocol
// client that talks to a helper started in --fake mode without the HelperConnection layer.

import Darwin
import Foundation
import Testing
@testable import CameraKit

/// osascript in tests. A test never waits for a script on a thread of the Swift cooperative pool: macOS starts no
/// block on the global dispatch queues while as many pool threads as the Mac has cores are in use, running or
/// blocked. The launcher and the camera service client used to read osascript's stderr in such a block, so on the
/// 3-core CI runner three test bodies blocked in waits (two of them for scripts) kept those readers from starting:
/// the scripts never "finished" although osascript had exited, and no other test of the target ended for six
/// minutes, until the launcher test's three 120 s waits ran out (CI run 42973936645; the machine was idle
/// meanwhile). The readers are threads now; the tests still wait on threads of their own and run their scripts in
/// the order of the launcher's serial authorization queue.
enum OSAScriptInTests {
    /// How long a test waits for its script: every script queued before it runs first, each several seconds on the
    /// runner. It only catches an osascript that never ends.
    static let timeout: TimeInterval = 900

    /// Runs `work` (which may start osascript itself) on the authorization queue, without holding a thread of the
    /// Swift cooperative pool while it waits its turn or runs.
    static func serialized<T>(_ work: @escaping () throws -> T) async throws -> T {
        try await HelperCameraSession.blocking { try HelperLauncher.authorizationQueue.sync(execute: work) }
    }

    /// Waits for scripts the launcher has queued (HelperLauncher.runAppleScript) off the cooperative pool. When
    /// one is not done after two minutes, the osascript children and a sample of this process (where the
    /// authorization queue and the pool threads wait) go to the log once, so a stuck run says why; the wait goes on
    /// to `timeout`.
    static func wait(for handles: [HelperProcessHandle]) async throws -> Bool {
        try await HelperCameraSession.blocking {
            let deadline = Date(timeIntervalSinceNow: timeout)
            var reported = false
            for handle in handles {
                while !handle.waitForExit(timeout: 120) {
                    if !reported {
                        reported = true
                        reportStuckScripts()
                    }
                    if Date() > deadline { return false }
                }
            }
            return true
        }
    }

    private static func reportStuckScripts() {
        func output(_ tool: String, _ arguments: [String]) -> String {
            let process = Process()
            process.executableURL = URL(fileURLWithPath: tool)
            process.arguments = arguments
            let pipe = Pipe()
            process.standardOutput = pipe
            process.standardError = pipe
            guard (try? process.run()) != nil else { return "\(tool) did not start" }
            let data = pipe.fileHandleForReading.readDataToEndOfFile()
            process.waitUntilExit()
            return String(decoding: data, as: UTF8.self)
        }
        let own = ProcessInfo.processInfo.processIdentifier
        var lines = ["an osascript run of the tests is not done after 120 s:"]
        // Only this process's own osascript children, by exact name (a pattern would match unrelated commands).
        let pids = output("/usr/bin/pgrep", ["-P", String(own), "-x", "osascript"])
            .split(separator: "\n").compactMap { Int32($0.trimmingCharacters(in: .whitespaces)) }
        if pids.isEmpty {
            lines.append("no osascript child is running: the wait is in this process")
        } else {
            lines.append(output("/bin/ps", ["-o", "pid,etime,command", "-p", pids.map(String.init).joined(separator: ",")]))
            for pid in pids.prefix(2) {
                lines.append(String(output("/usr/bin/sample", [String(pid), "1", "-mayDie"]).prefix(6000)))
            }
        }
        // This process: which threads wait where (the authorization queue's thread, the cooperative pool's).
        let sample = output("/usr/bin/sample", [String(own), "1", "-mayDie"])
        lines.append(String((sample.components(separatedBy: "Binary Images:").first ?? sample).prefix(40000)))
        FileHandle.standardError.write(Data((lines.joined(separator: "\n") + "\n").utf8))
    }
}

enum TestHelper {
    /// $IFS_HELPER_PATH or build/helper/IntelFaceScanHelper of this checkout (scripts/build.sh or the helper's CMake
    /// build); the integration tests are skipped without it.
    static let url: URL? = {
        if let path = ProcessInfo.processInfo.environment["IFS_HELPER_PATH"], !path.isEmpty,
           FileManager.default.isExecutableFile(atPath: path) {
            return URL(fileURLWithPath: path)
        }
        let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent()
            .deletingLastPathComponent()
        let built = root.appendingPathComponent("build/helper/IntelFaceScanHelper")
        return FileManager.default.isExecutableFile(atPath: built.path) ? built : nil
    }()

    static let available = url != nil
    static let skipReason: Comment =
        "IntelFaceScanHelper not built: run the helper CMake build (build/helper) or set IFS_HELPER_PATH"

    /// make-test-recording (synthetic .db3/.bag recordings): $IFS_MAKE_TEST_RECORDING, or the one the helper build
    /// makes with -DIFS_HELPER_TESTS=ON (build/helper/tests), or the standalone helper-tests build.
    static let recordingTool: URL? = {
        if let path = ProcessInfo.processInfo.environment["IFS_MAKE_TEST_RECORDING"], !path.isEmpty,
           FileManager.default.isExecutableFile(atPath: path) {
            return URL(fileURLWithPath: path)
        }
        let root = URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent()
            .deletingLastPathComponent()
        var candidates = [root.appendingPathComponent("build/helper/tests/make-test-recording"),
                          root.appendingPathComponent("build/helper-tests/make-test-recording")]
        if let helper = url {
            candidates.insert(helper.deletingLastPathComponent().appendingPathComponent("tests/make-test-recording"), at: 0)
        }
        return candidates.first { FileManager.default.isExecutableFile(atPath: $0.path) }
    }()

    static let playbackAvailable = url != nil && recordingTool != nil
    static let playbackSkipReason: Comment =
        "IntelFaceScanHelper or make-test-recording not built: configure the helper with -DIFS_HELPER_TESTS=ON"

    /// Writes a synthetic recording (make-test-recording, no IR, small frames so tests stay fast) and returns it.
    static func makeRecording(in dir: URL, frames: Int, size: String = "640x480") throws -> URL {
        guard let tool = recordingTool else { throw CameraError.helperNotFound("make-test-recording") }
        let file = dir.appendingPathComponent("synthetic-\(frames).db3")
        let process = Process()
        process.executableURL = tool
        process.arguments = ["-o", file.path, "--frames", String(frames), "--depth", size, "--color", size,
                             "--no-ir", "--fast"]
        process.standardOutput = FileHandle.nullDevice
        let errors = Pipe()
        process.standardError = errors
        try process.run()
        process.waitUntilExit()
        let message = String(decoding: errors.fileHandleForReading.readDataToEndOfFile(), as: UTF8.self)
        guard process.terminationStatus == 0, FileManager.default.fileExists(atPath: file.path) else {
            throw CameraError.commandFailed("make-test-recording failed (\(process.terminationStatus)): \(message)")
        }
        return file
    }

    static func scratchDirectory() throws -> URL {
        let dir = FileManager.default.temporaryDirectory
            .appendingPathComponent("IntelFaceScanTests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }
}

struct TestTimeout: Error, CustomStringConvertible {
    var what: String
    var description: String { "timed out: \(what)" }
}

func withTimeout<T: Sendable>(_ seconds: Double, _ what: String = "operation",
                              _ body: @escaping @Sendable () async throws -> T) async throws -> T {
    try await withThrowingTaskGroup(of: T.self) { group in
        group.addTask { try await body() }
        group.addTask {
            try await Task.sleep(for: .seconds(seconds))
            throw TestTimeout(what: what)
        }
        defer { group.cancelAll() }
        guard let first = try await group.next() else { throw TestTimeout(what: what) }
        return first
    }
}

func collectFrames(_ stream: AsyncStream<CameraFrame>, count: Int, timeout: Double = 10) async throws -> [CameraFrame] {
    try await withTimeout(timeout, "\(count) frames") {
        var frames: [CameraFrame] = []
        for await frame in stream {
            frames.append(frame)
            if frames.count == count { break }
        }
        return frames
    }
}

/// Collects a source's events in the background.
final class EventRecorder: @unchecked Sendable {
    private let lock = NSLock()
    private var recorded: [CameraEvent] = []
    private var finished = false
    private var task: Task<Void, Never>?

    init(_ stream: AsyncStream<CameraEvent>) {
        task = Task { [weak self] in
            for await event in stream { self?.append(event) }
            self?.lock.withLock { self?.finished = true }
        }
    }

    deinit { task?.cancel() }

    private func append(_ event: CameraEvent) {
        lock.withLock { recorded.append(event) }
    }

    var events: [CameraEvent] { lock.withLock { recorded } }
    var streamFinished: Bool { lock.withLock { finished } }

    func wait(timeout: Double, _ predicate: (CameraEvent) -> Bool) async -> CameraEvent? {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while Date() < deadline {
            if let event = events.first(where: predicate) { return event }
            try? await Task.sleep(for: .milliseconds(20))
        }
        return events.first(where: predicate)
    }

    func waitUntilFinished(timeout: Double) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while Date() < deadline && !streamFinished { try? await Task.sleep(for: .milliseconds(20)) }
        return streamFinished
    }
}

extension CameraEvent {
    var helperExitStatus: Int32? {
        if case .helperExited(let status) = self { return status }
        return nil
    }
}

/// A helper in --fake mode driven message by message.
final class RawHelperClient {
    let server: HelperSocketServer
    let process: HelperProcessHandle
    let io: HelperSocketIO
    let hello: HelperHello
    private var nextId: UInt64 = 1

    init(helper: URL, extraArguments: [String] = ["--fake"], parentPid: pid_t = getpid()) throws {
        server = try HelperSocketServer()
        let args = ["--socket", server.socketPath, "--uid", String(getuid()), "--parent-pid", String(parentPid)]
            + extraArguments
        process = try HelperLauncher.launchAsUser(helper: helper, arguments: args)
        let fd: Int32
        do {
            fd = try server.accept(timeout: 10)
        } catch {
            process.terminate()
            server.close()
            throw error
        }
        server.stopListening()
        // Nothing in a test may block forever.
        var tv = timeval(tv_sec: 10, tv_usec: 0)
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        io = HelperSocketIO(fd: fd)
        guard let first = try io.readMessage(), first.type == HelperMessageType.hello.rawValue else {
            throw CameraError.protocolError("no HELLO")
        }
        hello = try HelperJSON.decoder.decode(HelperHello.self, from: first.json)
    }

    deinit {
        io.shutdown()
        if !process.waitForExit(timeout: 3) {
            process.terminate()
            process.waitForExit(timeout: 3)
        }
        io.close()
        server.close()
    }

    @discardableResult
    func send(_ type: UInt16, _ body: [String: Any] = [:], withId: Bool = true, descriptor: Int32? = nil) throws -> UInt64 {
        var object = body
        let id = nextId
        if withId {
            object["request_id"] = id
            nextId += 1
        }
        let json = try JSONSerialization.data(withJSONObject: object)
        try io.writeMessage(type: type, json: json, descriptor: descriptor)
        return id
    }

    /// Reads until a message of `type` arrives (nil: the helper closed the connection).
    func read(until type: HelperMessageType, limit: Int = 10_000) throws -> WireMessage? {
        for _ in 0..<limit {
            guard let message = try io.readMessage() else { return nil }
            if message.type == type.rawValue { return message }
        }
        return nil
    }

    func reply(to id: UInt64) throws -> [String: Any]? {
        while let message = try read(until: .reply) {
            let object = try JSONSerialization.jsonObject(with: message.json) as? [String: Any]
            if (object?["request_id"] as? NSNumber)?.uint64Value == id { return object }
        }
        return nil
    }
}
