// License: Apache 2.0. See LICENSE file in root directory.
//
// Starting the helper. The live camera needs root: the helper runs through
// `do shell script ... with administrator privileges` (the standard macOS password prompt, as the
// realsense-viewer-macos launcher does), which blocks until the helper exits, so it runs on its own queue.
// Playback and the fake camera run the helper directly as the user.

import Darwin
import Foundation

/// A launched helper process: its exit status once it ended, and how the launch failed if it did.
final class HelperProcessHandle: @unchecked Sendable {
    private let condition = NSCondition()
    private var exiting = false
    private var exited = false
    private var status: Int32 = 0
    private var error: CameraError?
    private var handlers: [(Int32, CameraError?) -> Void] = []
    private var stderrTail = Data()
    private var process: Process?

    init() {}

    fileprivate func attach(_ process: Process) {
        condition.withLock { self.process = process }
    }

    fileprivate func appendStderr(_ data: Data) {
        condition.withLock {
            stderrTail.append(data)
            if stderrTail.count > 8192 { stderrTail.removeFirst(stderrTail.count - 8192) }
        }
    }

    /// Records the end of the process. Exit handlers run before waitForExit() returns, so whoever waited for the
    /// exit sees what the handlers did.
    func markExited(status: Int32, error: CameraError?, message: String? = nil) {
        let pending: [(Int32, CameraError?) -> Void] = condition.withLock {
            guard !exiting else { return [] }
            exiting = true
            self.status = status
            self.error = error
            if let message, !message.isEmpty { stderrTail = Data(message.utf8) }
            defer { handlers.removeAll() }
            return handlers
        }
        pending.forEach { $0(status, error) }
        condition.withLock {
            exited = true
            condition.broadcast()
        }
    }

    /// Calls `handler` when the process exits (right away when it already did).
    func onExit(_ handler: @escaping (Int32, CameraError?) -> Void) {
        let immediate: (Int32, CameraError?)? = condition.withLock {
            if exiting { return (status, error) }
            handlers.append(handler)
            return nil
        }
        if let immediate { handler(immediate.0, immediate.1) }
    }

    var hasExited: Bool { condition.withLock { exited } }
    var exitStatus: Int32? { condition.withLock { exited ? status : nil } }
    var launchError: CameraError? { condition.withLock { error } }
    /// What the helper wrote to stderr (startup errors; it logs over the socket once connected).
    var diagnostics: String {
        condition.withLock { String(decoding: stderrTail, as: UTF8.self) }
            .trimmingCharacters(in: .whitespacesAndNewlines)
    }

    @discardableResult
    func waitForExit(timeout: TimeInterval) -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        condition.lock()
        defer { condition.unlock() }
        while !exited {
            if !condition.wait(until: deadline) { return exited }
        }
        return true
    }

    /// Only possible for a helper running as the user: asks it to exit (SIGTERM, handled like QUIT).
    func terminate() {
        let p = condition.withLock { process }
        if let p, p.isRunning { p.terminate() }
    }

    /// Last resort for a user helper that does not react (never used for the root helper, which may be in the middle
    /// of a camera command).
    func kill() {
        let p = condition.withLock { process }
        if let p, p.isRunning { Darwin.kill(p.processIdentifier, SIGKILL) }
    }
}

enum HelperLauncher {
    static let authorizationPrompt =
        "IntelFaceScan needs administrator access to control the RealSense camera over USB."

    private static let authorizationQueue = DispatchQueue(
        label: "io.github.dolegadolegowski.IntelFaceScan.authorization", qos: .userInitiated)

    /// POSIX shell single quoting.
    static func shellQuote(_ s: String) -> String {
        "'" + s.replacingOccurrences(of: "'", with: "'\\''") + "'"
    }

    /// AppleScript string literal.
    static func appleScriptQuote(_ s: String) -> String {
        "\"" + s.replacingOccurrences(of: "\\", with: "\\\\").replacingOccurrences(of: "\"", with: "\\\"") + "\""
    }

    /// The command run as root: an empty environment (nothing from the app's environment reaches the root process,
    /// including DYLD_* variables), no input, output discarded; stderr stays the error message of the script when the
    /// helper fails before it can report over the socket.
    static func privilegedShellCommand(helper: String, arguments: [String]) -> String {
        (["exec", "/usr/bin/env", "-i", shellQuote(helper)] + arguments.map(shellQuote)).joined(separator: " ")
            + " </dev/null >/dev/null"
    }

    static func appleScriptSource(command: String, prompt: String) -> String {
        "do shell script \(appleScriptQuote(command)) with administrator privileges with prompt \(appleScriptQuote(prompt))"
    }

    static func checkExecutable(_ helper: URL) throws {
        guard FileManager.default.isExecutableFile(atPath: helper.path) else {
            throw CameraError.helperNotFound(helper.path)
        }
    }

    /// Runs the helper as root after the administrator prompt. The returned handle reports the exit; a cancelled
    /// prompt ends it with CameraError.authorizationCancelled.
    static func launchPrivileged(helper: URL, arguments: [String], prompt: String = authorizationPrompt)
        -> HelperProcessHandle {
        let handle = HelperProcessHandle()
        let source = appleScriptSource(command: privilegedShellCommand(helper: helper.path, arguments: arguments),
                                       prompt: prompt)
        authorizationQueue.async {
            var info: NSDictionary?
            guard let script = NSAppleScript(source: source) else {
                handle.markExited(status: -1, error: .launchFailed("cannot create the authorization script"))
                return
            }
            _ = script.executeAndReturnError(&info)
            guard let info else {
                handle.markExited(status: 0, error: nil)
                return
            }
            let number = (info[NSAppleScript.errorNumber] as? NSNumber)?.intValue ?? 0
            let message = info[NSAppleScript.errorMessage] as? String ?? "unknown error"
            if number == -128 {
                handle.markExited(status: -1, error: .authorizationCancelled)
            } else if (1...255).contains(number) {
                // "do shell script" reports a non-zero exit status as the error number, with stderr as the message.
                handle.markExited(status: Int32(number), error: nil, message: message)
            } else {
                handle.markExited(status: -1, error: .launchFailed("\(message) (\(number))"))
            }
        }
        return handle
    }

    /// Runs the helper as the user (playback, fake camera, --release without privileges).
    static func launchAsUser(helper: URL, arguments: [String]) throws -> HelperProcessHandle {
        let handle = HelperProcessHandle()
        let process = Process()
        process.executableURL = helper
        process.arguments = arguments
        process.environment = [:]
        process.standardInput = FileHandle.nullDevice
        process.standardOutput = FileHandle.nullDevice
        let stderr = Pipe()
        process.standardError = stderr
        let stderrDone = DispatchSemaphore(value: 0)
        process.terminationHandler = { p in
            let status = p.terminationReason == .uncaughtSignal ? 128 + p.terminationStatus : p.terminationStatus
            // The diagnostics should be complete when the exit is published.
            _ = stderrDone.wait(timeout: .now() + 1)
            handle.markExited(status: status, error: nil)
        }
        handle.attach(process)
        do {
            try process.run()
        } catch {
            try? stderr.fileHandleForWriting.close()
            try? stderr.fileHandleForReading.close()
            throw CameraError.launchFailed("\(helper.path): \(error.localizedDescription)")
        }
        let reader = stderr.fileHandleForReading
        DispatchQueue.global(qos: .utility).async {
            while let data = try? reader.read(upToCount: 4096), !data.isEmpty {
                handle.appendStderr(data)
            }
            try? reader.close()
            stderrDone.signal()
        }
        return handle
    }
}
