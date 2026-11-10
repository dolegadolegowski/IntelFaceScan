// License: Apache 2.0. See LICENSE file in root directory.
//
// Starting the helper. Playback, the fake camera and the live camera through the camera service run the helper
// directly as the user. The live camera without the service needs root: the helper then runs through
// `do shell script ... with administrator privileges` (the standard macOS password prompt), which blocks until the
// helper exits, so it runs on its own queue.
//
// The script runs in-process (NSAppleScript) only in an AppKit application (NSApp exists, as in IntelFaceScan.app):
// there the prompt belongs to the app. Without one - a command line tool, the test runner - an in-process
// "do shell script" can wait forever for an Apple event that nobody dispatches, so the same script goes through
// /usr/bin/osascript instead (the prompt then names osascript).

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

    /// The helper's own pid when the app started it directly (playback, fake, live through the camera service). nil
    /// for the privileged helper: the administrator launch runs it through a root shell whose pid the app never
    /// learns.
    var processIdentifier: pid_t? {
        condition.withLock {
            guard let process, process.processIdentifier > 0 else { return nil }
            return process.processIdentifier
        }
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
    /// What the helper wrote to stderr (startup errors; it logs over the socket once connected), or everything it
    /// printed for a privileged run that captures its output (HelperLauncher.launchPrivileged(captureOutput:)).
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

    /// Everything that shows an administrator prompt runs here, one after another: the root helper (for its whole
    /// life), the release tool, and the camera service's installer (CameraService.install). Two prompts never show
    /// at once, and the in-process AppleScript always runs in the same kind of place.
    static let authorizationQueue = DispatchQueue(
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
    /// helper fails before it can report over the socket. `captureOutput` (short tools such as `--release`) sends
    /// stderr to stdout instead: "do shell script" returns stdout on success and, when stderr is empty, reports it as
    /// the error message on failure, so the tool's lines come back either way.
    static func privilegedShellCommand(helper: String, arguments: [String], captureOutput: Bool = false) -> String {
        (["exec", "/usr/bin/env", "-i", shellQuote(helper)] + arguments.map(shellQuote)).joined(separator: " ")
            + (captureOutput ? " </dev/null 2>&1" : " </dev/null >/dev/null")
    }

    /// `privileged: false` only exists so tests can run the exact quoting through AppleScript and sh without a prompt.
    static func appleScriptSource(command: String, prompt: String, privileged: Bool = true) -> String {
        guard privileged else { return "do shell script \(appleScriptQuote(command))" }
        return "do shell script \(appleScriptQuote(command)) with administrator privileges with prompt \(appleScriptQuote(prompt))"
    }

    static func checkExecutable(_ helper: URL) throws {
        guard FileManager.default.isExecutableFile(atPath: helper.path) else {
            throw CameraError.helperNotFound(helper.path)
        }
    }

    /// Runs the helper as root after the administrator prompt. The returned handle reports the exit; a cancelled
    /// prompt ends it with CameraError.authorizationCancelled. With `captureOutput` the handle's diagnostics hold
    /// what the helper printed (stdout and stderr), also when it succeeded.
    static func launchPrivileged(helper: URL, arguments: [String], prompt: String = authorizationPrompt,
                                 captureOutput: Bool = false) -> HelperProcessHandle {
        let command = privilegedShellCommand(helper: helper.path, arguments: arguments, captureOutput: captureOutput)
        return runAppleScript(appleScriptSource(command: command, prompt: prompt))
    }

    /// True in an AppKit application (NSApp exists), looked up at run time so CameraKit does not link AppKit.
    static var hasAppKitApplication: Bool {
        // RTLD_DEFAULT: every image loaded in the process. Without AppKit there is no NSApp symbol at all.
        guard let symbol = dlsym(UnsafeMutableRawPointer(bitPattern: -2), "NSApp") else { return false }
        return symbol.load(as: UnsafeRawPointer?.self) != nil
    }

    /// Runs an AppleScript on the authorization queue and reports its end on the returned handle: in-process in an
    /// AppKit application, through /usr/bin/osascript otherwise (see the top of this file). `inProcess` overrides the
    /// choice (tests).
    static func runAppleScript(_ source: String, inProcess: Bool? = nil) -> HelperProcessHandle {
        let handle = HelperProcessHandle()
        let local = inProcess ?? hasAppKitApplication
        authorizationQueue.async {
            if local {
                var info: NSDictionary?
                guard let script = NSAppleScript(source: source) else {
                    handle.markExited(status: -1, error: .launchFailed("cannot create the authorization script"))
                    return
                }
                let result = script.executeAndReturnError(&info)
                if let info {
                    finish(handle, errorNumber: (info[NSAppleScript.errorNumber] as? NSNumber)?.intValue ?? 0,
                           message: info[NSAppleScript.errorMessage] as? String ?? "unknown error")
                } else {
                    finish(handle, output: result.stringValue ?? "")
                }
            } else {
                runOSAScript(source, handle: handle)
            }
        }
        return handle
    }

    /// The end of a "do shell script": its output on success, or AppleScript's error number and message.
    private static func finish(_ handle: HelperProcessHandle, output: String) {
        handle.markExited(status: 0, error: nil, message: normalizedLines(output))
    }

    private static func finish(_ handle: HelperProcessHandle, errorNumber number: Int, message: String) {
        if number == -128 {
            handle.markExited(status: -1, error: .authorizationCancelled)
        } else if (1...255).contains(number) {
            // "do shell script" reports a non-zero exit status as the error number, with stderr as the message.
            handle.markExited(status: Int32(number), error: nil, message: normalizedLines(message))
        } else {
            handle.markExited(status: -1, error: .launchFailed("\(message) (\(number))"))
        }
    }

    /// AppleScript returns the shell's lines separated by carriage returns.
    static func normalizedLines(_ text: String) -> String {
        text.replacingOccurrences(of: "\r\n", with: "\n").replacingOccurrences(of: "\r", with: "\n")
            .trimmingCharacters(in: .whitespacesAndNewlines)
    }

    /// osascript prints the result on stdout, or "<range>: execution error: <message> (<number>)" on stderr.
    private static func runOSAScript(_ source: String, handle: HelperProcessHandle) {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/osascript")
        process.arguments = ["-e", source]
        process.environment = [:]
        process.standardInput = FileHandle.nullDevice
        let stdout = Pipe(), stderr = Pipe()
        process.standardOutput = stdout
        process.standardError = stderr
        let exited = DispatchSemaphore(value: 0)
        process.terminationHandler = { _ in exited.signal() }
        do {
            try process.run()
        } catch {
            handle.markExited(status: -1, error: .launchFailed("/usr/bin/osascript: \(error.localizedDescription)"))
            return
        }
        // Both pipes at once: a full one would block osascript before it exits.
        let errors = OSAScriptErrorOutput()
        DispatchQueue.global(qos: .utility).async { errors.read(stderr.fileHandleForReading) }
        let outputData = stdout.fileHandleForReading.readDataToEndOfFile()
        let errorData = errors.wait()
        exited.wait()
        if process.terminationStatus == 0 {
            finish(handle, output: String(decoding: outputData, as: UTF8.self))
            return
        }
        let text = String(decoding: errorData, as: UTF8.self).trimmingCharacters(in: .whitespacesAndNewlines)
        guard let parsed = parseOSAScriptError(text) else {
            handle.markExited(status: -1, error: .launchFailed("osascript failed: \(text)"))
            return
        }
        finish(handle, errorNumber: parsed.number, message: parsed.message)
    }

    private final class OSAScriptErrorOutput: @unchecked Sendable {
        private let done = DispatchSemaphore(value: 0)
        private var data = Data()

        func read(_ handle: FileHandle) {
            data = handle.readDataToEndOfFile()
            done.signal()
        }

        func wait() -> Data {
            done.wait()
            return data
        }
    }

    /// "0:42: execution error: <message> (<number>)" -> (number, message).
    static func parseOSAScriptError(_ text: String) -> (number: Int, message: String)? {
        guard let marker = text.range(of: "execution error: "), text.hasSuffix(")"),
              let open = text.range(of: " (", options: .backwards), open.lowerBound >= marker.upperBound,
              let number = Int(text[open.upperBound..<text.index(before: text.endIndex)]) else { return nil }
        return (number, String(text[marker.upperBound..<open.lowerBound]))
    }

    /// Runs the helper as the user (playback, fake camera, live through the camera service).
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
