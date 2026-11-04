// License: Apache 2.0. See LICENSE file in root directory.
//
// The listening end the helper connects to: a socket in a fresh 0700 directory of the user's temporary directory
// (path without symbolic links, as the helper requires), accepting exactly one peer that must be the helper binary
// running as the expected user.

import Darwin
import Foundation

final class HelperSocketServer: @unchecked Sendable {
    static let socketName = "camera.sock"

    let directory: String
    let socketPath: String
    private let lock = NSLock()
    private var listenFD: Int32
    private var abortRead: Int32
    private var abortWrite: Int32
    private var aborted = false
    private var removed = false

    init() throws {
        guard let base = realpath(NSTemporaryDirectory(), nil) else {
            throw CameraError.connectionFailed("no temporary directory: \(errnoDescription())")
        }
        let basePath = String(cString: base)
        free(base)
        var template = Array("\(basePath)/IntelFaceScan.XXXXXX".utf8CString)
        guard let made = template.withUnsafeMutableBufferPointer({ mkdtemp($0.baseAddress!) }) else {
            throw CameraError.connectionFailed("cannot create the socket directory: \(errnoDescription())")
        }
        directory = String(cString: made)
        socketPath = directory + "/" + Self.socketName
        listenFD = -1
        abortRead = -1
        abortWrite = -1

        do {
            // mkdtemp already uses 0700; say so explicitly because the helper rejects anything else.
            guard chmod(directory, 0o700) == 0 else {
                throw CameraError.connectionFailed("cannot protect the socket directory: \(errnoDescription())")
            }
            var addr = sockaddr_un()
            let pathBytes = Array(socketPath.utf8)
            guard pathBytes.count < MemoryLayout.size(ofValue: addr.sun_path) else {
                throw CameraError.connectionFailed("socket path too long: \(socketPath)")
            }
            addr.sun_family = sa_family_t(AF_UNIX)
            withUnsafeMutableBytes(of: &addr.sun_path) { raw in
                raw.copyBytes(from: pathBytes)
                raw[pathBytes.count] = 0
            }
            addr.sun_len = UInt8(MemoryLayout<sockaddr_un>.size)

            listenFD = socket(AF_UNIX, SOCK_STREAM, 0)
            guard listenFD >= 0 else { throw CameraError.connectionFailed("socket(): \(errnoDescription())") }
            _ = fcntl(listenFD, F_SETFD, FD_CLOEXEC)
            let bound = withUnsafePointer(to: &addr) {
                $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                    bind(listenFD, $0, socklen_t(MemoryLayout<sockaddr_un>.size))
                }
            }
            guard bound == 0 else { throw CameraError.connectionFailed("bind(): \(errnoDescription())") }
            _ = chmod(socketPath, 0o600)
            guard listen(listenFD, 1) == 0 else { throw CameraError.connectionFailed("listen(): \(errnoDescription())") }

            var fds: [Int32] = [-1, -1]
            guard pipe(&fds) == 0 else { throw CameraError.connectionFailed("pipe(): \(errnoDescription())") }
            abortRead = fds[0]
            abortWrite = fds[1]
            _ = fcntl(abortRead, F_SETFD, FD_CLOEXEC)
            _ = fcntl(abortWrite, F_SETFD, FD_CLOEXEC)
        } catch {
            close()
            throw error
        }
    }

    deinit {
        close()
    }

    /// Waits for the helper to connect. Throws CameraError.timeout, or CameraError.connectionFailed after abort().
    func accept(timeout: TimeInterval) throws -> Int32 {
        let (fd, abortFD): (Int32, Int32) = lock.withLock { (listenFD, abortRead) }
        guard fd >= 0, abortFD >= 0 else { throw CameraError.notConnected }
        let deadline = Date(timeIntervalSinceNow: timeout)
        while true {
            if lock.withLock({ aborted }) { throw CameraError.connectionFailed("connection attempt aborted") }
            let remaining = deadline.timeIntervalSinceNow
            guard remaining > 0 else {
                throw CameraError.timeout("the camera helper did not connect in \(Int(timeout)) s")
            }
            var polls = [pollfd(fd: fd, events: Int16(POLLIN), revents: 0),
                         pollfd(fd: abortFD, events: Int16(POLLIN), revents: 0)]
            let n = poll(&polls, 2, Int32(min(remaining, 1.0) * 1000))
            if n < 0 {
                if errno == EINTR { continue }
                throw CameraError.connectionFailed("poll(): \(errnoDescription())")
            }
            if polls[1].revents != 0 { throw CameraError.connectionFailed("connection attempt aborted") }
            if polls[0].revents & Int16(POLLIN) != 0 {
                let client = Darwin.accept(fd, nil, nil)
                if client < 0 {
                    if errno == EINTR || errno == ECONNABORTED || errno == EAGAIN { continue }
                    throw CameraError.connectionFailed("accept(): \(errnoDescription())")
                }
                _ = fcntl(client, F_SETFD, FD_CLOEXEC)
                return client
            }
        }
    }

    /// Makes a waiting accept() fail right away (helper exited, user cancelled, shutdown).
    func abort() {
        lock.withLock {
            aborted = true
            if abortWrite >= 0 {
                var byte: UInt8 = 1
                _ = write(abortWrite, &byte, 1)
            }
        }
    }

    /// No second peer can connect once the helper is accepted.
    func stopListening() {
        lock.withLock {
            if listenFD >= 0 {
                Darwin.close(listenFD)
                listenFD = -1
            }
            unlink(socketPath)
        }
    }

    /// Closes everything and removes the socket and its directory. Idempotent.
    func close() {
        stopListening()
        lock.withLock {
            for fd in [abortRead, abortWrite] where fd >= 0 { Darwin.close(fd) }
            abortRead = -1
            abortWrite = -1
            if !removed {
                removed = true
                rmdir(directory)
            }
        }
    }

    // MARK: Peer verification

    /// Checks the connected peer: effective uid (LOCAL_PEERCRED), executable (LOCAL_PEERPID + proc_pidpath, the
    /// same file as `helperPath`) and, when the app knows it (a helper it started directly), the pid. Returns the
    /// peer's pid.
    static func verifyPeer(fd: Int32, expectedUID: uid_t, helperPath: String, expectedPID: pid_t? = nil) throws -> pid_t {
        var cred = xucred()
        var length = socklen_t(MemoryLayout<xucred>.size)
        guard getsockopt(fd, SOL_LOCAL, LOCAL_PEERCRED, &cred, &length) == 0,
              cred.cr_version == UInt32(XUCRED_VERSION) else {
            throw CameraError.connectionFailed("cannot read the credentials of the connected process")
        }
        guard cred.cr_uid == expectedUID else {
            throw CameraError.connectionFailed(
                "the connected process runs as uid \(cred.cr_uid), expected \(expectedUID)")
        }
        var pid: pid_t = 0
        length = socklen_t(MemoryLayout<pid_t>.size)
        guard getsockopt(fd, SOL_LOCAL, LOCAL_PEERPID, &pid, &length) == 0, pid > 0 else {
            throw CameraError.connectionFailed("cannot read the pid of the connected process")
        }
        // A process of the same user that connected first and then executed the real helper would pass the checks
        // below; it cannot have the pid of the process the app started.
        if let expectedPID, pid != expectedPID {
            throw CameraError.connectionFailed(
                "the connected process (pid \(pid)) is not the camera helper the app started (pid \(expectedPID))")
        }
        var buffer = [CChar](repeating: 0, count: 4 * Int(MAXPATHLEN))
        guard proc_pidpath(pid, &buffer, UInt32(buffer.count)) > 0 else {
            throw CameraError.connectionFailed("cannot read the executable of process \(pid)")
        }
        let peerPath = String(cString: buffer)
        guard sameFile(peerPath, helperPath) else {
            throw CameraError.connectionFailed("the connected process (\(peerPath)) is not the camera helper")
        }
        return pid
    }

    /// Same inode on the same device (the paths may differ by symbolic links or firmlinks).
    static func sameFile(_ a: String, _ b: String) -> Bool {
        var sa = stat(), sb = stat()
        guard stat(a, &sa) == 0, stat(b, &sb) == 0 else { return false }
        return sa.st_dev == sb.st_dev && sa.st_ino == sb.st_ino
    }
}
