// License: Apache 2.0. See LICENSE file in root directory.
//
// Framing of docs/PROTOCOL.md on the app side: the 24-byte header (validated by the shared C code of IFSProtocol),
// blocking reads of whole messages and writes that can carry one descriptor (SCM_RIGHTS) on their first byte.

import Darwin
import Foundation
import IFSProtocol

#if _endian(big)
#error("the wire header is exchanged in host byte order, which must be little endian")
#endif

/// Message types of ifs_protocol.h (`ifs_msg_type`).
enum HelperMessageType: UInt16, CaseIterable, Sendable {
    case hello = 1
    case reply = 2
    case frameset = 3
    case event = 4
    case log = 5
    case bye = 6

    case queryDevice = 100
    case configure = 101
    case start = 102
    case stop = 103
    case setOption = 104
    case setROI = 105
    case setFilters = 106
    case recordStart = 107
    case recordStop = 108
    case playback = 109
    case hardwareReset = 110
    case quit = 111
}

enum HelperWire {
    static let headerSize = 24
    /// IFS_MAX_JSON_BYTES / IFS_MAX_PAYLOAD_BYTES (the C macros are expressions Swift does not import; the tests
    /// check these values against ifs_wire_header_valid).
    static let maxJSONBytes = 256 * 1024
    static let maxPayloadBytes = 64 * 1024 * 1024
    static let flagHasDescriptor: UInt32 = 0x1
    /// Socket buffers large enough that a 5 MB frameset is not moved in 8 KiB steps.
    static let socketBufferBytes: Int32 = 4 * 1024 * 1024
}

struct WireHeader: Equatable, Sendable {
    var type: UInt16
    var jsonBytes: UInt32
    var flags: UInt32
    var payloadBytes: UInt64

    func encoded() -> [UInt8] {
        var header = ifs_wire_header()
        ifs_wire_header_init(&header, type, jsonBytes, payloadBytes, flags)
        return withUnsafeBytes(of: &header) { Array($0) }
    }

    /// Decodes and validates a received header (magic, version, sizes, flags).
    static func decode(_ bytes: UnsafeRawBufferPointer) throws -> WireHeader {
        guard bytes.count == HelperWire.headerSize else {
            throw CameraError.protocolError("truncated message header")
        }
        var header = ifs_wire_header()
        withUnsafeMutableBytes(of: &header) { $0.copyMemory(from: bytes) }
        guard ifs_wire_header_valid(&header) == 0 else {
            throw CameraError.protocolError("invalid message header")
        }
        return WireHeader(type: header.type, jsonBytes: header.json_bytes, flags: header.flags,
                          payloadBytes: header.payload_bytes)
    }
}

/// One received message.
struct WireMessage: Sendable {
    var type: UInt16
    var json: Data
    var payload: Data
    /// Only when the reader was asked to accept one (tests acting as the helper); owned by the receiver.
    var descriptor: Int32?
}

// MARK: - Control messages (CMSG_* are macros Swift does not import)

enum ControlMessage {
    private static func align(_ n: Int) -> Int {
        let a = MemoryLayout<UInt32>.size
        return (n + a - 1) & ~(a - 1)
    }

    static var headerSpace: Int { align(MemoryLayout<cmsghdr>.size) }
    /// CMSG_SPACE(sizeof(int) * count)
    static func space(descriptors count: Int) -> Int { headerSpace + align(MemoryLayout<Int32>.size * count) }

    /// A control buffer carrying one descriptor as SCM_RIGHTS.
    static func rights(_ descriptor: Int32) -> [UInt8] {
        var buffer = [UInt8](repeating: 0, count: space(descriptors: 1))
        buffer.withUnsafeMutableBytes { raw in
            let header = cmsghdr(cmsg_len: socklen_t(headerSpace + MemoryLayout<Int32>.size),
                                 cmsg_level: SOL_SOCKET, cmsg_type: SCM_RIGHTS)
            raw.storeBytes(of: header, toByteOffset: 0, as: cmsghdr.self)
            raw.storeBytes(of: descriptor, toByteOffset: headerSpace, as: Int32.self)
        }
        return buffer
    }

    /// The descriptors in a received control buffer.
    static func descriptors(in raw: UnsafeRawBufferPointer, length: Int) -> [Int32] {
        var result: [Int32] = []
        var offset = 0
        let end = min(length, raw.count)
        while offset + MemoryLayout<cmsghdr>.size <= end {
            let header = raw.loadUnaligned(fromByteOffset: offset, as: cmsghdr.self)
            let len = Int(header.cmsg_len)
            guard len >= MemoryLayout<cmsghdr>.size, offset + len <= end else { break }
            if header.cmsg_level == SOL_SOCKET && header.cmsg_type == SCM_RIGHTS {
                let count = (len - headerSpace) / MemoryLayout<Int32>.size
                for i in 0..<max(0, count) {
                    result.append(raw.loadUnaligned(fromByteOffset: offset + headerSpace + i * 4, as: Int32.self))
                }
            }
            offset += align(len)
        }
        return result
    }
}

func errnoDescription(_ code: Int32 = errno) -> String {
    String(cString: strerror(code))
}

// MARK: - Socket I/O

/// Blocking message I/O on a connected AF_UNIX stream socket. Reads happen on one thread (the reader); writes are
/// serialized by a lock and may come from any thread.
final class HelperSocketIO: @unchecked Sendable {
    let fd: Int32
    private let writeLock = NSLock()
    private let stateLock = NSLock()
    private var closed = false

    init(fd: Int32) {
        self.fd = fd
        var one: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, socklen_t(MemoryLayout<Int32>.size))
        var size = HelperWire.socketBufferBytes
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size, socklen_t(MemoryLayout<Int32>.size))
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &size, socklen_t(MemoryLayout<Int32>.size))
        _ = fcntl(fd, F_SETFD, FD_CLOEXEC)
    }

    deinit {
        close()
    }

    /// Reads one message; nil when the peer closed the connection between two messages. Descriptors are rejected
    /// (closed, protocol error) unless `acceptDescriptor`, and then only on the first byte of the message.
    func readMessage(acceptDescriptor: Bool = false) throws -> WireMessage? {
        var headerBytes = [UInt8](repeating: 0, count: HelperWire.headerSize)
        var descriptors: [Int32] = []
        let complete = try headerBytes.withUnsafeMutableBytes { raw in
            try readExact(raw, descriptors: &descriptors, acceptDescriptors: acceptDescriptor, eofOK: true)
        }
        guard complete else { return nil }
        var keep = false
        defer {
            if !keep { descriptors.forEach { _ = Darwin.close($0) } }
        }
        let header = try headerBytes.withUnsafeBytes { try WireHeader.decode($0) }
        let flagged = header.flags & HelperWire.flagHasDescriptor != 0
        guard descriptors.count == (flagged ? 1 : 0) else {
            throw CameraError.protocolError(flagged ? "descriptor missing" : "unexpected file descriptor")
        }
        guard header.jsonBytes > 0 else { throw CameraError.protocolError("message without a JSON body") }

        var none: [Int32] = []
        var json = Data(count: Int(header.jsonBytes))
        _ = try json.withUnsafeMutableBytes {
            try readExact($0, descriptors: &none, acceptDescriptors: false, eofOK: false)
        }
        var payload = Data()
        if header.payloadBytes > 0 {
            payload = Data(count: Int(header.payloadBytes))
            _ = try payload.withUnsafeMutableBytes {
                try readExact($0, descriptors: &none, acceptDescriptors: false, eofOK: false)
            }
        }
        keep = true
        return WireMessage(type: header.type, json: json, payload: payload, descriptor: descriptors.first)
    }

    /// Returns false on a clean EOF before the first byte (only with eofOK).
    private func readExact(_ buffer: UnsafeMutableRawBufferPointer, descriptors: inout [Int32], acceptDescriptors: Bool,
                           eofOK: Bool) throws -> Bool {
        guard let base = buffer.baseAddress else { return true }
        var done = 0
        let controlSize = ControlMessage.space(descriptors: 16)
        var control = [UInt8](repeating: 0, count: controlSize)
        while done < buffer.count {
            var iov = iovec(iov_base: base + done, iov_len: buffer.count - done)
            let (n, flags, received): (Int, Int32, [Int32]) = control.withUnsafeMutableBytes { controlRaw in
                withUnsafeMutablePointer(to: &iov) { iovPointer in
                    var msg = msghdr()
                    msg.msg_iov = iovPointer
                    msg.msg_iovlen = 1
                    msg.msg_control = controlRaw.baseAddress
                    msg.msg_controllen = socklen_t(controlSize)
                    let n = recvmsg(fd, &msg, 0)
                    let fds = n >= 0
                        ? ControlMessage.descriptors(in: UnsafeRawBufferPointer(controlRaw), length: Int(msg.msg_controllen))
                        : []
                    return (n, msg.msg_flags, fds)
                }
            }
            if n < 0 {
                if errno == EINTR { continue }
                throw CameraError.connectionFailed("socket read failed: \(errnoDescription())")
            }
            if flags & MSG_CTRUNC != 0 {
                received.forEach { _ = Darwin.close($0) }
                throw CameraError.protocolError("too many file descriptors in one message")
            }
            if !received.isEmpty {
                guard done == 0, acceptDescriptors else {
                    received.forEach { _ = Darwin.close($0) }
                    throw CameraError.protocolError("unexpected file descriptor")
                }
                descriptors.append(contentsOf: received)
            }
            if n == 0 {
                if done == 0 && eofOK { return false }
                throw CameraError.connectionFailed("connection closed in the middle of a message")
            }
            done += n
        }
        return true
    }

    /// Writes one message. A descriptor travels as SCM_RIGHTS with the first byte and stays owned by the caller.
    func writeMessage(type: UInt16, json: Data, payload: Data = Data(), descriptor: Int32? = nil) throws {
        guard json.count <= HelperWire.maxJSONBytes, payload.count <= HelperWire.maxPayloadBytes else {
            throw CameraError.protocolError("message exceeds the protocol size limits")
        }
        let header = WireHeader(type: type, jsonBytes: UInt32(json.count),
                                flags: descriptor == nil ? 0 : HelperWire.flagHasDescriptor,
                                payloadBytes: UInt64(payload.count))
        var message = Data(header.encoded())
        message.append(json)
        message.append(payload)

        writeLock.lock()
        defer { writeLock.unlock() }
        guard !isClosed else { throw CameraError.notConnected }
        try message.withUnsafeBytes { raw in
            guard let base = raw.baseAddress else { return }
            var sent = 0
            if let descriptor {
                var control = ControlMessage.rights(descriptor)
                while true {
                    var iov = iovec(iov_base: UnsafeMutableRawPointer(mutating: base), iov_len: raw.count)
                    let n: Int = control.withUnsafeMutableBytes { controlRaw in
                        withUnsafeMutablePointer(to: &iov) { iovPointer in
                            var msg = msghdr()
                            msg.msg_iov = iovPointer
                            msg.msg_iovlen = 1
                            msg.msg_control = controlRaw.baseAddress
                            msg.msg_controllen = socklen_t(controlRaw.count)
                            return sendmsg(fd, &msg, 0)
                        }
                    }
                    if n < 0 && errno == EINTR { continue }
                    guard n > 0 else {
                        throw CameraError.connectionFailed("socket write failed: \(errnoDescription())")
                    }
                    sent = n
                    break
                }
            }
            while sent < raw.count {
                let n = send(fd, base + sent, raw.count - sent, 0)
                if n < 0 {
                    if errno == EINTR { continue }
                    throw CameraError.connectionFailed("socket write failed: \(errnoDescription())")
                }
                sent += n
            }
        }
    }

    private var isClosed: Bool {
        stateLock.lock()
        defer { stateLock.unlock() }
        return closed
    }

    /// Wakes a reader blocked in recvmsg() (it sees EOF) without releasing the descriptor it uses.
    func shutdown() {
        stateLock.lock()
        defer { stateLock.unlock() }
        if !closed { _ = Darwin.shutdown(fd, SHUT_RDWR) }
    }

    /// Closes the descriptor. Only call once no thread reads from or writes to it any more.
    func close() {
        stateLock.lock()
        defer { stateLock.unlock() }
        if !closed {
            closed = true
            _ = Darwin.close(fd)
        }
    }
}
