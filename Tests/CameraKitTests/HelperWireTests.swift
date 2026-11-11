// License: Apache 2.0. See LICENSE file in root directory.

import Darwin
import Foundation
import IFSProtocol
import Testing
@testable import CameraKit

@Suite("Helper wire codec")
struct HelperWireTests {
    @Test func messageTypesMatchTheCHeader() {
        let c: [(HelperMessageType, ifs_msg_type)] = [
            (.hello, IFS_MSG_HELLO), (.reply, IFS_MSG_REPLY), (.frameset, IFS_MSG_FRAMESET), (.event, IFS_MSG_EVENT),
            (.log, IFS_MSG_LOG), (.bye, IFS_MSG_BYE), (.queryDevice, IFS_CMD_QUERY_DEVICE),
            (.configure, IFS_CMD_CONFIGURE), (.start, IFS_CMD_START), (.stop, IFS_CMD_STOP),
            (.setOption, IFS_CMD_SET_OPTION), (.setROI, IFS_CMD_SET_ROI), (.setFilters, IFS_CMD_SET_FILTERS),
            (.recordStart, IFS_CMD_RECORD_START), (.recordStop, IFS_CMD_RECORD_STOP), (.playback, IFS_CMD_PLAYBACK),
            (.hardwareReset, IFS_CMD_HARDWARE_RESET), (.quit, IFS_CMD_QUIT),
        ]
        #expect(c.count == HelperMessageType.allCases.count)
        for (swift, header) in c {
            #expect(UInt32(swift.rawValue) == header.rawValue, "\(swift)")
        }
        #expect(HelperWire.flagHasDescriptor == IFS_FLAG_HAS_FD)
        #expect(MemoryLayout<ifs_wire_header>.size == HelperWire.headerSize)
    }

    @Test func limitsMatchTheCHeader() throws {
        func valid(json: UInt32, payload: UInt64) -> Bool {
            var h = ifs_wire_header()
            ifs_wire_header_init(&h, HelperMessageType.frameset.rawValue, json, payload, 0)
            return ifs_wire_header_valid(&h) == 0
        }
        #expect(valid(json: UInt32(HelperWire.maxJSONBytes), payload: UInt64(HelperWire.maxPayloadBytes)))
        #expect(!valid(json: UInt32(HelperWire.maxJSONBytes) + 1, payload: 0))
        #expect(!valid(json: 10, payload: UInt64(HelperWire.maxPayloadBytes) + 1))
    }

    @Test func headerRoundTrip() throws {
        let header = WireHeader(type: 3, jsonBytes: 1234, flags: 0, payloadBytes: 5_120_000)
        let bytes = header.encoded()
        #expect(bytes.count == 24)
        #expect(Array(bytes[0..<4]) == Array("IFS1".utf8))
        let decoded = try bytes.withUnsafeBytes { try WireHeader.decode($0) }
        #expect(decoded == header)
    }

    @Test func invalidHeadersAreRejected() {
        let good = WireHeader(type: 100, jsonBytes: 10, flags: 0, payloadBytes: 0).encoded()
        var badMagic = good
        badMagic[0] = 0x58
        var badVersion = good
        badVersion[4] = 2
        var badFlags = good
        badFlags[12] = 0x2
        var tooLarge = good
        withUnsafeBytes(of: UInt32(HelperWire.maxJSONBytes + 1).littleEndian) { tooLarge.replaceSubrange(8..<12, with: $0) }
        for bytes in [badMagic, badVersion, badFlags, tooLarge, Array(good.prefix(20))] {
            #expect(throws: CameraError.self) { try bytes.withUnsafeBytes { try WireHeader.decode($0) } }
        }
    }

    @Test func controlMessageLayout() {
        let control = ControlMessage.rights(42)
        #expect(control.count == 16)  // CMSG_SPACE(sizeof(int)) on Darwin
        control.withUnsafeBytes { raw in
            let header = raw.loadUnaligned(as: cmsghdr.self)
            #expect(header.cmsg_len == 16)
            #expect(header.cmsg_level == SOL_SOCKET)
            #expect(header.cmsg_type == SCM_RIGHTS)
            #expect(ControlMessage.descriptors(in: raw, length: raw.count) == [42])
        }
    }

    private func socketPair() throws -> (HelperSocketIO, HelperSocketIO) {
        var fds: [Int32] = [-1, -1]
        guard socketpair(AF_UNIX, SOCK_STREAM, 0, &fds) == 0 else { throw CameraError.connectionFailed("socketpair") }
        return (HelperSocketIO(fd: fds[0]), HelperSocketIO(fd: fds[1]))
    }

    @Test func messageRoundTripOverASocket() throws {
        let (a, b) = try socketPair()
        let json = Data(#"{"request_id": 1}"#.utf8)
        let payload = Data((0..<300_000).map { UInt8(truncatingIfNeeded: $0 * 7) })
        // Large enough that the write blocks until the reader drains it: write from another thread. A thread, not a
        // block on a global dispatch queue: such a block does not start while the Swift cooperative pool is full,
        // and this test then held one of the 3-core runner's three pool threads in the read below (18 to 400 s).
        let done = DispatchSemaphore(value: 0)
        Thread {
            try? a.writeMessage(type: HelperMessageType.frameset.rawValue, json: json, payload: payload)
            try? a.writeMessage(type: HelperMessageType.queryDevice.rawValue, json: json)
            done.signal()
        }.start()
        let first = try #require(try b.readMessage())
        #expect(first.type == HelperMessageType.frameset.rawValue)
        #expect(first.json == json)
        #expect(first.payload == payload)
        let second = try #require(try b.readMessage())
        #expect(second.type == HelperMessageType.queryDevice.rawValue)
        #expect(second.payload.isEmpty)
        done.wait()
        a.close()
        #expect(try b.readMessage() == nil)  // clean EOF between messages
    }

    @Test func eofInsideAMessageIsAnError() throws {
        let (a, b) = try socketPair()
        let header = WireHeader(type: 2, jsonBytes: 100, flags: 0, payloadBytes: 0).encoded()
        _ = header.withUnsafeBytes { send(a.fd, $0.baseAddress, $0.count, 0) }
        a.close()
        #expect(throws: CameraError.self) { _ = try b.readMessage() }
    }

    @Test func descriptorIsPassedWithTheFirstByte() throws {
        let (a, b) = try socketPair()
        var pipeFDs: [Int32] = [-1, -1]
        #expect(pipe(&pipeFDs) == 0)
        try a.writeMessage(type: HelperMessageType.recordStart.rawValue, json: Data(#"{"request_id":1}"#.utf8),
                           descriptor: pipeFDs[1])
        close(pipeFDs[1])
        let message = try #require(try b.readMessage(acceptDescriptor: true))
        let received = try #require(message.descriptor)
        // The received descriptor is the pipe's write end: what goes in comes out of the read end.
        var byte: UInt8 = 0x5A
        #expect(write(received, &byte, 1) == 1)
        close(received)
        var out: UInt8 = 0
        #expect(read(pipeFDs[0], &out, 1) == 1)
        #expect(out == 0x5A)
        close(pipeFDs[0])
    }

    @Test func unexpectedDescriptorIsRejected() throws {
        let (a, b) = try socketPair()
        var pipeFDs: [Int32] = [-1, -1]
        #expect(pipe(&pipeFDs) == 0)
        defer { close(pipeFDs[0]); close(pipeFDs[1]) }
        try a.writeMessage(type: HelperMessageType.event.rawValue, json: Data(#"{"event":"x"}"#.utf8),
                           descriptor: pipeFDs[1])
        #expect(throws: CameraError.self) { _ = try b.readMessage() }
    }

    @Test func oversizedMessagesAreNotSent() throws {
        let (a, _) = try socketPair()
        let json = Data(repeating: 0x20, count: HelperWire.maxJSONBytes + 1)
        #expect(throws: CameraError.self) { try a.writeMessage(type: 100, json: json) }
    }
}
