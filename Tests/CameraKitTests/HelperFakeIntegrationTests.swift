// License: Apache 2.0. See LICENSE file in root directory.
//
// The real helper binary in --fake mode: full socket round trips through FakeCameraSource (the same code as the live
// camera), and the helper's own checks driven message by message.

import Darwin
import Foundation
import Testing
@testable import CameraKit

@Suite("Helper (fake camera)", .serialized, .enabled(if: TestHelper.available, TestHelper.skipReason),
       .timeLimit(.minutes(2)))
struct HelperFakeIntegrationTests {
    private var helper: URL { TestHelper.url ?? URL(fileURLWithPath: "/nonexistent") }

    @Test func fullSession() async throws {
        let source = FakeCameraSource(helperURL: helper)
        let events = EventRecorder(source.events())
        let info = try await source.connect()
        #expect(info.source == "fake")
        #expect(info.name == "IntelFaceScan Fake Camera")
        #expect(info.sensors.map(\.kind) == [.depth, .color, .motion])
        #expect(info.presets.contains("High Accuracy"))
        #expect(source.isConnected)
        let hello = try #require(source.helperHello)
        #expect(hello.mode == "fake" && hello.protocol == 1 && hello.euid == getuid())
        #expect(await events.wait(timeout: 2) { if case .connected = $0 { return true }; return false } != nil)

        // laser_power null on the wire means "maximum": visible in the frame metadata.
        try await source.configure(.faceScanD455)
        let first = source.frames(bufferingNewest: 4)
        let second = source.frames(bufferingNewest: 4)
        try await source.start()
        #expect(await events.wait(timeout: 2) { if case .streamingStarted = $0 { return true }; return false } != nil)

        async let a = collectFrames(first, count: 8)
        async let b = collectFrames(second, count: 8)
        let (framesA, framesB) = try await (a, b)
        #expect(framesA.count == 8 && framesB.count == 8)
        let sequences = framesA.map(\.sequence)
        #expect(sequences == sequences.sorted())

        let frame = try #require(framesA.last)
        #expect(frame.color.width == 1280 && frame.color.height == 800 && frame.color.bytesPerRow == 3840)
        #expect(frame.color.pixels.count == 1280 * 800 * 3)
        #expect(frame.alignedDepth.width == 1280 && frame.alignedDepth.height == 800)
        #expect(frame.alignedDepth.pixels.count == 1280 * 800 * 2)
        #expect(frame.alignedDepth.depthScale == 0.0001)
        #expect(abs(frame.color.intrinsics.fx - 640) < 0.01)
        #expect(frame.alignedDepth.intrinsics == frame.color.intrinsics)
        let center = frame.alignedDepth.meters(x: 640, y: 400)
        #expect(center > 0.30 && center < 0.40, "sphere depth at the image center: \(center)")
        #expect(frame.alignedDepth.meters(x: 0, y: 0) == 0)
        #expect(frame.metadata["laser_power"] == 360)
        #expect(frame.metadata["color_exposure_us"] == 8000)
        #expect(abs((frame.imu?.accel?[1] ?? 0) + 9.81) < 0.01)
        #expect(abs(frame.depthToColor.translation[0] + 0.059) < 1e-6)
        expectPattern(frame)

        // Errors come back as the helper's messages.
        await #expect(throws: CameraError.commandFailed("Laser Power: value out of range [0, 360]")) {
            try await source.setOption(sensor: .depth, name: "Laser Power", value: 999)
        }
        await #expect(throws: CameraError.commandFailed("the color sensor has no option 'Warp Drive'")) {
            try await source.setOption(sensor: .color, name: "Warp Drive", value: 1)
        }
        try await source.setOption(sensor: .depth, name: "Laser Power", value: 240)
        try await source.setAutoExposureROI(CGRect(x: 500, y: 220, width: 300, height: 380), distanceMeters: 0.6)
        try await source.setAutoExposureROI(CGRect(x: 0, y: 0, width: 10, height: 10), distanceMeters: nil)
        try await source.setFilters(.faceScan)
        let later = try await collectFrames(source.frames(bufferingNewest: 1), count: 3)
        #expect(later.last?.metadata["laser_power"] == 240)

        try await source.stop()
        #expect(await events.wait(timeout: 2) { if case .streamingStopped = $0 { return true }; return false } != nil)

        await source.shutdown()
        #expect(!source.isConnected)
        let exit = await events.wait(timeout: 5) { $0.helperExitStatus != nil }
        #expect(exit?.helperExitStatus == 0)
        #expect(await events.waitUntilFinished(timeout: 2), "shutdown ends the event streams")
        await #expect(throws: CameraError.notConnected) { try await source.start() }
        await source.shutdown()  // idempotent
    }

    /// red = x + 3n, green = y + 2n, blue = (x ^ y) + n (mod 256) wherever the sphere is not.
    private func expectPattern(_ frame: CameraFrame, sourceLine: SourceLocation = #_sourceLocation) {
        let n = Int(frame.color.frameNumber)
        frame.color.pixels.withUnsafeBytes { p in
            for (x, y) in [(0, 0), (10, 0), (1279, 3), (200, 799)] {
                let i = y * frame.color.bytesPerRow + x * 3
                #expect(Int(p[i]) == (x + 3 * n) & 255, sourceLocation: sourceLine)
                #expect(Int(p[i + 1]) == (y + 2 * n) & 255, sourceLocation: sourceLine)
                #expect(Int(p[i + 2]) == ((x ^ y) + n) & 255, sourceLocation: sourceLine)
            }
        }
    }

    @Test func paddedColorRowsAreRepacked() async throws {
        let source = FakeCameraSource(helperURL: helper, colorRowPadding: 13)
        _ = try await source.connect()
        let frames = source.frames(bufferingNewest: 2)
        try await source.start()
        let frame = try #require(try await collectFrames(frames, count: 2).last)
        #expect(frame.color.bytesPerRow == 1280 * 3)
        #expect(frame.color.pixels.count == 1280 * 800 * 3)
        expectPattern(frame)
        await source.shutdown()
    }

    @Test func recordingIsCopiedIntoTheAppsFile() async throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("session.db3")
        let source = FakeCameraSource(helperURL: helper)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        try await source.start()
        try await source.startRecording(to: file)
        #expect(await events.wait(timeout: 2) { if case .recordingStarted = $0 { return true }; return false } != nil)
        await #expect(throws: CameraError.commandFailed("a recording is already running")) {
            try await source.startRecording(to: dir.appendingPathComponent("second.db3"))
        }
        #expect(!FileManager.default.fileExists(atPath: dir.appendingPathComponent("second.db3").path))
        try await Task.sleep(for: .milliseconds(400))
        let bytes = try await source.stopRecording()
        let size = try #require(try FileManager.default.attributesOfItem(atPath: file.path)[.size] as? NSNumber)
        #expect(bytes > 64 * 1024)
        #expect(size.int64Value == bytes)
        let head = try #require(try FileHandle(forReadingFrom: file).read(upToCount: 19))
        #expect(String(decoding: head, as: UTF8.self) == "IFSFAKE1 format=db3")
        let finished = await events.wait(timeout: 2) { if case .recordingFinished = $0 { return true }; return false }
        if case .recordingFinished(let reported)? = finished { #expect(reported == bytes) } else { Issue.record("no recordingFinished") }
        await #expect(throws: CameraError.commandFailed("no recording is running")) { _ = try await source.stopRecording() }
        await source.shutdown()
    }

    @Test func lostConnectionEndsTheHelper() async throws {
        let source = FakeCameraSource(helperURL: helper)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        try await source.start()
        source.helperSession.dropConnectionForTesting()
        // Socket EOF: the helper stops and exits by itself.
        let exit = await events.wait(timeout: 5) { $0.helperExitStatus != nil }
        #expect(exit?.helperExitStatus == 0)
        #expect(await events.wait(timeout: 1) { if case .error = $0 { return true }; return false } != nil)
        #expect(!source.isConnected)
        // A new helper can be started afterwards.
        let info = try await source.connect()
        #expect(info.source == "fake")
        await source.shutdown()
    }

    @Test func hardwareResetReconnects() async throws {
        let source = FakeCameraSource(helperURL: helper)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        try await source.hardwareReset()
        #expect(await events.wait(timeout: 2) { if case .disconnected = $0 { return true }; return false } != nil)
        // The device comes back after the disconnect (the first .connected is from connect()).
        let deadline = Date(timeIntervalSinceNow: 3)
        var reconnected = false
        while !reconnected && Date() < deadline {
            let all = events.events
            if let lost = all.firstIndex(where: { if case .disconnected = $0 { return true }; return false }) {
                reconnected = all[lost...].contains { if case .connected = $0 { return true }; return false }
            }
            try await Task.sleep(for: .milliseconds(20))
        }
        #expect(reconnected)
        await source.shutdown()
    }

    @Test func playbackErrorsReachTheApp() async throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let file = dir.appendingPathComponent("empty.db3")
        #expect(FileManager.default.createFile(atPath: file.path, contents: Data()))
        let source = PlaybackCameraSource(file: file, helperURL: helper, realTime: false)
        // An empty file is no recording: whatever the engine says, connect() reports it and the helper stays clean.
        do {
            _ = try await source.connect()
            Issue.record("an empty recording should not open")
        } catch let error as CameraError {
            guard case .commandFailed = error else { Issue.record("unexpected \(error)"); return }
        }
        await source.shutdown()
        #expect(!source.isConnected)
    }

    // MARK: The helper's checks

    private func runHelper(_ arguments: [String]) throws -> (Int32, String) {
        let handle = try HelperLauncher.launchAsUser(helper: helper, arguments: arguments)
        guard handle.waitForExit(timeout: 10) else {
            handle.kill()
            throw TestTimeout(what: "helper exit")
        }
        return (handle.exitStatus ?? -1, handle.diagnostics)
    }

    @Test func helperRejectsUnsafeSocketPaths() throws {
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let real = try #require(realpath(dir.path, nil).map { p in defer { free(p) }; return String(cString: p) })
        let common = ["--uid", String(getuid()), "--parent-pid", String(getpid()), "--fake"]

        var (status, message) = try runHelper(["--socket", "relative/camera.sock"] + common)
        #expect(status == 69 && message.contains("absolute"), "\(message)")

        // a directory others can enter
        let open = real + "/open"
        #expect(mkdir(open, 0o755) == 0 && chmod(open, 0o755) == 0)
        (status, message) = try runHelper(["--socket", open + "/camera.sock"] + common)
        #expect(status == 69 && message.contains("0700"), "\(message)")

        // a link to a private directory
        let target = real + "/private"
        #expect(mkdir(target, 0o700) == 0)
        #expect(symlink(target, real + "/link") == 0)
        (status, message) = try runHelper(["--socket", real + "/link/camera.sock"] + common)
        #expect(status == 69 && message.contains("symbolic"), "\(message)")

        // not canonical
        (status, message) = try runHelper(["--socket", target + "/../private/camera.sock"] + common)
        #expect(status == 69 && message.contains("canonical"), "\(message)")

        // no socket there
        (status, message) = try runHelper(["--socket", target + "/camera.sock"] + common)
        #expect(status == 69, "\(message)")

        // the /var symlink of the temporary directory itself is refused too
        if NSTemporaryDirectory().hasPrefix("/var/") {
            (status, message) = try runHelper(["--socket", NSTemporaryDirectory() + "x/camera.sock"] + common)
            #expect(status == 69 && message.contains("symbolic"), "\(message)")
        }
    }

    @Test func helperChecksPrivilegesAndArguments() throws {
        var (status, message) = try runHelper(["--socket", "/x/y", "--uid", "0", "--parent-pid", String(getpid()), "--fake"])
        #expect(status == 77 && message.contains("--uid"), "\(message)")
        (status, message) = try runHelper(["--socket", "/x/y", "--uid", String(getuid()), "--parent-pid", String(getpid())])
        #expect(status == 77 && message.contains("root"), "\(message)")
        (status, message) = try runHelper(["--fake", "--playback", "/x"])
        #expect(status == 64, "\(message)")
        (status, message) = try runHelper(["--socket", "/x/y", "--uid", "abc", "--parent-pid", "5", "--fake"])
        #expect(status == 64, "\(message)")
        (status, message) = try runHelper(["--bogus"])
        #expect(status == 64, "\(message)")
        (status, _) = try runHelper(["--help"])
        #expect(status == 0)
    }

    @Test func helperChecksThePeerProcess() throws {
        // The listening process is this test, but the helper is told the app is our parent: it must hang up.
        let server = try HelperSocketServer()
        defer { server.close() }
        let handle = try HelperLauncher.launchAsUser(helper: helper, arguments: [
            "--socket", server.socketPath, "--uid", String(getuid()), "--parent-pid", String(getppid()), "--fake"])
        let fd = try server.accept(timeout: 10)
        defer { close(fd) }
        #expect(handle.waitForExit(timeout: 10))
        #expect(handle.exitStatus == 77)
        #expect(handle.diagnostics.contains("expected \(getppid())"), "\(handle.diagnostics)")
    }

    @Test func appChecksThePeerProcess() throws {
        let server = try HelperSocketServer()
        defer { server.close() }
        let client = socket(AF_UNIX, SOCK_STREAM, 0)
        defer { close(client) }
        var addr = sockaddr_un()
        addr.sun_family = sa_family_t(AF_UNIX)
        withUnsafeMutableBytes(of: &addr.sun_path) { raw in
            let bytes = Array(server.socketPath.utf8)
            raw.copyBytes(from: bytes)
            raw[bytes.count] = 0
        }
        let connected = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                Darwin.connect(client, $0, socklen_t(MemoryLayout<sockaddr_un>.size))
            }
        }
        #expect(connected == 0)
        let fd = try server.accept(timeout: 5)
        defer { close(fd) }
        // This process is no helper, and does not run as root.
        #expect(throws: CameraError.self) {
            _ = try HelperSocketServer.verifyPeer(fd: fd, expectedUID: getuid(), helperPath: helper.path)
        }
        #expect(throws: CameraError.self) {
            _ = try HelperSocketServer.verifyPeer(fd: fd, expectedUID: 0, helperPath: CommandLine.arguments[0])
        }
        let me = try #require(Bundle.main.executableURL?.path)
        #expect(try HelperSocketServer.verifyPeer(fd: fd, expectedUID: getuid(), helperPath: me) == getpid())
    }

    @Test func everyCommandIsAnsweredOnce() throws {
        let client = try RawHelperClient(helper: helper)
        #expect(client.hello.mode == "fake")
        var ids: [UInt64] = []
        for _ in 0..<20 { ids.append(try client.send(HelperMessageType.queryDevice.rawValue)) }
        ids.append(try client.send(HelperMessageType.playback.rawValue, ["action": "pause"]))
        ids.append(try client.send(150))  // unknown command
        ids.append(try client.send(HelperMessageType.setOption.rawValue, ["sensor": "depth", "option": "Laser Power",
                                                                         "value": 90, "extra": true]))
        ids.append(try client.send(HelperMessageType.setROI.rawValue, ["x": 1.5, "y": 0, "width": 4, "height": 4]))
        ids.append(try client.send(HelperMessageType.recordStart.rawValue, ["format": "db3"]))
        var replies: [UInt64: [String: Any]] = [:]
        while replies.count < ids.count {
            let message = try #require(try client.read(until: .reply))
            let object = try #require(try JSONSerialization.jsonObject(with: message.json) as? [String: Any])
            let id = try #require((object["request_id"] as? NSNumber)?.uint64Value)
            #expect(replies[id] == nil, "request \(id) answered twice")
            replies[id] = object
        }
        #expect(Set(replies.keys) == Set(ids))
        func error(_ id: UInt64) -> String? { replies[id]?["error"] as? String }
        #expect(replies[ids[0]]?["ok"] as? Bool == true)
        #expect(error(ids[20]) == "playback control is only available for recordings")
        #expect(error(ids[21]) == "unknown command 150")
        #expect(error(ids[22]) == "unknown field 'extra'")
        #expect(error(ids[23]) == "field 'x' must be an integer")
        #expect(error(ids[24]) == "RECORD_START needs the descriptor of the destination file")

        let quit = try client.send(HelperMessageType.quit.rawValue)
        #expect(try client.reply(to: quit)?["ok"] as? Bool == true)
        let bye = try #require(try client.read(until: .bye))
        #expect(String(decoding: bye.json, as: UTF8.self).contains("quit"))
        #expect(client.process.waitForExit(timeout: 5) && client.process.exitStatus == 0)
    }

    @Test func protocolViolationsCloseTheConnection() throws {
        // A broken header.
        var client = try RawHelperClient(helper: helper)
        var garbage = WireHeader(type: 100, jsonBytes: 2, flags: 0, payloadBytes: 0).encoded()
        garbage[0] = 0
        _ = garbage.withUnsafeBytes { send(client.io.fd, $0.baseAddress, $0.count, 0) }
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 5) && client.process.exitStatus == 0)

        // A command without request_id.
        client = try RawHelperClient(helper: helper)
        try client.send(HelperMessageType.queryDevice.rawValue, withId: false)
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 5))

        // A body that is no JSON object.
        client = try RawHelperClient(helper: helper)
        try client.io.writeMessage(type: HelperMessageType.queryDevice.rawValue, json: Data("[1]".utf8))
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 5))

        // A descriptor on a command that takes none.
        client = try RawHelperClient(helper: helper)
        var fds: [Int32] = [-1, -1]
        #expect(pipe(&fds) == 0)
        defer { close(fds[0]); close(fds[1]) }
        try client.send(HelperMessageType.queryDevice.rawValue, descriptor: fds[1])
        #expect(try client.read(until: .reply) == nil)
        #expect(client.process.waitForExit(timeout: 5))
    }

    @Test func recordingDestinationMustBeWritable() throws {
        let client = try RawHelperClient(helper: helper)
        let dir = try TestHelper.scratchDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let path = dir.appendingPathComponent("ro.db3").path
        #expect(FileManager.default.createFile(atPath: path, contents: Data()))
        let readOnly = open(path, O_RDONLY)
        defer { close(readOnly) }
        let id = try client.send(HelperMessageType.recordStart.rawValue, ["format": "db3"], descriptor: readOnly)
        #expect(try client.reply(to: id)?["error"] as? String == "the recording destination is not open for writing")
        let dirFD = open(dir.path, O_RDONLY)
        defer { close(dirFD) }
        let id2 = try client.send(HelperMessageType.recordStart.rawValue, ["format": "db3"], descriptor: dirFD)
        #expect(try client.reply(to: id2)?["error"] as? String == "the recording destination is not a regular file")
        let writable = open(path, O_WRONLY)
        defer { close(writable) }
        let id3 = try client.send(HelperMessageType.recordStart.rawValue, ["format": "mp4"], descriptor: writable)
        #expect((try client.reply(to: id3)?["error"] as? String)?.contains("unsupported recording format") == true)
    }

    @Test func slowReaderLosesOldFramesetsOnly() throws {
        let client = try RawHelperClient(helper: helper)
        let start = try client.send(HelperMessageType.start.rawValue)
        #expect(try client.reply(to: start)?["ok"] as? Bool == true)
        _ = try client.read(until: .frameset)
        // Stop reading while ~30 framesets are produced.
        Thread.sleep(forTimeInterval: 1.0)
        let query = try client.send(HelperMessageType.queryDevice.rawValue)
        var sequences: [UInt64] = []
        var answered = false
        while sequences.count < 10 || !answered {
            let message = try #require(try client.io.readMessage())
            if message.type == HelperMessageType.frameset.rawValue {
                let frame = try HelperFrameDecoder.decode(json: message.json, payload: message.payload)
                sequences.append(frame.sequence)
            } else if message.type == HelperMessageType.reply.rawValue {
                answered = true  // replies are never dropped
            }
        }
        let gaps = zip(sequences, sequences.dropFirst()).filter { $1 != $0 + 1 }.count
        #expect(gaps >= 1, "framesets queued during the stall should have been dropped: \(sequences)")
        // after the backlog, the stream is continuous again
        let tail = Array(sequences.suffix(5))
        #expect(zip(tail, tail.dropFirst()).allSatisfy { $1 == $0 + 1 }, "\(sequences)")
        _ = query
    }

    @Test func throughput() async throws {
        // Not a benchmark, a floor: the socket path must keep up with 30 fps of 1280x800 RGB + depth.
        let source = FakeCameraSource(helperURL: helper)
        _ = try await source.connect()
        let frames = source.frames(bufferingNewest: 64)
        try await source.start()
        _ = try await collectFrames(frames, count: 1)
        let t0 = Date()
        let received = try await collectFrames(frames, count: 45, timeout: 10)
        let fps = Double(received.count) / Date().timeIntervalSince(t0)
        #expect(fps > 24, "received \(fps) fps")
        await source.shutdown()
    }
}
