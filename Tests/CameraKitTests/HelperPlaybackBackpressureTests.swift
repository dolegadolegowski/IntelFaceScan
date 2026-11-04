// License: Apache 2.0. See LICENSE file in root directory.
//
// Lossless non-real-time playback (docs/PROTOCOL.md, backpressure): the real helper plays a synthetic recording made
// with make-test-recording. A slow reader or consumer gets every frameset in order, and stop, QUIT, shutdown, a lost
// connection or a signal while the helper waits for room in its queue still end promptly.

import Darwin
import Foundation
import Testing
@testable import CameraKit

/// What a raw protocol client saw.
private struct RawPlaybackLog {
    var sequences: [UInt64] = []
    var colorFrames: [UInt64] = []
    var events: [String] = []
    var replies: [UInt64: [String: Any]] = [:]
    var bye = false

    /// Reads messages until `done` is true; false when the helper closed the connection first.
    @discardableResult
    mutating func read(_ client: RawHelperClient, until done: (RawPlaybackLog) -> Bool) throws -> Bool {
        while !done(self) {
            guard let message = try client.io.readMessage() else { return false }
            switch HelperMessageType(rawValue: message.type) {
            case .frameset:
                let frame = try HelperFrameDecoder.decode(json: message.json, payload: message.payload)
                sequences.append(frame.sequence)
                colorFrames.append(frame.color.frameNumber)
            case .event:
                events.append(try HelperJSON.decoder.decode(WireEvent.self, from: message.json).event)
            case .reply:
                let object = try JSONSerialization.jsonObject(with: message.json) as? [String: Any] ?? [:]
                if let id = (object["request_id"] as? NSNumber)?.uint64Value { replies[id] = object }
            case .bye:
                bye = true
            default:
                break
            }
        }
        return true
    }
}

/// Consumes a frame stream on its own task, optionally slowly, and can be held (stops taking frames while the
/// stream stays alive, like a consumer busy with something else).
private final class FrameConsumer: @unchecked Sendable {
    private let lock = NSLock()
    private var sequences: [UInt64] = []
    private var colorFrames: [UInt64] = []
    private var holding = false
    private var ended = false
    private var lastFrame = Date()
    private var task: Task<Void, Never>?

    init(_ stream: AsyncStream<CameraFrame>, delay: Duration = .zero, holdAfter: Int? = nil) {
        task = Task { [weak self] in
            for await frame in stream {
                guard let self else { return }
                let count: Int = self.lock.withLock {
                    self.sequences.append(frame.sequence)
                    self.colorFrames.append(frame.color.frameNumber)
                    self.lastFrame = Date()
                    if let holdAfter, self.sequences.count == holdAfter { self.holding = true }
                    return self.sequences.count
                }
                _ = count
                if delay > .zero { try? await Task.sleep(for: delay) }
                while self.lock.withLock({ self.holding }) {
                    try? await Task.sleep(for: .milliseconds(10))
                }
            }
            self?.lock.withLock { self?.ended = true }
        }
    }

    deinit { task?.cancel() }

    var received: [UInt64] { lock.withLock { sequences } }
    var receivedColorFrames: [UInt64] { lock.withLock { colorFrames } }
    var count: Int { lock.withLock { sequences.count } }
    var streamEnded: Bool { lock.withLock { ended } }
    var secondsSinceLastFrame: Double { lock.withLock { Date().timeIntervalSince(lastFrame) } }
    func resume() { lock.withLock { holding = false } }

    func wait(timeout: Double, _ condition: (FrameConsumer) -> Bool) async -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        while Date() < deadline {
            if condition(self) { return true }
            try? await Task.sleep(for: .milliseconds(20))
        }
        return condition(self)
    }
}

private func isConsecutive(_ values: [UInt64]) -> Bool {
    zip(values, values.dropFirst()).allSatisfy { $1 == $0 + 1 }
}

@Suite("Helper playback backpressure", .serialized,
       .enabled(if: TestHelper.playbackAvailable, TestHelper.playbackSkipReason), .timeLimit(.minutes(3)))
final class HelperPlaybackBackpressureTests {
    private let dir: URL
    private let helper: URL

    init() throws {
        dir = try TestHelper.scratchDirectory()
        helper = TestHelper.url ?? URL(fileURLWithPath: "/nonexistent")
    }

    deinit { try? FileManager.default.removeItem(at: dir) }

    private func playbackClient(_ file: URL) throws -> RawHelperClient {
        try RawHelperClient(helper: helper, extraArguments: ["--playback", file.path, "--no-real-time"])
    }

    /// Starts the playback, takes the first frameset and stops reading long enough for the helper to fill the socket
    /// and its queue: its processing thread then waits in send_frameset.
    private func startAndStall(_ client: RawHelperClient, _ log: inout RawPlaybackLog, seconds: Double = 1.5) throws {
        let start = try client.send(HelperMessageType.start.rawValue)
        try log.read(client) { $0.replies[start] != nil && !$0.sequences.isEmpty }
        #expect(log.replies[start]?["ok"] as? Bool == true, "\(log.replies[start] ?? [:])")
        Thread.sleep(forTimeInterval: seconds)
    }

    private func quit(_ client: RawHelperClient, _ log: inout RawPlaybackLog) throws {
        let quit = try client.send(HelperMessageType.quit.rawValue)
        let t0 = Date()
        try log.read(client) { $0.replies[quit] != nil && $0.bye }
        #expect(log.replies[quit]?["ok"] as? Bool == true)
        #expect(client.process.waitForExit(timeout: 5) && client.process.exitStatus == 0)
        #expect(Date().timeIntervalSince(t0) < 5, "QUIT took \(Date().timeIntervalSince(t0)) s")
    }

    // MARK: The helper

    @Test func stalledReaderGetsEveryFramesetInOrder() throws {
        let frames = 40
        let client = try playbackClient(try TestHelper.makeRecording(in: dir, frames: frames))
        #expect(client.hello.mode == "playback")
        var log = RawPlaybackLog()
        try startAndStall(client, &log)
        try log.read(client) { $0.events.contains("playback_finished") }
        #expect(log.sequences == Array(1...UInt64(frames)), "\(log.sequences)")
        #expect(log.colorFrames == Array(200..<200 + UInt64(frames)), "\(log.colorFrames)")
        try quit(client, &log)
    }

    @Test func stopWhileTheHelperWaitsForRoomIsPrompt() throws {
        let client = try playbackClient(try TestHelper.makeRecording(in: dir, frames: 60))
        var log = RawPlaybackLog()
        try startAndStall(client, &log)
        let t0 = Date()
        let stop = try client.send(HelperMessageType.stop.rawValue)
        try log.read(client) { $0.replies[stop] != nil }
        let elapsed = Date().timeIntervalSince(t0)
        #expect(log.replies[stop]?["ok"] as? Bool == true, "\(log.replies[stop] ?? [:])")
        #expect(elapsed < 3, "STOP took \(elapsed) s")
        #expect(log.sequences.count < 60, "the playback should have been stopped before its end")
        #expect(isConsecutive(log.sequences), "\(log.sequences)")
        #expect(log.events.contains("streaming_stopped"))
        // Started again, the playback keeps waiting for the reader instead of dropping.
        let restart = try client.send(HelperMessageType.start.rawValue)
        let before = log.sequences.count
        try log.read(client) { $0.replies[restart] != nil && $0.sequences.count > before + 2 }
        #expect(isConsecutive(Array(log.sequences[before...])), "\(log.sequences)")
        try quit(client, &log)
    }

    @Test func quitWhileTheHelperWaitsForRoomIsPrompt() throws {
        let client = try playbackClient(try TestHelper.makeRecording(in: dir, frames: 60))
        var log = RawPlaybackLog()
        try startAndStall(client, &log)
        try quit(client, &log)
        #expect(log.sequences.count < 60)
    }

    @Test func lostConnectionWhileTheHelperWaitsForRoomEndsIt() throws {
        let client = try playbackClient(try TestHelper.makeRecording(in: dir, frames: 60))
        var log = RawPlaybackLog()
        try startAndStall(client, &log)
        let t0 = Date()
        client.io.shutdown()
        #expect(client.process.waitForExit(timeout: 5), "the helper did not exit after EOF")
        #expect(client.process.exitStatus == 0)
        #expect(Date().timeIntervalSince(t0) < 5)
    }

    @Test func signalWhileTheHelperWaitsForRoomEndsIt() throws {
        let client = try playbackClient(try TestHelper.makeRecording(in: dir, frames: 60))
        var log = RawPlaybackLog()
        try startAndStall(client, &log)
        let t0 = Date()
        client.process.terminate()  // SIGTERM: handled like QUIT
        try log.read(client) { $0.bye }
        #expect(client.process.waitForExit(timeout: 5), "the helper did not exit after SIGTERM")
        #expect(client.process.exitStatus == 0)
        #expect(Date().timeIntervalSince(t0) < 5)
    }

    // MARK: PlaybackCameraSource

    @Test func slowConsumerReceivesEveryFrameInOrder() async throws {
        let frames = 40
        let file = try TestHelper.makeRecording(in: dir, frames: frames)
        let source = PlaybackCameraSource(file: file, helperURL: helper, realTime: false)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        // Two frames of buffer and 30 ms per frame: far slower than the helper plays the file.
        let consumer = FrameConsumer(source.frames(bufferingNewest: 2), delay: .milliseconds(30))
        try await source.start()
        let complete = await consumer.wait(timeout: 60) { c in
            c.count >= frames
                || (events.events.contains { if case .playbackFinished = $0 { return true }; return false }
                    && c.secondsSinceLastFrame > 3)
        }
        #expect(complete)
        #expect(consumer.received == Array(1...UInt64(frames)), "\(consumer.received)")
        #expect(consumer.receivedColorFrames == Array(200..<200 + UInt64(frames)))
        #expect(await events.wait(timeout: 5) { if case .playbackFinished = $0 { return true }; return false } != nil)
        await source.shutdown()
        #expect(await consumer.wait(timeout: 2) { $0.streamEnded }, "shutdown ends the frame stream")
        #expect(await events.waitUntilFinished(timeout: 2), "shutdown ends the event stream")
    }

    @Test func stopAndShutdownWhileTheConsumerHoldsArePrompt() async throws {
        let file = try TestHelper.makeRecording(in: dir, frames: 60)
        let source = PlaybackCameraSource(file: file, helperURL: helper, realTime: false)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        let consumer = FrameConsumer(source.frames(bufferingNewest: 2), holdAfter: 1)
        try await source.start()
        #expect(await consumer.wait(timeout: 10) { $0.count == 1 })
        try await Task.sleep(for: .milliseconds(1500))  // the app's reader and the helper now wait for the consumer

        let t0 = Date()
        try await withTimeout(5, "stop() while the consumer holds") { try await source.stop() }
        #expect(Date().timeIntervalSince(t0) < 5)
        #expect(await events.wait(timeout: 5) { if case .streamingStopped = $0 { return true }; return false } != nil)

        // Whatever was played before the stop is still there, in order.
        consumer.resume()
        #expect(await consumer.wait(timeout: 10) { $0.count > 1 && $0.secondsSinceLastFrame > 1 })
        #expect(consumer.received.first == 1 && isConsecutive(consumer.received), "\(consumer.received)")
        #expect(consumer.count < 60)

        let t1 = Date()
        try await withTimeout(8, "shutdown()") { await source.shutdown() }
        #expect(Date().timeIntervalSince(t1) < 8)
        let exit = await events.wait(timeout: 5) { $0.helperExitStatus != nil }
        #expect(exit?.helperExitStatus == 0, "the helper must exit by itself, not by a signal")
        #expect(await consumer.wait(timeout: 2) { $0.streamEnded })
    }

    @Test func shutdownWhileTheConsumerHoldsIsPrompt() async throws {
        let file = try TestHelper.makeRecording(in: dir, frames: 60)
        let source = PlaybackCameraSource(file: file, helperURL: helper, realTime: false)
        let events = EventRecorder(source.events())
        _ = try await source.connect()
        let consumer = FrameConsumer(source.frames(bufferingNewest: 2), holdAfter: 1)
        try await source.start()
        #expect(await consumer.wait(timeout: 10) { $0.count == 1 })
        try await Task.sleep(for: .milliseconds(1500))

        let t0 = Date()
        try await withTimeout(8, "shutdown() while the consumer holds") { await source.shutdown() }
        #expect(Date().timeIntervalSince(t0) < 8, "shutdown took \(Date().timeIntervalSince(t0)) s")
        let exit = await events.wait(timeout: 5) { $0.helperExitStatus != nil }
        #expect(exit?.helperExitStatus == 0, "the helper must exit by itself, not by a signal")
        consumer.resume()
        #expect(await consumer.wait(timeout: 5) { $0.streamEnded }, "shutdown ends the frame stream")
        #expect(isConsecutive(consumer.received), "\(consumer.received)")
    }
}
