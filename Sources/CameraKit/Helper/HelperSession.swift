// License: Apache 2.0. See LICENSE file in root directory.
//
// Everything the helper-backed camera sources share: launching the helper in its mode, the connection, commands
// with their timeouts, frame/event fan-out to any number of consumers, and a shutdown that always ends with the
// helper gone (QUIT -> BYE -> exit, bounded by a timeout). Nothing here needs the main thread, so shutdown can run
// from applicationShouldTerminate.

import CoreGraphics
import Darwin
import Foundation
import os

/// Fan-out of values to any number of AsyncStream consumers.
///
/// Lossy subscribers (`subscribe(bufferingNewest:)`) keep only their newest undelivered values and never slow the
/// producer down. Lossless subscribers (`subscribeLossless(capacity:)`, non-real-time playback) get every value:
/// `yield` waits until each of them has room, so the helper's reader stops reading and the helper waits in turn
/// (docs/PROTOCOL.md, backpressure). The wait is skipped - the value is queued beyond the capacity, still not lost -
/// while a bypass is held (a command waits for its reply, which arrives behind the frames the reader has not read
/// yet) and after `release()` (shutdown, the helper exited). `finishAll()` ends every stream after its queued values.
final class Broadcaster<Element: Sendable>: @unchecked Sendable {
    private let lock = NSLock()
    private var continuations: [UUID: AsyncStream<Element>.Continuation] = [:]

    // Lossless subscriptions and the producer's wait, all guarded by `condition`.
    private let condition = NSCondition()
    private var queues: [UUID: LosslessQueue] = [:]
    private var waits = true
    private var bypasses = 0
    private var released = false

    private final class LosslessQueue: @unchecked Sendable {
        let capacity: Int
        var buffer: [Element] = []
        var waiter: CheckedContinuation<Element?, Never>?
        var ended = false

        init(capacity: Int) { self.capacity = capacity }
    }

    /// The consumer's end of a lossless subscription. Dropping the stream (or cancelling the task that iterates it)
    /// unsubscribes, so a consumer that went away never keeps the producer waiting.
    private final class Subscription: @unchecked Sendable {
        let broadcaster: Broadcaster
        let id: UUID
        let queue: LosslessQueue

        init(broadcaster: Broadcaster, id: UUID, queue: LosslessQueue) {
            self.broadcaster = broadcaster
            self.id = id
            self.queue = queue
        }

        deinit { broadcaster.unsubscribe(id, queue) }

        func next() async -> Element? {
            await withCheckedContinuation { continuation in broadcaster.take(from: queue, continuation) }
        }

        func cancel() { broadcaster.unsubscribe(id, queue) }
    }

    func subscribe(bufferingNewest count: Int) -> AsyncStream<Element> {
        let (stream, continuation) = AsyncStream.makeStream(of: Element.self,
                                                            bufferingPolicy: .bufferingNewest(max(1, count)))
        let id = UUID()
        continuation.onTermination = { [weak self] _ in
            self?.lock.withLock { _ = self?.continuations.removeValue(forKey: id) }
        }
        lock.withLock { continuations[id] = continuation }
        return stream
    }

    /// A stream that receives every value in order: at most `capacity` wait for the consumer, then `yield` waits.
    func subscribeLossless(capacity: Int) -> AsyncStream<Element> {
        let id = UUID()
        let queue = LosslessQueue(capacity: max(1, capacity))
        condition.withLock { queues[id] = queue }
        let subscription = Subscription(broadcaster: self, id: id, queue: queue)
        return AsyncStream(unfolding: { await subscription.next() }, onCancel: { subscription.cancel() })
    }

    /// Delivers `value` to every subscriber; may wait for lossless ones (see the type).
    func yield(_ value: Element) {
        let targets = lock.withLock { Array(continuations.values) }
        for continuation in targets { continuation.yield(value) }

        condition.lock()
        defer { condition.unlock() }
        for id in Array(queues.keys) {
            while let queue = queues[id], !queue.ended, queue.waiter == nil, queue.buffer.count >= queue.capacity,
                  waits, bypasses == 0, !released {
                condition.wait()
            }
            guard let queue = queues[id], !queue.ended else { continue }
            if let waiter = queue.waiter {
                queue.waiter = nil
                waiter.resume(returning: value)
            } else {
                // Lossy again (real-time playback): the oldest value makes room, as with .bufferingNewest.
                if !waits && queue.buffer.count >= queue.capacity { queue.buffer.removeFirst() }
                queue.buffer.append(value)
            }
        }
    }

    private func take(from queue: LosslessQueue, _ continuation: CheckedContinuation<Element?, Never>) {
        condition.lock()
        if !queue.buffer.isEmpty {
            let value = queue.buffer.removeFirst()
            condition.broadcast()
            condition.unlock()
            continuation.resume(returning: value)
        } else if queue.ended || queue.waiter != nil {
            // Ended, or a second concurrent next() (not allowed on one stream): no value.
            condition.unlock()
            continuation.resume(returning: nil)
        } else {
            queue.waiter = continuation
            condition.unlock()
        }
    }

    private func unsubscribe(_ id: UUID, _ queue: LosslessQueue) {
        condition.lock()
        queues[id] = nil
        queue.ended = true
        queue.buffer.removeAll()
        let waiter = queue.waiter
        queue.waiter = nil
        condition.broadcast()
        condition.unlock()
        waiter?.resume(returning: nil)
    }

    /// Lossless subscribers make the producer wait (true, the default) or lose their oldest values like lossy ones.
    func setLossless(_ on: Bool) {
        condition.withLock {
            waits = on
            condition.broadcast()
        }
    }

    /// While held, `yield` never waits: values are queued beyond the capacity instead. Balance with `endBypass()`.
    func beginBypass() {
        condition.withLock {
            bypasses += 1
            condition.broadcast()
        }
    }

    func endBypass() {
        condition.withLock { bypasses = max(0, bypasses - 1) }
    }

    /// `yield` stops waiting until `resetRelease()`, so no consumer can hold up a shutdown.
    func release() {
        condition.withLock {
            released = true
            condition.broadcast()
        }
    }

    func resetRelease() {
        condition.withLock { released = false }
    }

    /// Ends the current streams (lossless ones after the values already queued); later subscriptions get new ones.
    func finishAll() {
        let targets: [AsyncStream<Element>.Continuation] = lock.withLock {
            defer { continuations.removeAll() }
            return Array(continuations.values)
        }
        for continuation in targets { continuation.finish() }

        let waiters: [CheckedContinuation<Element?, Never>] = condition.withLock {
            let all = Array(queues.values)
            queues.removeAll()
            condition.broadcast()
            return all.compactMap { queue in
                queue.ended = true
                defer { queue.waiter = nil }
                return queue.waiter
            }
        }
        waiters.forEach { $0.resume(returning: nil) }
    }

    var subscriberCount: Int { lock.withLock { continuations.count } + condition.withLock { queues.count } }
}

/// Per-command reply timeouts. Some commands legitimately take long: enabling advanced mode re-enumerates the
/// camera, and RECORD_STOP copies a whole recording.
enum HelperTimeouts {
    static let hello: TimeInterval = 15
    static let query: TimeInterval = 20
    static let configure: TimeInterval = 60
    static let streaming: TimeInterval = 30
    static let option: TimeInterval = 15
    static let recordStart: TimeInterval = 30
    static let recordStop: TimeInterval = 600
    static let hardwareReset: TimeInterval = 60
    static let quit: TimeInterval = 10
}

final class HelperCameraSession: @unchecked Sendable {
    enum Mode: Sendable, Equatable {
        case live
        case playback(file: URL, realTime: Bool)
        case fake(colorRowPadding: Int)

        var name: String {
            switch self {
            case .live: return "live"
            case .playback: return "playback"
            case .fake: return "fake"
            }
        }
    }

    /// One running helper.
    private final class Active: @unchecked Sendable {
        let server: HelperSocketServer
        let process: HelperProcessHandle
        var connection: HelperConnection?
        var hello: HelperHello?
        var byeReceived = false
        var connectionClosed = false

        init(server: HelperSocketServer, process: HelperProcessHandle) {
            self.server = server
            self.process = process
        }
    }

    let mode: Mode
    let helperURL: URL
    let frames = Broadcaster<CameraFrame>()
    let events = Broadcaster<CameraEvent>()
    /// How long to wait for the helper to connect: the live camera waits for the user to type the password.
    var connectTimeout: TimeInterval

    private let logger = Logger(subsystem: "io.github.dolegadolegowski.IntelFaceScan", category: "camera-helper")
    private let lock = NSLock()
    private var active: Active?
    private var starting: Active?
    private var connectTask: Task<DeviceInfo, Error>?
    private var shuttingDown = false
    private var invalidFrames = 0
    /// A non-real-time playback delivers every frame: frame streams are lossless and the reader waits for them.
    private var losslessFrames: Bool

    init(mode: Mode, helperURL: URL) {
        self.mode = mode
        self.helperURL = helperURL
        connectTimeout = mode == .live ? 600 : 20
        losslessFrames = Self.startsLossless(mode)
    }

    private static func startsLossless(_ mode: Mode) -> Bool {
        if case .playback(_, let realTime) = mode { return !realTime }
        return false
    }

    /// A new frame stream: lossless (every frame, the reader waits while `bufferingNewest` frames are queued) during a
    /// non-real-time playback, otherwise only the newest `bufferingNewest` undelivered frames. A stream keeps the kind
    /// it was created with; a lossless one drops its oldest frames while the playback runs in real time.
    func subscribeFrames(bufferingNewest count: Int) -> AsyncStream<CameraFrame> {
        lock.withLock { losslessFrames }
            ? frames.subscribeLossless(capacity: count)
            : frames.subscribe(bufferingNewest: count)
    }

    private func setLosslessFrames(_ lossless: Bool) {
        lock.withLock { losslessFrames = lossless }
        frames.setLossless(lossless)
    }

    // MARK: Connecting

    func connect() async throws -> DeviceInfo {
        if let connection = lock.withLock({ active?.connection }) {
            return try await queryDevice(connection)
        }
        let task: Task<DeviceInfo, Error> = try lock.withLock {
            if shuttingDown { throw CameraError.notConnected }
            if let connectTask { return connectTask }
            let task = Task.detached { [self] in try await establish() }
            connectTask = task
            return task
        }
        return try await withTaskCancellationHandler {
            try await task.value
        } onCancel: {
            task.cancel()
            self.abortStart()
        }
    }

    private func abortStart() {
        lock.withLock { starting }?.server.abort()
    }

    private func resolvedHelperURL() throws -> URL {
        if FileManager.default.isExecutableFile(atPath: helperURL.path) { return helperURL }
        // A development build (swift run, tests) has no bundle: find the helper next to it.
        if helperURL == CameraSources.bundledHelperURL, let found = CameraSources.locateHelper() { return found }
        throw CameraError.helperNotFound(helperURL.path)
    }

    private func arguments(socket: String, helper: URL) -> [String] {
        var args = ["--socket", socket, "--uid", String(getuid())]
        if mode == .live { args += ["--gid", String(getgid())] }
        args += ["--parent-pid", String(getpid())]
        switch mode {
        case .live:
            break
        case .playback(let file, let realTime):
            args += ["--playback", file.path]
            if !realTime { args.append("--no-real-time") }
        case .fake(let padding):
            args.append("--fake")
            if padding > 0 { args += ["--fake-row-padding", String(padding)] }
        }
        return args
    }

    private func establish() async throws -> DeviceInfo {
        defer { lock.withLock { connectTask = nil } }
        let helper = try resolvedHelperURL()
        let server = try HelperSocketServer()
        let args = arguments(socket: server.socketPath, helper: helper)
        let process: HelperProcessHandle
        do {
            process = mode == .live
                ? HelperLauncher.launchPrivileged(helper: helper, arguments: args)
                : try HelperLauncher.launchAsUser(helper: helper, arguments: args)
        } catch {
            server.close()
            throw error
        }
        let session = Active(server: server, process: process)
        let raced: Bool = lock.withLock {
            starting = session
            return shuttingDown
        }
        if raced { server.abort() }
        defer { lock.withLock { if starting === session { starting = nil } } }
        // A helper that ends before connecting (cancelled prompt, bad arguments) must not leave us waiting.
        process.onExit { [weak self, weak server] status, error in
            server?.abort()
            self?.helperExited(session, status: status, error: error)
        }

        do {
            let timeout = connectTimeout
            let fd: Int32 = try await Self.blocking {
                do {
                    return try server.accept(timeout: timeout)
                } catch {
                    throw Self.launchFailure(process, waitingError: error)
                }
            }
            do {
                _ = try HelperSocketServer.verifyPeer(fd: fd, expectedUID: mode == .live ? 0 : getuid(),
                                                      helperPath: helper.path)
            } catch {
                Darwin.close(fd)
                throw error
            }
            server.stopListening()

            let connection = HelperConnection(fd: fd, handlers: handlers(for: session))
            session.connection = connection
            // A new helper plays at the speed it was started with, and its reader may wait for consumers again.
            setLosslessFrames(Self.startsLossless(mode))
            frames.resetRelease()
            connection.start()
            let hello = try await connection.waitForHello(timeout: HelperTimeouts.hello)
            guard hello.protocol == 1 else {
                throw CameraError.protocolError("the camera helper speaks protocol \(hello.protocol), expected 1")
            }
            guard hello.mode == mode.name else {
                throw CameraError.protocolError("the camera helper runs in \(hello.mode) mode, expected \(mode.name)")
            }
            lock.withLock { session.hello = hello }
            logger.info("camera helper \(hello.helperVersion, privacy: .public) (librealsense \(hello.sdkVersion, privacy: .public), pid \(hello.pid)) connected in \(hello.mode, privacy: .public) mode")

            // The helper may already be gone (e.g. it crashed opening the device right after HELLO): connectionClosed()
            // and helperExited() then found no active session to end, so it must not become one now.
            let (cancelled, gone): (Bool, Bool) = lock.withLock {
                if shuttingDown { return (true, false) }
                if session.connectionClosed || session.process.hasExited { return (false, true) }
                active = session
                return (false, false)
            }
            if cancelled { throw CancellationError() }
            if gone {
                throw Self.launchFailure(process, waitingError:
                    CameraError.connectionFailed("the camera helper closed the connection right after connecting"))
            }
        } catch {
            _ = try? await Self.blocking { self.stop(session, timeout: 15) }
            throw Task.isCancelled ? CancellationError() : error
        }
        // From here the helper stays up even when the device query fails (e.g. no camera plugged in yet): the next
        // connect() asks again without another password prompt.
        guard let connection = session.connection else { throw CameraError.notConnected }
        do {
            return try await queryDevice(connection)
        } catch where connection.closed {
            // The helper went away instead of answering: its exit status and stderr say more than the lost socket.
            throw Self.launchFailure(process, waitingError: error)
        }
    }

    /// Why the helper never connected.
    private static func launchFailure(_ process: HelperProcessHandle, waitingError: Error) -> Error {
        if !process.hasExited { process.waitForExit(timeout: 0.5) }
        if let error = process.launchError { return error }
        if let status = process.exitStatus {
            let detail = process.diagnostics
            return CameraError.launchFailed("the camera helper exited with status \(status)"
                + (detail.isEmpty ? "" : ": \(detail)"))
        }
        return waitingError
    }

    private func queryDevice(_ connection: HelperConnection) async throws -> DeviceInfo {
        let json = try await request(connection, .queryDevice, EmptyBody(), timeout: HelperTimeouts.query)
        let info = try Self.decodeData(DeviceInfo.self, from: json)
        events.yield(.connected(info))
        return info
    }

    static func decodeData<T: Decodable>(_ type: T.Type, from json: Data) throws -> T {
        do {
            return try HelperJSON.snakeCaseDecoder.decode(WireReplyData<T>.self, from: json).data
        } catch {
            throw CameraError.protocolError("invalid \(T.self) from the camera helper: \(error)")
        }
    }

    // MARK: Messages from the helper

    private func handlers(for session: Active) -> HelperConnection.Handlers {
        var h = HelperConnection.Handlers()
        h.frame = { [weak self] frame in self?.frames.yield(frame) }
        h.event = { [weak self] event in self?.handleEvent(event) }
        h.log = { [weak self] level, message in
            guard let self else { return }
            switch level {
            case .debug: logger.debug("helper: \(message, privacy: .public)")
            case .info: logger.info("helper: \(message, privacy: .public)")
            case .warning: logger.warning("helper: \(message, privacy: .public)")
            case .error: logger.error("helper: \(message, privacy: .public)")
            }
            events.yield(.log(level, message))
        }
        h.invalidFrame = { [weak self] reason in
            guard let self else { return }
            let count = lock.withLock { () -> Int in
                self.invalidFrames += 1
                return self.invalidFrames
            }
            // One message per burst is enough; a broken helper would otherwise flood the log.
            if count == 1 || count % 100 == 0 {
                logger.error("dropped frameset (\(count) so far): \(reason, privacy: .public)")
                events.yield(.log(.warning, "dropped invalid frameset: \(reason)"))
            }
        }
        // Weak: the session record owns the connection that owns these handlers.
        h.bye = { [weak self, weak session] reason in
            guard let self, let session else { return }
            lock.withLock { session.byeReceived = true }
            logger.info("camera helper said goodbye: \(reason ?? "", privacy: .public)")
        }
        h.closed = { [weak self, weak session] error in
            guard let self, let session else { return }
            connectionClosed(session, error: error)
        }
        return h
    }

    private func handleEvent(_ event: WireEvent) {
        let message = event.message ?? ""
        switch event.event {
        case "device_connected":
            if let data = event.data, let json = try? JSONEncoder().encode(data),
               let info = try? HelperJSON.snakeCaseDecoder.decode(DeviceInfo.self, from: json) {
                events.yield(.connected(info))
            } else if let connection = lock.withLock({ active?.connection }) {
                Task.detached { [weak self] in _ = try? await self?.queryDevice(connection) }
            }
        case "device_disconnected":
            events.yield(.disconnected(reason: message.isEmpty ? "the camera was disconnected" : message))
        case "streaming_started":
            events.yield(.streamingStarted)
        case "streaming_stopped":
            events.yield(.streamingStopped)
        case "stream_restarted":
            events.yield(.streamRestarted(message))
        case "recording_started":
            events.yield(.recordingStarted)
        case "recording_finished":
            let bytes = event.data?["bytes"]?.number.map { Int64($0) } ?? 0
            events.yield(.recordingFinished(bytes: bytes))
        case "playback_finished":
            events.yield(.playbackFinished)
        case "error":
            events.yield(.error(message.isEmpty ? "camera error" : message))
        default:
            events.yield(.log(.debug, "helper event \(event.event): \(message)"))
        }
    }

    private func connectionClosed(_ session: Active, error: CameraError?) {
        let (wasActive, expected): (Bool, Bool) = lock.withLock {
            session.connectionClosed = true
            let wasActive = active === session
            if wasActive { active = nil }
            return (wasActive, shuttingDown || session.byeReceived)
        }
        if wasActive && !expected {
            let reason = error?.localizedDescription ?? "the camera helper closed the connection"
            logger.error("camera helper connection lost: \(reason, privacy: .public)")
            events.yield(.error("Lost the connection to the camera helper: \(reason)"))
        }
        if wasActive {
            // Release the socket off the reader thread that called us.
            DispatchQueue.global().async { session.connection?.close(); session.server.close() }
        }
    }

    private func helperExited(_ session: Active, status: Int32, error: CameraError?) {
        let (reported, current): (Bool, Bool) = lock.withLock {
            let current = active === session
            if current { active = nil }
            // A helper that never connected is reported by connect() throwing.
            return (session.hello != nil, current)
        }
        // Nothing more will come: the reader must reach the end of the connection even if no consumer reads.
        if current { frames.release() }
        logger.info("camera helper exited with status \(status)")
        if reported { events.yield(.helperExited(status: status)) }
    }

    // MARK: Commands

    func command<Body: Encodable>(_ type: HelperMessageType, _ body: Body, explicitNulls: [String] = [],
                                  descriptor: Int32? = nil, timeout: TimeInterval) async throws -> Data {
        guard let connection = lock.withLock({ active?.connection }) else {
            if let descriptor { Darwin.close(descriptor) }
            throw CameraError.notConnected
        }
        return try await request(connection, type, body, explicitNulls: explicitNulls, descriptor: descriptor,
                                 timeout: timeout)
    }

    /// The reply arrives behind the frames the helper sent before it: while it is awaited, the reader queues frames
    /// for lossless consumers instead of waiting for them (a consumer may itself be the one awaiting the reply).
    private func request<Body: Encodable>(_ connection: HelperConnection, _ type: HelperMessageType, _ body: Body,
                                          explicitNulls: [String] = [], descriptor: Int32? = nil,
                                          timeout: TimeInterval) async throws -> Data {
        frames.beginBypass()
        defer { frames.endBypass() }
        return try await connection.request(type, body, explicitNulls: explicitNulls, descriptor: descriptor,
                                            timeout: timeout)
    }

    func configure(_ configuration: CaptureConfiguration) async throws {
        // The protocol gives "laser_power": null a meaning (maximum); the synthesized encoder would omit the key.
        _ = try await command(.configure, configuration,
                              explicitNulls: configuration.laserPower == nil ? ["laserPower"] : [],
                              timeout: HelperTimeouts.configure)
    }

    func setFilters(_ filters: FilterConfiguration) async throws {
        _ = try await command(.setFilters, filters, timeout: HelperTimeouts.option)
    }

    func start() async throws {
        _ = try await command(.start, EmptyBody(), timeout: HelperTimeouts.streaming)
    }

    func stop() async throws {
        _ = try await command(.stop, EmptyBody(), timeout: HelperTimeouts.streaming)
    }

    func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        guard sensor != .motion else { throw CameraError.unsupported("options of the motion module") }
        guard value.isFinite else { throw CameraError.commandFailed("\(name): invalid value") }
        _ = try await command(.setOption, SetOptionBody(sensor: sensor.rawValue, option: name, value: value),
                              timeout: HelperTimeouts.option)
    }

    func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {
        let r = rect.standardized.integral
        guard r.width.isFinite, r.height.isFinite, r.width >= 1, r.height >= 1,
              abs(r.minX) < 1e6, abs(r.minY) < 1e6, r.width < 1e6, r.height < 1e6 else {
            throw CameraError.commandFailed("invalid auto-exposure region \(rect)")
        }
        let distance = distanceMeters.flatMap { $0.isFinite && $0 > 0 ? $0 : nil }
        _ = try await command(.setROI, SetROIBody(x: Int(r.minX), y: Int(r.minY), width: Int(r.width),
                                                  height: Int(r.height), distanceM: distance),
                              timeout: HelperTimeouts.option)
    }

    func startRecording(to url: URL) async throws {
        guard url.isFileURL else { throw CameraError.unsupported("recording to \(url)") }
        let format = url.pathExtension.lowercased() == "bag" ? "bag" : "db3"
        // Created by the app as the user; the helper only ever gets this descriptor, never the path.
        let fd = open(url.path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0o644)
        guard fd >= 0 else {
            throw CameraError.commandFailed("Cannot create \(url.path): \(errnoDescription())")
        }
        do {
            _ = try await command(.recordStart, RecordStartBody(format: format), descriptor: fd,
                                  timeout: HelperTimeouts.recordStart)
        } catch {
            var st = stat()
            if stat(url.path, &st) == 0, st.st_size == 0 { unlink(url.path) }
            throw error
        }
    }

    func stopRecording() async throws -> Int64 {
        let json = try await command(.recordStop, EmptyBody(), timeout: HelperTimeouts.recordStop)
        return try Self.decodeData(WireBytes.self, from: json).bytes
    }

    func playback(pause: Bool, realTime: Bool?) async throws {
        _ = try await command(.playback, PlaybackBody(action: pause ? "pause" : "resume", realTime: realTime),
                              timeout: HelperTimeouts.option)
        // The helper applies real_time with either action and from then on drops frames (real time) or waits.
        if let realTime { setLosslessFrames(!realTime) }
    }

    func hardwareReset() async throws {
        _ = try await command(.hardwareReset, EmptyBody(), timeout: HelperTimeouts.hardwareReset)
    }

    var isConnected: Bool { lock.withLock { active?.connection != nil } }

    /// Tests: makes the connection fail as if the app had died (the helper sees EOF).
    func dropConnectionForTesting() {
        lock.withLock { active?.connection }?.io.shutdown()
    }
    var hello: HelperHello? { lock.withLock { active?.hello } }

    // MARK: Shutdown

    func shutdown(timeout: TimeInterval) async {
        await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
            let thread = Thread { [self] in
                shutdownBlocking(timeout: timeout)
                continuation.resume()
            }
            thread.name = "IntelFaceScan helper shutdown"
            thread.start()
        }
    }

    /// Ends the session: stops a pending connect, asks the helper to quit and waits (bounded) for its exit, then ends
    /// the frame and event streams. Idempotent; never needs the main thread.
    func shutdownBlocking(timeout: TimeInterval) {
        let deadline = Date(timeIntervalSinceNow: timeout)
        let (session, pending, task): (Active?, Active?, Task<DeviceInfo, Error>?) = lock.withLock {
            shuttingDown = true
            return (active, starting, connectTask)
        }
        // A lossless consumer that stopped reading must not keep the reader from QUIT's reply and the helper's BYE.
        frames.release()
        pending?.server.abort()
        if let task {
            let done = DispatchSemaphore(value: 0)
            Task.detached {
                _ = try? await task.value
                done.signal()
            }
            _ = done.wait(timeout: .now() + max(0, deadline.timeIntervalSinceNow))
        }
        if let session { stop(session, timeout: max(1, deadline.timeIntervalSinceNow)) }
        events.finishAll()
        frames.finishAll()
        lock.withLock {
            if active === session { active = nil }
            shuttingDown = false
        }
    }

    /// QUIT -> BYE / socket closed -> process exit. The helper releases the camera between QUIT and BYE.
    private func stop(_ session: Active, timeout: TimeInterval) {
        let deadline = Date(timeIntervalSinceNow: timeout)
        func remaining() -> TimeInterval { max(0, deadline.timeIntervalSinceNow) }

        session.server.abort()
        if let connection = session.connection {
            if !connection.closed {
                frames.beginBypass()
                _ = try? connection.requestBlocking(.quit, EmptyBody(), timeout: min(HelperTimeouts.quit, remaining()))
                connection.waitUntilClosed(timeout: remaining())
                frames.endBypass()
            }
            connection.close()
        }
        if !session.process.waitForExit(timeout: remaining()) {
            if mode == .live {
                // Root cannot be signalled from here; it exits by itself once it notices the closed socket.
                logger.error("the camera helper did not exit in time")
            } else {
                session.process.terminate()
                if !session.process.waitForExit(timeout: 3) {
                    session.process.kill()
                    session.process.waitForExit(timeout: 2)
                }
            }
        }
        session.server.close()
    }

    // MARK: Helpers

    /// Runs blocking work on its own thread (accept() can wait minutes for the password prompt).
    static func blocking<T>(_ work: @escaping () throws -> T) async throws -> T {
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<T, Error>) in
            let thread = Thread {
                continuation.resume(with: Result { try work() })
            }
            thread.start()
        }
    }
}
