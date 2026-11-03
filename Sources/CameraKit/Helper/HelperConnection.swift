// License: Apache 2.0. See LICENSE file in root directory.
//
// The app's end of one helper connection: a reader thread that decodes every message, requests with unique ids that
// complete exactly once (reply, timeout, cancellation or connection loss), and callbacks for framesets, events,
// logs and the end of the connection.

import Darwin
import Foundation

final class HelperConnection: @unchecked Sendable {
    struct Handlers {
        var frame: (CameraFrame) -> Void = { _ in }
        var event: (WireEvent) -> Void = { _ in }
        var log: (LogLevel, String) -> Void = { _, _ in }
        /// A FRAMESET that could not be used (dropped; the connection stays up).
        var invalidFrame: (String) -> Void = { _ in }
        var bye: (String?) -> Void = { _ in }
        /// Called once when the connection ended: nil after a clean close, otherwise why.
        var closed: (CameraError?) -> Void = { _ in }
    }

    private struct Pending {
        let completion: (Result<Data, Error>) -> Void
        let timeout: DispatchWorkItem
    }

    let io: HelperSocketIO
    private let handlers: Handlers
    private let lock = NSLock()
    private let closedCondition = NSCondition()
    private var pending: [UInt64: Pending] = [:]
    private var allocated: Set<UInt64> = []  // ids handed out but not registered yet
    private var cancelled: Set<UInt64> = []
    private var nextRequestId: UInt64 = 1
    private var isClosed = false
    private var closeError: CameraError?
    private var hello: HelperHello?
    private var helloWaiters: [(Result<HelperHello, CameraError>) -> Void] = []
    private var readerFinished = false
    private let writeQueue = DispatchQueue(label: "io.github.dolegadolegowski.IntelFaceScan.helper-write")

    init(fd: Int32, handlers: Handlers) {
        io = HelperSocketIO(fd: fd)
        self.handlers = handlers
    }

    func start() {
        let thread = Thread { [self] in readLoop() }
        thread.name = "IntelFaceScan helper reader"
        thread.qualityOfService = .userInteractive
        thread.start()
    }

    // MARK: Requests

    /// Sends a command and calls `completion` exactly once. A descriptor is owned by the connection from here on
    /// (closed once it was sent or the request failed).
    func send<Body: Encodable>(_ type: HelperMessageType, _ body: Body, explicitNulls: [String] = [],
                               descriptor: Int32? = nil, timeout: TimeInterval, id preallocated: UInt64? = nil,
                               completion: @escaping (Result<Data, Error>) -> Void) {
        let id = preallocated ?? allocateId()
        let json: Data
        do {
            json = try HelperJSON.encodeCommand(requestId: id, body: body, explicitNulls: explicitNulls)
        } catch {
            _ = lock.withLock { allocated.remove(id) }
            if let descriptor { Darwin.close(descriptor) }
            completion(.failure(CameraError.protocolError("cannot encode \(type): \(error)")))
            return
        }

        let timeoutItem = DispatchWorkItem { [weak self] in
            self?.complete(id, .failure(CameraError.timeout("camera helper did not answer \(type) in \(Int(timeout)) s")))
        }
        let registered: Result<Void, Error> = lock.withLock {
            allocated.remove(id)
            if cancelled.remove(id) != nil { return .failure(CancellationError()) }
            if isClosed { return .failure(closeError ?? CameraError.notConnected) }
            pending[id] = Pending(completion: completion, timeout: timeoutItem)
            return .success(())
        }
        if case .failure(let error) = registered {
            if let descriptor { Darwin.close(descriptor) }
            completion(.failure(error))
            return
        }
        DispatchQueue.global().asyncAfter(deadline: .now() + timeout, execute: timeoutItem)
        writeQueue.async { [self] in
            do {
                try io.writeMessage(type: type.rawValue, json: json, descriptor: descriptor)
            } catch {
                complete(id, .failure(error))
            }
            if let descriptor { Darwin.close(descriptor) }
        }
    }

    private func allocateId() -> UInt64 {
        lock.withLock {
            let id = nextRequestId
            nextRequestId += 1
            allocated.insert(id)
            return id
        }
    }

    func request<Body: Encodable>(_ type: HelperMessageType, _ body: Body, explicitNulls: [String] = [],
                                  descriptor: Int32? = nil, timeout: TimeInterval) async throws -> Data {
        let id = allocateId()
        return try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Data, Error>) in
                send(type, body, explicitNulls: explicitNulls, descriptor: descriptor, timeout: timeout, id: id) {
                    continuation.resume(with: $0)
                }
            }
        } onCancel: {
            self.cancel(id)
        }
    }

    /// For shutdown paths that cannot be async (the timeout bounds the wait).
    func requestBlocking<Body: Encodable>(_ type: HelperMessageType, _ body: Body, timeout: TimeInterval) throws -> Data {
        let semaphore = DispatchSemaphore(value: 0)
        let box = ResultBox()
        send(type, body, timeout: timeout) { result in
            box.result = result
            semaphore.signal()
        }
        semaphore.wait()
        guard let result = box.result else { throw CameraError.notConnected }
        return try result.get()
    }

    private func cancel(_ id: UInt64) {
        let found = lock.withLock { () -> Bool in
            if pending[id] != nil { return true }
            // Not registered yet: send() fails it when it gets there.
            if allocated.contains(id) { cancelled.insert(id) }
            return false
        }
        if found { complete(id, .failure(CancellationError())) }
    }

    private func complete(_ id: UInt64, _ result: Result<Data, Error>) {
        guard let request = lock.withLock({ pending.removeValue(forKey: id) }) else { return }
        request.timeout.cancel()
        request.completion(result)
    }

    // MARK: HELLO

    func waitForHello(timeout: TimeInterval) async throws -> HelperHello {
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<HelperHello, Error>) in
            let once = OnceFlag()
            let deliver: (Result<HelperHello, CameraError>) -> Void = { result in
                if once.claim() { continuation.resume(with: result) }
            }
            let immediate: Result<HelperHello, CameraError>? = lock.withLock {
                if let hello { return .success(hello) }
                if isClosed { return .failure(closeError ?? CameraError.connectionFailed("the camera helper disconnected")) }
                helloWaiters.append(deliver)
                return nil
            }
            if let immediate {
                deliver(immediate)
                return
            }
            DispatchQueue.global().asyncAfter(deadline: .now() + timeout) {
                deliver(.failure(CameraError.timeout("the camera helper did not say hello in \(Int(timeout)) s")))
            }
        }
    }

    // MARK: Reading

    private func readLoop() {
        while true {
            let message: WireMessage?
            do {
                message = try io.readMessage()
            } catch let error as CameraError {
                finish(error)
                return
            } catch {
                finish(.connectionFailed("\(error)"))
                return
            }
            guard let message else {
                finish(nil)
                return
            }
            do {
                try handle(message)
            } catch let error as CameraError {
                // The peer does not speak the protocol: close, as docs/PROTOCOL.md requires.
                io.shutdown()
                finish(error)
                return
            } catch {
                io.shutdown()
                finish(.protocolError("\(error)"))
                return
            }
        }
    }

    private func handle(_ message: WireMessage) throws {
        guard let type = HelperMessageType(rawValue: message.type) else {
            throw CameraError.protocolError("unknown message type \(message.type)")
        }
        switch type {
        case .frameset:
            do {
                handlers.frame(try HelperFrameDecoder.decode(json: message.json, payload: message.payload))
            } catch let error as CameraError {
                _ = try HelperJSON.object(message.json)  // a body that is no JSON object still ends the connection
                handlers.invalidFrame(error.localizedDescription)
            }
        case .reply:
            let reply = try decode(WireReply.self, message)
            let result: Result<Data, Error> = reply.ok
                ? .success(message.json)
                : .failure(CameraError.commandFailed(reply.error ?? "the camera helper reported an error"))
            complete(reply.requestId, result)
        case .event:
            handlers.event(try decode(WireEvent.self, message))
        case .log:
            let log = try decode(WireLog.self, message)
            handlers.log(LogLevel(rawValue: log.level) ?? .info, log.message)
        case .hello:
            let hello = try decode(HelperHello.self, message)
            let waiters: [(Result<HelperHello, CameraError>) -> Void] = lock.withLock {
                self.hello = hello
                defer { helloWaiters.removeAll() }
                return helloWaiters
            }
            waiters.forEach { $0(.success(hello)) }
        case .bye:
            handlers.bye(try decode(WireBye.self, message).reason)
        default:
            throw CameraError.protocolError("the helper sent a command (\(type))")
        }
    }

    private func decode<T: Decodable>(_ type: T.Type, _ message: WireMessage) throws -> T {
        do {
            return try HelperJSON.decoder.decode(T.self, from: message.json)
        } catch {
            throw CameraError.protocolError("invalid \(T.self): \(error)")
        }
    }

    private func finish(_ error: CameraError?) {
        let (requests, waiters): ([Pending], [(Result<HelperHello, CameraError>) -> Void]) = lock.withLock {
            isClosed = true
            closeError = error
            let requests = Array(pending.values)
            pending.removeAll()
            let waiters = helloWaiters
            helloWaiters.removeAll()
            return (requests, waiters)
        }
        let failure = error ?? CameraError.connectionFailed("the camera helper closed the connection")
        for request in requests {
            request.timeout.cancel()
            request.completion(.failure(failure))
        }
        waiters.forEach { $0(.failure(failure)) }
        closedCondition.lock()
        readerFinished = true
        closedCondition.broadcast()
        closedCondition.unlock()
        handlers.closed(error)
    }

    // MARK: Teardown

    var closed: Bool { lock.withLock { isClosed } }

    /// Waits until the helper closed the connection (after BYE) or the timeout passed.
    @discardableResult
    func waitUntilClosed(timeout: TimeInterval) -> Bool {
        let deadline = Date(timeIntervalSinceNow: timeout)
        closedCondition.lock()
        defer { closedCondition.unlock() }
        while !readerFinished {
            if !closedCondition.wait(until: deadline) { return readerFinished }
        }
        return true
    }

    /// Ends the connection: wakes the reader, waits for it and for queued writes, then releases the socket.
    func close() {
        io.shutdown()
        waitUntilClosed(timeout: 5)
        writeQueue.sync {}
        if waitUntilClosed(timeout: 0) { io.close() }
    }
}

private final class ResultBox: @unchecked Sendable {
    var result: Result<Data, Error>?
}

final class OnceFlag: @unchecked Sendable {
    private let lock = NSLock()
    private var claimed = false
    /// True for the first caller only.
    func claim() -> Bool {
        lock.withLock {
            if claimed { return false }
            claimed = true
            return true
        }
    }
}
