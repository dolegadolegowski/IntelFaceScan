// License: Apache 2.0. See LICENSE file in root directory.
//
// Broadcaster (HelperSession.swift) without a helper: lossless subscriptions deliver every value in order and make
// the producer wait; bypass, release, finishAll, dropping or cancelling a stream all let a waiting producer go.

import Foundation
import Testing
@testable import CameraKit

/// Runs `yield` on its own thread (as the helper reader does) and tells whether it has returned.
private final class Producer: @unchecked Sendable {
    private let done = DispatchSemaphore(value: 0)

    init(_ broadcaster: Broadcaster<Int>, _ values: [Int]) {
        let thread = Thread { [done] in
            for value in values { broadcaster.yield(value) }
            done.signal()
        }
        thread.start()
    }

    /// True when every value was yielded within `seconds`.
    func finished(within seconds: Double) -> Bool { done.wait(timeout: .now() + seconds) == .success }
}

private func take(_ iterator: inout AsyncStream<Int>.Iterator, _ count: Int) async -> [Int] {
    var values: [Int] = []
    for _ in 0..<count {
        guard let value = await iterator.next() else { break }
        values.append(value)
    }
    return values
}

@Suite("Helper broadcaster", .timeLimit(.minutes(1)))
struct HelperBroadcasterTests {
    @Test func losslessDeliversEveryValueInOrderAndMakesTheProducerWait() async {
        let broadcaster = Broadcaster<Int>()
        let stream = broadcaster.subscribeLossless(capacity: 2)
        let producer = Producer(broadcaster, Array(0..<10))
        #expect(!producer.finished(within: 0.3), "two values fit, the third must wait for the consumer")
        var iterator = stream.makeAsyncIterator()
        var values: [Int] = []
        for _ in 0..<10 {
            if let value = await iterator.next() { values.append(value) }
            try? await Task.sleep(for: .milliseconds(5))
        }
        #expect(values == Array(0..<10))
        #expect(producer.finished(within: 2))
    }

    @Test func lossyAndLosslessSubscribersCoexist() async {
        let broadcaster = Broadcaster<Int>()
        let lossy = broadcaster.subscribe(bufferingNewest: 1)
        let lossless = broadcaster.subscribeLossless(capacity: 8)
        #expect(broadcaster.subscriberCount == 2)
        let producer = Producer(broadcaster, Array(0..<5))
        #expect(producer.finished(within: 2))
        var a = lossy.makeAsyncIterator()
        var b = lossless.makeAsyncIterator()
        #expect(await a.next() == 4, "a lossy subscriber keeps only the newest value")
        #expect(await take(&b, 5) == Array(0..<5))
    }

    @Test func bypassQueuesWithoutWaiting() async {
        let broadcaster = Broadcaster<Int>()
        let stream = broadcaster.subscribeLossless(capacity: 1)
        broadcaster.beginBypass()
        #expect(Producer(broadcaster, Array(0..<6)).finished(within: 2), "no wait while a command awaits its reply")
        broadcaster.endBypass()
        let waiting = Producer(broadcaster, [6])
        #expect(!waiting.finished(within: 0.2), "waits again after the bypass")
        var iterator = stream.makeAsyncIterator()
        #expect(await take(&iterator, 7) == Array(0..<7), "nothing was lost")
        #expect(waiting.finished(within: 2))
    }

    @Test func releaseAndFinishLetAWaitingProducerGo() async {
        let broadcaster = Broadcaster<Int>()
        let stream = broadcaster.subscribeLossless(capacity: 1)
        let producer = Producer(broadcaster, [0, 1, 2])
        #expect(!producer.finished(within: 0.2))
        broadcaster.release()
        #expect(producer.finished(within: 2))
        broadcaster.finishAll()
        var iterator = stream.makeAsyncIterator()
        #expect(await take(&iterator, 10) == [0, 1, 2], "queued values are delivered, then the stream ends")
        #expect(await iterator.next() == nil)
        #expect(broadcaster.subscriberCount == 0)

        broadcaster.resetRelease()
        let next = broadcaster.subscribeLossless(capacity: 1)
        let waiting = Producer(broadcaster, [3, 4])
        #expect(!waiting.finished(within: 0.2), "a new session waits again")
        broadcaster.finishAll()
        #expect(waiting.finished(within: 2), "finishAll() ends the wait")
        var it = next.makeAsyncIterator()
        #expect(await take(&it, 10) == [3])
    }

    @Test func droppedStreamUnsubscribes() {
        let broadcaster = Broadcaster<Int>()
        var producer: Producer?
        do {
            let stream = broadcaster.subscribeLossless(capacity: 1)
            producer = Producer(broadcaster, [0, 1, 2])
            #expect(producer?.finished(within: 0.2) == false)
            withExtendedLifetime(stream) {}
        }
        #expect(producer?.finished(within: 2) == true, "nobody holds the stream any more")
        #expect(broadcaster.subscriberCount == 0)
    }

    @Test func cancelledConsumerUnsubscribes() async {
        let broadcaster = Broadcaster<Int>()
        let stream = broadcaster.subscribeLossless(capacity: 1)
        let consumer = Task { () -> [Int] in
            var values: [Int] = []
            for await value in stream { values.append(value) }
            return values
        }
        broadcaster.yield(0)
        try? await Task.sleep(for: .milliseconds(100))
        consumer.cancel()
        #expect(await consumer.value == [0])
        #expect(Producer(broadcaster, [1, 2, 3]).finished(within: 2), "a cancelled consumer holds nobody up")
    }

    @Test func lossyModeDropsTheOldest() async {
        let broadcaster = Broadcaster<Int>()
        let stream = broadcaster.subscribeLossless(capacity: 2)
        broadcaster.setLossless(false)
        #expect(Producer(broadcaster, Array(0..<6)).finished(within: 2))
        broadcaster.finishAll()
        var iterator = stream.makeAsyncIterator()
        #expect(await take(&iterator, 10) == [4, 5])
    }
}
