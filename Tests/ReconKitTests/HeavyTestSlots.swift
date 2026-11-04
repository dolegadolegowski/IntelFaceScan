// License: Apache 2.0. See LICENSE file in root directory.
//
// Swift Testing starts every test at once, and each reconstruction (synthetic 1280x800 scan, TSDF, cleanup,
// texture) peaks at about 0.5 GB and uses every core it gets. With about twenty of them at once, no test finished for
// 35 minutes on the 3-core, 7 GB CI runner (memory pressure or CPU oversubscription: neither was measured there).
// Suites and tests marked .heavyReconstruction share a few slots, sized by the cores and the memory of the machine
// (IFS_TEST_HEAVY_SLOTS overrides): 3 on the runner, where ReconKitTests then passed in about 400 s, and no limit in
// practice on a workstation. An exclusive test takes every slot, i.e. runs with no other marked test. A .timeLimit
// counts the wait for a slot too.

import Foundation
import Testing

actor HeavyTestSlots {
    static let shared = HeavyTestSlots(count: HeavyTestSlots.defaultCount())

    let total: Int
    private var free: Int
    /// First in, first out: an exclusive test waiting for every slot is not overtaken by single-slot tests.
    private var waiters: [(count: Int, continuation: CheckedContinuation<Void, Never>)] = []

    init(count: Int) {
        total = max(1, count)
        free = total
    }

    static func defaultCount() -> Int {
        if let text = ProcessInfo.processInfo.environment["IFS_TEST_HEAVY_SLOTS"], let n = Int(text), n > 0 { return n }
        let cores = ProcessInfo.processInfo.activeProcessorCount
        let memoryGB = Int(ProcessInfo.processInfo.physicalMemory >> 30)
        // About 0.5 GB per reconstruction; 3 GB stay for the system and the lighter tests running alongside.
        return max(1, min(cores, (memoryGB - 3) * 2))
    }

    func acquire(_ count: Int = 1) async {
        let count = min(max(1, count), total)
        if waiters.isEmpty && free >= count {
            free -= count
            return
        }
        await withCheckedContinuation { waiters.append((count, $0)) }
    }

    func release(_ count: Int = 1) {
        free += min(max(1, count), total)
        while let next = waiters.first, free >= next.count {
            waiters.removeFirst()
            free -= next.count
            next.continuation.resume()
        }
    }
}

/// Runs each test case of the marked suite or test inside one of the HeavyTestSlots, or all of them (exclusive).
struct HeavyReconstructionTrait: SuiteTrait, TestTrait, TestScoping {
    var exclusive = false

    var isRecursive: Bool { true }

    func scopeProvider(for test: Test, testCase: Test.Case?) -> Self? { testCase == nil ? nil : self }

    func provideScope(for test: Test, testCase: Test.Case?, performing function: @Sendable () async throws -> Void) async throws {
        let slots = exclusive ? HeavyTestSlots.shared.total : 1
        await HeavyTestSlots.shared.acquire(slots)
        do {
            try await function()
        } catch {
            await HeavyTestSlots.shared.release(slots)
            throw error
        }
        await HeavyTestSlots.shared.release(slots)
    }
}

extension Trait where Self == HeavyReconstructionTrait {
    static var heavyReconstruction: Self { Self() }
    /// For a test whose timing breaks when other reconstructions compete for the CPU.
    static func heavyReconstruction(exclusive: Bool) -> Self { Self(exclusive: exclusive) }
}
