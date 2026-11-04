// License: Apache 2.0. See LICENSE file in root directory.
//
// Swift Testing starts every test at once, and each reconstruction (synthetic 1280x800 scan, TSDF, cleanup,
// texture) peaks at about 0.5 GB. Twenty of them at once fit a workstation but not a 3-core, 7 GB CI runner: there
// the run swaps until the step times out. Suites and tests marked .heavyReconstruction share a few slots, sized by
// the cores and the memory of the machine (IFS_TEST_HEAVY_SLOTS overrides), so a small machine runs them a few at a
// time and a large one all at once. A .timeLimit counts the wait for a slot too.

import Foundation
import Testing

actor HeavyTestSlots {
    static let shared = HeavyTestSlots(count: HeavyTestSlots.defaultCount())

    private var free: Int
    private var waiters: [CheckedContinuation<Void, Never>] = []

    init(count: Int) { free = max(1, count) }

    static func defaultCount() -> Int {
        if let text = ProcessInfo.processInfo.environment["IFS_TEST_HEAVY_SLOTS"], let n = Int(text), n > 0 { return n }
        let cores = ProcessInfo.processInfo.activeProcessorCount
        let memoryGB = Int(ProcessInfo.processInfo.physicalMemory >> 30)
        // About 0.5 GB per reconstruction; 3 GB stay for the system and the lighter tests running alongside.
        return max(1, min(cores, (memoryGB - 3) * 2))
    }

    func acquire() async {
        if free > 0 {
            free -= 1
            return
        }
        await withCheckedContinuation { waiters.append($0) }
    }

    func release() {
        if waiters.isEmpty { free += 1 } else { waiters.removeFirst().resume() }
    }
}

/// Runs each test case of the marked suite or test inside one of the HeavyTestSlots.
struct HeavyReconstructionTrait: SuiteTrait, TestTrait, TestScoping {
    var isRecursive: Bool { true }

    func scopeProvider(for test: Test, testCase: Test.Case?) -> Self? { testCase == nil ? nil : self }

    func provideScope(for test: Test, testCase: Test.Case?, performing function: @Sendable () async throws -> Void) async throws {
        await HeavyTestSlots.shared.acquire()
        do {
            try await function()
        } catch {
            await HeavyTestSlots.shared.release()
            throw error
        }
        await HeavyTestSlots.shared.release()
    }
}

extension Trait where Self == HeavyReconstructionTrait {
    static var heavyReconstruction: Self { Self() }
}
