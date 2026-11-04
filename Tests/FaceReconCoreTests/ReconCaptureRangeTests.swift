// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Foundation
import Testing

/// Capture range of the whole-scan registration against the initial pose error.
/// Opt-in: IFS_BENCH=1 scripts/test.sh -c release --filter ReconCaptureRange
@Suite("ReconCaptureRange", .enabled(if: ProcessInfo.processInfo.environment["IFS_BENCH"] == "1"))
struct ReconCaptureRange {
    @Test func initialPoseErrorSweep() throws {
        var rows = ["| initial rotation error | initial translation error | colored ICP | worst misalignment (mm) | max rejected edges | time (s) |",
                    "|---|---|---|---|---|---|"]
        for (rot, trans) in [(2.0, 0.003), (4.0, 0.006), (6.0, 0.01), (8.0, 0.015), (10.0, 0.02)] {
            for colored in [true, false] {
                var params = fr_registration_params()
                fr_registration_params_default(&params)
                params.icp.colored = colored ? 1 : 0
                var worst = 0.0
                var rejected: UInt32 = 0
                var time = 0.0
                for seed in 1 ... 3 {
                    let initial = ReconRegisterFramesTests.perturbedInitialPoses(seed: UInt64(seed * 17), rotationDeg: rot, translation: trans)
                    let start = Date()
                    let (poses, report) = try ReconRegisterFramesTests.register(initial: initial, params: params)
                    time += Date().timeIntervalSince(start) / 3
                    rejected = max(rejected, report.rejected_edges)
                    for i in 0 ..< poses.count {
                        worst = max(worst, ReconRegisterFramesTests.headError(poses[i], ReconRegisterFramesTests.views[i].worldFromCamera))
                    }
                }
                rows.append(String(format: "| %.0f deg | %.0f mm | %@ | %.2f | %u | %.2f |", rot, trans * 1000, colored ? "yes" : "no",
                                   worst * 1000, rejected, time))
            }
        }
        print("capture range (11 views 640x400, 3 seeds each):\n" + rows.joined(separator: "\n"))
    }
}
