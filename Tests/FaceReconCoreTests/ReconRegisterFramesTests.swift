// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Foundation
import Testing

@Suite("Recon whole-scan registration")
struct ReconRegisterFramesTests {
    static let head = HeadScene()
    static let camera = SynthCamera.d455(width: 640, height: 400)

    /// Scan order of the guided scan with the target angles of ScanKit.ScanPose (yaw, pitch), small hold jitter.
    static let scanAngles: [(Double, Double)] = [
        (0, 0), (1.5, -1), (-1, 1.5),   // front
        (0, 25), (2, 24),               // up
        (0, -25), (-2, -24),            // down
        (45, 0), (43, 2),               // left
        (-45, 0), (-44, -2),            // right
    ]

    static let views: [RenderedFrame] = scanAngles.enumerated().map { i, a in
        let truth = headPoseInWorld(headRotation(yaw: a.0, pitch: a.1), center: head.center)
        return render(head, camera: camera, worldFromCamera: truth, noiseAt1m: 0.0025, seed: UInt64(100 + i))
    }

    /// Head points in the world frame, for misalignment in millimeters as fusion sees it.
    static let headPoints = bumpySpherePoints(count: 64, radius: 0.09, center: head.center, jitterSeed: 21)

    static func headError(_ a: Rigid, _ b: Rigid) -> Double {
        headPoints.map { (a.apply($0) - b.apply($0)).length }.max() ?? 0
    }

    /// Initial poses as Vision would give them: rotation off by a few degrees, translation by millimeters.
    static func perturbedInitialPoses(seed: UInt64, rotationDeg: Double, translation: Double) -> [Rigid] {
        var rng = SplitMix64(seed: seed)
        return views.map { v in
            let axis = V3(rng.gaussian(), rng.gaussian(), rng.gaussian())
            let rot = Rigid(axis: axis, angleDeg: rotationDeg)
            let c = head.center
            let d = Rigid(r: rot.r, t: c - rot.rotate(c) + V3(rng.gaussian(), rng.gaussian(), rng.gaussian()).normalized * translation)
            return d * v.worldFromCamera
        }
    }

    static func register(initial: [Rigid], params: fr_registration_params) throws -> ([Rigid], fr_registration_report) {
        var frames = zip(views, initial).map { $0.0.coreFrame(initialPose: $0.1) }
        var p = params
        var out = [Float](repeating: 0, count: 16 * frames.count)
        var report = fr_registration_report()
        let status = fr_register_frames(&frames, UInt32(frames.count), &p, &out, &report, nil, nil)
        try #require(status == FR_OK, "\(String(cString: fr_last_error()))")
        let poses = (0 ..< frames.count).map { Rigid(floats: Array(out[16 * $0 ..< 16 * $0 + 16])) }
        return (poses, report)
    }

    @Test func registersRenderedHeadViewsFromPerturbedInitialPoses() throws {
        var params = fr_registration_params()
        fr_registration_params_default(&params)
        let initial = Self.perturbedInitialPoses(seed: 5, rotationDeg: 4, translation: 0.006)
        let (poses, report) = try Self.register(initial: initial, params: params)

        // The world frame is frame 0 (truth[0] is the identity: the frontal view).
        #expect(Self.headError(poses[0], .identity) < 1e-6)
        var worstInitial = 0.0, worst = 0.0, worstRotation = 0.0
        for i in 0 ..< poses.count {
            let truth = Self.views[i].worldFromCamera
            worstInitial = max(worstInitial, Self.headError(initial[0].inverse * initial[i], truth))
            worst = max(worst, Self.headError(poses[i], truth))
            worstRotation = max(worstRotation, Rigid.error(poses[i], truth).rotationDeg)
        }
        print("registration: worst head misalignment \(worst * 1000) mm (initial \(worstInitial * 1000) mm), "
            + "worst rotation error \(worstRotation) deg, odometry \(report.odometry_edges), loops \(report.loop_closures), "
            + "rejected \(report.rejected_edges), mean RMSE \(report.mean_rmse_m * 1000) mm, fitness \(report.mean_fitness)")
        #expect(worst < 0.0008, "worst head misalignment \(worst * 1000) mm")
        #expect(worstRotation < 0.4)
        #expect(report.frames == UInt32(Self.views.count))
        #expect(report.odometry_edges >= UInt32(Self.views.count - 1))
        #expect(report.loop_closures > 0)
        #expect(report.mean_rmse_m > 0 && report.mean_rmse_m < 0.0015)
        #expect(report.mean_fitness > 0.5)
    }

    @Test func geometryOnlyRegistrationWithoutPoseGraphAlsoConverges() throws {
        var params = fr_registration_params()
        fr_registration_params_default(&params)
        params.icp.colored = 0
        params.global_optimization = 0
        let initial = Self.perturbedInitialPoses(seed: 8, rotationDeg: 3, translation: 0.004)
        let (poses, report) = try Self.register(initial: initial, params: params)
        var worst = 0.0
        for i in 0 ..< poses.count { worst = max(worst, Self.headError(poses[i], Self.views[i].worldFromCamera)) }
        #expect(worst < 0.0012, "worst head misalignment \(worst * 1000) mm")
        #expect(report.loop_closures == 0)
    }

    @Test func registrationCanBeCancelled() throws {
        var frames = Self.views.prefix(4).map { $0.coreFrame(initialPose: $0.worldFromCamera) }
        var params = fr_registration_params()
        fr_registration_params_default(&params)
        var out = [Float](repeating: 0, count: 16 * frames.count)
        let cancel: fr_progress_cb = { _, fraction, _ in fraction > 0.3 ? 1 : 0 }
        #expect(fr_register_frames(&frames, UInt32(frames.count), &params, &out, nil, cancel, nil) == FR_ERR_CANCELLED)
        #expect(fr_register_frames(&frames, 0, &params, &out, nil, nil, nil) == FR_ERR_INVALID_ARGUMENT)
    }
}
