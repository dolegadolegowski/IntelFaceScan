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

    static func register(_ views: [RenderedFrame] = views, initial: [Rigid],
                         params: fr_registration_params) throws -> ([Rigid], fr_registration_report) {
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

    /// Side views at 65° that share only a smooth patch with the frontal views, as in a real scan: no depth beyond 60°
    /// incidence and frontal depth only within 5.4 cm of the head center across the image (a D455 head mask measured
    /// 10.7 cm wide). Scan order front x3, left x2, right x3.
    static let sideAngles: [(Double, Double)] = [(0, 0), (1.5, -1), (-1, 1.5), (65, 0), (64, 2), (-65, 0), (-64, -2), (-66, 1)]

    static func renderSideViews(noiseAt1m: Double) -> [RenderedFrame] {
        sideAngles.enumerated().map { i, a in
            let truth = headPoseInWorld(headRotation(yaw: a.0, pitch: a.1), center: head.center)
            let frontal = abs(a.0) < 10
            return render(head, camera: camera, worldFromCamera: truth, noiseAt1m: noiseAt1m, seed: UInt64(300 + i),
                          maxIncidenceDeg: 60, keep: frontal ? { abs($0.x - head.center.x) <= 0.054 } : nil)
        }
    }

    static let sideViews = renderSideViews(noiseAt1m: 0.0025)

    /// Initial poses with the side frames off like Vision's: turn overstated by yawDeg, spurious pitch and roll about
    /// the head center, plus a shift (meters).
    static func sideInitialPoses(yawDeg: Double, pitchDeg: Double, rollDeg: Double, shift: Double) -> [Rigid] {
        zip(sideViews, sideAngles).map { view, a in
            guard abs(a.0) > 10 else { return view.worldFromCamera }
            let wrong = headPoseInWorld(headRotation(yaw: a.0 + (a.0 > 0 ? yawDeg : -yawDeg), pitch: a.1 + pitchDeg, roll: rollDeg),
                                        center: head.center)
            return Rigid(r: wrong.r, t: wrong.t + V3(0.6, -0.8, 0) * shift)
        }
    }

    /// Depth noise sigma = noiseAt1m * z^2; the shared surface lies 0.50-0.56 m from the camera: 0.6-0.8 mm there at
    /// 0.0025, and 1.0-1.2 mm at 0.0039, about the most at which the guard still separates the slides. At 0.0045
    /// (1.2-1.4 mm) slid edges score like correct ones: the right side frames were accepted 9.3-10.0 mm off from the
    /// exact and the 2° start (docs/RECONSTRUCTION.md, "Known limits").
    @Test(arguments: [0.0025, 0.0039])
    func degenerateSideOverlapIsNeverAccepted(noiseAt1m: Double) throws {
        // Side-to-front fitness at the true pose is as low as a real scan's 60-68° side edges (0.065-0.077), so these
        // edges can only pass the overlap test. Its statistics cannot see ICP sliding along the smooth shared patch;
        // without the guard (overlap_constraint) the side frames were accepted 8.5-51 mm off, even from the truth.
        let views = noiseAt1m == 0.0025 ? Self.sideViews : Self.renderSideViews(noiseAt1m: noiseAt1m)
        var icp = fr_icp_params()
        fr_icp_params_default(&icp)
        icp.max_iterations = 0
        icp.levels = 1
        icp.colored = 0
        var cloudParams = fr_cloud_params()
        fr_cloud_params_default(&cloudParams)
        var clouds: [CoreCloud] = []
        for i in [0, 3] {
            var frame = views[i].coreFrame(useColor: false)
            var out: OpaquePointer?
            try #require(fr_cloud_from_rgbd(&frame, &cloudParams, &out) == FR_OK)
            clouds.append(CoreCloud(try #require(out)))
        }
        let relative = (views[0].worldFromCamera.inverse * views[3].worldFromCamera).floats
        var atTruth = fr_icp_result()
        _ = relative.withUnsafeBufferPointer { fr_icp(clouds[1].pointer, clouds[0].pointer, $0.baseAddress, &icp, &atTruth) }
        #expect(atTruth.fitness < 0.15, "side-to-front fitness at the truth \(atTruth.fitness)")

        var params = fr_registration_params()
        fr_registration_params_default(&params)
        params.icp.colored = 0
        let cases: [(String, Double, Double, Double, Double)] = [
            ("exact", 0, 0, 0, 0), ("2°", 2, 1, 0, 0.003), ("8°", 5, 5, 3, 0.008), ("rec4-like", 6, 9, 7, 0.015),
            ("15° yaw", 15, 0, 0, 0.010), ("25° toward the front", -25, 0, 0, 0),
        ]
        for (name, yaw, pitch, roll, shift) in cases {
            let initial = Self.sideInitialPoses(yawDeg: yaw, pitchDeg: pitch, rollDeg: roll, shift: shift)
            let (poses, _) = try Self.register(views, initial: initial, params: params)
            for i in 3 ..< poses.count {
                // Either registered correctly or left unregistered (initial pose; the registration check drops it).
                let error = Self.headError(poses[i], views[i].worldFromCamera)
                let unchanged = Self.headError(poses[i], initial[i]) < 1e-5
                #expect(error < 0.001 || unchanged, "\(name): side frame \(i) accepted \(error * 1000) mm off")
            }
        }
    }

    @Test func frameOverlapOfTurnedAndFrontalView() throws {
        // fr_frame_overlap / fr_overlap_fits as the registration check calls them (stride 2, 1.5 mm voxels, 3 mm),
        // left view (45°) against the frontal one.
        var left = Self.views[7].coreFrame(useColor: false), front = Self.views[0].coreFrame(useColor: false)
        func overlap(_ frontFromLeft: Rigid, stride: Int32 = 2) throws -> fr_overlap_result {
            var cloudParams = fr_cloud_params()
            fr_cloud_params_default(&cloudParams)
            cloudParams.stride = stride
            var out = fr_overlap_result()
            let m = frontFromLeft.floats
            let status = m.withUnsafeBufferPointer { fr_frame_overlap(&left, &front, $0.baseAddress, &cloudParams, 0.0015, 0.003, &out) }
            try #require(status == FR_OK)
            return out
        }
        let truth = Self.views[0].worldFromCamera.inverse * Self.views[7].worldFromCamera
        var correct = try overlap(truth)
        #expect(min(correct.fitness.0, correct.fitness.1) > 0.95 && max(correct.free_space_ratio.0, correct.free_space_ratio.1) < 0.02
                && min(correct.inlier_area_m2.0, correct.inlier_area_m2.1) > 0.003 && fr_overlap_fits(&correct) == 1, "\(correct)")
        // Stride 4 covers the same surface: the lookup window grows with the stride, since the view map holds every
        // 4th pixel. A fixed 3 x 3 window found 63-75 % of the stride-2 inlier area here and failed the true pose at
        // stride 8 (1 of 11 visible points in free space, 0.09).
        var coarse = try overlap(truth, stride: 4)
        #expect(fr_overlap_fits(&coarse) == 1 && min(coarse.fitness.0, coarse.fitness.1) > 0.95
                && coarse.inlier_area_m2.0 > 0.85 * correct.inlier_area_m2.0 && coarse.inlier_area_m2.1 > 0.85 * correct.inlier_area_m2.1,
                "\(coarse)")
        // 5 mm toward the frontal camera: the left view's surface lies in space the frontal camera saw through.
        var wrong = try overlap(Rigid(r: Rigid.identity.r, t: V3(0, 0, -0.005)) * truth)
        #expect(wrong.free_space_ratio.0 > 0.2 && min(wrong.fitness.0, wrong.fitness.1) < 0.5 && fr_overlap_fits(&wrong) == 0, "\(wrong)")

        var out = fr_overlap_result()
        // Default cloud parameters (NULL), no voxels.
        #expect(fr_frame_overlap(&left, &front, truth.floats, nil, 0, 0.003, &out) == FR_OK && out.fitness.0 > 0.9)
        // An empty cloud (nothing within the depth range): zeros, FR_OK.
        var nearOnly = fr_cloud_params()
        fr_cloud_params_default(&nearOnly)
        nearOnly.max_depth_m = 0.3
        out.fitness.0 = 1
        #expect(fr_frame_overlap(&left, &front, truth.floats, &nearOnly, 0.0015, 0.003, &out) == FR_OK
                && out.fitness == (0, 0) && out.expected == (0, 0) && out.inlier_area_m2 == (0, 0))
        // Invalid arguments.
        #expect(fr_frame_overlap(&left, nil, truth.floats, nil, 0, 0.003, &out) == FR_ERR_INVALID_ARGUMENT)
        for (voxel, distance) in [(Float(0), Float(0)), (-0.001, 0.003), (.nan, 0.003), (.infinity, 0.003), (0, .nan), (0, .infinity)] {
            #expect(fr_frame_overlap(&left, &front, truth.floats, nil, voxel, distance, &out) == FR_ERR_INVALID_ARGUMENT,
                    "voxel \(voxel), distance \(distance)")
        }
        var scaled = truth.floats
        for k in [0, 1, 2, 4, 5, 6, 8, 9, 10] { scaled[k] *= 1.01 }   // rotation scaled by 1 %
        var mirrored = truth.floats
        for k in [0, 1, 2] { mirrored[k] = -mirrored[k] }   // first column negated: a reflection
        var infinite = truth.floats
        infinite[12] = .infinity
        for m in [scaled, mirrored, infinite] {
            #expect(fr_frame_overlap(&left, &front, m, nil, 0, 0.003, &out) == FR_ERR_INVALID_ARGUMENT)
        }
        #expect(fr_overlap_fits(nil) == 0)
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
