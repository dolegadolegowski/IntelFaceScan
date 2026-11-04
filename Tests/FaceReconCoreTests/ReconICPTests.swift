// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Foundation
import Testing

@Suite("Recon ICP")
struct ReconICPTests {
    private func icpParams(colored: Bool, voxel: Float, maxCorrespondence: Float) -> fr_icp_params {
        var p = fr_icp_params()
        fr_icp_params_default(&p)
        p.colored = colored ? 1 : 0
        p.voxel_size_m = voxel
        p.max_correspondence_m = maxCorrespondence
        p.max_iterations = 60
        p.convergence_rotation_deg = 0.0005
        p.convergence_translation_m = 1e-7
        return p
    }

    @Test func pointToPlaneRecoversKnownTransformOnBumpySphere() throws {
        let center = V3(0, 0, 0.5)
        let truth = Rigid(axis: V3(1, 2, 3), angleDeg: 6, t: V3(0.004, -0.003, 0.002))   // target_from_source
        // Two different samplings of the same surface, so the result is not a trivial point-to-point identity.
        let target = bumpySpherePoints(count: 80000, radius: 0.1, center: center, jitterSeed: 1)
        let source = bumpySpherePoints(count: 60000, radius: 0.1, center: center, jitterSeed: 2).map { truth.inverse.apply($0) }
        let s = try #require(CoreCloud.make(points: source))
        let t = try #require(CoreCloud.make(points: target))

        var params = icpParams(colored: false, voxel: 0.0015, maxCorrespondence: 0.005)
        var result = fr_icp_result()
        #expect(fr_icp(s.pointer, t.pointer, Rigid.identity.floats, &params, &result) == FR_OK, "\(String(cString: fr_last_error()))")
        let estimate = Rigid(floats: withUnsafeBytes(of: result.target_from_source) { Array($0.bindMemory(to: Float.self)) })
        let err = Rigid.error(estimate, truth)
        #expect(err.rotationDeg < 0.1, "rotation error \(err.rotationDeg) deg")
        #expect(err.translation < 0.0001, "translation error \(err.translation * 1000) mm")
        #expect(result.fitness > 0.9)
        #expect(result.inlier_rmse_m < 0.0005)
        #expect(result.converged == 1)
    }

    @Test func coloredICPResolvesInPlaneShiftOnTexturedPlane() throws {
        // A flat plane facing the camera: geometry constrains only depth and two tilts, the texture fixes the rest.
        func intensity(_ x: Double, _ y: Double) -> UInt8 {
            UInt8(128 + 100 * sin(2 * Double.pi * x / 0.012) * cos(2 * Double.pi * y / 0.015))
        }
        let shift = V3(0.003, -0.002, 0)   // target_from_source translation, in the plane
        var tp: [V3] = [], tc: [(UInt8, UInt8, UInt8)] = []
        var sp: [V3] = [], sc: [(UInt8, UInt8, UInt8)] = []
        let n = 280
        let spacing = 0.0005
        for j in 0 ..< n {
            for i in 0 ..< n {
                let x = (Double(i) - Double(n) / 2) * spacing, y = (Double(j) - Double(n) / 2) * spacing
                let v = intensity(x, y)
                tp.append(V3(x, y, 0.6)); tc.append((v, v, v))
                // Source: a different sampling (offset grid); its texture moves with the surface.
                let xs = x + spacing / 2, ys = y + spacing / 3
                let vs = intensity(xs, ys)
                sp.append(V3(xs, ys, 0.6) - shift); sc.append((vs, vs, vs))
            }
        }
        let normals = [V3](repeating: V3(0, 0, -1), count: tp.count)
        let s = try #require(CoreCloud.make(points: sp, normals: normals, colors: sc))
        let t = try #require(CoreCloud.make(points: tp, normals: normals, colors: tc))
        let truth = Rigid(axis: V3(0, 0, 1), angleDeg: 0, t: shift)

        var geometric = icpParams(colored: false, voxel: 0.001, maxCorrespondence: 0.004)
        var r1 = fr_icp_result()
        #expect(fr_icp(s.pointer, t.pointer, Rigid.identity.floats, &geometric, &r1) == FR_OK)
        let e1 = Rigid.error(Rigid(floats: withUnsafeBytes(of: r1.target_from_source) { Array($0.bindMemory(to: Float.self)) }), truth)
        #expect(e1.translation > 0.002, "point-to-plane cannot see the in-plane shift (error \(e1.translation * 1000) mm)")

        var colored = icpParams(colored: true, voxel: 0.001, maxCorrespondence: 0.004)
        var r2 = fr_icp_result()
        #expect(fr_icp(s.pointer, t.pointer, Rigid.identity.floats, &colored, &r2) == FR_OK, "\(String(cString: fr_last_error()))")
        let e2 = Rigid.error(Rigid(floats: withUnsafeBytes(of: r2.target_from_source) { Array($0.bindMemory(to: Float.self)) }), truth)
        #expect(e2.translation < 0.0002, "colored ICP translation error \(e2.translation * 1000) mm")
        #expect(e2.rotationDeg < 0.1, "colored ICP rotation error \(e2.rotationDeg) deg")
    }

    /// Head geometry with sharp-edged dark spots (brows, pupils, freckles): where the linear intensity model of colored
    /// ICP fails.
    struct SpottedHeadScene: SynthScene {
        var head = HeadScene()
        func intersect(origin: V3, dir: V3) -> SurfaceHit? { head.intersect(origin: origin, dir: dir) }
        func color(at p: V3) -> (UInt8, UInt8, UInt8) {
            let d = p - head.center
            let s = sin(310 * d.x + 2 * sin(90 * d.y)) * sin(270 * d.y + 40 * d.z) * sin(230 * d.z + 20 * d.x)
            let spot = 0.5 + 0.5 * tanh((s - 0.3) * 25)   // ~0.3 mm transition
            let skin = 0.75 + 0.1 * sin(60 * d.x) * sin(50 * d.y)
            let v = skin * (1 - 0.7 * spot)
            return (UInt8(200 * v), UInt8(160 * v), UInt8(135 * v))   // no clipping after a 20% gain
        }
    }

    @Test func coloredICPStaysAtTheTruthDespiteTextureEdgesAndExposure() throws {
        // Two views 20 degrees apart, the source 20% brighter (auto exposure). Started at the true pose, colored ICP
        // must stay there. Without a robust photometric kernel the correspondences on spot edges, with residuals
        // 10-50x the noise, dominated the normal equations, and the additive intensity offset could not absorb the
        // gain: 2.2-2.4 degrees / 1.2-1.6 mm off. Now 0.17 degrees / 0.05 mm (point-to-plane: 0.03 degrees / 0.05 mm).
        let scene = SpottedHeadScene()
        let camera = SynthCamera.d455(width: 640, height: 400)
        let a = 20 * Double.pi / 180
        let poses = lookAtPoses(target: scene.head.center, distance: 0.6, directions: [V3(0, 0, -1), V3(sin(a), 0, -cos(a))])
        let target = render(scene, camera: camera, worldFromCamera: poses[0], noiseAt1m: 0.002, seed: 1)
        let source = render(scene, camera: camera, worldFromCamera: poses[1], noiseAt1m: 0.002, seed: 2)
        for i in 0 ..< source.rgb.count { source.rgb[i] = UInt8(min(255, (Double(source.rgb[i]) * 1.2).rounded())) }
        func cloud(_ f: RenderedFrame) throws -> CoreCloud {
            var frame = f.coreFrame()
            var p = fr_cloud_params()
            fr_cloud_params_default(&p)
            var out: OpaquePointer?
            try #require(fr_cloud_from_rgbd(&frame, &p, &out) == FR_OK)
            return CoreCloud(try #require(out))
        }
        let s = try cloud(source), t = try cloud(target)
        let truth = poses[0].inverse * poses[1]   // target_from_source
        let headInSource = poses[1].inverse.apply(scene.head.center)
        for start in [truth, Rigid(axis: V3(1, -2, 1), angleDeg: 2, t: V3(0.002, -0.001, 0.002)) * truth] {
            var params = fr_icp_params()
            fr_icp_params_default(&params)   // colored, 3 levels, 1 mm voxels
            var result = fr_icp_result()
            try #require(fr_icp(s.pointer, t.pointer, start.floats, &params, &result) == FR_OK, "\(String(cString: fr_last_error()))")
            let estimate = Rigid(floats: withUnsafeBytes(of: result.target_from_source) { Array($0.bindMemory(to: Float.self)) })
            let rotation = Rigid.error(estimate, truth).rotationDeg
            let displacement = (estimate.apply(headInSource) - truth.apply(headInSource)).length
            #expect(rotation < 0.3, "rotation error \(rotation) deg")
            #expect(displacement < 0.0003, "head displacement \(displacement * 1000) mm")
        }
    }

    @Test func zeroIterationsEvaluatesTheInitialAlignment() throws {
        let pts = bumpySpherePoints(count: 20000, radius: 0.08, center: V3(0, 0, 0.5), jitterSeed: 3)
        let c = try #require(CoreCloud.make(points: pts))
        var params = icpParams(colored: false, voxel: 0, maxCorrespondence: 0.002)
        params.max_iterations = 0
        var result = fr_icp_result()
        #expect(fr_icp(c.pointer, c.pointer, Rigid.identity.floats, &params, &result) == FR_OK)
        #expect(result.fitness == 1)
        #expect(result.inlier_rmse_m < 1e-6)
        #expect(result.iterations == 0)
        let info = withUnsafeBytes(of: result.information) { Array($0.bindMemory(to: Double.self)) }
        #expect(info[35] == Double(result.correspondences))   // (5,5) = correspondence count
    }

    @Test func icpRejectsInvalidInput() throws {
        let pts = bumpySpherePoints(count: 1000, radius: 0.08, center: V3(0, 0, 0.5), jitterSeed: 4)
        let c = try #require(CoreCloud.make(points: pts))
        var params = icpParams(colored: false, voxel: 0.001, maxCorrespondence: 0)
        var result = fr_icp_result()
        #expect(fr_icp(c.pointer, c.pointer, nil, &params, &result) == FR_ERR_INVALID_ARGUMENT)
        #expect(!String(cString: fr_last_error()).isEmpty)
        params.max_correspondence_m = 0.002
        let empty = try #require(CoreCloud.make(points: []))
        #expect(fr_icp(empty.pointer, c.pointer, nil, &params, &result) == FR_ERR_NOT_ENOUGH_DATA)
    }
}

@Suite("Recon pose graph")
struct ReconPoseGraphTests {
    /// Open3D-style ICP information (sum of G^T G, G = [-[p]x | I]) for n correspondences on a head-sized sphere
    /// 0.6 m in front of the target camera.
    private func information(correspondences n: Double) -> [Double] {
        let samples = bumpySpherePoints(count: 400, radius: 0.08, center: V3(0, 0, 0.6), jitterSeed: 11)
        var m = [Double](repeating: 0, count: 36)
        let scale = n / Double(samples.count)
        for p in samples {
            // Rows of G for x, y, z.
            let rows: [[Double]] = [[0, p.z, -p.y, 1, 0, 0], [-p.z, 0, p.x, 0, 1, 0], [p.y, -p.x, 0, 0, 0, 1]]
            for row in rows {
                for i in 0 ..< 6 { for j in 0 ..< 6 { m[j * 6 + i] += scale * row[i] * row[j] } }
            }
        }
        return m
    }

    @Test func poseGraphReducesDriftAndPrunesWrongLoopClosure() throws {
        var rng = SplitMix64(seed: 7)
        let n = 12
        // Ground truth: views around a head, rotating about the vertical axis.
        let truth: [Rigid] = (0 ..< n).map { i in
            let rot = Rigid(axis: V3(0, 1, 0), angleDeg: 25 * Double(i))
            let pivot = V3(0, 0, 0.6)
            return Rigid(r: rot.r, t: pivot - rot.rotate(pivot))
        }
        // ICP errors: small rotations about the head (0.6 m in front of the target camera) plus a translation.
        func noisy(_ z: Rigid, rotDeg: Double, trans: Double) -> Rigid {
            let axis = V3(rng.gaussian(), rng.gaussian(), rng.gaussian())
            let rot = Rigid(axis: axis, angleDeg: rotDeg * rng.gaussian())
            let head = V3(0, 0, 0.6)
            let d = Rigid(r: rot.r, t: head - rot.rotate(head) + V3(rng.gaussian(), rng.gaussian(), rng.gaussian()) * trans)
            return d * z
        }
        var graph: OpaquePointer?
        #expect(fr_pose_graph_create(&graph) == FR_OK)
        let g = try #require(graph)
        defer { fr_pose_graph_free(g) }

        // Odometry chain (each edge source i -> target i-1) accumulates drift.
        var odometry: [Rigid] = []
        var chain = [Rigid.identity]
        for i in 1 ..< n {
            let z = noisy(truth[i - 1].inverse * truth[i], rotDeg: 0.1, trans: 0.00015)
            odometry.append(z)
            chain.append(chain[i - 1] * z)
        }
        for i in 0 ..< n { _ = fr_pose_graph_add_node(g, chain[i].floats) }
        let info = information(correspondences: 20000)
        for i in 1 ..< n {
            #expect(fr_pose_graph_add_edge(g, UInt32(i), UInt32(i - 1), odometry[i - 1].floats, info, 0) == FR_OK)
        }
        // Accurate loop closures, and one that is 5 cm off.
        for (s, t) in [(11, 0), (6, 0), (9, 3)] {
            let z = noisy(truth[t].inverse * truth[s], rotDeg: 0.02, trans: 0.00003)
            #expect(fr_pose_graph_add_edge(g, UInt32(s), UInt32(t), z.floats, info, 1) == FR_OK)
        }
        let wrong = Rigid(axis: V3(0, 1, 0), angleDeg: 0, t: V3(0.05, 0, 0)) * (truth[2].inverse * truth[8])
        #expect(fr_pose_graph_add_edge(g, 8, 2, wrong.floats, info, 1) == FR_OK)

        var params = fr_pose_graph_params()
        fr_pose_graph_params_default(&params)
        var report = fr_pose_graph_report()
        #expect(fr_pose_graph_optimize(g, &params, &report) == FR_OK, "\(String(cString: fr_last_error()))")
        #expect(report.pruned_edges == 1)
        #expect(report.final_error < report.initial_error)

        // Misalignment of head points (what fusion sees), not of the camera centers 0.6 m away.
        let headPoints = bumpySpherePoints(count: 64, radius: 0.09, center: V3(0, 0, 0.6), jitterSeed: 5)
        func headError(_ a: Rigid, _ b: Rigid) -> Double {
            headPoints.map { (a.apply($0) - b.apply($0)).length }.max() ?? 0
        }
        var chainError = 0.0, optimizedError = 0.0
        for i in 0 ..< n {
            var m = [Float](repeating: 0, count: 16)
            #expect(fr_pose_graph_get_node(g, UInt32(i), &m) == FR_OK)
            let pose = Rigid(floats: m)
            if i == 0 { #expect(Rigid.error(pose, .identity).translation < 1e-6) }
            chainError = max(chainError, headError(chain[i], truth[i]))
            optimizedError = max(optimizedError, headError(pose, truth[i]))
        }
        #expect(optimizedError < chainError * 0.6, "optimized \(optimizedError * 1000) mm vs chain \(chainError * 1000) mm")
        #expect(optimizedError < 0.0008, "optimized \(optimizedError * 1000) mm vs chain \(chainError * 1000) mm")
    }

    @Test func exactMeasurementsRecoverTruthFromPerturbedStart() throws {
        var rng = SplitMix64(seed: 9)
        let n = 9
        let truth: [Rigid] = (0 ..< n).map { i in
            let rot = Rigid(axis: V3(0.2 * Double(i % 3), 1, 0.1), angleDeg: 20 * Double(i) - 60)
            let pivot = V3(0, 0, 0.6)
            return Rigid(r: rot.r, t: pivot - rot.rotate(pivot))
        }
        var graph: OpaquePointer?
        #expect(fr_pose_graph_create(&graph) == FR_OK)
        let g = try #require(graph)
        defer { fr_pose_graph_free(g) }
        for i in 0 ..< n {
            // Start several degrees and centimeters away from the truth (node 0 is the fixed reference).
            let d = i == 0 ? Rigid.identity
                : Rigid(axis: V3(rng.gaussian(), rng.gaussian(), rng.gaussian()), angleDeg: 6,
                        t: V3(rng.gaussian(), rng.gaussian(), rng.gaussian()) * 0.02)
            _ = fr_pose_graph_add_node(g, (truth[i] * d).floats)
        }
        let info = information(correspondences: 5000)
        for i in 1 ..< n {
            #expect(fr_pose_graph_add_edge(g, UInt32(i), UInt32(i - 1), (truth[i - 1].inverse * truth[i]).floats, info, 0) == FR_OK)
        }
        for (s, t) in [(8, 0), (4, 0), (6, 2)] {
            #expect(fr_pose_graph_add_edge(g, UInt32(s), UInt32(t), (truth[t].inverse * truth[s]).floats, info, 1) == FR_OK)
        }
        var params = fr_pose_graph_params()
        fr_pose_graph_params_default(&params)
        var report = fr_pose_graph_report()
        #expect(fr_pose_graph_optimize(g, &params, &report) == FR_OK)
        #expect(report.pruned_edges == 0)
        for i in 0 ..< n {
            var m = [Float](repeating: 0, count: 16)
            #expect(fr_pose_graph_get_node(g, UInt32(i), &m) == FR_OK)
            let err = Rigid.error(Rigid(floats: m), truth[i])
            #expect(err.rotationDeg < 0.002 && err.translation < 2e-6, "node \(i): \(err.rotationDeg) deg, \(err.translation * 1000) mm")
        }
    }

    @Test func consistentGraphKeepsMeasurements() throws {
        var graph: OpaquePointer?
        #expect(fr_pose_graph_create(&graph) == FR_OK)
        let g = try #require(graph)
        defer { fr_pose_graph_free(g) }
        let a = Rigid.identity
        let b = Rigid(axis: V3(0, 1, 0), angleDeg: 40, t: V3(0.1, 0, 0.02))
        _ = fr_pose_graph_add_node(g, a.floats)
        _ = fr_pose_graph_add_node(g, Rigid.identity.floats)   // poor initial guess
        let info = information(correspondences: 1000)
        #expect(fr_pose_graph_add_edge(g, 1, 0, (a.inverse * b).floats, info, 0) == FR_OK)
        var params = fr_pose_graph_params()
        fr_pose_graph_params_default(&params)
        #expect(fr_pose_graph_optimize(g, &params, nil) == FR_OK)
        var m = [Float](repeating: 0, count: 16)
        #expect(fr_pose_graph_get_node(g, 1, &m) == FR_OK)
        let err = Rigid.error(Rigid(floats: m), b)
        #expect(err.rotationDeg < 0.01 && err.translation < 1e-5)
        #expect(fr_pose_graph_add_edge(g, 0, 0, a.floats, info, 0) == FR_ERR_INVALID_ARGUMENT)
        #expect(fr_pose_graph_add_edge(g, 0, 5, a.floats, info, 0) == FR_ERR_INVALID_ARGUMENT)
    }
}
