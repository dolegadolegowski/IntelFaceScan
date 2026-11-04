// License: Apache 2.0. See LICENSE file in root directory.
//
// Geometry around the whole-scan registration: initial poses from the Vision head pose (rotation) and the head
// position (landmarks with depth, or the visible-surface centroid), the head frame of the reference keyframe
// (orientation with the eye line as roll reference, chin point), registration verification, and the bounds of the
// registered clouds.

import CameraKit
import FaceReconCore
import Foundation
import ScanKit
import simd

enum InitialPoseEstimator {
    /// The visible surface of a head (radius ~9 cm) has its centroid ~r/2 in front of the head center.
    static let headCenterOffset: Float = 0.045
    static let minLandmarkPairs = 6

    struct Result {
        var worldFromCamera: [simd_float4x4]
        /// Frames whose pose came from a landmark Kabsch fit (rotation and translation).
        var kabschFrames: Int
        /// Frames with the Vision rotation and a landmark translation.
        var landmarkTranslationFrames: Int
    }

    /// Head rotation in the camera frame (ScanKit.HeadPose convention); non-finite angles count as 0.
    static func rotation(_ pose: HeadPose) -> simd_float3x3 {
        let clean = HeadPose(yaw: pose.yaw.isFinite ? pose.yaw : 0, pitch: pose.pitch.isFinite ? pose.pitch : 0,
                             roll: pose.roll.isFinite ? pose.roll : 0)
        return simd_float3x3(clean.cameraFrameRotation)
    }

    /// Initial world_from_camera per keyframe; world = camera frame of keyframes[reference].
    static func estimate(keyframes: [Keyframe], segments: [SegmentedKeyframe], reference: Int) -> Result {
        let rRef = rotation(keyframes[reference].headPose)
        let centers = segments.map { headCenter($0) }
        let landmarks = zip(keyframes, segments).map { landmarkPoints($0.0, $0.1) }
        var poses: [simd_float4x4] = []
        var kabsch = 0, translated = 0
        for i in keyframes.indices {
            if i == reference {
                poses.append(matrix_identity_float4x4)
                continue
            }
            // A head point p (world) appears in frame i at R_i R_ref^T (p - c_ref) + c_i: world_from_camera_i has
            // rotation R_ref R_i^T.
            let r = rRef * rotation(keyframes[i].headPose).transpose
            if let fit = landmarkKabsch(reference: landmarks[reference], frame: landmarks[i], visionRotation: r) {
                poses.append(fit)
                kabsch += 1
            } else if let t = landmarkTranslation(rotation: r, reference: landmarks[reference], frame: landmarks[i]) {
                poses.append(simd_float4x4(rotation: r, translation: t))
                translated += 1
            } else {
                poses.append(simd_float4x4(rotation: r, translation: centers[reference] - r * centers[i]))
            }
        }
        return Result(worldFromCamera: poses, kabschFrames: kabsch, landmarkTranslationFrames: translated)
    }

    /// Approximate head center in the camera frame: centroid of the head-mask points pushed back along the ray.
    static func headCenter(_ s: SegmentedKeyframe) -> SIMD3<Float> {
        let d = s.depth
        var sum = SIMD3<Double>.zero
        var n = 0
        let step = max(1, Int((Double(s.validPoints) / 20000).squareRoot()))
        s.headMask.withUnsafeBytes { (mask: UnsafeRawBufferPointer) in
            for y in stride(from: 0, to: d.height, by: step) {
                for x in stride(from: 0, to: d.width, by: step) where mask[y * d.width + x] != 0 {
                    let z = d.meters(x, y)
                    guard z > 0 else { continue }
                    let p = d.point(SIMD2(Float(x), Float(y)), depth: z)
                    sum += SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z))
                    n += 1
                }
            }
        }
        guard n > 0 else { return SIMD3(0, 0, s.faceDepth + headCenterOffset) }
        let c = SIMD3<Float>(sum / Double(n))
        return c + simd_normalize(c) * headCenterOffset
    }

    /// 3D landmark positions (nil where the landmark has no depth on the head).
    static func landmarkPoints(_ kf: Keyframe, _ s: SegmentedKeyframe) -> [SIMD3<Float>?] {
        let band = (s.faceDepth - HeadSegmentation.bandInFront) ... (s.faceDepth + HeadSegmentation.bandBehind)
        return kf.landmarks.map { l in
            guard let z = s.depth.median(around: l, radius: 2, band: band) else { return nil }
            return s.depth.point(l, depth: z)
        }
    }

    /// Rigid fit of the frame's 3D landmarks onto the reference's (Horn's closed form, iteratively trimmed against
    /// landmarks Vision places on occluded parts of a turned face). Accepted only when well supported, spread in 3D,
    /// and within 12 degrees of the Vision rotation; Vision is noisier, but never wildly wrong.
    static func landmarkKabsch(reference: [SIMD3<Float>?], frame: [SIMD3<Float>?], visionRotation: simd_float3x3) -> simd_float4x4? {
        guard reference.count == frame.count else { return nil }
        var pairs: [(SIMD3<Float>, SIMD3<Float>)] = []
        for (a, b) in zip(reference, frame) {
            if let a, let b { pairs.append((a, b)) }
        }
        guard pairs.count >= 8 else { return nil }
        var inliers = pairs
        var pose = matrix_identity_float4x4
        var rms: Float = .infinity
        for _ in 0 ..< 5 {
            guard inliers.count >= 8, let fit = horn(inliers) else { return nil }
            pose = fit
            let residuals = pairs.map { simd_length($0.0 - transform(fit, $0.1)) }
            let median = residuals.sorted()[residuals.count / 2]
            let limit = max(3 * median, 0.004)
            inliers = zip(pairs, residuals).filter { $0.1 < limit }.map(\.0)
            let used = residuals.filter { $0 < limit }
            rms = used.isEmpty ? .infinity : (used.map { $0 * $0 }.reduce(0, +) / Float(used.count)).squareRoot()
        }
        guard inliers.count >= 8, rms < 0.004 else { return nil }
        // Spread: landmarks along a line (or on a tiny patch) do not fix the rotation. Require a covariance whose
        // geometric-mean standard deviation exceeds 6 mm.
        let mean = inliers.map(\.1).reduce(.zero, +) / Float(inliers.count)
        var cov = simd_float3x3()
        for (_, b) in inliers {
            let d = b - mean
            cov += simd_float3x3(columns: (d * d.x, d * d.y, d * d.z))
        }
        cov *= 1 / Float(inliers.count)
        let det = cov.determinant
        guard det.isFinite, det > 0, pow(det, 1.0 / 6.0) > 0.006 else { return nil }
        let rotationError = simd_quatf(visionRotation.transpose * pose.upperLeft3x3).angle
        guard min(rotationError, 2 * .pi - rotationError) < 12 * .pi / 180 else { return nil }
        return pose
    }

    static func transform(_ m: simd_float4x4, _ p: SIMD3<Float>) -> SIMD3<Float> {
        let q = m * SIMD4(p, 1)
        return SIMD3(q.x, q.y, q.z)
    }

    /// Least-squares rigid transform mapping pairs.1 onto pairs.0 (Horn 1987: the rotation quaternion is the
    /// dominant eigenvector of a symmetric 4x4 matrix built from the cross-covariance).
    static func horn(_ pairs: [(SIMD3<Float>, SIMD3<Float>)]) -> simd_float4x4? {
        let n = Double(pairs.count)
        var ca = SIMD3<Double>.zero, cb = SIMD3<Double>.zero
        for (a, b) in pairs {
            ca += SIMD3<Double>(a)
            cb += SIMD3<Double>(b)
        }
        ca /= n
        cb /= n
        var s = [[Double]](repeating: [0, 0, 0], count: 3)   // s[i][j] = sum of b_i a_j (centered)
        for (a, b) in pairs {
            let da = SIMD3<Double>(a) - ca, db = SIMD3<Double>(b) - cb
            for i in 0 ..< 3 { for j in 0 ..< 3 { s[i][j] += db[i] * da[j] } }
        }
        let (sxx, sxy, sxz) = (s[0][0], s[0][1], s[0][2])
        let (syx, syy, syz) = (s[1][0], s[1][1], s[1][2])
        let (szx, szy, szz) = (s[2][0], s[2][1], s[2][2])
        var m: [[Double]] = [
            [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
            [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
            [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
            [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
        ]
        guard let q = dominantEigenvector(&m) else { return nil }
        let quat = simd_quatd(ix: q[1], iy: q[2], iz: q[3], r: q[0]).normalized
        let r = simd_double3x3(quat)
        let t = ca - r * cb
        let rf = simd_float3x3(SIMD3<Float>(r.columns.0), SIMD3<Float>(r.columns.1), SIMD3<Float>(r.columns.2))
        return simd_float4x4(rotation: rf, translation: SIMD3<Float>(t))
    }

    /// Eigenvector of the largest eigenvalue of a symmetric 4x4 matrix (cyclic Jacobi rotations).
    static func dominantEigenvector(_ a: inout [[Double]]) -> [Double]? {
        var v: [[Double]] = (0 ..< 4).map { i in (0 ..< 4).map { $0 == i ? 1 : 0 } }
        for _ in 0 ..< 50 {
            var off = 0.0
            for p in 0 ..< 4 { for q in (p + 1) ..< 4 { off += a[p][q] * a[p][q] } }
            if off < 1e-24 { break }
            for p in 0 ..< 4 {
                for q in (p + 1) ..< 4 where abs(a[p][q]) > 1e-300 {
                    let theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
                    let t = (theta >= 0 ? 1.0 : -1.0) / (abs(theta) + (theta * theta + 1).squareRoot())
                    let c = 1 / (t * t + 1).squareRoot(), sn = t * c
                    for k in 0 ..< 4 {
                        let akp = a[k][p], akq = a[k][q]
                        a[k][p] = c * akp - sn * akq
                        a[k][q] = sn * akp + c * akq
                    }
                    for k in 0 ..< 4 {
                        let apk = a[p][k], aqk = a[q][k]
                        a[p][k] = c * apk - sn * aqk
                        a[q][k] = sn * apk + c * aqk
                    }
                    for k in 0 ..< 4 {
                        let vkp = v[k][p], vkq = v[k][q]
                        v[k][p] = c * vkp - sn * vkq
                        v[k][q] = sn * vkp + c * vkq
                    }
                }
            }
        }
        var best = 0
        for i in 1 ..< 4 where a[i][i] > a[best][best] { best = i }
        let vec = (0 ..< 4).map { v[$0][best] }
        return vec.allSatisfy { $0.isFinite } ? vec : nil
    }

    /// Least-squares translation for a fixed rotation over corresponding landmarks, robust to landmarks Vision
    /// places on occluded parts of a turned face.
    static func landmarkTranslation(rotation r: simd_float3x3, reference: [SIMD3<Float>?], frame: [SIMD3<Float>?]) -> SIMD3<Float>? {
        guard reference.count == frame.count else { return nil }
        var candidates: [SIMD3<Float>] = []
        for (a, b) in zip(reference, frame) {
            if let a, let b { candidates.append(a - r * b) }
        }
        guard candidates.count >= minLandmarkPairs else { return nil }
        func median(_ v: [Float]) -> Float { let s = v.sorted(); return s[s.count / 2] }
        let med = SIMD3(median(candidates.map(\.x)), median(candidates.map(\.y)), median(candidates.map(\.z)))
        let inliers = candidates.filter { simd_length($0 - med) < 0.015 }
        guard inliers.count >= minLandmarkPairs else { return nil }
        return inliers.reduce(.zero, +) / Float(inliers.count)
    }
}

/// Orientation and chin of the head in the world (= camera frame of the reference frontal keyframe).
struct HeadFrame {
    /// Head rotation in the world: columns are the head's axes (x toward the person's left, y down, z from the face
    /// toward the back of the head). Yaw and pitch: mean of all keyframes' Vision head poses carried into the world;
    /// roll: the reference keyframe's eye line when landmarks allow.
    var rotation: simd_float3x3
    /// Point under the chin (world), nil when no depth was found there.
    var chin: SIMD3<Float>?

    var up: SIMD3<Float> { -rotation.columns.1 }

    /// - Parameters:
    ///   - kf, s: the reference (frontal) keyframe, whose camera frame is the world.
    ///   - keyframes, poses: all registered keyframes with their world_from_camera poses. Each one's Vision head
    ///     rotation, carried into the world, is an independent estimate of the head orientation; their robust mean
    ///     is far less noisy than the reference keyframe's alone.
    init(reference kf: Keyframe, segment s: SegmentedKeyframe, keyframes: [Keyframe], poses: [simd_float4x4]) {
        var estimates: [simd_quatf] = []
        for (k, pose) in zip(keyframes, poses) {
            let p = k.headPose
            guard p.yaw.isFinite, p.pitch.isFinite, p.roll.isFinite else { continue }
            estimates.append(simd_quatf(pose.upperLeft3x3 * InitialPoseEstimator.rotation(p)))
        }
        let mean = HeadFrame.robustMean(estimates) ?? kf.headPose.cameraFrameRotation
        // Yaw and pitch from where the mean face points; roll from the reference eye line when available.
        let f = mean.act(SIMD3<Float>(0, 0, -1))
        let yaw = atan2(f.x, -f.z), pitch = asin(min(1, max(-1, -f.y)))
        let toDegrees: Float = 180 / .pi
        var roll: Float
        if s.rollFromLandmarks {
            roll = s.rollRadians
        } else {
            let yawPitch = InitialPoseEstimator.rotation(HeadPose(yaw: yaw * toDegrees, pitch: pitch * toDegrees, roll: 0))
            let rz = yawPitch.transpose * simd_float3x3(mean)
            roll = atan2(rz.columns.0.y, rz.columns.0.x)
        }
        rotation = InitialPoseEstimator.rotation(HeadPose(yaw: yaw * toDegrees, pitch: pitch * toDegrees, roll: roll * toDegrees))
        let rect = HeadSegmentation.clampedFaceRect(kf.faceRect, width: s.depth.width, height: s.depth.height)
        let down = SIMD2<Float>(-sin(s.rollRadians), cos(s.rollRadians))
        let chinPixel = HeadSegmentation.chinPixel(landmarks: kf.landmarks, faceRect: rect, down: down)
        let band = (s.faceDepth - HeadSegmentation.bandInFront) ... (s.faceDepth + HeadSegmentation.bandBehind)
        // The chin landmark sits on the silhouette; walk up the face until there is depth.
        chin = nil
        let steps = max(1, Int(rect.height * 0.15))
        for k in 0 ... steps {
            let p = chinPixel - down * Float(k)
            if let z = s.depth.median(around: p, radius: 2, band: band) {
                chin = s.depth.point(p, depth: z)
                break
            }
        }
    }

    /// Mean of rotations near each other, ignoring those more than 10 degrees from the first estimate of the mean.
    static func robustMean(_ rotations: [simd_quatf]) -> simd_quatf? {
        guard let first = rotations.first else { return nil }
        func average(_ qs: [simd_quatf], reference: simd_quatf) -> simd_quatf {
            var sum = SIMD4<Float>.zero
            for q in qs { sum += simd_dot(q.vector, reference.vector) < 0 ? -q.vector : q.vector }
            return simd_quatf(vector: simd_normalize(sum))
        }
        var mean = average(rotations, reference: first)
        let inliers = rotations.filter { abs((mean.inverse * $0).angle.remainder(dividingBy: 2 * .pi)) < 10 * .pi / 180 }
        if !inliers.isEmpty { mean = average(inliers, reference: mean) }
        return mean
    }

    /// World -> export frame rotation: head upright (+Y up), face toward +Z (a 180 degree turn about X of the
    /// head's camera-convention axes).
    var exportRotation: simd_float3x3 {
        simd_float3x3(diagonal: SIMD3(1, -1, -1)) * rotation.transpose
    }
}

enum RegistrationCheck {
    struct FrameScore {
        var fitness: Float
        var rmse: Float
    }

    /// Views closer than this (relative rotation) form one pose group.
    static let groupAngle: Float = 12 * .pi / 180
    /// Views farther apart than this overlap too little to be compared.
    static let maxPairAngle: Float = 60 * .pi / 180
    static let minFitness: Float = 0.25

    /// Scores every frame's registered cloud against the most similar view of its own pose group and of every other
    /// pose group within maxPairAngle. A pair fits when the surfaces overlap (fitness) with an RMSE close to the
    /// typical pair's. A frame passes when a chain of fitting pairs links it to frame 0 (the world reference, which
    /// always passes). Judging each frame by its single most similar view failed both ways: a misregistered pose
    /// group fits itself (its frames are each other's most similar views) and passed, while a correct frame whose
    /// most similar view was the misregistered one was dropped. The comparisons across pose groups are what link a
    /// group to the reference.
    static func verify(frames: RGBDFrameSet, poses: [simd_float4x4]) throws -> (passed: [Bool], scores: [FrameScore]) {
        let n = frames.count
        guard n > 1 else { return (Array(repeating: true, count: n), Array(repeating: FrameScore(fitness: 1, rmse: 0), count: n)) }
        var cloudParams = fr_cloud_params()
        fr_cloud_params_default(&cloudParams)
        cloudParams.stride = 2
        var clouds: [OpaquePointer] = []
        defer { clouds.forEach { fr_cloud_free($0) } }
        for k in 0 ..< n {
            var frame = frames.frames[k]
            var raw: OpaquePointer?
            try checkCore(fr_cloud_from_rgbd(&frame, &cloudParams, &raw), "registration check")
            guard let cloud = raw else { throw ReconstructionError.core(stage: "registration check", status: -1, message: "no cloud") }
            defer { fr_cloud_free(cloud) }
            var down: OpaquePointer?
            try checkCore(fr_cloud_voxel_downsample(cloud, 0.0015, &down), "registration check")
            guard let d = down else { throw ReconstructionError.core(stage: "registration check", status: -1, message: "no cloud") }
            clouds.append(d)
            try poses[k].floats.withUnsafeBufferPointer { try checkCore(fr_cloud_transform(d, $0.baseAddress), "registration check") }
        }
        var icp = fr_icp_params()
        fr_icp_params_default(&icp)
        icp.levels = 1
        icp.voxel_size_m = 0
        icp.max_iterations = 0
        icp.colored = 0
        icp.max_correspondence_m = 0.003
        let identity = matrix_identity_float4x4.floats
        let enough = (0 ..< n).map { fr_cloud_size(clouds[$0]) >= 50 }
        func angle(_ i: Int, _ j: Int) -> Float {
            let a = simd_quatf(poses[j].upperLeft3x3.transpose * poses[i].upperLeft3x3).angle
            return min(a, 2 * .pi - a)
        }
        // Pose groups (greedy, by the rotation relative to each group's first frame).
        var group = [Int](repeating: -1, count: n)
        var groupHeads: [Int] = []
        for i in 0 ..< n where enough[i] {
            if let g = groupHeads.firstIndex(where: { angle($0, i) < groupAngle }) {
                group[i] = g
            } else {
                group[i] = groupHeads.count
                groupHeads.append(i)
            }
        }
        // Pairs: each frame with the most similar view of every pose group in reach (unordered, evaluated once).
        var pairs: [(Int, Int)] = []
        var seen = Set<Int>()
        for i in 0 ..< n where enough[i] {
            for g in groupHeads.indices {
                let candidates = (0 ..< n).filter { $0 != i && group[$0] == g }
                guard let j = candidates.min(by: { angle(i, $0) < angle(i, $1) }), angle(i, j) <= maxPairAngle else { continue }
                if seen.insert(min(i, j) * n + max(i, j)).inserted { pairs.append((i, j)) }
            }
        }
        var pairScores: [FrameScore] = []
        for (i, j) in pairs {
            var result = fr_icp_result()
            let status = identity.withUnsafeBufferPointer { fr_icp(clouds[i], clouds[j], $0.baseAddress, &icp, &result) }
            pairScores.append(status == FR_OK ? FrameScore(fitness: result.fitness, rmse: result.inlier_rmse_m)
                                              : FrameScore(fitness: 0, rmse: .infinity))
        }
        let finite = pairScores.filter { $0.fitness >= minFitness }.map(\.rmse).filter { $0.isFinite }.sorted()
        let median = finite.isEmpty ? 0 : finite[finite.count / 2]
        let limit = max(2 * median, median + 0.0007)
        // Fitting pairs are links (either direction); frames linked to frame 0 pass. Each frame reports its best pair.
        var links = [[Int]](repeating: [], count: n)
        var scores = Array(repeating: FrameScore(fitness: 0, rmse: .infinity), count: n)
        for ((i, j), s) in zip(pairs, pairScores) {
            if s.fitness >= minFitness && s.rmse <= limit {
                links[i].append(j)
                links[j].append(i)
            }
            for k in [i, j] where s.rmse < scores[k].rmse || !scores[k].rmse.isFinite && s.fitness > scores[k].fitness {
                scores[k] = s
            }
        }
        var passed = [Bool](repeating: false, count: n)
        passed[0] = true
        var stack = [0]
        while let k = stack.popLast() {
            for j in links[k] where !passed[j] {
                passed[j] = true
                stack.append(j)
            }
        }
        return (passed, scores)
    }

    /// Axis-aligned bounds of the registered frames (robust percentiles) plus a margin, in the world frame.
    static func bounds(frames: RGBDFrameSet, poses: [simd_float4x4], margin: Float) throws -> (min: SIMD3<Float>, max: SIMD3<Float>) {
        var cloudParams = fr_cloud_params()
        fr_cloud_params_default(&cloudParams)
        cloudParams.stride = 4
        var xs: [Float] = [], ys: [Float] = [], zs: [Float] = []
        for k in 0 ..< frames.count {
            var frame = frames.frames[k]
            var raw: OpaquePointer?
            try checkCore(fr_cloud_from_rgbd(&frame, &cloudParams, &raw), "bounds")
            guard let cloud = raw else { continue }
            defer { fr_cloud_free(cloud) }
            try poses[k].floats.withUnsafeBufferPointer { try checkCore(fr_cloud_transform(cloud, $0.baseAddress), "bounds") }
            let count = Int(fr_cloud_size(cloud))
            guard count > 0, let p = fr_cloud_points(cloud) else { continue }
            for i in 0 ..< count {
                xs.append(p[3 * i]); ys.append(p[3 * i + 1]); zs.append(p[3 * i + 2])
            }
        }
        guard xs.count >= 100 else {
            throw ReconstructionError.core(stage: "bounds", status: Int32(bitPattern: FR_ERR_NOT_ENOUGH_DATA.rawValue),
                                           message: "too few registered points (\(xs.count))")
        }
        func range(_ v: inout [Float]) -> (Float, Float) {
            v.sort()
            let lo = v[Int(Double(v.count - 1) * 0.002)], hi = v[Int(Double(v.count - 1) * 0.998)]
            return (lo, hi)
        }
        let (x0, x1) = range(&xs), (y0, y1) = range(&ys), (z0, z1) = range(&zs)
        return (SIMD3(x0, y0, z0) - margin, SIMD3(x1, y1, z1) + margin)
    }
}

extension simd_float4x4 {
    init(rotation r: simd_float3x3, translation t: SIMD3<Float>) {
        self.init(columns: (SIMD4(r.columns.0, 0), SIMD4(r.columns.1, 0), SIMD4(r.columns.2, 0), SIMD4(t, 1)))
    }

    var upperLeft3x3: simd_float3x3 {
        simd_float3x3(SIMD3(columns.0.x, columns.0.y, columns.0.z), SIMD3(columns.1.x, columns.1.y, columns.1.z),
                      SIMD3(columns.2.x, columns.2.y, columns.2.z))
    }
}
