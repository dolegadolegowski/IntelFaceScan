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
        /// Frames whose translation came from landmarks (the others from the head centroid).
        var landmarkFrames: Int
    }

    /// Head rotation in the camera frame (ScanKit.HeadPose convention).
    static func rotation(_ pose: HeadPose) -> simd_float3x3 { simd_float3x3(pose.cameraFrameRotation) }

    /// Initial world_from_camera per keyframe; world = camera frame of keyframes[reference].
    static func estimate(keyframes: [Keyframe], segments: [SegmentedKeyframe], reference: Int) -> Result {
        let rRef = rotation(keyframes[reference].headPose)
        let centers = segments.map { headCenter($0) }
        let landmarks = zip(keyframes, segments).map { landmarkPoints($0.0, $0.1) }
        var poses: [simd_float4x4] = []
        var fromLandmarks = 0
        for i in keyframes.indices {
            // A head point p (world) appears in frame i at R_i R_ref^T (p - c_ref) + c_i: world_from_camera_i has
            // rotation R_ref R_i^T.
            let r = rRef * rotation(keyframes[i].headPose).transpose
            var t = centers[reference] - r * centers[i]
            if i != reference, let lt = landmarkTranslation(rotation: r, reference: landmarks[reference], frame: landmarks[i]) {
                t = lt
                fromLandmarks += 1
            }
            if i == reference { t = .zero }
            poses.append(simd_float4x4(rotation: i == reference ? matrix_identity_float3x3 : r, translation: t))
        }
        return Result(worldFromCamera: poses, landmarkFrames: fromLandmarks)
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

    /// Scores every frame's registered cloud against its most similar view; frames whose surface does not fit
    /// (failed or ambiguous registration) are marked false. Frame 0 (the world reference) always passes.
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
        var scores = Array(repeating: FrameScore(fitness: 1, rmse: 0), count: n)
        let identity = matrix_identity_float4x4.floats
        for i in 0 ..< n {
            guard fr_cloud_size(clouds[i]) >= 50 else { scores[i] = FrameScore(fitness: 0, rmse: .infinity); continue }
            // The most similar view (smallest relative rotation) overlaps most.
            var best = -1
            var bestAngle = Float.infinity
            for j in 0 ..< n where j != i && fr_cloud_size(clouds[j]) >= 50 {
                let rel = simd_quatf(poses[j].upperLeft3x3.transpose * poses[i].upperLeft3x3)
                let angle = abs(rel.angle)
                if angle < bestAngle { bestAngle = angle; best = j }
            }
            guard best >= 0 else { continue }
            var result = fr_icp_result()
            let status = identity.withUnsafeBufferPointer { fr_icp(clouds[i], clouds[best], $0.baseAddress, &icp, &result) }
            scores[i] = status == FR_OK ? FrameScore(fitness: result.fitness, rmse: result.inlier_rmse_m)
                                        : FrameScore(fitness: 0, rmse: .infinity)
        }
        let finite = scores.map(\.rmse).filter { $0.isFinite }.sorted()
        let median = finite.isEmpty ? 0 : finite[finite.count / 2]
        let limit = max(2 * median, median + 0.0007)
        let passed = scores.enumerated().map { i, s in i == 0 || (s.fitness >= 0.25 && s.rmse <= limit) }
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
