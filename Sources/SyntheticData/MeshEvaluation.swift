// License: Apache 2.0. See LICENSE file in root directory.
//
// Swift side of fr_eval.h (surface distances, plane fits, pose errors) and probes that tell whether the
// reconstruction core is still a placeholder (tests that need it are skipped until it is integrated).

import FaceReconCore
import Foundation
import ReconKit
import simd

/// One-sided surface distance statistics (millimeters).
public struct SurfaceDistance: Codable, Sendable, Equatable {
    public var meanMm: Double
    public var rmsMm: Double
    public var medianMm: Double
    public var p95Mm: Double
    public var maxMm: Double
    public var samples: Int

    public init(meanMm: Double, rmsMm: Double, medianMm: Double, p95Mm: Double, maxMm: Double, samples: Int) {
        self.meanMm = meanMm; self.rmsMm = rmsMm; self.medianMm = medianMm
        self.p95Mm = p95Mm; self.maxMm = maxMm; self.samples = samples
    }
}

public struct PlaneFit: Codable, Sendable, Equatable {
    /// Unit normal oriented toward the origin (the camera for camera-frame points).
    public var normal: SIMD3<Float>
    /// Plane: dot(normal, x) + d = 0.
    public var d: Float
    public var rmsM: Double
    public var inliers: Int
}

public struct PoseError: Codable, Sendable, Equatable {
    public var rotationDeg: Float
    public var translationM: Float
}

public enum MeshEvaluation {
    /// Distances from `samples` area-uniform points on `from` to the surface of `to` (both in the same frame and
    /// unit; statistics are reported in millimeters assuming meters, scaled by `unitsToMM`).
    public static func distance(from: ReconMesh, to: ReconMesh, samples: Int = 200_000,
                                unitsToMM: Double = 1000) throws -> SurfaceDistance {
        guard samples > 0, samples <= 100_000_000 else {
            throw ReconstructionError.core(stage: "evaluation", status: Int32(FR_ERR_INVALID_ARGUMENT.rawValue),
                                           message: "samples must be in 1...100000000")
        }
        let a = try from.makeCoreMesh()
        defer { fr_mesh_free(a) }
        let b = try to.makeCoreMesh()
        defer { fr_mesh_free(b) }
        var stats = fr_distance_stats()
        try check(fr_mesh_distance(a, b, UInt32(samples), &stats))
        return SurfaceDistance(meanMm: stats.mean_m * unitsToMM, rmsMm: stats.rms_m * unitsToMM,
                               medianMm: stats.median_m * unitsToMM, p95Mm: stats.p95_m * unitsToMM,
                               maxMm: stats.max_m * unitsToMM, samples: Int(stats.samples))
    }

    /// Plane through points (meters); RANSAC-robust when ransacThresholdM > 0.
    public static func fitPlane(_ points: [SIMD3<Float>], ransacThresholdM: Float = 0) throws -> PlaneFit {
        var flat = [Float]()
        flat.reserveCapacity(points.count * 3)
        for p in points { flat += [p.x, p.y, p.z] }
        var fit = fr_plane_fit()
        try check(flat.withUnsafeBufferPointer { fr_fit_plane($0.baseAddress, UInt32(points.count), ransacThresholdM, &fit) })
        return PlaneFit(normal: SIMD3(fit.normal.0, fit.normal.1, fit.normal.2), d: fit.d, rmsM: fit.rms_m,
                        inliers: Int(fit.inliers))
    }

    /// Rotation angle and translation distance between two poses.
    public static func poseError(_ a: simd_float4x4, _ b: simd_float4x4) -> PoseError {
        var ma = a, mb = b
        var rotation: Float = -1, translation: Float = -1
        withUnsafeBytes(of: &ma) { pa in
            withUnsafeBytes(of: &mb) { pb in
                fr_pose_error(pa.bindMemory(to: Float.self).baseAddress, pb.bindMemory(to: Float.self).baseAddress,
                              &rotation, &translation)
            }
        }
        return PoseError(rotationDeg: rotation, translationM: translation)
    }

    /// Per-frame errors of estimated world_from_camera poses after expressing both sets relative to their first frame
    /// (the reconstruction's world frame is the first keyframe's camera frame; this removes any gauge difference).
    public static func relativePoseErrors(estimated: [simd_float4x4], groundTruth: [simd_float4x4]) -> [PoseError] {
        guard let e0 = estimated.first, let g0 = groundTruth.first else { return [] }
        let e0inv = e0.inverse, g0inv = g0.inverse
        return zip(estimated, groundTruth).map { poseError(e0inv * $0.0, g0inv * $0.1) }
    }

    private static func check(_ status: fr_status) throws {
        guard status == FR_OK else {
            let detail = String(cString: fr_last_error())
            throw ReconstructionError.core(stage: "evaluation", status: Int32(bitPattern: status.rawValue),
                                           message: detail.isEmpty ? String(cString: fr_status_string(status)) : detail)
        }
    }
}

/// Which parts of FaceReconCore are real implementations (placeholders return FR_ERR_NOT_IMPLEMENTED).
/// Each probe calls the function with arguments a real implementation rejects without side effects.
public enum CoreAvailability {
    /// fr_register_frames (registration work stream).
    public static let registration: Bool = {
        var params = fr_registration_params()
        fr_registration_params_default(&params)
        var frame = fr_rgbd_frame()
        var pose = [Float](repeating: 0, count: 16)
        let status = withUnsafePointer(to: &frame) { fr_register_frames($0, 0, &params, &pose, nil, nil, nil) }
        return status != FR_ERR_NOT_IMPLEMENTED
    }()

    /// fr_tsdf_create (fusion work stream).
    public static let fusion: Bool = {
        var params = fr_tsdf_params()
        fr_tsdf_params_default(&params)
        var tsdf: OpaquePointer?
        let status = fr_tsdf_create(&params, &tsdf)
        if let tsdf { fr_tsdf_free(tsdf) }
        return status != FR_ERR_NOT_IMPLEMENTED
    }()

    /// fr_export_obj (mesh/export work stream).
    public static let objExport: Bool = {
        var mesh: UnsafeMutablePointer<fr_mesh>?
        guard fr_mesh_create(0, 0, 0, &mesh) == FR_OK, let mesh else { return false }
        defer { fr_mesh_free(mesh) }
        var options = fr_export_options()
        fr_export_options_default(&options)
        return fr_export_obj(mesh, nil, "", &options) != FR_ERR_NOT_IMPLEMENTED
    }()

    /// Registration and fusion: what an end-to-end reconstruction needs from the core.
    public static var reconstruction: Bool { registration && fusion }

    /// Human-readable list of the placeholders still in place.
    public static var missingComponents: [String] {
        var missing: [String] = []
        if !registration { missing.append("registration (fr_register_frames)") }
        if !fusion { missing.append("fusion (fr_tsdf_create)") }
        if !objExport { missing.append("OBJ export (fr_export_obj)") }
        return missing
    }
}
