// License: Apache 2.0. See LICENSE file in root directory.
//
// ScanCapture -> registered, fused, cleaned, textured mesh. Stages (docs/RECONSTRUCTION.md):
//   preparing      scan order, lens undistortion of every keyframe (the core works with an ideal pinhole)
//   segmentation   per-keyframe head masks (depth band, face component, chin cut)
//   registration   initial poses from Vision + landmarks, pairwise ICP, loop closures, pose graph (FaceReconCore)
//   fusion         TSDF on the GPU (or screened Poisson), marching cubes
//   cleanup        crop below the chin, MeshProcessing.clean
//   texturing      vertex colors and texture atlas (Texturing)
// The heavy work runs on a background dispatch queue (never on the main actor or the cooperative pool); task
// cancellation is forwarded to FaceReconCore through the progress callbacks.

import CameraKit
import FaceReconCore
import Foundation
import ScanKit
import simd

public final class ReconstructionPipeline: Sendable {
    public let options: ReconstructionOptions
    /// Lens undistortion of the keyframes (off only to measure what ignoring the distortion costs).
    let undistortLens: Bool
    /// Overrides whether the aligned depth is distorted like the color image (LensDistortion.alignedDepthFollowsDistortion).
    let depthFollowsDistortion: Bool?

    public init(options: ReconstructionOptions = ReconstructionOptions()) {
        self.options = options
        undistortLens = true
        depthFollowsDistortion = nil
    }

    init(options: ReconstructionOptions, undistortLens: Bool, depthFollowsDistortion: Bool? = nil) {
        self.options = options
        self.undistortLens = undistortLens
        self.depthFollowsDistortion = depthFollowsDistortion
    }

    public func run(_ capture: ScanCapture,
                    progress: @escaping @Sendable (ReconstructionProgress) -> Void = { _ in }) async throws -> ReconstructionResult {
        if Task.isCancelled { throw ReconstructionError.cancelled }
        let cancellation = CancellationFlag()
        let options = self.options
        let undistortLens = self.undistortLens, depthFollowsDistortion = self.depthFollowsDistortion
        return try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<ReconstructionResult, Error>) in
                DispatchQueue.global(qos: .userInitiated).async {
                    let run = PipelineRun(options: options, progress: progress, cancellation: cancellation)
                    run.undistortLens = undistortLens
                    run.depthFollowsDistortion = depthFollowsDistortion
                    do {
                        continuation.resume(returning: try run.execute(capture))
                    } catch {
                        continuation.resume(throwing: error)
                    }
                }
            }
        } onCancel: {
            cancellation.cancel()
        }
    }
}

/// Set from the task's cancellation handler, polled by the pipeline and FaceReconCore's progress callbacks.
final class CancellationFlag: @unchecked Sendable {
    private let lock = NSLock()
    private var cancelled = false

    func cancel() {
        lock.lock()
        cancelled = true
        lock.unlock()
    }

    var isCancelled: Bool {
        lock.lock()
        defer { lock.unlock() }
        return cancelled
    }
}

/// One execution of the pipeline (all state of a run; used from one background thread, plus FaceReconCore's
/// serialized progress callbacks).
final class PipelineRun: @unchecked Sendable {
    let options: ReconstructionOptions
    let progress: @Sendable (ReconstructionProgress) -> Void
    let cancellation: CancellationFlag
    private(set) var report = ReconstructionReport()
    var undistortLens = true
    var depthFollowsDistortion: Bool?

    /// Lowest observation weight of a voxel that may become surface (1 = one frontal view at 0.6 m).
    static let minSurfaceWeight: Float = 0.25
    /// Volumes beyond this size get a coarser voxel instead of failing (8 bytes per voxel).
    static let maxVoxels = 600_000_000
    /// Mesh crop: this far below the chin, perpendicular to the head's vertical axis.
    static let chinCropOffset: Float = 0.012

    init(options: ReconstructionOptions, progress: @escaping @Sendable (ReconstructionProgress) -> Void,
         cancellation: CancellationFlag) {
        self.options = options
        self.progress = progress
        self.cancellation = cancellation
    }

    private func emit(_ stage: ReconstructionProgress.Stage, _ fraction: Double, _ message: String = "") {
        progress(ReconstructionProgress(stage: stage, fraction: min(1, max(0, fraction)), message: message))
    }

    private func checkCancelled() throws {
        if cancellation.isCancelled { throw ReconstructionError.cancelled }
    }

    private func timed<T>(_ key: String, _ body: () throws -> T) rethrows -> T {
        let start = DispatchTime.now().uptimeNanoseconds
        defer { report.durations[key, default: 0] += Double(DispatchTime.now().uptimeNanoseconds - start) / 1e9 }
        return try body()
    }

    /// Runs a step owned by another module; its failure (other than cancellation) degrades the result, it does not
    /// lose it.
    private func tolerant(_ what: String, _ body: () throws -> Void) throws {
        do {
            try body()
        } catch ReconstructionError.cancelled {
            throw ReconstructionError.cancelled
        } catch {
            report.notes.append("\(what) failed: \(error.localizedDescription)")
        }
    }

    func execute(_ capture: ScanCapture) throws -> ReconstructionResult {
        let total = DispatchTime.now().uptimeNanoseconds
        let captured = capture.keyframes
        report.keyframes = captured.count
        report.method = options.method
        emit(.preparing, 0, "\(captured.count) keyframes")

        // Scan order (front first, stable within a pose), structurally valid frames only.
        let rank = Dictionary(uniqueKeysWithValues: ScanPose.allCases.enumerated().map { ($0.element, $0.offset) })
        let ordered = captured.indices.sorted { (rank[captured[$0].scanPose] ?? 99, $0) < (rank[captured[$1].scanPose] ?? 99, $1) }
        let valid = ordered.filter { Self.isStructurallyValid(captured[$0]) }
        if valid.count < captured.count {
            report.notes.append("\(captured.count - valid.count) keyframes with inconsistent image data skipped")
        }
        try checkCancelled()

        // Lens distortion: every later stage (core clouds, TSDF, texture projection, landmarks) assumes a pinhole.
        var keyframes = captured
        if undistortLens {
            let undistorted = timed("undistortion") {
                KeyframeUndistortion.undistort(captured, indices: valid, depthFollowsDistortion: depthFollowsDistortion)
            }
            keyframes = undistorted.keyframes
            if !undistorted.removed.isEmpty {
                report.notes.append("lens distortion removed: \(undistorted.removed.map(\.rawValue).joined(separator: ", "))")
            }
            if !undistorted.unknownModels.isEmpty {
                report.notes.append("unknown lens distortion model \(undistorted.unknownModels.joined(separator: ", ")): treated as a pinhole")
            }
        }
        try checkCancelled()

        // Segmentation.
        emit(.segmentation, 0.02, "head masks")
        let segments: [SegmentedKeyframe?] = timed("segmentation") {
            let results = LockedArray<SegmentedKeyframe?>(count: keyframes.count, value: nil)
            let keep = options.keepShoulders
            DispatchQueue.concurrentPerform(iterations: valid.count) { k in
                let index = valid[k]
                results.set(index, HeadSegmentation.segment(keyframes[index], keepShoulders: keep))
            }
            return results.values
        }
        try checkCancelled()
        let usable = valid.filter { segments[$0] != nil }
        guard usable.count >= 5 else { throw ReconstructionError.notEnoughKeyframes(usable.count) }
        guard let reference = usable.first, keyframes[reference].scanPose == .front else {
            throw ReconstructionError.core(stage: "validation", status: Int32(bitPattern: FR_ERR_NOT_ENOUGH_DATA.rawValue),
                                           message: "no usable frontal keyframe (face depth missing or too few head points)")
        }
        if usable.count < valid.count {
            report.notes.append("\(valid.count - usable.count) keyframes without usable head depth skipped")
        }
        emit(.segmentation, 0.08, "\(usable.count) usable keyframes")

        // Registration (frame 0 = reference frontal keyframe = world).
        let regKeyframes = usable.map { i -> Keyframe in
            var k = keyframes[i]
            k.mask = segments[i]?.headMask
            return k
        }
        let regSegments = usable.compactMap { segments[$0] }
        let initial = InitialPoseEstimator.estimate(keyframes: regKeyframes, segments: regSegments, reference: 0)
        report.notes.append("initial poses: \(initial.kabschFrames) landmark fits, \(initial.landmarkTranslationFrames) Vision "
                            + "rotation + landmark translation, "
                            + "\(usable.count - 1 - initial.kabschFrames - initial.landmarkTranslationFrames) Vision rotation + head centroid")
        let weights = Self.frameWeights(regKeyframes)
        let regFrames = RGBDFrameSet(keyframes: regKeyframes, initialPoses: initial.worldFromCamera, weights: weights,
                                     includeColor: options.useColoredICP)
        var regParams = fr_registration_params()
        fr_registration_params_default(&regParams)
        regParams.icp.colored = options.useColoredICP ? 1 : 0
        regParams.global_optimization = options.globalOptimization ? 1 : 0
        var poseFloats = [Float](repeating: 0, count: 16 * regFrames.count)
        var regReport = fr_registration_report()
        try timed("registration") {
            let box = CoreProgressBox { [weak self] fraction, stage in
                guard let self else { return false }
                self.emit(stage.contains("pose graph") ? .globalOptimization : .registration, 0.08 + 0.3 * Double(fraction), stage)
                return !self.cancellation.isCancelled
            }
            try box.withUserPointer { user in
                try regFrames.withFrames { frames, count in
                    try checkCore(fr_register_frames(frames, count, &regParams, &poseFloats, &regReport, CoreProgressBox.callback, user),
                                  "registration")
                }
            }
        }
        let poses = unpackPoses(poseFloats)
        report.odometryEdges = Int(regReport.odometry_edges)
        report.loopClosures = Int(regReport.loop_closures)
        report.rejectedEdges = Int(regReport.rejected_edges)
        report.meanRegistrationRMSEmm = regReport.mean_rmse_m * 1000
        report.maxRegistrationRMSEmm = regReport.max_rmse_m * 1000
        report.meanFitness = regReport.mean_fitness
        try checkCancelled()

        // Frames whose registered surface does not fit their neighbors would smear the fusion.
        emit(.globalOptimization, 0.38, "checking registration")
        let check = try timed("registration check") {
            try RegistrationCheck.verify(frames: regFrames, poses: poses, landmarks: initial.landmarks)
        }
        let used = (0 ..< usable.count).filter { check.passed[$0] }
        let landmarkDropped = (0 ..< usable.count).filter { !check.passed[$0] && check.landmarkFailed[$0] }
        let surfaceDropped = (0 ..< usable.count).filter { !check.passed[$0] && !check.landmarkFailed[$0] }
        for (frames, reason) in [(surfaceDropped, "surface does not fit"), (landmarkDropped, "face landmarks do not fit")] where !frames.isEmpty {
            let names = frames.map { "#\(keyframes[usable[$0]].id)" }
            report.notes.append("keyframes excluded after registration (\(reason)): \(names.joined(separator: ", "))")
        }
        guard used.count >= 5 else { throw ReconstructionError.notEnoughKeyframes(used.count) }
        report.usedFrames = used.count

        // Fusion input: fusion masks, original capture order (also used for texturing and the result poses).
        let fusionOrder = used.sorted { usable[$0] < usable[$1] }
        let fusionKeyframes = fusionOrder.map { k -> Keyframe in
            var kf = keyframes[usable[k]]
            kf.mask = segments[usable[k]]?.fusionMask
            return kf
        }
        let fusionPoses = fusionOrder.map { poses[$0] }
        let fusionFrames = RGBDFrameSet(keyframes: fusionKeyframes, initialPoses: nil,
                                        weights: fusionOrder.map { weights[$0] }, includeColor: true)
        try checkCancelled()

        var mesh: ReconMesh
        var voxel = options.voxelSizeMM / 1000
        var truncation = max(options.truncationMM / 1000, 2 * voxel)
        let margin = max(3 * truncation, 0.015)
        let bounds = try timed("bounds") { try RegistrationCheck.bounds(frames: fusionFrames, poses: fusionPoses, margin: margin) }
        switch options.method {
        case .tsdf:
            let extent = bounds.max - bounds.min
            let voxels = Double(extent.x / voxel + 1) * Double(extent.y / voxel + 1) * Double(extent.z / voxel + 1)
            if voxels > Double(Self.maxVoxels) {
                voxel *= Float(pow(voxels / Double(Self.maxVoxels), 1.0 / 3.0)) * 1.01
                truncation = max(truncation, 2 * voxel)   // fr_tsdf_create needs truncation > voxel
                report.notes.append(String(format: "volume too large for %.2f mm voxels; using %.2f mm",
                                           options.voxelSizeMM, voxel * 1000))
            }
            mesh = try timed("fusion") { try fuseTSDF(fusionFrames, poses: fusionPoses, voxel: voxel, truncation: truncation, bounds: bounds) }
        case .poisson:
            mesh = try timed("poisson") { try poisson(fusionFrames, poses: fusionPoses, voxel: voxel) }
        }
        report.voxelSizeMM = voxel * 1000
        try checkCancelled()

        // Cleanup.
        emit(.cleanup, 0.78, "cleanup")
        let headFrame = HeadFrame(reference: regKeyframes[0], segment: regSegments[0], keyframes: fusionKeyframes, poses: fusionPoses)
        try timed("cleanup") {
            if !options.keepShoulders {
                if let chin = headFrame.chin {
                    cropBelowChin(&mesh, point: chin - headFrame.up * Self.chinCropOffset, normal: headFrame.up)
                } else {
                    report.notes.append("no depth at the chin: neck not cropped")
                }
            }
            try checkCancelled()
            try tolerant("mesh cleanup") { try MeshProcessing.clean(&mesh, options: options) }
        }
        try checkCancelled()

        // Texturing.
        emit(.texturing, 0.86, "texturing")
        var texture: TextureImage?
        try timed("texturing") {
            try tolerant("vertex colors") {
                try Texturing.colorizeVertices(&mesh, keyframes: fusionKeyframes, poses: fusionPoses, options: options)
            }
            try checkCancelled()
            try tolerant("texture") {
                texture = try Texturing.unwrapAndBake(&mesh, keyframes: fusionKeyframes, poses: fusionPoses, options: options)
            }
        }
        try checkCancelled()

        let exportTransform = Self.exportTransform(mesh: mesh, headFrame: headFrame, belowChin: Self.chinCropOffset)
        report.vertexCount = mesh.vertexCount
        report.triangleCount = mesh.triangleCount
        report.durations["total"] = Double(DispatchTime.now().uptimeNanoseconds - total) / 1e9
        emit(.finished, 1, "\(mesh.vertexCount) vertices, \(mesh.triangleCount) triangles")
        return ReconstructionResult(mesh: mesh, texture: texture, cameraPoses: fusionPoses, exportTransform: exportTransform,
                                    report: report)
    }

    // MARK: - Stages

    private func fuseTSDF(_ frames: RGBDFrameSet, poses: [simd_float4x4], voxel: Float, truncation: Float,
                          bounds: (min: SIMD3<Float>, max: SIMD3<Float>)) throws -> ReconMesh {
        var params = fr_tsdf_params()
        fr_tsdf_params_default(&params)
        params.voxel_size_m = voxel
        params.truncation_m = truncation
        params.use_gpu = options.useGPU ? 1 : 0
        params.integrate_color = 1
        params.weight_mode = 1
        params.bounds_min = (bounds.min.x, bounds.min.y, bounds.min.z)
        params.bounds_max = (bounds.max.x, bounds.max.y, bounds.max.z)
        var raw: OpaquePointer?
        try checkCore(fr_tsdf_create(&params, &raw), "fusion")
        guard let tsdf = raw else { throw ReconstructionError.core(stage: "fusion", status: -1, message: "no volume") }
        defer { fr_tsdf_free(tsdf) }
        report.usedGPU = fr_tsdf_uses_gpu(tsdf) == 1
        if options.useGPU && !report.usedGPU { report.notes.append("Metal unavailable: fused on the CPU") }
        for k in 0 ..< frames.count {
            try checkCancelled()
            emit(.fusion, 0.4 + 0.3 * Double(k) / Double(frames.count), "integrating frame \(k + 1) of \(frames.count)")
            var frame = frames.frames[k]
            try poses[k].floats.withUnsafeBufferPointer { try checkCore(fr_tsdf_integrate(tsdf, &frame, $0.baseAddress), "fusion") }
        }
        try checkCancelled()
        emit(.meshing, 0.7, "marching cubes")
        var out: UnsafeMutablePointer<fr_mesh>?
        try checkCore(fr_tsdf_extract_mesh(tsdf, Self.minSurfaceWeight, &out), "meshing")
        guard let coreMesh = out else { throw ReconstructionError.core(stage: "meshing", status: -1, message: "no mesh") }
        defer { fr_mesh_free(coreMesh) }
        guard coreMesh.pointee.triangle_count > 0 else {
            throw ReconstructionError.core(stage: "meshing", status: Int32(bitPattern: FR_ERR_NOT_ENOUGH_DATA.rawValue),
                                           message: "the fused volume contains no surface")
        }
        return ReconMesh(core: coreMesh)
    }

    private func poisson(_ frames: RGBDFrameSet, poses: [simd_float4x4], voxel: Float) throws -> ReconMesh {
        emit(.fusion, 0.4, "merging registered clouds")
        var cloudParams = fr_cloud_params()
        fr_cloud_params_default(&cloudParams)
        var mergedRaw: OpaquePointer?
        try checkCore(fr_cloud_create(nil, nil, nil, 0, &mergedRaw), "poisson")
        guard let merged = mergedRaw else { throw ReconstructionError.core(stage: "poisson", status: -1, message: "no cloud") }
        defer { fr_cloud_free(merged) }
        for k in 0 ..< frames.count {
            try checkCancelled()
            var frame = frames.frames[k]
            var raw: OpaquePointer?
            try checkCore(fr_cloud_from_rgbd(&frame, &cloudParams, &raw), "poisson")
            guard let cloud = raw else { continue }
            defer { fr_cloud_free(cloud) }
            try poses[k].floats.withUnsafeBufferPointer { try checkCore(fr_cloud_transform(cloud, $0.baseAddress), "poisson") }
            try checkCore(fr_cloud_append(merged, cloud), "poisson")
        }
        var downRaw: OpaquePointer?
        try checkCore(fr_cloud_voxel_downsample(merged, voxel, &downRaw), "poisson")
        guard let down = downRaw else { throw ReconstructionError.core(stage: "poisson", status: -1, message: "no cloud") }
        defer { fr_cloud_free(down) }
        var params = fr_poisson_params()
        fr_poisson_params_default(&params)
        params.depth = Int32(min(13, max(5, options.poissonDepth)))
        emit(.meshing, 0.55, "screened Poisson (depth \(params.depth))")
        let box = CoreProgressBox { [weak self] fraction, stage in
            guard let self else { return false }
            self.emit(.meshing, 0.55 + 0.2 * Double(fraction), stage)
            return !self.cancellation.isCancelled
        }
        var out: UnsafeMutablePointer<fr_mesh>?
        try box.withUserPointer { user in
            try checkCore(fr_poisson_reconstruct(down, &params, &out, CoreProgressBox.callback, user), "poisson")
        }
        guard let coreMesh = out else { throw ReconstructionError.core(stage: "poisson", status: -1, message: "no mesh") }
        defer { fr_mesh_free(coreMesh) }
        return ReconMesh(core: coreMesh)
    }

    /// Removes the part of the mesh below the plane (point, normal = head up).
    private func cropBelowChin(_ mesh: inout ReconMesh, point: SIMD3<Float>, normal: SIMD3<Float>) {
        do {
            try mesh.withCoreMesh { m in
                try [point.x, point.y, point.z].withUnsafeBufferPointer { p in
                    try [normal.x, normal.y, normal.z].withUnsafeBufferPointer { n in
                        try checkCore(fr_mesh_crop_plane(m, p.baseAddress, n.baseAddress), "crop")
                    }
                }
            }
        } catch {
            // Until the mesh module provides fr_mesh_crop_plane (or if it fails): drop whole triangles below.
            mesh = Self.cropFallback(mesh, point: point, normal: normal)
            if case ReconstructionError.core(_, let status, _) = error, status == Int32(bitPattern: FR_ERR_NOT_IMPLEMENTED.rawValue) {
                report.notes.append("chin crop: triangle-level fallback (fr_mesh_crop_plane not implemented)")
            } else {
                report.notes.append("chin crop: triangle-level fallback (\(error.localizedDescription))")
            }
        }
    }

    // MARK: - Helpers

    static func isStructurallyValid(_ kf: Keyframe) -> Bool {
        let c = kf.frame.color, d = kf.frame.alignedDepth
        return c.width > 0 && c.height > 0 && c.width == d.width && c.height == d.height
            && c.bytesPerRow == c.width * 3 && c.pixels.count >= c.width * c.height * 3
            && d.pixels.count >= d.width * d.height * 2 && d.depthScale > 0 && d.depthScale.isFinite
            && c.intrinsics.fx > 0 && c.intrinsics.fy > 0 && c.intrinsics.fx.isFinite && c.intrinsics.fy.isFinite
            && c.intrinsics.ppx.isFinite && c.intrinsics.ppy.isFinite
    }

    /// Quality weights in [0.6, 1] from the sharpness relative to the sharpest keyframe.
    static func frameWeights(_ keyframes: [Keyframe]) -> [Float] {
        let sharpness = keyframes.map { $0.quality.sharpness.isFinite ? max(0, $0.quality.sharpness) : 0 }
        let best = sharpness.max() ?? 0
        guard best > 0 else { return Array(repeating: 1, count: keyframes.count) }
        return sharpness.map { 0.6 + 0.4 * $0 / best }
    }

    static func cropFallback(_ mesh: ReconMesh, point: SIMD3<Float>, normal: SIMD3<Float>) -> ReconMesh {
        let n = simd_normalize(normal)
        let keepVertex = mesh.positions.map { simd_dot($0 - point, n) >= 0 }
        var remap = [Int](repeating: -1, count: mesh.vertexCount)
        var out = ReconMesh()
        var indices: [UInt32] = []
        indices.reserveCapacity(mesh.indices.count)
        for t in 0 ..< mesh.triangleCount {
            let tri = [Int(mesh.indices[3 * t]), Int(mesh.indices[3 * t + 1]), Int(mesh.indices[3 * t + 2])]
            guard tri.allSatisfy({ $0 < mesh.vertexCount && keepVertex[$0] }) else { continue }
            for v in tri {
                if remap[v] < 0 {
                    remap[v] = out.positions.count
                    out.positions.append(mesh.positions[v])
                    if mesh.normals.count == mesh.vertexCount { out.normals.append(mesh.normals[v]) }
                    if mesh.colors.count == mesh.vertexCount { out.colors.append(mesh.colors[v]) }
                    if mesh.uvs.count == mesh.vertexCount { out.uvs.append(mesh.uvs[v]) }
                }
                indices.append(UInt32(remap[v]))
            }
        }
        out.indices = indices
        return out
    }

    /// World (camera frame of the reference keyframe: +Y down, face toward -Z) -> export frame (+Y up, face toward
    /// +Z, origin at the center of the head's bounding box in that orientation). Rigid. With shoulders kept, only the
    /// part above the chin plane counts as the head.
    static func exportTransform(mesh: ReconMesh, headFrame: HeadFrame, belowChin: Float) -> simd_float4x4 {
        let r = headFrame.exportRotation
        var lo = SIMD3<Float>(repeating: .infinity), hi = SIMD3<Float>(repeating: -.infinity)
        var count = 0
        for p in mesh.positions {
            if let chin = headFrame.chin, simd_dot(p - chin, headFrame.up) < -belowChin { continue }
            let q = r * p
            lo = simd_min(lo, q)
            hi = simd_max(hi, q)
            count += 1
        }
        let center = count == 0 ? SIMD3<Float>.zero : (lo + hi) / 2
        return simd_float4x4(rotation: r, translation: -center)
    }
}

/// Fixed-size array written concurrently by index.
final class LockedArray<T>: @unchecked Sendable {
    private let lock = NSLock()
    private var storage: [T]

    init(count: Int, value: T) { storage = Array(repeating: value, count: count) }

    func set(_ index: Int, _ value: T) {
        lock.lock()
        storage[index] = value
        lock.unlock()
    }

    var values: [T] {
        lock.lock()
        defer { lock.unlock() }
        return storage
    }
}
