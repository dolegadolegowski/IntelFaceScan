// License: Apache 2.0. See LICENSE file in root directory.
//
// UV unwrap, texture baking and vertex colors from the keyframes.

import CameraKit
import FaceReconCore
import Foundation
import ScanKit
import simd

public enum Texturing {
    /// Unwraps UVs (splitting seam vertices) and bakes a texture from the keyframes at their poses (synthesized surface:
    /// interpolated from the measured colors around it, never projected from the keyframes).
    public static func unwrapAndBake(_ mesh: inout ReconMesh, keyframes: [Keyframe], poses: [simd_float4x4],
                                     options: ReconstructionOptions) throws -> TextureImage? {
        guard mesh.triangleCount > 0 else { return nil }
        try MeshProcessing.validate(mesh, stage: "texturing")
        try validate(keyframes: keyframes, poses: poses)
        let size = textureSize(options)
        var uv = fr_uv_params()
        fr_uv_params_default(&uv)
        uv.texture_size = Int32(size)
        // 4 px at 4096: enough gutter for bilinear filtering and the first mip levels.
        uv.padding_px = Int32(max(2, size / 1024))
        try mesh.withCoreMesh { m in
            if m.pointee.normals == nil { try checkCore(fr_mesh_compute_normals(m), "uv unwrap") }
            try checkCore(fr_mesh_uv_unwrap(m, &uv, nil, nil), "uv unwrap")
        }

        var params = textureParameters(size: size)
        let frames = RGBDFrameSet(keyframes: keyframes)
        let weights = frameWeights(for: keyframes)
        let packed = packPoses(poses)
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var texture: UnsafeMutablePointer<fr_texture>?
        // Synthesized surface (seam copies included, withCoreMesh carried the flags) takes interpolated colors.
        let synthesized = mesh.synthesizedBytes
        try frames.withFrames { framePointer, count in
            try packed.withUnsafeBufferPointer { posePointer in
                try weights.withUnsafeBufferPointer { weightPointer in
                    try synthesized.withUnsafeBufferPointer { flags in
                        params.synthesized = synthesized.isEmpty ? nil : flags.baseAddress
                        defer { params.synthesized = nil }
                        try checkCore(fr_bake_texture(core, framePointer, posePointer.baseAddress, weightPointer.baseAddress,
                                                      count, &params, &texture, nil, nil), "texturing")
                    }
                }
            }
        }
        guard let baked = texture else {
            throw ReconstructionError.core(stage: "texturing", status: Int32(bitPattern: FR_ERR_FAILED.rawValue),
                                           message: "no texture returned")
        }
        defer { fr_texture_free(baked) }
        return TextureImage(core: baked)
    }

    /// Per-vertex colors from the keyframes (synthesized vertices: interpolated from the measured colors around them).
    public static func colorizeVertices(_ mesh: inout ReconMesh, keyframes: [Keyframe], poses: [simd_float4x4],
                                        options: ReconstructionOptions) throws {
        guard mesh.triangleCount > 0 else { return }
        try MeshProcessing.validate(mesh, stage: "vertex colors")
        try validate(keyframes: keyframes, poses: poses)
        var params = textureParameters(size: textureSize(options))
        let frames = RGBDFrameSet(keyframes: keyframes)
        let weights = frameWeights(for: keyframes)
        let packed = packPoses(poses)
        let synthesized = mesh.synthesizedBytes
        try mesh.withCoreMesh { m in
            try frames.withFrames { framePointer, count in
                try packed.withUnsafeBufferPointer { posePointer in
                    try weights.withUnsafeBufferPointer { weightPointer in
                        try synthesized.withUnsafeBufferPointer { flags in
                            params.synthesized = synthesized.isEmpty ? nil : flags.baseAddress
                            defer { params.synthesized = nil }
                            try checkCore(fr_colorize_vertices(m, framePointer, posePointer.baseAddress,
                                                               weightPointer.baseAddress, count, &params), "vertex colors")
                        }
                    }
                }
            }
        }
    }

    // MARK: - Parameters

    static func textureSize(_ options: ReconstructionOptions) -> Int {
        min(16384, max(256, options.textureSize))
    }

    static func textureParameters(size: Int) -> fr_texture_params {
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        params.texture_size = Int32(size)
        params.dilate_px = Int32(max(4, size / 512))
        return params
    }

    /// Quality weight per keyframe in (0, 1]: sharp, frontal and still frames dominate the blend where several
    /// views see a surface point equally well (the per-point view angle weighting comes on top).
    static func frameWeights(for keyframes: [Keyframe]) -> [Float] {
        let sharpness = keyframes.map { $0.quality.sharpness }.filter { $0.isFinite && $0 > 0 }.sorted()
        let reference = sharpness.isEmpty ? 0 : sharpness[sharpness.count / 2]
        return keyframes.map { kf in
            let q = kf.quality
            // Relative to the median keyframe: sharper frames saturate at 1, blurrier ones lose weight.
            var sharp: Float = 1
            if reference > 0 {
                sharp = q.sharpness.isFinite && q.sharpness > 0 ? min(1, (q.sharpness / reference).squareRoot()) : 0.2
            }
            let pose = kf.headPose
            let yaw = pose.yaw.isFinite ? pose.yaw * .pi / 180 : 0, pitch = pose.pitch.isFinite ? pose.pitch * .pi / 180 : 0
            let frontal = 0.6 + 0.4 * max(0, cos(yaw) * cos(pitch))
            let speed = q.angularSpeedDegPerSec.isFinite ? max(0, q.angularSpeedDegPerSec) : 0
            let still = 1 / (1 + speed / 30)
            return min(1, max(0.05, max(0.2, sharp) * frontal * still))
        }
    }

    /// The core reads keyframe pixels through raw pointers: every buffer must match its declared size. (Poses are
    /// checked by the core.)
    static func validate(keyframes: [Keyframe], poses: [simd_float4x4]) throws {
        func invalid(_ message: String) -> ReconstructionError {
            .core(stage: "texturing", status: Int32(bitPattern: FR_ERR_INVALID_ARGUMENT.rawValue), message: message)
        }
        guard !keyframes.isEmpty else { throw ReconstructionError.notEnoughKeyframes(0) }
        guard poses.count == keyframes.count else {
            throw invalid("\(poses.count) poses for \(keyframes.count) keyframes")
        }
        for (i, kf) in keyframes.enumerated() {
            let color = kf.frame.color, depth = kf.frame.alignedDepth
            let w = color.width, h = color.height
            guard w > 0, h > 0, color.bytesPerRow == w * 3, color.pixels.count >= w * h * 3,
                  color.intrinsics.width == w, color.intrinsics.height == h else {
                throw invalid("keyframe \(i): inconsistent color image")
            }
            guard depth.width == w, depth.height == h, depth.pixels.count >= w * h * 2 else {
                throw invalid("keyframe \(i): aligned depth does not match the color image")
            }
        }
    }
}
