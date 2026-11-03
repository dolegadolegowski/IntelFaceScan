// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: mesh/texture/export module) - UV unwrap, texture baking, vertex colors.

import Foundation
import ScanKit
import simd

public enum Texturing {
    /// Unwraps UVs (splitting seam vertices) and bakes a texture from the keyframes at their poses.
    public static func unwrapAndBake(_ mesh: inout ReconMesh, keyframes: [Keyframe], poses: [simd_float4x4],
                                     options: ReconstructionOptions) throws -> TextureImage? { nil }

    /// Per-vertex colors from the keyframes.
    public static func colorizeVertices(_ mesh: inout ReconMesh, keyframes: [Keyframe], poses: [simd_float4x4],
                                        options: ReconstructionOptions) throws {}
}
