// License: Apache 2.0. See LICENSE file in root directory.
//
// Mesh post-processing of the reconstructed surface: cleanup, detail-preserving smoothing, hole filling, decimation.

import FaceReconCore
import Foundation

public enum MeshProcessing {
    /// Vertices closer than this are merged before the topology-based steps (exact duplicates from meshing; far
    /// below the 0.5-1 mm voxel size).
    static let weldEpsilonM: Float = 1e-6

    /// Removes small components, smooths (detail preserving), fills small holes, optionally decimates.
    public static func clean(_ mesh: inout ReconMesh, options: ReconstructionOptions) throws {
        guard mesh.triangleCount > 0 else { return }
        try validate(mesh)
        try mesh.withCoreMesh { m in
            try checkCore(fr_mesh_clean(m, weldEpsilonM), "cleanup")
            if options.minComponentFraction > 0 {
                try checkCore(fr_mesh_remove_small_components(m, options.minComponentFraction, 0, nil), "cleanup")
            }
            if options.smoothingIterations > 0 {
                var params = smoothingParameters(iterations: options.smoothingIterations)
                try checkCore(fr_mesh_smooth(m, &params), "smoothing")
            }
            if options.fillHoles && options.maxHoleEdges >= 3 {
                try checkCore(fr_mesh_fill_holes(m, UInt32(clamping: options.maxHoleEdges), nil), "hole filling")
            }
            if let target = options.targetTriangles, target > 0, target < Int(m.pointee.triangle_count) {
                try checkCore(fr_mesh_decimate(m, UInt32(clamping: target), 0), "decimation")
            }
            try checkCore(fr_mesh_compute_normals(m), "normals")
        }
    }

    /// Taubin passes as requested, preceded by bilateral normal filtering that keeps creases (lips, eyelids, nostrils)
    /// sharp; the spatial sigma follows the mesh resolution.
    static func smoothingParameters(iterations: Int) -> fr_smooth_params {
        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        params.taubin_iterations = Int32(clamping: iterations)
        params.bilateral_iterations = Int32(clamping: max(1, (iterations + 1) / 3))
        params.bilateral_sigma_spatial_m = 0  // automatic: 1.5 x mean edge length
        return params
    }

    /// Rejects meshes whose attribute arrays do not match the vertices (the core copy would silently drop them);
    /// index ranges are checked by the core.
    static func validate(_ mesh: ReconMesh, stage: String = "mesh") throws {
        let n = mesh.vertexCount
        func invalid(_ message: String) -> ReconstructionError {
            .core(stage: stage, status: Int32(bitPattern: FR_ERR_INVALID_ARGUMENT.rawValue), message: message)
        }
        guard mesh.indices.count % 3 == 0 else { throw invalid("index count is not a multiple of 3") }
        guard n <= Int(UInt32.max) else { throw invalid("too many vertices") }
        guard mesh.normals.isEmpty || mesh.normals.count == n, mesh.colors.isEmpty || mesh.colors.count == n,
              mesh.uvs.isEmpty || mesh.uvs.count == n else { throw invalid("attribute arrays do not match the vertices") }
    }
}
