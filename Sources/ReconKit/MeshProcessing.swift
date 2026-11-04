// License: Apache 2.0. See LICENSE file in root directory.
//
// Mesh post-processing of the reconstructed surface: cleanup, detail-preserving smoothing, hole filling (and closing
// the open back of the head), decimation.

import FaceReconCore
import Foundation

public enum MeshProcessing {
    /// Vertices closer than this are merged before the topology-based steps (exact duplicates from meshing; far
    /// below the 0.5-1 mm voxel size).
    static let weldEpsilonM: Float = 1e-6

    /// What the cleanup did to the boundary.
    public struct Summary: Sendable, Equatable {
        /// Holes filled (not counting the closed head).
        public var filledHoles = 0
        /// Boundary loops left open (0: watertight).
        public var openLoops = 0
        /// The open back of the head and the neck cut were closed.
        public var closedHead = false
        /// Fringe triangles removed along that border before closing it.
        public var trimmedTriangles = 0
        /// Filling triangles still folded against the surrounding surface.
        public var foldedTriangles = 0
        /// Area of all filling (synthesized) triangles, mm².
        public var filledAreaMM2: Float = 0
    }

    /// Removes small components, smooths (detail preserving), fills holes and (with `closeHead`) closes the head,
    /// optionally decimates. Sets `mesh.synthesized` for the vertices the filling created.
    @discardableResult
    public static func clean(_ mesh: inout ReconMesh, options: ReconstructionOptions) throws -> Summary {
        var summary = Summary()
        guard mesh.triangleCount > 0 else { return summary }
        try validate(mesh)
        var firstNew: Int?
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
                var params = fillParameters(options)
                var report = fr_fill_report()
                try checkCore(fr_mesh_fill_holes_ex(m, &params, &report), "hole filling")
                summary.filledHoles = Int(report.filled_holes)
                summary.openLoops = Int(report.open_loops)
                summary.closedHead = report.closed_largest != 0
                summary.trimmedTriangles = Int(report.trimmed_triangles)
                summary.foldedTriangles = Int(report.folded_triangles)
                summary.filledAreaMM2 = report.filled_area_m2 * 1e6
                firstNew = Int(report.first_new_vertex)
            }
            try checkCore(fr_mesh_compute_normals(m), "normals")
        }
        if let firstNew {
            // Flags from an earlier filling (carried through withCoreMesh) stay.
            let earlier = mesh.synthesized.count == mesh.vertexCount ? mesh.synthesized : []
            let flags = (0 ..< mesh.vertexCount).map { $0 >= firstNew || (!earlier.isEmpty && earlier[$0]) }
            mesh.synthesized = flags.contains(true) ? flags : []
        }
        // Decimation renumbers the vertices: separate, so that the flags follow them (withCoreMesh).
        if let target = options.targetTriangles, target > 0, target < mesh.triangleCount {
            try mesh.withCoreMesh { m in
                try checkCore(fr_mesh_decimate(m, UInt32(clamping: target), 0), "decimation")
                try checkCore(fr_mesh_compute_normals(m), "normals")
            }
        }
        return summary
    }

    static func fillParameters(_ options: ReconstructionOptions) -> fr_fill_params {
        var params = fr_fill_params()
        fr_fill_params_default(&params)
        params.max_hole_perimeter_m = options.maxHolePerimeterMM.isFinite ? max(0, options.maxHolePerimeterMM) / 1000 : 0
        params.max_hole_edges = UInt32(clamping: options.maxHoleEdges)
        params.close_largest = options.closeHead ? 1 : 0
        return params
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
        guard mesh.synthesized.isEmpty || mesh.synthesized.count == n else {
            throw invalid("synthesized flags do not match the vertices")
        }
    }
}
