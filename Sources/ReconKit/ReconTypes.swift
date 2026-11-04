// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared reconstruction types (Swift side of FaceReconCore).

import Foundation
import simd

public enum SurfaceMethod: String, Codable, Sendable, CaseIterable {
    /// Truncated signed distance fusion (Metal) + marching cubes.
    case tsdf
    /// Screened Poisson surface reconstruction of the registered oriented point cloud.
    case poisson
}

public struct ReconstructionOptions: Codable, Sendable, Hashable {
    /// TSDF voxel size in millimeters (target 0.5-1 mm, chosen from the measured depth noise).
    public var voxelSizeMM: Float = 0.75
    /// TSDF truncation distance in millimeters.
    public var truncationMM: Float = 4
    public var method: SurfaceMethod = .tsdf
    /// Keep neck/shoulders (from the frontal views) instead of cutting below the chin.
    public var keepShoulders: Bool = false
    /// Colored ICP (geometry + photometric term at the finest level) instead of point-to-plane only. Off by default:
    /// with the head turning under fixed room lights the skin color is view dependent, and on the synthetic guided
    /// scans point-to-plane is 10-40x more accurate (docs/RECONSTRUCTION.md, "Colored versus point-to-plane ICP").
    public var useColoredICP: Bool = false
    /// Pose graph with loop closures between poses (left<->front<->right, up/down<->front).
    public var globalOptimization: Bool = true
    public var useGPU: Bool = true
    public var smoothingIterations: Int = 5
    public var fillHoles: Bool = true
    /// Holes whose border is at most this long are filled (the openings behind the ears are 160-220 mm).
    public var maxHolePerimeterMM: Float = 250
    /// Safety cap on the border edges of a filled hole (about 0.4 mm per edge at 0.75 mm voxels).
    public var maxHoleEdges: Int = 4096
    /// Close the model into a watertight bust: the guided scan never sees the back of the head (|yaw| <= 45 degrees),
    /// so the mesh is open there and, joined with the neck cut, forms one long border. With `closeHead` the cleanup
    /// closes it with a smooth surface that continues the head's roundness and becomes a flat cap at the chin crop,
    /// and fills every other hole regardless of `maxHolePerimeterMM`. That surface is synthesized, not measured:
    /// `ReconMesh.synthesized` marks its vertices, texturing gives them interpolated colors instead of projecting the
    /// keyframes onto them, and the report gives its area. Off, the border stays open (as one-sided viewers then show
    /// the inside of the head from behind). Needs `fillHoles`.
    public var closeHead: Bool = true
    /// Decimate to this many triangles (nil = keep full resolution).
    public var targetTriangles: Int? = nil
    public var textureSize: Int = 4096
    /// Remove connected components smaller than this fraction of the largest one.
    public var minComponentFraction: Float = 0.05
    /// Screened Poisson octree depth (method = .poisson).
    public var poissonDepth: Int = 10

    public init() {}
}

/// Mesh in the world (head) frame, meters. Triangles are counter-clockwise seen from outside.
public struct ReconMesh: Sendable {
    public var positions: [SIMD3<Float>]
    public var normals: [SIMD3<Float>]       // empty or one per vertex
    public var colors: [SIMD3<UInt8>]        // empty or one per vertex
    public var uvs: [SIMD2<Float>]           // empty or one per vertex, (0,0) = bottom-left of the texture
    public var indices: [UInt32]             // 3 per triangle
    /// Empty or one per vertex: true where the cleanup synthesized the surface (filled holes, the closed back of the
    /// head) instead of measuring it. Kept through the mesh operations of `withCoreMesh` (by position where vertices
    /// are renumbered: decimation, UV seams).
    public var synthesized: [Bool]

    public init(positions: [SIMD3<Float>] = [], normals: [SIMD3<Float>] = [], colors: [SIMD3<UInt8>] = [],
                uvs: [SIMD2<Float>] = [], indices: [UInt32] = [], synthesized: [Bool] = []) {
        self.positions = positions; self.normals = normals; self.colors = colors; self.uvs = uvs; self.indices = indices
        self.synthesized = synthesized
    }

    public var vertexCount: Int { positions.count }
    public var triangleCount: Int { indices.count / 3 }

    /// Surface area in square meters.
    public var area: Double { area { _ in true } }

    /// The measured part: the triangles without a synthesized vertex (vertices and attributes unchanged). Accuracy
    /// against a ground truth belongs here; the synthesized part is a plausible guess of surface no keyframe saw.
    public var measuredSurface: ReconMesh {
        guard synthesized.count == vertexCount, synthesized.contains(true) else { return self }
        var out = self
        out.indices = []
        out.indices.reserveCapacity(indices.count)
        for t in 0 ..< triangleCount {
            let a = indices[3 * t], b = indices[3 * t + 1], c = indices[3 * t + 2]
            guard Int(max(a, b, c)) < vertexCount else { continue }
            if !synthesized[Int(a)] && !synthesized[Int(b)] && !synthesized[Int(c)] { out.indices += [a, b, c] }
        }
        return out
    }

    /// Area (square meters) of the triangles with a synthesized vertex: the surface the cleanup made up.
    public var synthesizedArea: Double {
        guard synthesized.count == vertexCount else { return 0 }
        return area { tri in synthesized[Int(tri.0)] || synthesized[Int(tri.1)] || synthesized[Int(tri.2)] }
    }

    private func area(where include: ((UInt32, UInt32, UInt32)) -> Bool) -> Double {
        var sum = 0.0
        for t in 0 ..< triangleCount {
            let tri = (indices[3 * t], indices[3 * t + 1], indices[3 * t + 2])
            guard Int(max(tri.0, tri.1, tri.2)) < vertexCount, include(tri) else { continue }
            let a = positions[Int(tri.0)], b = positions[Int(tri.1)], c = positions[Int(tri.2)]
            sum += Double(simd_length(simd_cross(b - a, c - a))) / 2
        }
        return sum
    }
}

/// RGBA8 texture, top-left origin.
public struct TextureImage: Sendable {
    public var width: Int
    public var height: Int
    public var rgba: Data
    public init(width: Int, height: Int, rgba: Data) { self.width = width; self.height = height; self.rgba = rgba }
}

public struct ReconstructionReport: Codable, Sendable {
    public var keyframes: Int = 0
    public var usedFrames: Int = 0
    public var odometryEdges: Int = 0
    public var loopClosures: Int = 0
    public var rejectedEdges: Int = 0
    public var meanRegistrationRMSEmm: Float = 0
    public var maxRegistrationRMSEmm: Float = 0
    public var meanFitness: Float = 0
    public var voxelSizeMM: Float = 0
    public var method: SurfaceMethod = .tsdf
    public var usedGPU: Bool = false
    public var vertexCount: Int = 0
    public var triangleCount: Int = 0
    /// Surface area of the result in mm².
    public var surfaceAreaMM2: Float = 0
    /// The part of it the cleanup synthesized (filled holes, the closed back of the head), mm²: not measured.
    public var synthesizedAreaMM2: Float = 0
    /// Stage name -> seconds.
    public var durations: [String: Double] = [:]
    public var notes: [String] = []
    public init() {}
}

public struct ReconstructionResult: Sendable {
    public var mesh: ReconMesh
    public var texture: TextureImage?
    /// world_from_camera per used keyframe (same order as ScanCapture.keyframes).
    public var cameraPoses: [simd_float4x4]
    /// World (head, meters) -> export frame (meters, before scaling to millimeters): head upright (+Y up),
    /// face looking toward +Z, origin at the head center. Rigid (det = +1).
    public var exportTransform: simd_float4x4
    public var report: ReconstructionReport

    /// The report's area figures are filled in from the mesh here when the report does not have them yet, so that every
    /// result says how much of its surface was synthesized.
    public init(mesh: ReconMesh, texture: TextureImage?, cameraPoses: [simd_float4x4],
                exportTransform: simd_float4x4, report: ReconstructionReport) {
        self.mesh = mesh; self.texture = texture; self.cameraPoses = cameraPoses
        self.exportTransform = exportTransform; self.report = report
        if self.report.surfaceAreaMM2 == 0 && mesh.triangleCount > 0 {
            let total = mesh.area, synthesized = mesh.synthesizedArea
            self.report.surfaceAreaMM2 = Float(total * 1e6)
            self.report.synthesizedAreaMM2 = Float(synthesized * 1e6)
            if synthesized > 0 {
                // Hole filling without closeHead, also the closed back of the head with it.
                self.report.notes.append(String(format: "%.0f of %.0f cm² synthesized by hole filling: not measured",
                                                synthesized * 1e4, total * 1e4))
            }
        }
    }
}

public struct ReconstructionProgress: Sendable {
    public enum Stage: String, Sendable, CaseIterable {
        case preparing, segmentation, registration, globalOptimization, fusion, meshing, cleanup, texturing, finished
    }
    public var stage: Stage
    /// Overall progress in [0, 1].
    public var fraction: Double
    public var message: String
    public init(stage: Stage, fraction: Double, message: String = "") {
        self.stage = stage; self.fraction = fraction; self.message = message
    }
}

public enum ReconstructionError: Error, LocalizedError, Sendable {
    case notEnoughKeyframes(Int)
    case core(stage: String, status: Int32, message: String)
    case cancelled

    public var errorDescription: String? {
        switch self {
        case .notEnoughKeyframes(let n): return "Not enough usable keyframes (\(n))."
        case .core(let stage, _, let message): return "\(stage) failed: \(message)"
        case .cancelled: return "Reconstruction cancelled."
        }
    }
}

public struct ExportOptions: Sendable, Hashable {
    /// 1000 = millimeters.
    public var unitsScale: Float = 1000
    public var includeTexture: Bool = true
    /// Vertex colors in an untextured OBJ ("v x y z r g b") and in the PLY. A textured OBJ carries its color in the
    /// texture only (viewers multiply both), whatever this says.
    public var includeVertexColors: Bool = true
    public var includeNormals: Bool = true
    public init() {}
}
