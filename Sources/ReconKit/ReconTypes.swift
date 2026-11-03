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
    public var useColoredICP: Bool = true
    /// Pose graph with loop closures between poses (left<->front<->right, up/down<->front).
    public var globalOptimization: Bool = true
    public var useGPU: Bool = true
    public var smoothingIterations: Int = 5
    public var fillHoles: Bool = true
    public var maxHoleEdges: Int = 400
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

    public init(positions: [SIMD3<Float>] = [], normals: [SIMD3<Float>] = [], colors: [SIMD3<UInt8>] = [],
                uvs: [SIMD2<Float>] = [], indices: [UInt32] = []) {
        self.positions = positions; self.normals = normals; self.colors = colors; self.uvs = uvs; self.indices = indices
    }

    public var vertexCount: Int { positions.count }
    public var triangleCount: Int { indices.count / 3 }
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

    public init(mesh: ReconMesh, texture: TextureImage?, cameraPoses: [simd_float4x4],
                exportTransform: simd_float4x4, report: ReconstructionReport) {
        self.mesh = mesh; self.texture = texture; self.cameraPoses = cameraPoses
        self.exportTransform = exportTransform; self.report = report
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
    public var includeVertexColors: Bool = true
    public var includeNormals: Bool = true
    public init() {}
}
