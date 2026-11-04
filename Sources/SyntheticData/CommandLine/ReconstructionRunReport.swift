// License: Apache 2.0. See LICENSE file in root directory.
//
// The JSON report of `ifscan reconstruct --report` (snake_case keys).

import Foundation
import ReconKit
import simd

public struct ReconstructionRunReport: Codable, Sendable {
    public var tool: String
    public var createdAt: String
    public var input: String
    /// "dataset" or "recording"
    public var inputKind: String
    public var outputs: [String]
    public var keyframes: Int
    public var options: ReconstructionOptions
    public var report: ReconstructionReport
    /// World (head, meters) -> export frame (meters), column-major. Exported coordinates are
    /// exportScale * exportTransform * p_world.
    public var exportTransform: [Float]
    public var exportScale: Float
    /// world_from_camera per used keyframe, column-major.
    public var cameraPoses: [[Float]]
    /// Seconds: input (reading / guided scan), reconstruction, export, total.
    public var timings: [String: Double]
    public var groundTruth: GroundTruthEvaluation?
    public var notes: [String]
}

/// Errors against a synthetic dataset's ground truth.
public struct GroundTruthEvaluation: Codable, Sendable {
    public struct Frame: Codable, Sendable {
        public var id: Int
        public var rotationDeg: Float
        /// Distance between the estimated and true camera centers (includes the lever arm of rotation errors).
        public var translationMm: Float
        /// Largest displacement of points on a 9 cm sphere around the head center (what the surface sees).
        public var headMisalignmentMm: Float
    }

    /// Poses relative to keyframe 0 (the reconstruction's world frame).
    public var poses: [Frame]
    public var meanRotationDeg: Float
    public var maxRotationDeg: Float
    public var meanTranslationMm: Float
    public var maxTranslationMm: Float
    public var meanHeadMisalignmentMm: Float
    public var maxHeadMisalignmentMm: Float
    /// Reconstructed surface -> ground-truth surface (world frame).
    public var surfaceToGroundTruth: SurfaceDistance?
    public var notes: [String]

    public static func evaluate(result: ReconstructionResult, groundTruth: SyntheticGroundTruth, keyframeIDs: [Int],
                                mesh: ReconMesh?, samples: Int = 200_000) -> GroundTruthEvaluation {
        var notes: [String] = []
        var frames: [Frame] = []
        let truth = groundTruth.worldFromCamera
        if result.cameraPoses.count == truth.count {
            let errors = MeshEvaluation.relativePoseErrors(estimated: result.cameraPoses, groundTruth: truth)
            let center = groundTruth.headCenter.count == 3
                ? SIMD3(groundTruth.headCenter[0], groundTruth.headCenter[1], groundTruth.headCenter[2]) : SIMD3<Float>(0, 0, 0.68)
            let misalignment = MeshEvaluation.relativeHeadMisalignment(estimated: result.cameraPoses, groundTruth: truth,
                                                                       center: center)
            for (i, e) in errors.enumerated() {
                frames.append(Frame(id: i < keyframeIDs.count ? keyframeIDs[i] : i, rotationDeg: e.rotationDeg,
                                    translationMm: e.translationM * 1000, headMisalignmentMm: misalignment[i] * 1000))
            }
        } else {
            notes.append("pose errors skipped: \(result.cameraPoses.count) poses for \(truth.count) ground-truth frames")
        }
        var surface: SurfaceDistance?
        if let mesh, result.mesh.triangleCount > 0 {
            do {
                surface = try MeshEvaluation.distance(from: result.mesh, to: mesh, samples: samples)
            } catch {
                notes.append("surface distance failed: \(error.localizedDescription)")
            }
        } else if mesh == nil {
            notes.append("surface distance skipped: no \(SyntheticGroundTruth.meshFileName)")
        }
        let rotations = frames.map(\.rotationDeg), translations = frames.map(\.translationMm)
        let misalignments = frames.map(\.headMisalignmentMm)
        return GroundTruthEvaluation(
            poses: frames,
            meanRotationDeg: rotations.isEmpty ? 0 : rotations.reduce(0, +) / Float(rotations.count),
            maxRotationDeg: rotations.max() ?? 0,
            meanTranslationMm: translations.isEmpty ? 0 : translations.reduce(0, +) / Float(translations.count),
            maxTranslationMm: translations.max() ?? 0,
            meanHeadMisalignmentMm: misalignments.isEmpty ? 0 : misalignments.reduce(0, +) / Float(misalignments.count),
            maxHeadMisalignmentMm: misalignments.max() ?? 0,
            surfaceToGroundTruth: surface, notes: notes)
    }
}

/// The part of a run report that maps an exported mesh back to the world frame (`ifscan eval --report`).
struct ExportFrame: Decodable {
    var exportTransform: [Float]
    var exportScale: Float

    static func load(_ url: URL) throws -> ExportFrame {
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let frame = try decoder.decode(ExportFrame.self, from: Data(contentsOf: url))
        guard simd_float4x4(columnMajor: frame.exportTransform) != nil, frame.exportScale > 0 else {
            throw ArgumentError("\(url.lastPathComponent): invalid export_transform/export_scale")
        }
        return frame
    }

    /// Exported coordinates -> world frame, in millimeters.
    func worldMillimeters(_ exported: SIMD3<Float>) -> SIMD3<Float> {
        let transform = simd_float4x4(columnMajor: exportTransform) ?? matrix_identity_float4x4
        let p = transform.inverse * SIMD4(exported / exportScale, 1)
        return SIMD3(p.x, p.y, p.z) * 1000
    }
}
