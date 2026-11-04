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
    /// "dataset", "recording" or "syntheticCamera"
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
    /// Measured reconstructed surface -> ground-truth surface (in the ground truth's world frame). Synthesized surface
    /// (filled holes and, with `ReconstructionOptions.closeHead`, the closed back of the head) is left out: it is a
    /// guess, not a measurement.
    public var surfaceToGroundTruth: SurfaceDistance?
    /// The whole mesh, synthesized surface included, -> ground-truth surface: what `ifscan eval` measures on the
    /// exported OBJ. nil when nothing was synthesized (then it equals `surfaceToGroundTruth`).
    public var wholeMeshToGroundTruth: SurfaceDistance?
    /// The reconstruction's world frame (camera frame of its first frontal keyframe) -> the ground truth's world
    /// frame (camera frame of keyframe 0), column-major, meters: G0 * E0^-1 through keyframe 0's estimated pose E0.
    /// The two differ when the dataset does not start with a frontal keyframe. nil = identity assumed.
    public var groundTruthFromWorld: [Float]?
    public var notes: [String]

    public static func evaluate(result: ReconstructionResult, groundTruth: SyntheticGroundTruth, keyframeIDs: [Int],
                                mesh: ReconMesh?, samples: Int = 200_000) -> GroundTruthEvaluation {
        var notes: [String] = []
        var frames: [Frame] = []
        let truth = groundTruth.worldFromCamera
        var groundTruthFromWorld: simd_float4x4?
        if result.cameraPoses.count == truth.count, let e0 = result.cameraPoses.first, let g0 = truth.first {
            groundTruthFromWorld = g0 * e0.inverse
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
            notes.append("surface distance assumes that the reconstruction's world frame is keyframe 0's camera frame")
        }
        var surface: SurfaceDistance?, wholeMesh: SurfaceDistance?
        if let mesh, result.mesh.triangleCount > 0 {
            do {
                var reconstructed = result.mesh
                if let transform = groundTruthFromWorld {
                    reconstructed.positions = reconstructed.positions.map { p in
                        let q = transform * SIMD4(p, 1)
                        return SIMD3(q.x, q.y, q.z)
                    }
                }
                let measured = reconstructed.measuredSurface
                surface = try MeshEvaluation.distance(from: measured, to: mesh, samples: samples)
                if measured.triangleCount != reconstructed.triangleCount {
                    wholeMesh = try MeshEvaluation.distance(from: reconstructed, to: mesh, samples: samples)
                }
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
            surfaceToGroundTruth: surface, wholeMeshToGroundTruth: wholeMesh,
            groundTruthFromWorld: groundTruthFromWorld?.columnMajor, notes: notes)
    }
}

/// The part of a run report that maps an exported mesh back to the world frame (`ifscan eval --report`): through the
/// inverse export transform to the reconstruction's world frame and, for synthetic datasets, on to the ground truth's.
struct ExportFrame: Decodable {
    struct GroundTruthFrame: Decodable {
        var groundTruthFromWorld: [Float]?
    }

    var exportTransform: [Float]
    var exportScale: Float
    var groundTruth: GroundTruthFrame?

    static func load(_ url: URL) throws -> ExportFrame {
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let frame = try decoder.decode(ExportFrame.self, from: Data(contentsOf: url))
        guard simd_float4x4(columnMajor: frame.exportTransform) != nil, frame.exportScale > 0 else {
            throw ArgumentError("\(url.lastPathComponent): invalid export_transform/export_scale")
        }
        if let m = frame.groundTruth?.groundTruthFromWorld, simd_float4x4(columnMajor: m) == nil {
            throw ArgumentError("\(url.lastPathComponent): invalid ground_truth.ground_truth_from_world")
        }
        return frame
    }

    /// Exported coordinates -> world frame (the ground truth's when the report has one), in millimeters.
    func worldMillimeters(_ exported: SIMD3<Float>) -> SIMD3<Float> {
        let transform = simd_float4x4(columnMajor: exportTransform) ?? matrix_identity_float4x4
        var p = transform.inverse * SIMD4(exported / exportScale, 1)
        if let m = groundTruth?.groundTruthFromWorld, let groundTruthFromWorld = simd_float4x4(columnMajor: m) {
            p = groundTruthFromWorld * p
        }
        return SIMD3(p.x, p.y, p.z) * 1000
    }
}
