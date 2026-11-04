// License: Apache 2.0. See LICENSE file in root directory.
//
// The pipeline on side views beyond 60°, where people turn in real scans although the guidance asks for 45°: the
// textured SyntheticData head with the left and right poses at 65°, no depth beyond 60° incidence (where a D455 stopped
// measuring on a real head) and the systematic Vision error of a real scan's side views. Every side edge then fails
// the fitness floor, and before the overlap test all six side frames were excluded (docs/RECONSTRUCTION.md, "Side
// views beyond 60°"). IFS_RECON_STUDY=1 prints the table, including the accuracy limit of a thin frontal band.

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

struct WideSideOutcome: Sendable {
    var usedFrames: Int
    var keyframes: Int
    /// Worst rotation error (degrees) and head misalignment (mm) of the side frames, and worst rotation error of the
    /// front and down frames, relative to the reference keyframe.
    var sideRotationDeg: Float
    var sideMisalignmentMm: Float
    var otherRotationDeg: Float
    var notes: [String]
}

enum WideSideStudy {
    static func run(sideYawDeg: Float = 65, sidePoseErrorDeg: SIMD3<Float>,
                    frontalHalfWidthM: Float? = nil) async throws -> WideSideOutcome {
        // 1280x800: at 640x400 the side frames of the default test reached 0.53-0.69° / 1.5-1.7 mm.
        var o = SyntheticScan.Options()
        o.width = 1280
        o.height = 800
        o.poses = [.front, .down, .left, .right]
        o.framesPerPose = 3
        o.maxIncidenceDeg = 60
        o.sideYawDeg = sideYawDeg
        o.sidePoseErrorDeg = sidePoseErrorDeg
        o.frontalHalfWidthM = frontalHalfWidthM
        let scan = SyntheticScan.make(o)
        var options = ReconstructionOptions()
        options.targetTriangles = 30_000
        options.textureSize = 256
        let result = try await ReconstructionPipeline(options: options).run(scan.capture)
        // Used keyframes, in capture order (ReconstructionResult.cameraPoses); excluded ones are named in the notes.
        let excluded = Set(result.report.notes.filter { $0.hasPrefix("keyframes excluded") }
            .flatMap { $0.split(separator: "#").dropFirst().compactMap { Int($0.prefix { $0.isNumber }) } })
        let keyframes = scan.capture.keyframes
        let used = keyframes.indices.filter { !excluded.contains(keyframes[$0].id) }
        // Keyframe 0 (frontal, the reference) is the world of both the ground truth and the reconstruction.
        let truth = used.map { scan.groundTruthPoses[$0] }
        let rotation = MeshEvaluation.relativePoseErrors(estimated: result.cameraPoses, groundTruth: truth).map(\.rotationDeg)
        let misalignment = MeshEvaluation.relativeHeadMisalignment(estimated: result.cameraPoses, groundTruth: truth,
                                                                   center: scan.headCenter)
        let side = used.indices.filter { [.left, .right].contains(keyframes[used[$0]].scanPose) }
        let other = used.indices.filter { !side.contains($0) }
        return WideSideOutcome(usedFrames: result.report.usedFrames, keyframes: keyframes.count,
                               sideRotationDeg: side.map { rotation[$0] }.max() ?? .nan,
                               sideMisalignmentMm: (side.map { misalignment[$0] }.max() ?? .nan) * 1000,
                               otherRotationDeg: other.map { rotation[$0] }.max() ?? .nan, notes: result.report.notes)
    }

    static func row(_ name: String, _ o: WideSideOutcome) -> String {
        String(format: "| %@ | %d/%d | %.2f° / %.1f mm | %.2f° |", name, o.usedFrames, o.keyframes, o.sideRotationDeg,
               o.sideMisalignmentMm, o.otherRotationDeg)
    }
}

@Suite("Pipeline wide side views", .enabled(if: CoreAvailability.reconstruction, "fr_register_frames / fr_tsdf_create are placeholders"),
       .timeLimit(.minutes(30)), .heavyReconstruction)
struct PipelineWideSideViewsTests {
    /// Side views at 65°: Vision overstates the turn by 6° and reads 9° of pitch and 7° of roll (a real scan's side
    /// views), or overstates the turn by 15°.
    @Test(arguments: [SIMD3<Float>(6, 9, 7), SIMD3<Float>(15, 0, 0)])
    func sideViewsAtSixtyFiveDegreesAreRegistered(visionError: SIMD3<Float>) async throws {
        let o = try await WideSideStudy.run(sidePoseErrorDeg: visionError)
        print("wide side views: " + WideSideStudy.row("65°, Vision side error \(visionError)", o) + " notes \(o.notes)")
        #expect(o.usedFrames == o.keyframes, "\(o.notes)")
        #expect(o.sideRotationDeg < 0.6)
        #expect(o.sideMisalignmentMm < 2.5)
        #expect(o.otherRotationDeg < 0.15)
    }

    /// IFS_RECON_STUDY=1: side yaw, Vision side error and a frontal band as narrow as a real D455 head mask
    /// (docs/RECONSTRUCTION.md, "Side views beyond 60°").
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func wideSideViewTable() async throws {
        let cases: [(String, Float, SIMD3<Float>, Float?)] = [
            ("65°, Vision error 2° (random only)", 65, .zero, nil),
            ("65°, side error (6, 9, 7)°", 65, SIMD3(6, 9, 7), nil),
            ("65°, side error (15, 0, 0)°", 65, SIMD3(15, 0, 0), nil),
            ("68°, Vision error 2°", 68, .zero, nil),
            ("65°, frontal band ±3.5 cm, Vision error 2°", 65, .zero, 0.035),
            ("65°, frontal band ±3.5 cm, side error (6, 9, 7)°", 65, SIMD3(6, 9, 7), 0.035),
        ]
        var rows = ["| scan | frames | side frames: worst rotation / misalignment | front, down: worst rotation |", "|---|---|---|---|"]
        for (name, yaw, sideError, band) in cases {
            do {
                let o = try await WideSideStudy.run(sideYawDeg: yaw, sidePoseErrorDeg: sideError, frontalHalfWidthM: band)
                rows.append(WideSideStudy.row(name, o))
            } catch {
                rows.append("| \(name) | failed: \(error.localizedDescription) | | |")
            }
        }
        print("wide side view table:\n" + rows.joined(separator: "\n"))
    }
}
