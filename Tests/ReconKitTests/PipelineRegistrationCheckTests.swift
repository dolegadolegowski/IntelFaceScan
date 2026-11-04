// License: Apache 2.0. See LICENSE file in root directory.

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

@Suite("Pipeline registration check")
struct PipelineRegistrationCheckTests {
    /// The scan's frames with the pipeline's head masks, as the registration check sees them.
    static func frames(_ scan: SynthScan) throws -> RGBDFrameSet {
        var keyframes = scan.capture.keyframes
        for i in keyframes.indices {
            keyframes[i].mask = try #require(HeadSegmentation.segment(keyframes[i], keepShoulders: false)).headMask
        }
        return RGBDFrameSet(keyframes: keyframes, includeColor: false)
    }

    /// A registration error that moves the head 3 mm sideways and 2 mm in depth (a rotation about the head center
    /// would mostly slide the near-spherical surface along itself, which no surface check can see).
    static let offset = simd_float4x4(rotation: matrix_identity_float3x3, translation: SIMD3(0.003, 0, 0.002))

    @Test func correctPosesPass() throws {
        let scan = PipelineTests.scan
        let check = try RegistrationCheck.verify(frames: try Self.frames(scan), poses: scan.truth)
        #expect(check.passed.allSatisfy { $0 }, "\(check.scores.map { ($0.fitness, $0.rmse * 1000) })")
    }

    @Test func correctPosesPassWithFourViewsPerPose() throws {
        // Like a real guided scan: several keyframes per pose, diagonals included. A frame's few most similar views
        // are then all its siblings; the pose groups must still be linked to the reference across groups (an earlier
        // version compared each frame with its 4 most similar views and dropped the left side of such a scan).
        var o = SynthScanOptions()
        o.width = 320
        o.height = 200
        var rng = Rng(s: 5)
        o.views = []
        for (pose, yaw, pitch) in [(ScanPose.front, 0.0, 0.0), (.up, 0, 25), (.down, 0, -25), (.left, 45, 0), (.right, -45, 0),
                                   (.upLeft, 30, 20), (.upRight, -30, 20), (.downLeft, 30, -20), (.downRight, -30, -20)] {
            for _ in 0 ..< 4 { o.views.append((pose, yaw + 1.5 * rng.gaussian(), pitch + 1.5 * rng.gaussian())) }
        }
        let scan = makeSyntheticScan(o)
        let truth = scan.truth.map { scan.truth[0].inverse * $0 }   // world = camera frame of keyframe 0
        let check = try RegistrationCheck.verify(frames: try Self.frames(scan), poses: truth)
        #expect(check.passed.allSatisfy { $0 }, "failed: \(check.passed.indices.filter { !check.passed[$0] })")
    }

    @Test func singleMisregisteredFrameIsExcluded() throws {
        let scan = PipelineTests.scan
        var poses = scan.truth
        poses[7] = Self.offset * poses[7]
        let check = try RegistrationCheck.verify(frames: try Self.frames(scan), poses: poses)
        #expect(!check.passed[7])
        #expect(check.passed.enumerated().allSatisfy { $0.offset == 7 || $0.element })
    }

    @Test func misregisteredPoseGroupIsExcluded() throws {
        // A wrong bridge puts a whole pose group (the two left views) off together. Each frame of the group fits its
        // sibling, the most similar view, perfectly; the check must compare across groups to see it.
        let scan = PipelineTests.scan
        var poses = scan.truth
        for i in [7, 8] { poses[i] = Self.offset * poses[i] }
        let check = try RegistrationCheck.verify(frames: try Self.frames(scan), poses: poses)
        #expect(!check.passed[7] && !check.passed[8], "\(check.scores.map { ($0.fitness, $0.rmse * 1000) })")
        #expect(check.passed.enumerated().allSatisfy { [7, 8].contains($0.offset) || $0.element })
    }

    /// The scan's camera-frame 3D landmarks, as the pipeline computes them for the initial poses.
    static func landmarks(_ scan: SynthScan) throws -> [[SIMD3<Float>?]] {
        try scan.capture.keyframes.map { kf in
            InitialPoseEstimator.landmarkPoints(kf, try #require(HeadSegmentation.segment(kf, keepShoulders: false)))
        }
    }

    /// A rotation of `degrees` about `axis` through the head center (world frame), applied to a frame's pose.
    static func rotationAboutHead(_ scan: SynthScan, degrees: Float, axis: SIMD3<Float>) -> simd_float4x4 {
        let c = scan.center.simd
        let r = simd_float3x3(simd_quatf(angle: degrees * .pi / 180, axis: simd_normalize(axis)))
        return simd_float4x4(rotation: r, translation: c - r * c)
    }

    @Test func rotationAboutTheHeadCenterIsCaughtByTheLandmarks() throws {
        let scan = PipelineTests.scan
        let frames = try Self.frames(scan), landmarks = try Self.landmarks(scan)
        let correct = try RegistrationCheck.verify(frames: frames, poses: scan.truth, landmarks: landmarks)
        #expect(correct.passed.allSatisfy { $0 }, "\(correct.landmarkResiduals)")
        for (frame, axis) in [(7, SIMD3<Float>(0, 1, 0)), (3, SIMD3<Float>(1, 0, 0))] {
            var poses = scan.truth
            poses[frame] = Self.rotationAboutHead(scan, degrees: 4, axis: axis) * poses[frame]
            // The surface alone slides along the near-spherical skull and passes the frame ...
            #expect(try RegistrationCheck.verify(frames: frames, poses: poses).passed[frame])
            // ... the face landmarks (5-7 mm off) do not.
            let check = try RegistrationCheck.verify(frames: frames, poses: poses, landmarks: landmarks)
            #expect(!check.passed[frame] && check.landmarkFailed[frame], "#\(frame): \(check.landmarkResiduals)")
            #expect(check.passed.enumerated().allSatisfy { $0.offset == frame || $0.element })
        }
    }

    @Test func landmarksForAnotherFrameCountAreIgnored() throws {
        let scan = PipelineTests.scan
        let frames = try Self.frames(scan)
        for landmarks in [[], Array(try Self.landmarks(scan).dropLast())] {
            let check = try RegistrationCheck.verify(frames: frames, poses: scan.truth, landmarks: landmarks)
            #expect(check.passed.allSatisfy { $0 } && check.landmarkFailed.count == frames.count)
        }
    }

    @Test func landmarkTermOnTheTexturedHead() throws {
        // SyntheticScan: 76 Vision-like landmarks with 1 px noise (also on occluded face parts), 1280x800.
        var o = SyntheticScan.Options()
        o.width = 1280
        o.height = 800
        o.meshResolution = 96
        let result = SyntheticScan.make(o)
        let scan = SynthScan(capture: result.capture, truth: result.groundTruthPoses, center: P3(result.headCenter))
        let frames = try Self.frames(scan), landmarks = try Self.landmarks(scan)
        let correct = try RegistrationCheck.verify(frames: frames, poses: scan.truth, landmarks: landmarks)
        #expect(correct.passed.allSatisfy { $0 }, "\(correct.landmarkResiduals)")
        let left = try #require(scan.capture.keyframes.firstIndex { $0.scanPose == .left })
        var poses = scan.truth
        poses[left] = Self.rotationAboutHead(scan, degrees: 4, axis: SIMD3(0, 1, 0)) * poses[left]
        let check = try RegistrationCheck.verify(frames: frames, poses: poses, landmarks: landmarks)
        print("landmark term (textured head): residuals correct \(correct.landmarkResiduals.map { $0.map { $0 * 1000 } }), "
              + "left #\(left) turned 4 deg \((check.landmarkResiduals[left] ?? .nan) * 1000) mm")
        #expect(!check.passed[left])
        #expect(check.passed.enumerated().allSatisfy { $0.offset == left || $0.element })
    }

    @Test(.enabled(if: CoreAvailability.reconstruction, "the core is a placeholder"))
    func appleVisionLandmarksKeepEveryCorrectKeyframe() async throws {
        // The landmarks and face rectangles of Apple Vision itself on the rendered head (lighting 0.6 as for the
        // synthetic camera). Its landmarks drift with the view angle (left views of this scan 16-23 mm from the
        // reference's at the true poses, against 1-2 mm for the projected model points), so the landmark term, when
        // the pipeline used it, dropped both left views of this scan. The pipeline leaves it off.
        var o = SyntheticCameraSource.defaultOptions
        o.width = 1280
        o.height = 800
        o.seed = 5
        o.poses = ScanPose.required
        let result = SyntheticScan.make(o)
        let tracker = HeadTracker(convention: .documented)
        var keyframes: [Keyframe] = []
        for var kf in result.capture.keyframes {
            let observation = try #require(try tracker.analyze(kf.frame, segmentation: false), "no face in keyframe #\(kf.id)")
            kf.landmarks = observation.landmarks
            kf.faceRect = observation.faceRect
            keyframes.append(kf)
        }
        let out = try await ReconstructionPipeline().run(ScanCapture(keyframes: keyframes))
        #expect(out.report.usedFrames == keyframes.count, "\(out.report.notes)")
    }

    /// IFS_RECON_STUDY=1: the surface check and the landmark term against 2 and 4 degree rotations about the head
    /// center, per axis (docs/RECONSTRUCTION.md, "Registration check").
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func landmarkTermStudy() throws {
        let scan = PipelineTests.scan
        let frames = try Self.frames(scan), landmarks = try Self.landmarks(scan)
        var lines: [String] = []
        let correct = try RegistrationCheck.verify(frames: frames, poses: scan.truth, landmarks: landmarks)
        lines.append("correct: passed \(correct.passed.filter { $0 }.count)/\(correct.passed.count), residuals (mm) "
                     + correct.landmarkResiduals.map { $0.map { String(format: "%.2f", $0 * 1000) } ?? "-" }.joined(separator: " "))
        for (frame, name) in [(7, "left"), (3, "up"), (5, "down"), (9, "right"), (11, "up-left")] {
            for (axisName, axis) in [("yaw", SIMD3<Float>(0, 1, 0)), ("pitch", SIMD3<Float>(1, 0, 0)), ("roll", SIMD3<Float>(0, 0, 1))] {
                for degrees: Float in [2, 4] {
                    var poses = scan.truth
                    poses[frame] = Self.rotationAboutHead(scan, degrees: degrees, axis: axis) * poses[frame]
                    let surface = try RegistrationCheck.verify(frames: frames, poses: poses)
                    let both = try RegistrationCheck.verify(frames: frames, poses: poses, landmarks: landmarks)
                    lines.append(String(format: "%@ #%d %@ %.0f°: surface rmse %.2f mm passed %@; landmark residual %.2f mm, passed %@",
                                        name, frame, axisName, degrees, surface.scores[frame].rmse * 1000,
                                        surface.passed[frame] ? "yes" : "no", (both.landmarkResiduals[frame] ?? .nan) * 1000,
                                        both.passed[frame] ? "yes" : "no"))
                }
            }
        }
        print("landmark term:\n" + lines.joined(separator: "\n"))
    }
}
