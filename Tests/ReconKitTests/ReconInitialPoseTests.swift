// License: Apache 2.0. See LICENSE file in root directory.
//
// Initial rotations from the Vision head pose: Vision measures yaw and pitch relative to the line of sight through
// the face, so a face off the image center (a person sitting off the optical axis, or the face moving sideways as
// the head turns about the neck) reads extra yaw / pitch toward that side. InitialPoseEstimator takes the line of
// sight out (docs/RECONSTRUCTION.md, "Initial poses"); SyntheticScan simulates the effect with the gains measured
// on Apple Vision (IFS_RECON_STUDY=1 repeats that measurement).

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

@Suite("Recon initial poses", .heavyReconstruction)
struct ReconInitialPoseTests {
    /// Off-center head, no random Vision error: what remains is the line-of-sight effect.
    static let scan: SyntheticScanResult = {
        var o = SyntheticScan.Options()
        o.width = 1280
        o.height = 800
        o.headOffsetM = SIMD2(0.12, 0.06)
        o.headPoseErrorDeg = 0
        o.meshResolution = 96
        return SyntheticScan.make(o)
    }()

    static func angle(_ a: simd_float3x3, _ b: simd_float3x3) -> Float {
        let q = simd_quatf(a.transpose * b).angle
        return min(q, 2 * .pi - q) * 180 / .pi
    }

    @Test func lineOfSightIsTakenOutOfTheVisionPose() {
        let scan = Self.scan
        var rawYaw: [Float] = [], rawPitch: [Float] = [], yaw: [Float] = [], pitch: [Float] = []
        for (kf, truth) in zip(scan.capture.keyframes, scan.groundTruthHeadPoses) {
            let corrected = InitialPoseEstimator.cameraFramePose(kf)
            rawYaw.append(abs(kf.headPose.yaw - truth.yaw))
            rawPitch.append(abs(kf.headPose.pitch - truth.pitch))
            yaw.append(abs(corrected.yaw - truth.yaw))
            pitch.append(abs(corrected.pitch - truth.pitch))
        }
        func mean(_ v: [Float]) -> Float { v.reduce(0, +) / Float(v.count) }
        print("initial poses: |yaw error| mean \(mean(rawYaw)) -> \(mean(yaw)), max \(rawYaw.max()!) -> \(yaw.max()!); "
              + "|pitch error| mean \(mean(rawPitch)) -> \(mean(pitch)), max \(rawPitch.max()!) -> \(pitch.max()!)")
        // 12 cm right and 6 cm down at ~0.68 m: about 10 and 5 degrees off axis, more for the faces turned right.
        #expect(mean(rawYaw) > 5 && mean(rawPitch) > 2)
        #expect(yaw.max()! < 0.3 && pitch.max()! < 0.3)
    }

    @Test func visionRotationsAndExportFrameImproveOffCenter() throws {
        let scan = Self.scan
        let keyframes = scan.capture.keyframes
        let segments = try keyframes.map { try #require(HeadSegmentation.segment($0, keepShoulders: false)) }
        let truth = scan.groundTruthPoses
        // The Vision-only initial rotation (estimate path 3) relative to the reference, against the truth.
        func visionErrors(_ gain: SIMD2<Float>) -> [Float] {
            let rRef = InitialPoseEstimator.rotation(keyframes[0], lineOfSightGain: gain)
            return keyframes.indices.dropFirst().map { i in
                Self.angle(rRef * InitialPoseEstimator.rotation(keyframes[i], lineOfSightGain: gain).transpose, truth[i].upperLeft3x3)
            }
        }
        let before = visionErrors(.zero), after = visionErrors(InitialPoseEstimator.lineOfSightGain)
        // The export orientation's yaw and pitch (where the mean of all keyframes' Vision rotations, carried into the
        // world, points the face; roll comes from the eye line) against the true face direction, -Z (keyframe 0 is
        // exactly frontal).
        func exportError(_ gain: SIMD2<Float>) -> Float {
            let frame = HeadFrame(reference: keyframes[0], segment: segments[0], keyframes: keyframes, poses: truth, lineOfSightGain: gain)
            return acos(min(1, simd_dot(frame.rotation.columns.2, SIMD3(0, 0, 1)))) * 180 / .pi
        }
        let exportBefore = exportError(.zero), exportAfter = exportError(InitialPoseEstimator.lineOfSightGain)
        func mean(_ v: [Float]) -> Float { v.reduce(0, +) / Float(v.count) }
        print("initial poses: Vision rotation error mean \(mean(before)) -> \(mean(after)), max \(before.max()!) -> \(after.max()!); "
              + "export orientation error \(exportBefore) -> \(exportAfter) deg")
        #expect(mean(before) > 3 && mean(after) < 0.5)
        #expect(exportBefore > 5 && exportAfter < 0.5)
        // The landmark fits, gated by their agreement with the Vision rotation, are not lost.
        let initial = InitialPoseEstimator.estimate(keyframes: keyframes, segments: segments, reference: 0)
        let uncorrected = InitialPoseEstimator.estimate(keyframes: keyframes, segments: segments, reference: 0, lineOfSightGain: .zero)
        let initialErrors = keyframes.indices.dropFirst().map { Self.angle(initial.worldFromCamera[$0].upperLeft3x3, truth[$0].upperLeft3x3) }
        print("initial poses: landmark fits \(uncorrected.kabschFrames) -> \(initial.kabschFrames), errors \(initialErrors)")
        #expect(initial.kabschFrames >= uncorrected.kabschFrames)
    }

    /// IFS_RECON_STUDY=1: Apple Vision on the rendered head (lighting 0.6, 1280x800, 0.6 m) moved off center, at
    /// several head orientations; prints the readings and the least-squares slope of the Vision angle over the
    /// line-of-sight angle of the face rectangle's center for each orientation (the gains above are their medians).
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func visionLineOfSightGainStudy() throws {
        var options = SyntheticCameraSource.defaultOptions
        options.width = 1280
        options.height = 800
        let scene = SyntheticScene(options: options)
        let k = scene.intrinsics
        let tracker = HeadTracker(convention: .documented)
        var index: UInt64 = 0
        var lines: [String] = []
        let toDegrees: Float = 180 / .pi
        func slope(_ xs: [Float], _ ys: [Float]) -> Float {
            let mx = xs.reduce(0, +) / Float(xs.count), my = ys.reduce(0, +) / Float(ys.count)
            let sxy = zip(xs, ys).map { ($0 - mx) * ($1 - my) }.reduce(0, +), sxx = xs.map { ($0 - mx) * ($0 - mx) }.reduce(0, +)
            return sxy / sxx
        }
        for (yaw, pitch) in [(Float(0), Float(0)), (20, 0), (-20, 0), (0, 15), (0, -15), (40, 0), (-40, 0)] {
            for (axis, offsets) in [("x", [Float(-0.12), -0.06, 0, 0.06, 0.12]), ("y", [Float(-0.09), -0.045, 0, 0.045, 0.09])] {
                var angles: [Float] = [], readings: [Float] = []
                for d in offsets {
                    index += 1
                    let shift = axis == "x" ? SIMD3<Float>(d, 0, 0) : SIMD3<Float>(0, d, 0)
                    let m = scene.cameraFromWorld(headPose: HeadPose(yaw: yaw, pitch: pitch, roll: 0), translation: shift)
                    let view = scene.render(cameraFromWorld: m, frameIndex: index, sequence: index, timestampMs: 1000 + Double(index))
                    guard let o = try tracker.analyze(view.frame, segmentation: false) else { continue }
                    let a = axis == "x" ? atan((Float(o.faceRect.midX) - k.ppx) / k.fx) : atan((Float(o.faceRect.midY) - k.ppy) / k.fy)
                    angles.append(a * toDegrees)
                    readings.append(axis == "x" ? o.pose.yaw : -o.pose.pitch)
                }
                let pairs = zip(angles, readings).map { String(format: "%.1f:%.1f", $0.0, $0.1) }.joined(separator: " ")
                lines.append(String(format: "| %.0f / %.0f | %@ | %@ | %.2f |", yaw, pitch, axis == "x" ? "yaw" : "-pitch", pairs,
                                    angles.count >= 3 ? slope(angles, readings) : .nan))
            }
        }
        print("vision line of sight (true yaw / pitch | reading | line-of-sight angle:reading | slope):\n" + lines.joined(separator: "\n"))
    }
}
