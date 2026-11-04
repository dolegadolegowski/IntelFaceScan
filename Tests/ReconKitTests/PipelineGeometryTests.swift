// License: Apache 2.0. See LICENSE file in root directory.

import CoreGraphics
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

@Suite("Pipeline geometry")
struct PipelineGeometryTests {
    static let truth: simd_float4x4 = {
        let r = simd_float3x3(simd_quatf(angle: 0.6, axis: simd_normalize(SIMD3<Float>(0.2, 1, 0.1))))
        return simd_float4x4(rotation: r, translation: SIMD3(0.12, -0.01, 0.05))
    }()

    /// Landmark-like points on a face-sized patch 0.6 m in front of the camera.
    static func facePoints(count: Int, seed: UInt64) -> [SIMD3<Float>] {
        var rng = Rng(s: seed)
        return (0 ..< count).map { _ in
            SIMD3(Float(0.1 * (rng.uniform() - 0.5)), Float(0.12 * (rng.uniform() - 0.5)), Float(0.55 + 0.04 * rng.uniform()))
        }
    }

    @Test func hornRecoversARigidTransform() throws {
        let frame = Self.facePoints(count: 12, seed: 1)
        let reference = frame.map { InitialPoseEstimator.transform(Self.truth, $0) }
        let fit = try #require(InitialPoseEstimator.horn(Array(zip(reference, frame))))
        for c in 0 ..< 4 {
            #expect(simd_length(fit[c] - Self.truth[c]) < 1e-4)
        }
    }

    @Test func landmarkKabschIsRobustAndGuardedByVision() throws {
        var rng = Rng(s: 2)
        let frame = Self.facePoints(count: 23, seed: 3)
        var reference: [SIMD3<Float>?] = frame.map { p in
            InitialPoseEstimator.transform(Self.truth, p)
                + SIMD3(Float(0.0008 * rng.gaussian()), Float(0.0008 * rng.gaussian()), Float(0.0008 * rng.gaussian()))
        }
        // Occluded landmarks: Vision reports them, the depth belongs to another surface.
        for k in [2, 7, 11, 19] { reference[k]! += SIMD3(0, 0, 0.03) }
        reference[5] = nil
        let vision = Self.truth.upperLeft3x3 * simd_float3x3(simd_quatf(angle: 0.08, axis: SIMD3(0, 0, 1)))
        let fit = try #require(InitialPoseEstimator.landmarkKabsch(reference: reference, frame: frame, visionRotation: vision))
        let dr = simd_quatf(fit.upperLeft3x3.transpose * Self.truth.upperLeft3x3).angle
        #expect(min(dr, 2 * .pi - dr) * 180 / .pi < 1.5)
        let center = SIMD3<Float>(0, 0, 0.57)
        #expect(simd_length(InitialPoseEstimator.transform(fit, center) - InitialPoseEstimator.transform(Self.truth, center)) < 0.0015)
        // A fit 20 degrees away from Vision is not trusted.
        let farVision = Self.truth.upperLeft3x3 * simd_float3x3(simd_quatf(angle: 0.35, axis: SIMD3(0, 1, 0)))
        #expect(InitialPoseEstimator.landmarkKabsch(reference: reference, frame: frame, visionRotation: farVision) == nil)
        // Too few landmarks with depth.
        let sparse = reference.enumerated().map { $0.offset < 6 ? $0.element : nil }
        #expect(InitialPoseEstimator.landmarkKabsch(reference: sparse, frame: frame, visionRotation: vision) == nil)
    }

    @Test func imageRollFromSymmetricLandmarks() {
        let base: [SIMD2<Float>] = [(-48, -18), (-32, -20), (-16, -18), (16, -18), (32, -20), (48, -18), (-40, -35), (40, -35),
                                    (0, 0), (0, 12), (-25, 52), (25, 52), (0, 58), (-50, 65), (50, 65), (0, 97)].map { SIMD2($0.0, $0.1) }
        let angle: Float = 8 * .pi / 180
        let rotated = base.map { p in SIMD2(cos(angle) * p.x - sin(angle) * p.y, sin(angle) * p.x + cos(angle) * p.y) * 3 + SIMD2(640, 400) }
        let (roll, fromLandmarks) = HeadSegmentation.estimateImageRoll(landmarks: rotated, fallbackDegrees: 0)
        #expect(fromLandmarks)
        #expect(abs(roll - angle) < 0.5 * .pi / 180)
        let (fallback, used) = HeadSegmentation.estimateImageRoll(landmarks: Array(rotated.prefix(5)), fallbackDegrees: 3)
        #expect(!used && abs(fallback - 3 * .pi / 180) < 1e-6)
    }

    @Test func imageRollOfVisionLikeConstellations() {
        // SyntheticScan's 76 Vision-like landmarks of a near-frontal head (yaw and pitch +-3 deg, roll +-10 deg) with
        // 1 px noise at 1280x800. With hard limits for the upper-left / upper-right sets, the jaw contour points at
        // eye height switched sets: mean 1.38 deg, p95 3.47 deg error (the export frame of `ifscan synth`'s default
        // scan was rolled by 3.1 deg); with ramped weights 0.47 / 1.08 deg.
        var o = SyntheticScan.Options()
        o.width = 1280
        o.height = 800
        o.meshResolution = 48
        let scene = SyntheticScene(options: o)
        var rng = Rng(s: 7)
        var errors: [Float] = []
        for _ in 0 ..< 200 {
            let truth = HeadPose(yaw: Float(6 * (rng.uniform() - 0.5)), pitch: Float(6 * (rng.uniform() - 0.5)),
                                 roll: Float(20 * (rng.uniform() - 0.5)))
            let landmarks = scene.landmarks(cameraFromWorld: scene.cameraFromWorld(headPose: truth))
                .map { $0 + SIMD2(Float(rng.gaussian()), Float(rng.gaussian())) }
            let (roll, _) = HeadSegmentation.estimateImageRoll(landmarks: landmarks, fallbackDegrees: truth.roll + Float(2 * rng.gaussian()))
            errors.append(abs(roll * 180 / .pi - truth.roll))
        }
        errors.sort()
        let mean = errors.reduce(0, +) / Float(errors.count), p95 = errors[Int(0.95 * Float(errors.count - 1))]
        #expect(mean < 0.7 && p95 < 1.4, "roll error mean \(mean), p95 \(p95) deg")
    }

    @Test func robustMeanIgnoresOutlyingRotations() throws {
        var qs = (0 ..< 9).map { k in simd_quatf(angle: 0.02 * Float(k % 3 - 1), axis: SIMD3(0, 1, 0)) }
        qs.append(simd_quatf(angle: 0.6, axis: SIMD3(1, 0, 0)))
        qs.append(simd_quatf(vector: -qs[0].vector))   // the same rotation, opposite quaternion sign
        let mean = try #require(HeadFrame.robustMean(qs))
        let angle = mean.angle
        #expect(min(angle, 2 * .pi - angle) < 0.01)
    }

    @Test func cropFallbackKeepsTrianglesAboveThePlane() {
        // Two triangles: one above y = 0 (head up = -Y in the camera convention), one below.
        let mesh = ReconMesh(positions: [SIMD3(0, -1, 0), SIMD3(1, -1, 0), SIMD3(0, -2, 0), SIMD3(0, 1, 0), SIMD3(1, 1, 0), SIMD3(0, 2, 0)],
                             normals: Array(repeating: SIMD3(0, 0, -1), count: 6), colors: Array(repeating: SIMD3(1, 2, 3), count: 6),
                             indices: [0, 1, 2, 3, 4, 5])
        let cropped = PipelineRun.cropFallback(mesh, point: .zero, normal: SIMD3(0, -1, 0))
        #expect(cropped.triangleCount == 1 && cropped.vertexCount == 3)
        #expect(cropped.positions.allSatisfy { $0.y < 0 })
        #expect(cropped.normals.count == 3 && cropped.colors.count == 3 && cropped.uvs.isEmpty)
    }

    @Test func frameWeightsFollowSharpness() {
        func keyframe(_ sharpness: Float) -> Keyframe {
            let scan = PipelineTests.scan.capture.keyframes[0]
            var k = scan
            k.quality.sharpness = sharpness
            return k
        }
        let w = PipelineRun.frameWeights([keyframe(100), keyframe(50), keyframe(.nan)])
        #expect(w == [1, 0.8, 0.6])
    }
}
