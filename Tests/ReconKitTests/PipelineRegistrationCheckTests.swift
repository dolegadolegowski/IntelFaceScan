// License: Apache 2.0. See LICENSE file in root directory.

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
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
}
