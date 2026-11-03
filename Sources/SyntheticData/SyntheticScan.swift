// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests/data module) - procedurally rendered head scans with ground truth.

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd

public struct SyntheticScanResult: Sendable {
    public var capture: ScanCapture
    /// Ground-truth world_from_camera per keyframe.
    public var groundTruthPoses: [simd_float4x4]
    /// Ground-truth head surface in the world frame (meters).
    public var groundTruthMesh: ReconMesh
}

public enum SyntheticScan {
    public struct Options: Sendable {
        public var width = 640
        public var height = 400
        public var distanceM: Float = 0.6
        /// Depth noise sigma at 1 m (scaled with z^2), meters.
        public var depthNoiseAt1m: Float = 0.002
        public var poses: [ScanPose] = ScanPose.required
        public var framesPerPose = 2
        public var seed: UInt64 = 1
        public init() {}
    }

    public static func make(_ options: Options = Options()) -> SyntheticScanResult {
        SyntheticScanResult(capture: ScanCapture(keyframes: []), groundTruthPoses: [], groundTruthMesh: ReconMesh())
    }
}
