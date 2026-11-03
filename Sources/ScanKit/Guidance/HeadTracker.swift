// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: UI/scan module) - Vision face detection, landmarks, head pose and person segmentation.

import CameraKit
import CoreGraphics
import Foundation

public struct HeadObservation: Sendable {
    public var pose: HeadPose
    /// Face rectangle in color image pixels (top-left origin).
    public var faceRect: CGRect
    /// Landmarks in color image pixels (top-left origin).
    public var landmarks: [SIMD2<Float>]
    /// Person/head segmentation at color resolution (255 = person), nil when not computed.
    public var personMask: Data?

    public init(pose: HeadPose, faceRect: CGRect, landmarks: [SIMD2<Float>], personMask: Data?) {
        self.pose = pose; self.faceRect = faceRect; self.landmarks = landmarks; self.personMask = personMask
    }
}

public final class HeadTracker: @unchecked Sendable {
    public init() {}

    /// Detects the most prominent face. Returns nil when no face is visible.
    public func analyze(_ frame: CameraFrame, segmentation: Bool = false) throws -> HeadObservation? { nil }
}
