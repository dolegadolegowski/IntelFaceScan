// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared scan data types: head pose, target poses, frame quality, accepted keyframes and the scan result that is
// handed to the reconstruction (ReconKit) and saved as a ScanDataset.

import CameraKit
import CoreGraphics
import Foundation
import simd

/// Head orientation relative to the camera, in degrees, defined on the (non-mirrored) color image:
///   * yaw > 0: the face turns so that the nose moves toward the RIGHT edge of the image
///     (= the person turns to THEIR left),
///   * pitch > 0: the face turns up (nose moves toward the top of the image),
///   * roll > 0: the head tilts clockwise in the image.
/// (Vision's own sign conventions are converted to these by the head tracker.)
public struct HeadPose: Codable, Sendable, Equatable, Hashable {
    public var yaw: Float
    public var pitch: Float
    public var roll: Float
    /// Detection confidence in [0, 1].
    public var confidence: Float

    public init(yaw: Float, pitch: Float, roll: Float, confidence: Float = 1) {
        self.yaw = yaw; self.pitch = pitch; self.roll = roll; self.confidence = confidence
    }

    public static let frontal = HeadPose(yaw: 0, pitch: 0, roll: 0)

    /// Rotation R of the head relative to a frontal head, expressed in the camera frame (+X right, +Y down,
    /// +Z forward; a frontal nose points along -Z). A head point p (world = camera frame of the frontal pose)
    /// appears in this frame at R * (p - c) + c', with c the head center, so world_from_camera has rotation R^T.
    /// Poses measured by HeadTracker (Apple Vision) are relative to the line of sight: for a face off the optical axis
    /// they differ from this camera-frame rotation by roughly the face's angular offset (see VisionPoseConvention).
    public var cameraFrameRotation: simd_quatf {
        let deg = Float.pi / 180
        let qYaw = simd_quatf(angle: yaw * deg, axis: SIMD3(0, -1, 0))     // nose toward +X (image right)
        let qPitch = simd_quatf(angle: pitch * deg, axis: SIMD3(-1, 0, 0)) // nose toward -Y (image top)
        let qRoll = simd_quatf(angle: roll * deg, axis: SIMD3(0, 0, 1))    // top of the head toward +X (clockwise)
        return qYaw * qPitch * qRoll
    }
}

/// The guided scan poses, in the order the scan asks for them.
public enum ScanPose: String, Codable, Sendable, CaseIterable, Hashable {
    case front
    case up
    case down
    case left       // the person turns to THEIR left  (yaw > 0)
    case right      // the person turns to THEIR right (yaw < 0)
    case upLeft
    case upRight
    case downLeft
    case downRight

    /// The five required poses.
    public static let required: [ScanPose] = [.front, .up, .down, .left, .right]
    /// Optional diagonal poses.
    public static let diagonals: [ScanPose] = [.upLeft, .upRight, .downLeft, .downRight]

    /// Target head orientation (degrees). Tuned by the scan module; Vision tracks faces reliably up to ~60° yaw.
    public var target: (yaw: Float, pitch: Float) {
        switch self {
        case .front: return (0, 0)
        case .up: return (0, 25)
        case .down: return (0, -25)
        case .left: return (45, 0)
        case .right: return (-45, 0)
        case .upLeft: return (30, 20)
        case .upRight: return (-30, 20)
        case .downLeft: return (30, -20)
        case .downRight: return (-30, -20)
        }
    }
}

/// Quality measures of a candidate frame (computed on the face region).
public struct FrameQuality: Codable, Sendable, Hashable {
    /// Variance of the Laplacian of the gray face region (higher = sharper).
    public var sharpness: Float
    /// Head angular speed estimate in degrees per second (pose change between consecutive frames).
    public var angularSpeedDegPerSec: Float
    /// Fraction of face-region pixels with valid depth, [0, 1].
    public var validDepthRatio: Float
    /// Number of valid depth points inside the head mask.
    public var validDepthPoints: Int
    /// Median depth of the face region in meters.
    public var faceDistanceM: Float
    /// Camera motion from the IMU (gyro magnitude, deg/s); the camera must stay static.
    public var cameraMotionDegPerSec: Float

    public init(sharpness: Float, angularSpeedDegPerSec: Float, validDepthRatio: Float, validDepthPoints: Int,
                faceDistanceM: Float, cameraMotionDegPerSec: Float = 0) {
        self.sharpness = sharpness; self.angularSpeedDegPerSec = angularSpeedDegPerSec
        self.validDepthRatio = validDepthRatio; self.validDepthPoints = validDepthPoints
        self.faceDistanceM = faceDistanceM; self.cameraMotionDegPerSec = cameraMotionDegPerSec
    }
}

/// A frame accepted for reconstruction, with everything the reconstruction needs.
public struct Keyframe: Sendable {
    public var id: Int
    public var frame: CameraFrame
    public var scanPose: ScanPose
    public var headPose: HeadPose
    /// Face rectangle in color image pixels (top-left origin).
    public var faceRect: CGRect
    /// Vision face landmarks in color image pixels (top-left origin), all points of VNFaceLandmarks2D.allPoints.
    public var landmarks: [SIMD2<Float>]
    /// Head segmentation in color resolution: 255 = head (and, optionally, shoulders), 0 = background.
    public var mask: Data?
    public var quality: FrameQuality

    public init(id: Int, frame: CameraFrame, scanPose: ScanPose, headPose: HeadPose, faceRect: CGRect,
                landmarks: [SIMD2<Float>], mask: Data?, quality: FrameQuality) {
        self.id = id; self.frame = frame; self.scanPose = scanPose; self.headPose = headPose
        self.faceRect = faceRect; self.landmarks = landmarks; self.mask = mask; self.quality = quality
    }
}

/// Result of a guided scan: the input of the reconstruction.
public struct ScanCapture: Sendable {
    public var keyframes: [Keyframe]
    public var device: DeviceInfo?
    public var configuration: CaptureConfiguration?
    public var createdAt: Date
    /// Free-form notes (e.g. source recording file).
    public var notes: [String: String]

    public init(keyframes: [Keyframe], device: DeviceInfo? = nil, configuration: CaptureConfiguration? = nil,
                createdAt: Date = Date(), notes: [String: String] = [:]) {
        self.keyframes = keyframes; self.device = device; self.configuration = configuration
        self.createdAt = createdAt; self.notes = notes
    }
}
