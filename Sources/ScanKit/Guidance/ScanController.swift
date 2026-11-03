// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: UI/scan module) - the guided scan state machine (poses, stability, quality, coverage).
// Usable without UI: the CLI runs it over a recording with ScanConfiguration.offline.

import CameraKit
import CoreGraphics
import Foundation

public struct ScanConfiguration: Sendable, Hashable {
    public var poses: [ScanPose] = ScanPose.required
    /// Keyframes accepted per pose.
    public var framesPerPose: Int = 3
    /// Maximum angular distance (degrees) from a pose's target.
    public var angleTolerance: Float = 8
    /// The pose must be held (within tolerance, slow motion) this long before frames are accepted.
    public var holdSeconds: Double = 0.4
    public var minSharpness: Float = 0
    public var maxAngularSpeedDegPerSec: Float = 20
    public var minValidDepthRatio: Float = 0.6
    public var minValidDepthPoints: Int = 20_000
    /// Recommended face distance for the current camera configuration.
    public var distanceRangeM: ClosedRange<Float> = 0.55 ... 0.75
    public var keepShoulders: Bool = false
    /// false: offline processing of a recording (no wall-clock timing; frame timestamps only).
    public var realTime: Bool = true

    public init() {}

    public static var offline: ScanConfiguration {
        var c = ScanConfiguration()
        c.realTime = false
        return c
    }
}

public enum DistanceAdvice: String, Sendable {
    case unknown, tooClose, ok, tooFar
}

public enum ScanPhase: Sendable, Equatable {
    case idle
    /// Waiting for a face at the right distance.
    case positioning
    case capturing(ScanPose)
    case finished
    case failed(String)
}

/// Coverage of head orientations: bins over yaw [-90, 90] x pitch [-60, 60], values in [0, 1].
public struct CoverageMap: Sendable, Equatable {
    public static let yawBins = 12, pitchBins = 8
    public var values: [Float] = Array(repeating: 0, count: yawBins * pitchBins)
    public init() {}
}

public struct ScanUpdate: Sendable {
    public var phase: ScanPhase = .idle
    public var observation: HeadObservation?
    public var distanceM: Float?
    public var distanceAdvice: DistanceAdvice = .unknown
    /// Per pose: fraction of the required keyframes accepted.
    public var poseProgress: [ScanPose: Float] = [:]
    public var overallProgress: Float = 0
    public var quality: FrameQuality?
    public var rejectionReason: String?
    public var acceptedKeyframe: Bool = false
    public var coverage = CoverageMap()
    /// Short instruction for the user (localized by the UI).
    public var instruction: String = ""
    public init() {}
}

public final class ScanController: @unchecked Sendable {
    public let configuration: ScanConfiguration

    public init(configuration: ScanConfiguration = ScanConfiguration()) {
        self.configuration = configuration
    }

    public func start(device: DeviceInfo?, captureConfiguration: CaptureConfiguration?) {}
    /// Processes one frame; returns the state to show.
    public func process(_ frame: CameraFrame) async -> ScanUpdate { ScanUpdate() }
    /// Skips the current (optional) pose.
    public func skipPose() {}
    public func cancel() {}
    /// The accepted keyframes so far.
    public func finish() -> ScanCapture { ScanCapture(keyframes: []) }
}
