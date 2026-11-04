// License: Apache 2.0. See LICENSE file in root directory.
//
// What the guided scan tells the user (typed, localized by the UI; English text for the CLI), the coverage map
// math and the auto-exposure ROI policy.

import CoreGraphics
import Foundation
import simd

/// Why a frame was not accepted as a keyframe.
public enum ScanRejection: String, Sendable, CaseIterable, Codable, Hashable {
    case noFace
    case lowConfidence
    case trackingFailed
    case tooClose
    case tooFar
    case noDepth
    case headMoving
    case cameraMoving
    case rollTooLarge
    case blurry
    case lowDepthCoverage
    case tooFewDepthPoints

    public var englishText: String {
        switch self {
        case .noFace: return "No face detected"
        case .lowConfidence: return "Face detection is uncertain"
        case .trackingFailed: return "Face tracking failed"
        case .tooClose: return "Too close"
        case .tooFar: return "Too far"
        case .noDepth: return "No depth on the face"
        case .headMoving: return "Head is moving"
        case .cameraMoving: return "Camera moved"
        case .rollTooLarge: return "Head is tilted sideways"
        case .blurry: return "Image is blurry"
        case .lowDepthCoverage: return "Depth has holes on the face"
        case .tooFewDepthPoints: return "Too few depth points on the head"
        }
    }
}

/// Which way to correct a turn that is not within a pose's tolerance (ScanUpdate.turnHint): further toward the target,
/// back toward the front (turned too far), or across the pose direction. Person-relative, like the poses: faceLeft is
/// the person's left (HeadPose yaw > 0).
public enum TurnHint: String, Sendable, CaseIterable, Codable {
    case further, back, chinUp, chinDown, faceLeft, faceRight

    public var englishText: String {
        switch self {
        case .further: return "Turn further"
        case .back: return "Too far: turn back a little"
        case .chinUp: return "Raise your chin a little"
        case .chinDown: return "Lower your chin a little"
        case .faceLeft: return "Turn your face a little to your left"
        case .faceRight: return "Turn your face a little to your right"
        }
    }

    /// The hint for a remaining turn `delta` (target - current, yaw/pitch degrees) toward `pose`: the component along
    /// the pose direction against the one across it (scaled by along / across, like the tolerance ellipse). The front
    /// pose and an along component that dominates give further (or back when negative).
    public static func hint(pose: ScanPose, delta: SIMD2<Float>, along: Float, across: Float) -> TurnHint {
        let target = SIMD2(pose.target.yaw, pose.target.pitch)
        let magnitude = simd_length(target)
        guard magnitude > 0, along > 0, across > 0 else { return .further }
        let direction = target / magnitude
        let a = simd_dot(delta, direction)
        let p = delta - a * direction
        if abs(a) >= simd_length(p) * along / across { return a >= 0 ? .further : .back }
        if abs(p.y) >= abs(p.x) { return p.y > 0 ? .chinUp : .chinDown }
        return p.x > 0 ? .faceLeft : .faceRight
    }
}

/// What the user should do now.
public enum ScanInstruction: Sendable, Equatable {
    case idle
    case showFace
    case moveCloser
    case moveBack
    /// Positioning: face found at a good distance, hold still facing the camera.
    case lookAtCamera
    /// Turn the head toward the pose target; deltas = target - current, degrees, HeadPose convention
    /// (yawDelta > 0: turn so the nose moves toward the image right, pitchDelta > 0: turn the face up).
    case turn(ScanPose, yawDelta: Float, pitchDelta: Float)
    /// After the front pose, the head is turned toward no unfinished pose (ScanUpdate.activePose is nil): turn it
    /// toward any of them (ScanUpdate.unfinishedPoses), in any order.
    case turnAnyDirection
    case straightenHead
    /// Within tolerance: hold still until frames are accepted.
    case holdStill(ScanPose)
    /// Accepting keyframes for the pose.
    case capturing(ScanPose)
    case finished

    public var englishText: String {
        switch self {
        case .idle: return ""
        case .showFace: return "Look at the camera"
        case .moveCloser: return "Move closer"
        case .moveBack: return "Move back"
        case .lookAtCamera: return "Hold still, looking at the camera"
        case .turn(let pose, _, _): return "Turn your head: \(pose.englishName)"
        case .turnAnyDirection: return "Turn your head in any direction"
        case .straightenHead: return "Keep your head straight"
        case .holdStill(let pose): return "Hold still (\(pose.englishName))"
        case .capturing(let pose): return "Capturing \(pose.englishName)"
        case .finished: return "Scan complete"
        }
    }
}

extension ScanPose {
    public var englishName: String {
        switch self {
        case .front: return "front"
        case .up: return "up"
        case .down: return "down"
        case .left: return "left"
        case .right: return "right"
        case .upLeft: return "up-left"
        case .upRight: return "up-right"
        case .downLeft: return "down-left"
        case .downRight: return "down-right"
        }
    }
}

public enum ScanGuidance {
    /// Unit vector on screen (x right, y down) of the direction the user should turn, for a yaw/pitch delta in
    /// the HeadPose convention. Mirrored display: the nose moving toward the image right appears moving left.
    public static func screenDirection(yawDelta: Float, pitchDelta: Float, mirrored: Bool) -> SIMD2<Float> {
        let v = SIMD2(mirrored ? -yawDelta : yawDelta, -pitchDelta)
        let length = simd_length(v)
        return length > 1e-3 ? v / length : .zero
    }

    /// Screen position (x right, y down, in [-1, 1]) of the head region that a pose shows to the camera, as seen
    /// in the (optionally mirrored) preview: turning to the image right shows the side of the head that appears
    /// on the image left, turning up shows the chin.
    public static func coveredRegionPosition(yaw: Float, pitch: Float, mirrored: Bool) -> SIMD2<Float> {
        let x = max(-1, min(1, -yaw / 90))
        let y = max(-1, min(1, pitch / 60))
        return SIMD2(mirrored ? -x : x, y)
    }
}

extension CoverageMap {
    /// Gaussian footprint of one keyframe on the yaw/pitch grid: a view sees a broad part of the head.
    public static let footprintSigmaDegrees: Float = 15

    public static func index(yawBin: Int, pitchBin: Int) -> Int { pitchBin * yawBins + yawBin }

    /// Center (degrees) of a bin: yaw bins span [-90, 90] left to right, pitch bins [-60, 60] bottom to top.
    public static func binCenter(yawBin: Int, pitchBin: Int) -> (yaw: Float, pitch: Float) {
        (-90 + (Float(yawBin) + 0.5) * 180 / Float(yawBins), -60 + (Float(pitchBin) + 0.5) * 120 / Float(pitchBins))
    }

    public static func bin(yaw: Float, pitch: Float) -> (yawBin: Int, pitchBin: Int) {
        let yb = Int(((yaw + 90) / 180 * Float(yawBins)).rounded(.down))
        let pb = Int(((pitch + 60) / 120 * Float(pitchBins)).rounded(.down))
        return (max(0, min(yawBins - 1, yb)), max(0, min(pitchBins - 1, pb)))
    }

    public func value(yawBin: Int, pitchBin: Int) -> Float {
        let i = Self.index(yawBin: yawBin, pitchBin: pitchBin)
        return i >= 0 && i < values.count ? values[i] : 0
    }

    /// Coverage from the head poses of accepted keyframes: 1 - exp(-sum of footprints / k), with k such that
    /// `framesPerPose` frames at one pose cover its own bin to 95 %.
    public init(keyframePoses: [HeadPose], framesPerPose: Int) {
        self.init()
        guard !keyframePoses.isEmpty else { return }
        let k = Float(max(1, framesPerPose)) / 3
        let twoSigma2 = 2 * Self.footprintSigmaDegrees * Self.footprintSigmaDegrees
        for pb in 0 ..< Self.pitchBins {
            for yb in 0 ..< Self.yawBins {
                let c = Self.binCenter(yawBin: yb, pitchBin: pb)
                var sum: Float = 0
                for p in keyframePoses {
                    let dy = p.yaw - c.yaw, dp = p.pitch - c.pitch
                    sum += exp(-(dy * dy + dp * dp) / twoSigma2)
                }
                values[Self.index(yawBin: yb, pitchBin: pb)] = 1 - exp(-sum / k)
            }
        }
    }

    /// Mean coverage of the bins inside the yaw/pitch range a guided scan aims for (|yaw| <= 60, |pitch| <= 30).
    public var scanAreaCoverage: Float {
        var sum: Float = 0, n: Float = 0
        for pb in 0 ..< Self.pitchBins {
            for yb in 0 ..< Self.yawBins {
                let c = Self.binCenter(yawBin: yb, pitchBin: pb)
                guard abs(c.yaw) <= 60, abs(c.pitch) <= 30 else { continue }
                sum += value(yawBin: yb, pitchBin: pb); n += 1
            }
        }
        return n > 0 ? sum / n : 0
    }
}

/// Decides when to move the camera's auto-exposure ROI to the detected face: at most once per `minInterval`
/// and only when the face moved noticeably, because every ROI change is a USB control transfer and makes the
/// exposure hunt.
public struct ExposureROIPolicy: Sendable {
    public var minInterval: Double
    /// Update when the overlap (intersection over union) with the last ROI falls below this.
    public var maxOverlap: CGFloat
    /// The ROI is the face rectangle scaled by this factor (forehead and cheeks in the metering).
    public var expansion: CGFloat
    private var last: (rect: CGRect, time: Double)?

    public init(minInterval: Double = 1.0, maxOverlap: CGFloat = 0.7, expansion: CGFloat = 1.2) {
        self.minInterval = minInterval; self.maxOverlap = maxOverlap; self.expansion = expansion
    }

    /// The new ROI (color pixels) to send now, or nil.
    public mutating func roi(for faceRect: CGRect?, imageSize: CGSize, now: Double) -> CGRect? {
        guard let face = faceRect, face.width > 4, face.height > 4, imageSize.width > 0, imageSize.height > 0 else { return nil }
        let candidate = face.scaledAboutCenter(expansion, expansion)
            .intersection(CGRect(origin: .zero, size: imageSize)).integral
        guard !candidate.isNull, candidate.width > 4, candidate.height > 4 else { return nil }
        if let last {
            guard now - last.time >= minInterval || now < last.time else { return nil }
            if Self.overlap(last.rect, candidate) >= maxOverlap { return nil }
        }
        last = (candidate, now)
        return candidate
    }

    public mutating func reset() { last = nil }

    static func overlap(_ a: CGRect, _ b: CGRect) -> CGFloat {
        let i = a.intersection(b)
        guard !i.isNull else { return 0 }
        let inter = i.width * i.height
        let union = a.width * a.height + b.width * b.height - inter
        return union > 0 ? inter / union : 0
    }
}
