// License: Apache 2.0. See LICENSE file in root directory.
//
// The guided scan state machine: positioning -> for each pose: instruct, wait until the head is within tolerance
// of the target and held still, accept spaced keyframes that pass the quality gates. Usable without UI: the CLI
// runs it over a recording with ScanConfiguration.offline (frame timestamps only, no wall clock).

import CameraKit
import CoreGraphics
import Foundation
import simd

public struct ScanConfiguration: Sendable, Hashable {
    public var poses: [ScanPose] = ScanPose.required
    /// Keyframes accepted per pose.
    public var framesPerPose: Int = 3
    /// Maximum angular distance (degrees) from a pose's target, along the direction of the target (the front pose:
    /// in every direction).
    public var angleTolerance: Float = 8
    /// Maximum deviation (degrees) across the direction of a pose's target: pitch on the side poses, yaw on up and
    /// down. A side pose a little up or down still shows that side of the head (the keyframe keeps its measured
    /// pose), and Vision reads large yaw partly as pitch: the synthetic head at yaw +44..+52 reads pitch +4..+10 at
    /// 640x400 (mean ~+8), which an 8-degree circle rarely accepted. The tolerance is the ellipse of both semi-axes.
    public var perpendicularToleranceDeg: Float = 12
    /// Targets of the other poses are relative to the person's own frontal pose (mean of the front keyframes):
    /// Vision's yaw/pitch of a face looking straight at the camera is biased per person by a few degrees.
    public var targetsRelativeToFront: Bool = true
    /// Largest frontal offset (degrees) that is applied to the other targets.
    public var maxFrontOffsetDeg: Float = 10
    /// Turning further than a pose's target (away from the front) is accepted up to this many degrees beyond it:
    /// more rotation only adds coverage.
    public var overshootToleranceDeg: Float = 15
    /// The pose must be held (within tolerance, slow motion) this long before frames are accepted.
    public var holdSeconds: Double = 0.4
    /// Absolute minimum variance of Laplacian of the face (0 = off; the relative gate below adapts to the scene).
    public var minSharpness: Float = 0
    public var maxAngularSpeedDegPerSec: Float = 20
    public var minValidDepthRatio: Float = 0.6
    /// Valid depth points required inside the head mask, for a 1280x800 color image; scaled with the image area.
    public var minValidDepthPoints: Int = 12_000
    /// Recommended face distance for the current camera configuration.
    public var distanceRangeM: ClosedRange<Float> = 0.55 ... 0.75
    public var keepShoulders: Bool = false
    /// false: offline processing of a recording (no wall-clock timing; frame timestamps only).
    public var realTime: Bool = true

    /// start() replaces distanceRangeM by WorkingDistance.recommendedRange when it gets a device or a capture
    /// configuration.
    public var autoDistanceRange: Bool = true
    /// The face is "out of range" during capture only beyond the distance range widened by this (meters), so that
    /// turning the head does not interrupt a pose.
    public var distanceSlackM: Float = 0.03
    /// A candidate is blurry below this fraction of the recent sharpness of the same pose (0 = off).
    public var minRelativeSharpness: Float = 0.6
    /// Maximum camera rotation speed from the IMU gyro (deg/s); the camera has to stand still.
    public var maxCameraMotionDegPerSec: Float = 4
    /// Minimum time between two keyframes of a pose (independent depth noise, slight pose variation).
    public var minKeyframeInterval: Double = 0.2
    /// Larger head tilt (roll) makes the user straighten the head first.
    public var maxRollDegrees: Float = 25
    /// Face found at a good distance for this long before the first pose starts.
    public var positioningHoldSeconds: Double = 0.5
    /// Vision face confidence below this counts as no face.
    public var minFaceConfidence: Float = 0.5
    /// Run Vision person segmentation for keyframe masks (else a face-rectangle ellipse is used).
    public var useSegmentation: Bool = true
    /// Head mask (depth band, neck cut, erosion); keepShoulders above overrides mask.keepShoulders.
    public var mask = HeadMaskParameters()
    /// Poses skipPose() may skip.
    public var optionalPoses: Set<ScanPose> = Set(ScanPose.diagonals)
    /// Consecutive tracker errors after which the scan fails.
    public var maxConsecutiveTrackerErrors: Int = 30

    /// Color pixel count minValidDepthPoints is defined for.
    public static let referencePixelCount = 1280 * 800

    public init() {}

    public static var offline: ScanConfiguration {
        var c = ScanConfiguration()
        c.realTime = false
        return c
    }

    /// minValidDepthPoints for a color image of the given size.
    public func minValidDepthPoints(width: Int, height: Int) -> Int {
        guard minValidDepthPoints > 0, width > 0, height > 0 else { return 0 }
        let scale = Double(width * height) / Double(Self.referencePixelCount)
        return max(1, Int((Double(minValidDepthPoints) * scale).rounded()))
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
/// values[pitchBin * yawBins + yawBin]; yaw bin 0 is yaw -90, pitch bin 0 is pitch -60 (see ScanGuidance.swift).
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

    /// Typed instruction (the UI localizes this; `instruction` is its English text).
    public var instructionKind: ScanInstruction = .idle
    /// Typed rejection (`rejectionReason` is its English text).
    public var rejection: ScanRejection?
    /// The frame this update belongs to.
    public var sequence: UInt64 = 0
    public var timestampMs: Double = 0
    /// Head pose averaged over the last ~0.25 s (what the target check uses).
    public var smoothedPose: HeadPose?
    public var angularSpeedDegPerSec: Float?
    /// Remaining angular distance to the current pose target (degrees), with the component across the pose direction
    /// scaled by angleTolerance / perpendicularToleranceDeg: the pose is within tolerance when this is at most
    /// angleTolerance (what the UI compares).
    public var angularErrorDeg: Float?
    /// Target - current (yaw, pitch) in degrees.
    public var targetDelta: SIMD2<Float>?
    /// Fraction of holdSeconds the pose has been held.
    public var holdProgress: Float = 0
    /// Distance range the scan uses (recommended for the camera configuration).
    public var distanceRangeM: ClosedRange<Float>?
    /// True when the distance is an estimate from the face size (no valid depth on the face).
    public var distanceIsEstimate: Bool = false
    public var keyframeCount: Int = 0
    public var skippedPoses: Set<ScanPose> = []
    public var canSkipPose: Bool = false
    /// Index of the current pose in configuration.poses (nil outside capture).
    public var poseIndex: Int?
    /// The keyframe accepted with this frame.
    public var keyframeID: Int?

    public init() {}
}

public final class ScanController: @unchecked Sendable {
    public let configuration: ScanConfiguration
    private let tracker: HeadTracking

    // Serializes process(): the per-frame working state below is only touched while it is held.
    private let processingLock = NSLock()
    private var work = WorkState()

    // Guards `shared`, which skip/cancel/finish change from other threads (e.g. the main actor) while a frame is
    // being analyzed; process() commits its result only when the generation did not change meanwhile.
    private let stateLock = NSLock()
    private var shared = SharedState()

    public init(configuration: ScanConfiguration = ScanConfiguration()) {
        self.configuration = configuration
        self.tracker = HeadTracker()
    }

    /// - Parameter tracker: face tracker (tests inject scripted observations).
    public init(configuration: ScanConfiguration, tracker: HeadTracking) {
        self.configuration = configuration
        self.tracker = tracker
    }

    // MARK: - Control

    public func start(device: DeviceInfo?, captureConfiguration: CaptureConfiguration?) {
        stateLock.withLock {
            let generation = shared.generation &+ 1
            shared = SharedState()
            shared.generation = generation
            setCamera(device: device, captureConfiguration: captureConfiguration)
            shared.phase = configuration.poses.isEmpty ? .finished : .positioning
        }
    }

    /// Sets the camera description (and with it the distance range) without starting a scan: an idle controller
    /// still tracks the face and reports the distance advice, which the preview shows.
    public func prepare(device: DeviceInfo?, captureConfiguration: CaptureConfiguration?) {
        stateLock.withLock { setCamera(device: device, captureConfiguration: captureConfiguration) }
    }

    private func setCamera(device: DeviceInfo?, captureConfiguration: CaptureConfiguration?) {
        shared.device = device
        shared.captureConfiguration = captureConfiguration
        if configuration.autoDistanceRange && (device != nil || captureConfiguration != nil) {
            shared.distanceRange = WorkingDistance.recommendedRange(device: device, configuration: captureConfiguration)
        } else {
            shared.distanceRange = configuration.distanceRangeM
        }
    }

    /// Processes one frame; returns the state to show.
    public func process(_ frame: CameraFrame) async -> ScanUpdate {
        processingLock.withLock { processLocked(frame) }
    }

    /// Skips the current pose when it is optional (or `allowRequired`). Returns whether it was skipped.
    @discardableResult
    public func skipPose(allowRequired: Bool = false) -> Bool {
        stateLock.withLock {
            guard case .capturing(let pose) = shared.phase,
                  allowRequired || configuration.optionalPoses.contains(pose) else { return false }
            shared.skipped.insert(pose)
            shared.generation &+= 1
            shared.phase = nextPhase(after: shared)
            return true
        }
    }

    /// Stops the scan and discards the keyframes (the camera description is kept).
    public func cancel() {
        stateLock.withLock {
            let previous = shared
            shared = SharedState()
            shared.generation = previous.generation &+ 1
            shared.device = previous.device
            shared.captureConfiguration = previous.captureConfiguration
            shared.distanceRange = previous.distanceRange
        }
    }

    /// The accepted keyframes so far; ends the scan.
    public func finish() -> ScanCapture {
        stateLock.withLock {
            switch shared.phase {
            case .positioning, .capturing:
                shared.phase = .finished
                shared.generation &+= 1
            default: break
            }
            var notes: [String: String] = [:]
            if !shared.skipped.isEmpty {
                notes["skipped_poses"] = configuration.poses.filter { shared.skipped.contains($0) }
                    .map(\.rawValue).joined(separator: ",")
            }
            notes["frames_per_pose"] = String(configuration.framesPerPose)
            return ScanCapture(keyframes: shared.keyframes, device: shared.device,
                               configuration: shared.captureConfiguration, createdAt: Date(), notes: notes)
        }
    }

    public var phase: ScanPhase { stateLock.withLock { shared.phase } }
    public var keyframeCount: Int { stateLock.withLock { shared.keyframes.count } }
    /// The distance range in use (recommended range after start()).
    public var distanceRange: ClosedRange<Float> { stateLock.withLock { shared.distanceRange } }

    public var canSkipCurrentPose: Bool {
        stateLock.withLock {
            guard case .capturing(let pose) = shared.phase else { return false }
            return configuration.optionalPoses.contains(pose)
        }
    }

    // MARK: - State

    private struct SharedState {
        var generation: UInt64 = 0
        var phase: ScanPhase = .idle
        var keyframes: [Keyframe] = []
        var accepted: [ScanPose: Int] = [:]
        var skipped: Set<ScanPose> = []
        var device: DeviceInfo?
        var captureConfiguration: CaptureConfiguration?
        var distanceRange: ClosedRange<Float> = 0.55 ... 0.75
        var trackerErrors = 0
    }

    private struct WorkState {
        var generation: UInt64 = .max
        var poseHistory = PoseHistory()
        var gyro = GyroHistory()
        var distance = DistanceClassifier(range: 0.55 ... 0.75)
        var sharpness = SharpnessReference()
        var holdSince: Double?
        var positioningSince: Double?
        var lastAccepted: Double?
        var lastTime: Double?
        var poseInProgress: ScanPose?

        mutating func resetPose() {
            holdSince = nil; positioningSince = nil; lastAccepted = nil
            sharpness.reset()
        }

        mutating func resetTiming() {
            resetPose()
            poseHistory.reset(); gyro.reset()
        }
    }

    private enum Decision {
        case none
        case startCapturing
        case accept(Keyframe)
        case trackerError(String)
    }

    // MARK: - Frame processing

    private func processLocked(_ frame: CameraFrame) -> ScanUpdate {
        let snapshot = stateLock.withLock { shared }
        if snapshot.generation != work.generation {
            work.generation = snapshot.generation
            work.resetPose()
            work.distance.reset()
        }
        work.distance.range = snapshot.distanceRange

        let t = time(of: frame)
        if let last = work.lastTime, t < last - 1e-3 { work.resetTiming() }   // stream restart / playback loop
        work.lastTime = t

        var update = ScanUpdate()
        update.sequence = frame.sequence
        update.timestampMs = frame.timestampMs
        update.distanceRangeM = snapshot.distanceRange

        // Tracking runs in every phase so the preview can show the face and the distance.
        var observation: HeadObservation?
        var trackerFailure: String?
        do {
            observation = try tracker.analyze(frame, segmentation: false)
        } catch {
            trackerFailure = error.localizedDescription
        }
        var lowConfidence = false
        if let o = observation, o.pose.confidence < configuration.minFaceConfidence {
            observation = nil
            lowConfidence = true
        }
        if let o = observation { work.poseHistory.add(o.pose, at: t) } else { work.poseHistory.reset() }
        work.gyro.add(CameraMotion.degreesPerSecond(frame.imu), at: t)

        let colorSize = CGSize(width: frame.color.width, height: frame.color.height)
        var distance: Float?
        if let o = observation {
            distance = DepthStatistics.medianDistance(frame.alignedDepth, in: DepthStatistics.distanceRegion(o.faceRect),
                                                      colorSize: colorSize)
            if distance == nil, let estimate = Self.distanceFromFaceSize(o.faceRect, intrinsics: frame.color.intrinsics) {
                // Below Min-Z (or beyond the threshold filter) the face has no depth at all; the face size still
                // tells which way to move.
                update.distanceIsEstimate = true
                distance = estimate
            }
        }
        update.observation = observation
        update.distanceM = distance
        update.distanceAdvice = work.distance.update(distance)
        update.smoothedPose = work.poseHistory.smoothed
        update.angularSpeedDegPerSec = work.poseHistory.angularSpeed()

        var decision = Decision.none
        if let trackerFailure {
            decision = .trackerError(trackerFailure)
            update.rejection = .trackingFailed
        }
        switch snapshot.phase {
        case .idle:
            update.instructionKind = .idle
        case .finished, .failed:
            update.instructionKind = snapshot.phase == .finished ? .finished : .idle
        case .positioning:
            if case .none = decision {
                decision = positioning(&update, observation: observation, lowConfidence: lowConfidence, t: t)
            }
        case .capturing(let pose):
            if work.poseInProgress != pose {
                work.poseInProgress = pose
                work.resetPose()
            }
            if trackerFailure == nil {
                decision = capture(&update, pose: pose, frame: frame, observation: observation,
                                   lowConfidence: lowConfidence, distance: distance, t: t,
                                   snapshot: snapshot)
            } else {
                work.holdSince = nil
            }
        }

        commit(decision, update: &update, generation: snapshot.generation, t: t)
        update.instruction = update.instructionKind.englishText
        update.rejectionReason = update.rejection?.englishText
        return update
    }

    private func positioning(_ update: inout ScanUpdate, observation: HeadObservation?, lowConfidence: Bool,
                             t: Double) -> Decision {
        guard observation != nil else {
            work.positioningSince = nil
            update.instructionKind = .showFace
            update.rejection = lowConfidence ? .lowConfidence : .noFace
            return .none
        }
        switch update.distanceAdvice {
        case .tooClose:
            work.positioningSince = nil
            update.instructionKind = .moveBack
            update.rejection = .tooClose
            return .none
        case .tooFar:
            work.positioningSince = nil
            update.instructionKind = .moveCloser
            update.rejection = .tooFar
            return .none
        case .unknown:
            work.positioningSince = nil
            update.instructionKind = .showFace
            update.rejection = .noDepth
            return .none
        case .ok where update.distanceIsEstimate:
            // The face size looks right but the face has no depth (emitter off, filters): cannot scan yet.
            work.positioningSince = nil
            update.instructionKind = .lookAtCamera
            update.rejection = .noDepth
            return .none
        case .ok:
            let since = work.positioningSince ?? t
            work.positioningSince = since
            update.instructionKind = .lookAtCamera
            update.holdProgress = Float(min(1, (t - since) / max(1e-6, configuration.positioningHoldSeconds)))
            return t - since >= configuration.positioningHoldSeconds ? .startCapturing : .none
        }
    }

    private func capture(_ update: inout ScanUpdate, pose: ScanPose, frame: CameraFrame, observation: HeadObservation?,
                         lowConfidence: Bool, distance: Float?, t: Double, snapshot: SharedState) -> Decision {
        let c = configuration
        guard let face = observation, let smoothed = work.poseHistory.smoothed else {
            work.holdSince = nil
            update.instructionKind = .showFace
            update.rejection = lowConfidence ? .lowConfidence : .noFace
            return .none
        }
        let delta = Self.poseDelta(pose: pose, current: SIMD2(smoothed.yaw, smoothed.pitch),
                                   frontOffset: frontOffset(snapshot), overshootDeg: c.overshootToleranceDeg)
        let error = Self.toleranceError(delta, pose: pose, along: c.angleTolerance, across: c.perpendicularToleranceDeg)
        update.targetDelta = delta
        update.angularErrorDeg = error

        // Distance (with slack, so turning the head does not break a pose).
        let range = snapshot.distanceRange
        guard let d = distance, !update.distanceIsEstimate else {
            work.holdSince = nil
            if let d = distance, d < range.lowerBound {
                update.instructionKind = .moveBack; update.rejection = .tooClose
            } else if let d = distance, d > range.upperBound {
                update.instructionKind = .moveCloser; update.rejection = .tooFar
            } else {
                update.instructionKind = .turn(pose, yawDelta: delta.x, pitchDelta: delta.y); update.rejection = .noDepth
            }
            return .none
        }
        if d < range.lowerBound - c.distanceSlackM {
            work.holdSince = nil
            update.instructionKind = .moveBack; update.rejection = .tooClose
            return .none
        }
        if d > range.upperBound + c.distanceSlackM {
            work.holdSince = nil
            update.instructionKind = .moveCloser; update.rejection = .tooFar
            return .none
        }
        if error > c.angleTolerance {
            work.holdSince = nil
            update.instructionKind = .turn(pose, yawDelta: delta.x, pitchDelta: delta.y)
            return .none
        }
        if abs(smoothed.roll) > c.maxRollDegrees {
            work.holdSince = nil
            update.instructionKind = .straightenHead; update.rejection = .rollTooLarge
            return .none
        }
        let speed = work.poseHistory.angularSpeed()
        if let speed, speed > c.maxAngularSpeedDegPerSec {
            work.holdSince = nil
            update.instructionKind = .holdStill(pose); update.rejection = .headMoving
            return .none
        }
        let cameraMotion = work.gyro.maximum
        if cameraMotion > c.maxCameraMotionDegPerSec {
            work.holdSince = nil
            update.instructionKind = .holdStill(pose); update.rejection = .cameraMoving
            return .none
        }

        // Within tolerance and still: measure sharpness on every such frame so the blur gate has a reference.
        let sharpness = SharpnessMetric.varianceOfLaplacian(frame.color, in: SharpnessMetric.faceRegion(face.faceRect))
        let since = work.holdSince ?? t
        work.holdSince = since
        update.holdProgress = Float(min(1, (t - since) / max(1e-6, c.holdSeconds)))
        let reference = work.sharpness.reference
        work.sharpness.add(sharpness)
        // Speed needs a few frames of history; until then the hold is not complete either.
        guard t - since >= c.holdSeconds, speed != nil else {
            update.instructionKind = .holdStill(pose)
            return .none
        }
        update.instructionKind = .capturing(pose)
        if let last = work.lastAccepted, t - last < c.minKeyframeInterval { return .none }

        let colorSize = CGSize(width: frame.color.width, height: frame.color.height)
        let depthRatio = DepthStatistics.validRatio(frame.alignedDepth, in: DepthStatistics.coverageRegion(face.faceRect),
                                                    colorSize: colorSize)
        var quality = FrameQuality(sharpness: sharpness, angularSpeedDegPerSec: speed ?? 0, validDepthRatio: depthRatio,
                                   validDepthPoints: 0, faceDistanceM: d, cameraMotionDegPerSec: cameraMotion)
        update.quality = quality
        if sharpness < c.minSharpness || (reference.map { sharpness < c.minRelativeSharpness * $0 } ?? false) {
            update.rejection = .blurry
            return .none
        }
        if depthRatio < c.minValidDepthRatio {
            update.rejection = .lowDepthCoverage
            return .none
        }

        var maskInput = face
        if maskInput.personMask == nil && c.useSegmentation {
            maskInput.personMask = try? tracker.segmentPerson(in: frame)
        }
        var maskParameters = c.mask
        maskParameters.keepShoulders = c.keepShoulders
        let depth = frame.alignedDepth
        let alignedDepth = depth.width == frame.color.width && depth.height == frame.color.height ? depth : nil
        let mask = HeadMaskBuilder.build(width: frame.color.width, height: frame.color.height, observation: maskInput,
                                         depth: alignedDepth, faceDistanceM: d, parameters: maskParameters)
        quality.validDepthPoints = alignedDepth.map { DepthStatistics.validPoints($0, mask: mask) } ?? 0
        update.quality = quality
        if quality.validDepthPoints < c.minValidDepthPoints(width: frame.color.width, height: frame.color.height) {
            update.rejection = .tooFewDepthPoints
            return .none
        }

        let keyframe = Keyframe(id: 0, frame: frame, scanPose: pose, headPose: smoothed, faceRect: face.faceRect,
                                landmarks: face.landmarks, mask: mask, quality: quality)
        return .accept(keyframe)
    }

    /// Mean yaw/pitch of the accepted front keyframes (clamped), or zero.
    private func frontOffset(_ state: SharedState) -> SIMD2<Float> {
        guard configuration.targetsRelativeToFront else { return .zero }
        let front = state.keyframes.filter { $0.scanPose == .front }
        guard !front.isEmpty else { return .zero }
        let sum = front.reduce(SIMD2<Float>.zero) { $0 + SIMD2($1.headPose.yaw, $1.headPose.pitch) }
        let limit = max(0, configuration.maxFrontOffsetDeg)
        return simd_clamp(sum / Float(front.count), SIMD2(repeating: -limit), SIMD2(repeating: limit))
    }

    /// Remaining turn (yaw, pitch degrees) from `current` to `pose`. The target is shifted by the person's frontal
    /// offset; along the direction away from the front, anything from the target to `overshootDeg` beyond it counts
    /// as reached (the along component is then 0, negative beyond that window).
    static func poseDelta(pose: ScanPose, current: SIMD2<Float>, frontOffset: SIMD2<Float>, overshootDeg: Float) -> SIMD2<Float> {
        let target = SIMD2(pose.target.yaw, pose.target.pitch)
        let offset = pose == .front ? SIMD2<Float>.zero : frontOffset
        let delta = target + offset - current
        let magnitude = simd_length(target)
        guard magnitude > 0, overshootDeg > 0 else { return delta }
        let direction = target / magnitude
        let along = simd_dot(current - offset, direction)          // how far the head turned toward the pose
        let perpendicular = delta - simd_dot(delta, direction) * direction
        let remaining: Float
        if along < magnitude {
            remaining = magnitude - along
        } else if along <= magnitude + overshootDeg {
            remaining = 0
        } else {
            remaining = magnitude + overshootDeg - along              // turned too far: turn back
        }
        return perpendicular + remaining * direction
    }

    /// A remaining turn (poseDelta) measured against the tolerance ellipse, in degrees on the `along` scale: the
    /// component across the direction of the pose's target counts along / across times, so the turn is within the
    /// ellipse (semi-axes `along` and `across`) when the result is at most `along`. The front pose: the plain length.
    static func toleranceError(_ delta: SIMD2<Float>, pose: ScanPose, along: Float, across: Float) -> Float {
        let target = SIMD2(pose.target.yaw, pose.target.pitch)
        let magnitude = simd_length(target)
        guard magnitude > 0, along > 0, across > 0 else { return simd_length(delta) }
        let direction = target / magnitude
        let a = simd_dot(delta, direction)
        let p = simd_length(delta - a * direction) * along / across
        return (a * a + p * p).squareRoot()
    }

    static func withinTolerance(_ delta: SIMD2<Float>, pose: ScanPose, along: Float, across: Float) -> Bool {
        toleranceError(delta, pose: pose, along: along, across: across) <= along
    }

    private func commit(_ decision: Decision, update: inout ScanUpdate, generation: UInt64, t: Double) {
        stateLock.withLock {
            if shared.generation == generation {
                switch decision {
                case .none:
                    shared.trackerErrors = 0
                case .trackerError(let message):
                    shared.trackerErrors += 1
                    if shared.trackerErrors >= configuration.maxConsecutiveTrackerErrors,
                       shared.phase != .idle, shared.phase != .finished {
                        shared.phase = .failed(message)
                        shared.generation &+= 1
                    }
                case .startCapturing:
                    shared.trackerErrors = 0
                    shared.phase = nextPhase(after: shared)
                case .accept(var keyframe):
                    shared.trackerErrors = 0
                    keyframe.id = shared.keyframes.count
                    shared.keyframes.append(keyframe)
                    shared.accepted[keyframe.scanPose, default: 0] += 1
                    update.acceptedKeyframe = true
                    update.keyframeID = keyframe.id
                    work.lastAccepted = t
                    if shared.accepted[keyframe.scanPose, default: 0] >= configuration.framesPerPose {
                        shared.phase = nextPhase(after: shared)
                    }
                }
            }
            fillProgress(&update, from: shared)
        }
        if case .finished = update.phase, update.instructionKind != .idle { update.instructionKind = .finished }
    }

    /// The first pose that is neither complete nor skipped, or finished.
    private func nextPhase(after state: SharedState) -> ScanPhase {
        for pose in configuration.poses where !state.skipped.contains(pose) {
            if state.accepted[pose, default: 0] < configuration.framesPerPose { return .capturing(pose) }
        }
        return .finished
    }

    private func fillProgress(_ update: inout ScanUpdate, from state: SharedState) {
        update.phase = state.phase
        let needed = max(1, configuration.framesPerPose)
        var done: Float = 0
        for pose in configuration.poses {
            let fraction = min(1, Float(state.accepted[pose, default: 0]) / Float(needed))
            update.poseProgress[pose] = fraction
            done += state.skipped.contains(pose) ? 1 : fraction
        }
        update.overallProgress = configuration.poses.isEmpty ? 1 : done / Float(configuration.poses.count)
        if case .finished = state.phase { update.overallProgress = 1 }
        update.coverage = CoverageMap(keyframePoses: state.keyframes.map(\.headPose), framesPerPose: needed)
        update.keyframeCount = state.keyframes.count
        update.skippedPoses = state.skipped
        if case .capturing(let pose) = state.phase {
            update.poseIndex = configuration.poses.firstIndex(of: pose)
            update.canSkipPose = configuration.optionalPoses.contains(pose)
        }
        if case .failed = state.phase { update.instructionKind = .idle }
    }

    // MARK: - Helpers

    /// Seconds on the frame clock. Frame timestamps drive all timing (also live, where they come from the
    /// camera); only a live source without timestamps falls back to the monotonic system clock, and an offline
    /// one to a nominal 30 fps.
    private func time(of frame: CameraFrame) -> Double {
        let t = frame.timestampMs / 1000
        if t.isFinite && t > 0 { return t }
        if configuration.realTime { return ProcessInfo.processInfo.systemUptime }
        return (work.lastTime ?? 0) + 1.0 / 30
    }

    /// Distance estimate from the face rectangle width (Vision's box is ~0.16 m wide on an adult face).
    static func distanceFromFaceSize(_ faceRect: CGRect, intrinsics: Intrinsics) -> Float? {
        guard faceRect.width > 4, intrinsics.fx > 0 else { return nil }
        return intrinsics.fx * 0.16 / Float(faceRect.width)
    }
}
