// License: Apache 2.0. See LICENSE file in root directory.
//
// The guided scan state machine: positioning -> the front pose -> the other poses in whatever order the person turns.
// After the front pose nothing is suggested: every frame picks the active pose, the unfinished one the head is within
// tolerance of or turned toward; the scan waits until the head is within that pose's tolerance (or a shorter turn is
// held) and held still, and accepts spaced keyframes that pass the quality gates. Usable without UI: the CLI runs it
// over a recording with ScanConfiguration.offline (frame timestamps only, no wall clock).

import CameraKit
import CoreGraphics
import Foundation
import os
import simd

public struct ScanConfiguration: Sendable, Hashable {
    public var poses: [ScanPose] = ScanPose.required
    /// Keyframes accepted per pose.
    public var framesPerPose: Int = 3
    /// Maximum angular distance (degrees) from a pose's target, along the direction of the target (the front pose:
    /// frontToleranceDeg).
    public var angleTolerance: Float = 8
    /// Maximum angular distance (degrees) of the front pose from the camera axis, in every direction. The front pose is
    /// the person's own straight-ahead view, which the other targets are measured from (targetsRelativeToFront, the
    /// shift limited by maxFrontOffsetDeg), so it needs a still head more than an exact angle: the live D455 session of
    /// 2026-11-04 spent 14.6 s on it with the 8-degree circle, 93 % of its frames outside, and the recording of 10:52
    /// read pitch +14..+16 while positioning asked to look at the camera. Vision's reading of a held head also steps
    /// by 3-6 degrees (VisionJitter in the tests), which an 8-degree circle around a head a few degrees off the axis
    /// keeps leaving.
    public var frontToleranceDeg: Float = 12
    /// Maximum deviation (degrees) across the direction of a pose's target: pitch on the side poses, yaw on up and
    /// down. A side pose a little up or down still shows that side of the head (the keyframe keeps its measured
    /// pose), and Vision reads large yaw partly as pitch: the synthetic head at yaw +44..+52 reads pitch +4..+10 at
    /// 640x400 (mean ~+8), which an 8-degree circle rarely accepted; a real D455 session read +10..+15 at yaw -74.
    /// The tolerance is the ellipse of both semi-axes.
    public var perpendicularToleranceDeg: Float = 15
    /// Beyond a pose's target the perpendicular tolerance grows by this much per degree of further turn: the real
    /// session read pitch 12-18 degrees off the front at yaw -60..-80.
    public var perpendicularGrowthPerDeg: Float = 0.3
    /// Targets of the other poses are relative to the person's own frontal pose (mean of the front keyframes):
    /// Vision's yaw/pitch of a face looking straight at the camera is biased per person by a few degrees.
    public var targetsRelativeToFront: Bool = true
    /// Largest frontal offset (degrees) that is applied to the other targets.
    public var maxFrontOffsetDeg: Float = 10
    /// Turning further than a pose's target (away from the front) is accepted up to this many degrees beyond it:
    /// more rotation only adds coverage. People turn much further than the target: a real D455 session read yaw -74
    /// (76 from the front) on a right turn and +70 on a left one, which a 15-degree window refused, so that scan never
    /// got past the right pose.
    public var overshootToleranceDeg: Float = 40
    /// true (the default): no order after the front pose. Every frame works on the active pose (ScanUpdate.activePose):
    /// the unfinished pose the head is within tolerance of (or holds a short turn toward), else the one it is turned
    /// toward (headingFraction), else none, and the instruction is to turn in any direction. The front pose always
    /// comes first: the other targets are relative to it. A fixed suggestion made people wait: the live session of
    /// 2026-11-04 logged 42.4 s on up, which it suggested first, 76 % of the frames not turned enough toward it.
    /// false: the former suggested order. The phase's pose (the first unfinished one in configuration order) is the
    /// active pose and the instruction names it; another unfinished pose still counts when the head is within its
    /// full tolerance, but a short turn counts only toward the suggested pose, and there is no turn in any direction.
    public var posesInAnyOrder: Bool = true
    /// After the front pose, the head counts as turned toward an unfinished pose (which becomes the active pose: the
    /// turn instruction, the short-turn rule) once it turned this fraction of the way to its target, along the target
    /// direction and within the perpendicular tolerance. Less is still the front: no pose, the instruction is to turn
    /// in any direction. 0.3 is 7.5 degrees toward up or down and 13.5 toward a side, beyond Vision's 3-6 degree steps
    /// on a held head (VisionJitter in the tests).
    public var headingFraction: Float = 0.3
    /// A turn that stops short of the active pose's target is accepted once it reaches this fraction of the target
    /// (from the front pose, within the perpendicular tolerance) and stays there, slowly, for shortTurnHoldSeconds:
    /// the person cannot turn further, or Vision reads less than the true angle (0 = off). People turn up and down
    /// little: the live session of 2026-11-04 got 20.1 degrees down (80 % of the target) in 19.6 s and 21.4 up.
    public var shortTurnFraction: Float = 0.6
    public var shortTurnHoldSeconds: Double = 1.5
    /// The pose must be held (within tolerance, slow motion) this long before frames are accepted. With the
    /// keyframe interval below, a pose takes about 0.6 s of holding still: a real session's holds at a pose were
    /// often under a second.
    public var holdSeconds: Double = 0.3
    /// Absolute minimum variance of Laplacian of the face (0 = off; the relative gate below adapts to the scene).
    public var minSharpness: Float = 0
    /// Head rotation speed (deg/s, smoothed Vision pose) above which the head counts as moving: no keyframe, and
    /// the hold restarts.
    public var maxAngularSpeedDegPerSec: Float = 20
    /// Valid depth in the face rectangle. A face turned down shows the hair, which a D455 measures poorly: 0.57 on a
    /// real session; minValidDepthPoints below is what guarantees the geometry.
    public var minValidDepthRatio: Float = 0.5
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
    public var minKeyframeInterval: Double = 0.15
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

    /// The tolerance a pose is within (ScanUpdate.angularErrorDeg at most this): frontToleranceDeg for the front pose,
    /// angleTolerance for the others.
    public func tolerance(for pose: ScanPose) -> Float {
        pose == .front ? frontToleranceDeg : angleTolerance
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
    /// Capturing keyframes. The pose is the front pose until it is complete; after it (with posesInAnyOrder) it is no
    /// instruction (ScanUpdate.activePose is the pose being worked on): the pose of the last accepted keyframe, and
    /// after a pose completes the first unfinished pose in configuration order.
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
    /// The pose this frame works on: the front pose until it is complete; after it the unfinished pose the head is
    /// within tolerance of, else the one it is turned toward (ScanConfiguration.headingFraction), else one it is
    /// turned toward beyond the sideways tolerance (for the instruction only: its turn hint says how to correct);
    /// nil when the head is turned toward no unfinished pose (near the front, the instruction is then
    /// .turnAnyDirection) or not seen. Without posesInAnyOrder: the phase's pose (suggested), or another unfinished
    /// pose within its tolerance. The UI points to it; the instructions .turn, .holdStill and .capturing name it.
    public var activePose: ScanPose?
    /// Remaining angular distance to the active pose's target (degrees), with the component across the pose
    /// direction scaled by angleTolerance / perpendicularToleranceDeg: the pose is within tolerance when this is at
    /// most ScanConfiguration.tolerance(for:) of the pose (what the UI compares). nil without an active pose.
    public var angularErrorDeg: Float?
    /// With a .turn instruction: which way to correct it (turned too far, off the pose direction, ...).
    public var turnHint: TurnHint?
    /// Target - current (yaw, pitch) in degrees, toward the active pose.
    public var targetDelta: SIMD2<Float>?
    /// Fraction of holdSeconds the pose has been held.
    public var holdProgress: Float = 0
    /// Distance range the scan uses (recommended for the camera configuration).
    public var distanceRangeM: ClosedRange<Float>?
    /// True when the distance is an estimate from the face size (no valid depth on the face).
    public var distanceIsEstimate: Bool = false
    public var keyframeCount: Int = 0
    public var skippedPoses: Set<ScanPose> = []
    /// The pose the skip button names and skips (ScanController.skipPose(_:)): the active pose when it is optional and
    /// still open, else nil. The button skips only what the UI shows: after the front pose no order is left to fall
    /// back to.
    public var skipPose: ScanPose?
    /// No active pose after the front pose, and only optional poses left: the UI offers to skip them all
    /// (ScanController.skipRemainingOptionalPoses()).
    public var canSkipRemainingPoses: Bool = false
    /// Index of the phase's pose in configuration.poses (nil outside capture).
    public var poseIndex: Int?
    /// The keyframe accepted with this frame.
    public var keyframeID: Int?
    /// The pose whose last keyframe was accepted with this frame (the app plays a sound).
    public var completedPose: ScanPose?

    public init() {}

    /// The configured poses that are neither complete nor skipped, in ScanPose order (poseProgress has every
    /// configured pose).
    public var unfinishedPoses: [ScanPose] {
        ScanPose.allCases.filter { pose in
            (poseProgress[pose].map { $0 < 1 } ?? false) && !skippedPoses.contains(pose)
        }
    }
}

public final class ScanController: @unchecked Sendable {
    public let configuration: ScanConfiguration
    private let tracker: HeadTracking
    private static let log = Logger(subsystem: "IntelFaceScan", category: "Guidance")

    // Serializes process(): the per-frame working state below is only touched while it is held.
    private let processingLock = NSLock()
    private var work = WorkState()

    // Frames are analyzed here, never on a thread of the Swift cooperative pool. Vision's perform blocks its thread in
    // -[VNControlledCapacityTasksQueue dispatchGroupWait:error:] until Vision's own work has run on another thread of
    // that pool (it never returns while every pool thread is busy). On a pool thread one frame therefore takes a
    // thread and needs a second free one: with the 3 threads of a 3-core machine (the CI runner), frames in flight
    // and a busy task left Vision's work no thread, no frame was analyzed, and every task of the process stopped.
    private let processingQueue = DispatchQueue(label: "IntelFaceScan.ScanController.process", qos: .userInitiated)

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

    /// Processes one frame on the controller's dispatch queue (off the Swift cooperative pool, see processingQueue)
    /// and hands the state to show to `completion`, on that queue.
    public func process(_ frame: CameraFrame, completion: @escaping @Sendable (ScanUpdate) -> Void) {
        processingQueue.async {
            completion(self.processingLock.withLock { self.processLocked(frame) })
        }
    }

    /// Processes one frame; returns the state to show. The frame is analyzed on the controller's dispatch queue,
    /// not on the caller's thread (see processingQueue).
    public func process(_ frame: CameraFrame) async -> ScanUpdate {
        await withCheckedContinuation { continuation in
            process(frame) { continuation.resume(returning: $0) }
        }
    }

    /// Skips `pose` when it is still open (configured, neither complete nor skipped, the scan capturing) and optional
    /// (or `allowRequired`). The app passes ScanUpdate.skipPose, the pose its button names. Returns whether it was
    /// skipped.
    @discardableResult
    public func skipPose(_ pose: ScanPose, allowRequired: Bool = false) -> Bool {
        stateLock.withLock { skipLocked([pose], allowRequired: allowRequired) }
    }

    /// Skips the pose being worked on: the active pose of the last frame, else the phase's pose (the first unfinished
    /// one in configuration order), when it is optional (or `allowRequired`). For callers without a frame to look at
    /// (tests, scripts); the app skips the pose it shows (skipPose(_:)). Returns whether a pose was skipped.
    @discardableResult
    public func skipPose(allowRequired: Bool = false) -> Bool {
        stateLock.withLock {
            guard let pose = skipCandidate(shared) else { return false }
            return skipLocked([pose], allowRequired: allowRequired)
        }
    }

    /// Skips every optional pose still open (the app offers it when no other pose is left: ScanUpdate
    /// .canSkipRemainingPoses). Returns whether any was skipped.
    @discardableResult
    public func skipRemainingOptionalPoses() -> Bool {
        stateLock.withLock {
            let open = configuration.poses.filter { isOpen($0, shared) && configuration.optionalPoses.contains($0) }
            return skipLocked(open, allowRequired: false)
        }
    }

    /// Configured, neither complete nor skipped.
    private func isOpen(_ pose: ScanPose, _ state: SharedState) -> Bool {
        configuration.poses.contains(pose) && !state.skipped.contains(pose)
            && state.accepted[pose, default: 0] < configuration.framesPerPose
    }

    /// Skips `poses` (all or none) while capturing. The phase moves on only when its own pose is skipped (it names the
    /// pose of the last keyframes, nothing the person is asked for); every skip starts a new generation, which resets
    /// the holds and short-turn timers on the next frame.
    private func skipLocked(_ poses: [ScanPose], allowRequired: Bool) -> Bool {
        guard case .capturing(let phasePose) = shared.phase, !poses.isEmpty,
              poses.allSatisfy({ isOpen($0, shared) && (allowRequired || configuration.optionalPoses.contains($0)) })
        else { return false }
        shared.skipped.formUnion(poses)
        shared.generation &+= 1
        if let active = shared.activePose, poses.contains(active) { shared.activePose = nil }
        if poses.contains(phasePose) { shared.phase = nextPhase(after: shared) }
        return true
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
            var open: [String] = []
            for pose in configuration.poses where shared.accepted[pose, default: 0] < configuration.framesPerPose {
                open.append(pose.rawValue)
                if let stats = shared.poseStats[pose] { Self.log.notice("pose \(pose.rawValue, privacy: .public) open: \(stats.summary, privacy: .public)") }
            }
            if shared.noActivePoseStats.frames > 0 || !shared.keyframes.isEmpty {
                let line = "scan end: \(shared.keyframes.count) keyframes, open poses: "
                    + (open.isEmpty ? "none" : open.joined(separator: ", "))
                    + "; no active pose: " + shared.noActivePoseStats.summary
                Self.log.notice("\(line, privacy: .public)")
            }
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
        stateLock.withLock { skipCandidate(shared).map { configuration.optionalPoses.contains($0) } ?? false }
    }

    /// The pose skipPose() without a pose skips: the last frame's active pose while it is still open, else the phase's
    /// pose.
    private func skipCandidate(_ state: SharedState) -> ScanPose? {
        guard case .capturing(let phasePose) = state.phase else { return nil }
        if let active = state.activePose, isOpen(active, state) { return active }
        return phasePose
    }

    /// The per-pose diagnostics so far (what finish() logs), for the tests.
    var diagnostics: (poses: [ScanPose: PoseStats], noActivePose: PoseStats) {
        stateLock.withLock { (shared.poseStats, shared.noActivePoseStats) }
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
        /// The active pose of the last committed frame (what skipPose() skips).
        var activePose: ScanPose?
        /// What held each pose back (logged when it completes, or open at finish()).
        var poseStats: [ScanPose: PoseStats] = [:]
        /// Capture frames that worked on no pose (the head turned toward none, or not seen): logged at finish().
        var noActivePoseStats = PoseStats()
    }

    /// Per pose: the outcome of every frame worked on it (it was the active pose), the time of those frames and the
    /// furthest turn toward the pose on the capture frames it was the active pose or the head within its sideways
    /// tolerance. Not on the others: a head turned to a side reads some pitch (Vision), which is no turn up.
    struct PoseStats {
        var frames = 0
        /// Seconds of the frames: the sum of their frame intervals. The span from the first frame to the last says
        /// little now that the person turns from pose to pose and back.
        var seconds: Double = 0
        var outcomes: [String: Int] = [:]
        var furthestDeg: Float = -.infinity

        mutating func add(_ outcome: String, dt: Double) {
            frames += 1
            seconds += dt
            outcomes[outcome, default: 0] += 1
        }

        mutating func turned(_ turnDeg: Float) { furthestDeg = max(furthestDeg, turnDeg) }

        /// "4.2 s, 126 frames: turnedTooFar 61%, holding 20%, ...; furthest turn 75.6 deg"
        var summary: String {
            let shares = outcomes.sorted { $0.value > $1.value || ($0.value == $1.value && $0.key < $1.key) }
                .map { "\($0.key) \(Int((100 * Double($0.value) / Double(max(1, frames))).rounded()))%" }
            let furthest = furthestDeg.isFinite ? String(format: "; furthest turn %.1f deg", furthestDeg) : ""
            return String(format: "%.1f s, %d frames", seconds, frames) + (shares.isEmpty ? "" : ": ")
                + shares.joined(separator: ", ") + furthest
        }
    }

    /// The outcome of a capture frame for the diagnostics: accepted, a rejection, the reason a turn is not within
    /// tolerance, holding, or waiting between keyframes.
    static func outcome(_ update: ScanUpdate) -> String {
        if update.acceptedKeyframe { return "accepted" }
        if let rejection = update.rejection { return rejection.rawValue }
        switch update.instructionKind {
        case .turn:
            switch update.turnHint {
            case .back: return "turnedTooFar"
            case .further, nil: return "notTurnedEnough"
            default: return "offAxis"
            }
        case .turnAnyDirection: return "turnAnyDirection"
        case .holdStill: return "holding"
        default: return "spacing"
        }
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
        /// The phase's pose the state below was reset for.
        var poseInProgress: ScanPose?
        /// The pose holdSince and the sharpness reference belong to (the last active pose, which need not be the
        /// phase's).
        var holdPose: ScanPose?
        /// Per pose: a turn short of its target, held since (see ScanConfiguration.shortTurnFraction). One start time
        /// per pose: Vision's jitter moving the head in and out of a neighbouring pose's range (the diagonals) switches
        /// the pose turned toward most, which must not restart the time of the pose that stays.
        var shortTurnSince: [ScanPose: Double] = [:]

        mutating func resetPose() {
            holdSince = nil; positioningSince = nil; lastAccepted = nil; holdPose = nil; shortTurnSince = [:]
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
        // The frame's share of the time in the diagnostics (a gap of dropped frames counts at most 0.25 s).
        let dt = work.lastTime.map { min(0.25, max(0, t - $0)) } ?? 0
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
                decision = capture(&update, phasePose: pose, frame: frame, observation: observation,
                                   lowConfidence: lowConfidence, distance: distance, t: t,
                                   snapshot: snapshot)
            } else {
                work.holdSince = nil
                if suggests(pose) { update.activePose = pose }
            }
        }

        var worked: Worked?
        if case .capturing(let phasePose) = snapshot.phase {
            // How far the head turned toward the active pose, and toward the open poses it is within the sideways
            // tolerance of (the furthest turn in the diagnostics).
            var turns: [(ScanPose, Float)] = []
            if let smoothed = update.smoothedPose, phasePose != .front {
                let offset = frontOffset(snapshot), current = SIMD2(smoothed.yaw, smoothed.pitch)
                for pose in openPoses(snapshot) {
                    let across = pose == update.activePose ? .infinity
                        : Self.acrossTolerance(pose: pose, current: current, frontOffset: offset,
                                               base: configuration.perpendicularToleranceDeg,
                                               growth: configuration.perpendicularGrowthPerDeg)
                    guard let f = Self.turnFraction(pose: pose, current: current, frontOffset: offset,
                                                    across: across) else { continue }
                    turns.append((pose, f * simd_length(SIMD2(pose.target.yaw, pose.target.pitch))))
                }
            }
            worked = Worked(pose: update.activePose, dt: dt, turns: turns)
        }
        commit(decision, update: &update, generation: snapshot.generation, t: t, worked: worked)
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

    private func capture(_ update: inout ScanUpdate, phasePose: ScanPose, frame: CameraFrame,
                         observation: HeadObservation?, lowConfidence: Bool, distance: Float?, t: Double,
                         snapshot: SharedState) -> Decision {
        let c = configuration
        guard let face = observation, let smoothed = work.poseHistory.smoothed else {
            work.holdSince = nil
            work.shortTurnSince = [:]
            if suggests(phasePose) { update.activePose = phasePose }
            update.instructionKind = .showFace
            update.rejection = lowConfidence ? .lowConfidence : .noFace
            return .none
        }
        let speed = work.poseHistory.angularSpeed()
        let active = chooseGoal(phasePose: phasePose, current: SIMD2(smoothed.yaw, smoothed.pitch), speed: speed,
                                frontOffset: frontOffset(snapshot), snapshot: snapshot, t: t)
        update.activePose = active?.pose
        if let goal = active {
            if work.holdPose != goal.pose {
                work.holdPose = goal.pose
                work.holdSince = nil
                work.sharpness.reset()
            }
            update.targetDelta = goal.delta
            // A short turn held long enough counts as reached (the UI compares the error with the pose's tolerance).
            update.angularErrorDeg = goal.shortTurnHeld ? min(goal.error, c.tolerance(for: goal.pose)) : goal.error
        }
        // Turn toward the active pose, or (none) toward any unfinished one.
        func turnInstruction() {
            guard let goal = active else {
                update.instructionKind = .turnAnyDirection
                return
            }
            update.instructionKind = .turn(goal.pose, yawDelta: goal.delta.x, pitchDelta: goal.delta.y)
            update.turnHint = TurnHint.hint(pose: goal.pose, delta: goal.delta, along: c.tolerance(for: goal.pose),
                                            across: goal.across)
        }

        // Distance (with slack, so turning the head does not break a pose).
        let range = snapshot.distanceRange
        guard let d = distance, !update.distanceIsEstimate else {
            work.holdSince = nil
            if let d = distance, d < range.lowerBound {
                update.instructionKind = .moveBack; update.rejection = .tooClose
            } else if let d = distance, d > range.upperBound {
                update.instructionKind = .moveCloser; update.rejection = .tooFar
            } else {
                turnInstruction(); update.rejection = .noDepth
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
        // Not within the active pose's tolerance (also a pose turned toward beyond its sideways tolerance, which only
        // the instruction names), or no active pose (the head turned toward no unfinished pose): turn.
        guard let goal = active, goal.error <= c.tolerance(for: goal.pose) || goal.shortTurnHeld else {
            work.holdSince = nil
            turnInstruction()
            return .none
        }
        let pose = goal.pose
        if abs(smoothed.roll) > c.maxRollDegrees {
            work.holdSince = nil
            update.instructionKind = .straightenHead; update.rejection = .rollTooLarge
            return .none
        }
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

    private struct PoseGoal {
        var pose: ScanPose
        var delta: SIMD2<Float>
        var error: Float
        /// The perpendicular tolerance used (grown beyond the target).
        var across: Float
        /// A turn short of the target, held for shortTurnHoldSeconds: accepted as if within tolerance.
        var shortTurnHeld = false
    }

    /// The phase's pose is the one to make: the front pose (the other targets are relative to it), or any pose
    /// without posesInAnyOrder (the scan suggests them in order).
    private func suggests(_ phasePose: ScanPose) -> Bool {
        phasePose == .front || !configuration.posesInAnyOrder
    }

    /// The poses after the front pose that are neither complete nor skipped, in configuration order.
    private func openPoses(_ state: SharedState) -> [ScanPose] {
        configuration.poses.filter { $0 != .front && isOpen($0, state) }
    }

    /// The active pose of this frame (nil: none). The front pose while it is open. After it, with posesInAnyOrder, no
    /// pose is suggested: the unfinished pose the head reached (within tolerance, or a short turn held: the previous
    /// active pose while it stays reached, else the smallest error); else the one it is turned toward (the largest
    /// turnFraction within the sideways tolerance, at least headingFraction); else the closest pose it is turned
    /// toward beyond the sideways tolerance, for the instruction only; else none. Without posesInAnyOrder: the
    /// suggested (phase's) pose within tolerance, else another unfinished one within tolerance, else the suggested
    /// one. A short turn (shortTurnFraction, held slowly for shortTurnHoldSeconds) counts as within tolerance; without
    /// posesInAnyOrder only toward the suggested pose.
    private func chooseGoal(phasePose: ScanPose, current: SIMD2<Float>, speed: Float?, frontOffset: SIMD2<Float>,
                            snapshot: SharedState, t: Double) -> PoseGoal? {
        let c = configuration
        func goal(_ pose: ScanPose) -> PoseGoal {
            let delta = Self.poseDelta(pose: pose, current: current, frontOffset: frontOffset,
                                       overshootDeg: c.overshootToleranceDeg)
            let across = Self.acrossTolerance(pose: pose, current: current, frontOffset: frontOffset,
                                              base: c.perpendicularToleranceDeg, growth: c.perpendicularGrowthPerDeg)
            let along = c.tolerance(for: pose)
            return PoseGoal(pose: pose, delta: delta,
                            error: Self.toleranceError(delta, pose: pose, along: along, across: across),
                            across: across)
        }
        func within(_ g: PoseGoal) -> Bool { g.error <= c.tolerance(for: g.pose) }
        /// How far the head turned toward the pose (nil: the front pose, or beyond the sideways tolerance).
        func fraction(_ g: PoseGoal) -> Float? {
            Self.turnFraction(pose: g.pose, current: current, frontOffset: frontOffset, across: g.across)
        }

        guard phasePose != .front else {
            work.shortTurnSince = [:]
            return goal(.front)
        }
        let open = openPoses(snapshot)
        var goals = (c.posesInAnyOrder ? open : [phasePose] + open.filter { $0 != phasePose }).map(goal)

        // Short turns, while the head moves slowly; short of the target only (a turn beyond the overshoot window is
        // asked to turn back instead). Each pose keeps its own start time while it qualifies.
        let slow = c.shortTurnFraction > 0 && speed.map { $0 <= c.maxAngularSpeedDegPerSec } == true
        var since: [ScanPose: Double] = [:]
        for i in goals.indices where slow && (c.posesInAnyOrder || goals[i].pose == phasePose) {
            guard let f = fraction(goals[i]), f >= c.shortTurnFraction, f < 1 else { continue }
            let start = work.shortTurnSince[goals[i].pose] ?? t
            since[goals[i].pose] = start
            goals[i].shortTurnHeld = t - start >= c.shortTurnHoldSeconds
        }
        work.shortTurnSince = since

        guard c.posesInAnyOrder else {
            return goals.first { within($0) && $0.pose == phasePose }
                ?? goals.filter(within).min { $0.error < $1.error } ?? goals[0]
        }
        let reached = goals.filter { within($0) || $0.shortTurnHeld }
        // Two reached poses overlap (a side and a diagonal): keeping the previous one keeps its hold, which Vision's
        // jitter switching between the two would restart frame after frame.
        if let previous = work.holdPose, let kept = reached.first(where: { $0.pose == previous }) { return kept }
        if let best = reached.min(by: { $0.error < $1.error }) { return best }

        // The pose the head is heading to: the largest fraction of its target, past headingFraction.
        var heading: (goal: PoseGoal, fraction: Float)?
        for g in goals {
            guard let f = fraction(g), f >= c.headingFraction, f > (heading?.fraction ?? -.infinity) else { continue }
            heading = (g, f)
        }
        if let toward = heading?.goal { return toward }

        // Turned toward an open pose but beyond its sideways tolerance (the chin raised with the head half turned to
        // a side, the side pose done): that pose, the closest by the tolerance error. Only the instruction uses it,
        // its turn hint saying which way to correct; it is not reached, and no short turn counts toward it. The head
        // must be turned more toward the pose than across it: at a finished side pose Vision reads 10-15 degrees of
        // pitch, which is no turn up to correct "a little" to the side. .turnAnyDirection is left for a head turned
        // toward no open pose: near the front, or at a finished one.
        var nearest: PoseGoal?
        for g in goals {
            let target = SIMD2(g.pose.target.yaw, g.pose.target.pitch), magnitude = simd_length(target)
            let relative = current - frontOffset, along = simd_dot(relative, target / magnitude)
            guard along >= c.headingFraction * magnitude,
                  simd_length(relative - along * target / magnitude) <= along,
                  g.error < (nearest?.error ?? .infinity) else { continue }
            nearest = g
        }
        return nearest
    }

    /// The perpendicular tolerance for `pose`: `base`, plus `growth` per degree the head turned beyond the target.
    static func acrossTolerance(pose: ScanPose, current: SIMD2<Float>, frontOffset: SIMD2<Float>, base: Float,
                                growth: Float) -> Float {
        let target = SIMD2(pose.target.yaw, pose.target.pitch)
        let magnitude = simd_length(target)
        guard magnitude > 0, growth > 0 else { return base }
        let along = simd_dot(current - frontOffset, target / magnitude)
        return base + growth * max(0, along - magnitude)
    }

    /// How far the head turned toward `pose` from the front offset, as a fraction of the target (along its
    /// direction); nil for the front pose or a head beyond the perpendicular tolerance (turned some other way).
    static func turnFraction(pose: ScanPose, current: SIMD2<Float>, frontOffset: SIMD2<Float>, across: Float) -> Float? {
        let target = SIMD2(pose.target.yaw, pose.target.pitch)
        let magnitude = simd_length(target)
        guard magnitude > 0 else { return nil }
        let direction = target / magnitude
        let relative = current - frontOffset
        let along = simd_dot(relative, direction)
        guard simd_length(relative - along * direction) <= across else { return nil }
        return along / magnitude
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

    /// A capture frame for the diagnostics: the active pose it worked on (nil: none), its share of the time and how far
    /// the head turned toward each open pose (degrees along the target direction).
    private struct Worked {
        var pose: ScanPose?
        var dt: Double
        var turns: [(ScanPose, Float)]
    }

    private func commit(_ decision: Decision, update: inout ScanUpdate, generation: UInt64, t: Double,
                        worked: Worked? = nil) {
        var logLine: String?
        stateLock.withLock {
            if shared.generation == generation {
                if let worked {
                    var accepted = update
                    if case .accept = decision { accepted.acceptedKeyframe = true }
                    let outcome = Self.outcome(accepted)
                    if let pose = worked.pose {
                        shared.poseStats[pose, default: PoseStats()].add(outcome, dt: worked.dt)
                    } else {
                        shared.noActivePoseStats.add(outcome, dt: worked.dt)
                    }
                    for (pose, deg) in worked.turns { shared.poseStats[pose, default: PoseStats()].turned(deg) }
                    shared.activePose = worked.pose
                }
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
                        update.completedPose = keyframe.scanPose
                        if let stats = shared.poseStats[keyframe.scanPose] {
                            logLine = "pose \(keyframe.scanPose.rawValue) complete: \(stats.summary)"
                        }
                        shared.phase = nextPhase(after: shared)
                    } else if shared.phase != .capturing(keyframe.scanPose) {
                        // The phase follows the pose being captured (CLI, trace), keeping its hold, which a change
                        // of the phase's pose would reset.
                        shared.phase = .capturing(keyframe.scanPose)
                        work.poseInProgress = keyframe.scanPose
                    }
                }
            }
            fillProgress(&update, from: shared)
        }
        if let logLine { Self.log.notice("\(logLine, privacy: .public)") }
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
            // The skip button names the pose the UI shows (the active pose), never a pose picked by an order.
            if let active = update.activePose, isOpen(active, state), configuration.optionalPoses.contains(active) {
                update.skipPose = active
            }
            let open = openPoses(state)
            update.canSkipRemainingPoses = update.activePose == nil && pose != .front && !open.isEmpty
                && open.allSatisfy { configuration.optionalPoses.contains($0) }
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
