// License: Apache 2.0. See LICENSE file in root directory.
//
// `ifscan reconstruct --trace trace.csv`: one CSV row per frame the guided scan processed (phase, pose, the active
// pose, the head pose Vision measured and the smoothed one the target check uses, the remaining turn, distance, the
// gates' measurements and the rejection). It shows why a real session did not accept a pose, frame by frame.

import Foundation
import ScanKit

public final class ScanTraceWriter: @unchecked Sendable {
    public static let header = [
        "sequence", "t_s", "phase", "pose", "active_pose", "instruction", "rejection", "accepted", "keyframes",
        "face", "confidence", "yaw", "pitch", "roll", "smoothed_yaw", "smoothed_pitch", "smoothed_roll",
        "target_dyaw", "target_dpitch", "angular_error", "turn_hint", "speed_deg_s", "hold",
        "distance_m", "distance_estimate", "distance_advice", "range_min_m", "range_max_m",
        "sharpness", "depth_ratio", "depth_points", "camera_motion_deg_s",
        "face_x", "face_y", "face_w", "face_h",
    ].joined(separator: ",")

    private let lock = NSLock()
    private let handle: FileHandle
    private var firstTimestampMs: Double?

    /// Creates (or truncates) the file and writes the header.
    public init(url: URL) throws {
        guard FileManager.default.createFile(atPath: url.path, contents: Data((Self.header + "\n").utf8)) else {
            throw CocoaError(.fileWriteUnknown, userInfo: [NSFilePathErrorKey: url.path])
        }
        handle = try FileHandle(forWritingTo: url)
        handle.seekToEndOfFile()
    }

    public func add(_ update: ScanUpdate) {
        lock.withLock {
            let first = firstTimestampMs ?? update.timestampMs
            firstTimestampMs = first
            // (write(contentsOf:) throws; write(_: Data) raises an Objective-C exception on a write error.)
            try? handle.write(contentsOf: Data((Self.row(update, startMs: first) + "\n").utf8))
        }
    }

    public func close() {
        lock.withLock { try? handle.close() }
    }

    static func row(_ u: ScanUpdate, startMs: Double) -> String {
        func f(_ value: Float?, _ digits: Int = 2) -> String { value.map { String(format: "%.\(digits)f", $0) } ?? "" }
        func d(_ value: Double?, _ digits: Int = 3) -> String { value.map { String(format: "%.\(digits)f", $0) } ?? "" }
        let pose = u.observation?.pose
        let smoothed = u.smoothedPose
        let rect = u.observation?.faceRect
        let fields: [String] = [
            String(u.sequence), d((u.timestampMs - startMs) / 1000), phaseName(u.phase), poseName(u.phase),
            u.activePose?.rawValue ?? "", instructionName(u.instructionKind), u.rejection?.rawValue ?? "",
            u.acceptedKeyframe ? "1" : "0", String(u.keyframeCount),
            u.observation == nil ? "0" : "1", f(pose?.confidence), f(pose?.yaw), f(pose?.pitch), f(pose?.roll),
            f(smoothed?.yaw), f(smoothed?.pitch), f(smoothed?.roll),
            f(u.targetDelta?.x), f(u.targetDelta?.y), f(u.angularErrorDeg), u.turnHint?.rawValue ?? "",
            f(u.angularSpeedDegPerSec), f(u.holdProgress),
            f(u.distanceM, 3), u.distanceIsEstimate ? "1" : "0", u.distanceAdvice.rawValue,
            f(u.distanceRangeM?.lowerBound), f(u.distanceRangeM?.upperBound),
            f(u.quality?.sharpness, 1), f(u.quality?.validDepthRatio, 3), u.quality.map { String($0.validDepthPoints) } ?? "",
            f(u.quality?.cameraMotionDegPerSec),
            rect.map { String(format: "%.0f", $0.minX) } ?? "", rect.map { String(format: "%.0f", $0.minY) } ?? "",
            rect.map { String(format: "%.0f", $0.width) } ?? "", rect.map { String(format: "%.0f", $0.height) } ?? "",
        ]
        return fields.joined(separator: ",")
    }

    static func phaseName(_ phase: ScanPhase) -> String {
        switch phase {
        case .idle: return "idle"
        case .positioning: return "positioning"
        case .capturing: return "capturing"
        case .finished: return "finished"
        case .failed: return "failed"
        }
    }

    static func poseName(_ phase: ScanPhase) -> String {
        if case .capturing(let pose) = phase { return pose.rawValue }
        return ""
    }

    static func instructionName(_ instruction: ScanInstruction) -> String {
        switch instruction {
        case .idle: return "idle"
        case .showFace: return "showFace"
        case .moveCloser: return "moveCloser"
        case .moveBack: return "moveBack"
        case .lookAtCamera: return "lookAtCamera"
        case .turn: return "turn"
        case .turnAnyDirection: return "turnAnyDirection"
        case .straightenHead: return "straightenHead"
        case .holdStill: return "holdStill"
        case .capturing: return "capturing"
        case .finished: return "finished"
        }
    }
}
