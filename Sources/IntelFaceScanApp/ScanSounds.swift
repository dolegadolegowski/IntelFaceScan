// License: Apache 2.0. See LICENSE file in root directory.
//
// Sounds of the guided scan, for a person turned away from the screen: a tick when the head reaches a pose (hold
// still), a chime when the pose is captured (move on), a fanfare when the scan is complete.

import AudioToolbox
import Foundation
import ScanKit

enum ScanSound: Equatable {
    case poseReached
    case poseRecognized
    case scanComplete

    /// System sound (/System/Library/Sounds).
    var systemName: String {
        switch self {
        case .poseReached: return "Tink"
        case .poseRecognized: return "Glass"
        case .scanComplete: return "Hero"
        }
    }
}

/// Turns consecutive scan updates into at most one sound each: the end of the scan, then a captured pose, then a
/// pose the head has just reached (the hold started; once per pose and 2 s of frame time, so Vision flicker at the
/// edge of the tolerance does not tick on every frame).
struct ScanCueTracker {
    private var holding: ScanPose?
    private var lastTick: [ScanPose: Double] = [:]
    static let tickInterval: Double = 2

    mutating func reset() { self = ScanCueTracker() }

    mutating func cue(for update: ScanUpdate) -> ScanSound? {
        var held: ScanPose?
        switch update.instructionKind {
        case .holdStill(let pose), .capturing(let pose):
            if update.holdProgress > 0 || update.instructionKind == .capturing(pose) { held = pose }
        default: break
        }
        defer { holding = held }
        if update.phase == .finished { return update.completedPose != nil ? .scanComplete : nil }
        if update.completedPose != nil { return .poseRecognized }
        guard let pose = held, pose != holding, (update.poseProgress[pose] ?? 0) < 1 else { return nil }
        let t = update.timestampMs / 1000
        if let last = lastTick[pose], t - last < Self.tickInterval, t >= last { return nil }
        lastTick[pose] = t
        return .poseReached
    }
}

@MainActor
protocol ScanSoundPlaying: AnyObject {
    func play(_ sound: ScanSound)
}

/// AudioToolbox system sounds: playing never blocks the main thread (NSSound.play took 33-176 ms there), and the
/// app's own setting decides, not the system's "Play user interface sound effects".
@MainActor
final class SystemScanSoundPlayer: ScanSoundPlaying {
    private var ids: [String: SystemSoundID] = [:]

    func play(_ sound: ScanSound) {
        if let id = soundID(sound.systemName) { AudioServicesPlaySystemSound(id) }
    }

    private func soundID(_ name: String) -> SystemSoundID? {
        if let id = ids[name] { return id }
        var id: SystemSoundID = 0
        let url = URL(fileURLWithPath: "/System/Library/Sounds/\(name).aiff") as CFURL
        guard AudioServicesCreateSystemSoundID(url, &id) == kAudioServicesNoError else { return nil }
        var isUISound: UInt32 = 0
        AudioServicesSetProperty(kAudioServicesPropertyIsUISound, UInt32(MemoryLayout<SystemSoundID>.size), &id,
                                 UInt32(MemoryLayout<UInt32>.size), &isUISound)
        ids[name] = id
        return id
    }
}
