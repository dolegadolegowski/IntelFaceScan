// License: Apache 2.0. See LICENSE file in root directory.
//
// The guided scan's sounds (when they play, without playing them) and the recording file name of the save panel.

import Foundation
import ScanKit
import Testing
import UniformTypeIdentifiers
@testable import IntelFaceScanApp

@MainActor
@Suite("Scan feedback")
struct AppScanFeedbackTests {
    private func update(_ phase: ScanPhase, _ instruction: ScanInstruction, hold: Float = 0, completed: ScanPose? = nil,
                        progress: [ScanPose: Float] = [:], t: Double) -> ScanUpdate {
        var u = ScanUpdate()
        u.phase = phase
        u.instructionKind = instruction
        u.holdProgress = hold
        u.completedPose = completed
        u.poseProgress = progress
        u.timestampMs = t * 1000
        return u
    }

    @Test("A tick when a pose is reached, a chime when it is captured, a fanfare at the end")
    func cuesFollowTheScan() {
        var cues = ScanCueTracker()
        #expect(cues.cue(for: update(.capturing(.up), .turn(.up, yawDelta: 0, pitchDelta: 20), t: 0)) == nil)
        // The person turns right while up is suggested: the hold on the right pose ticks once.
        #expect(cues.cue(for: update(.capturing(.up), .holdStill(.right), t: 0.1)) == nil, "hold not started yet")
        #expect(cues.cue(for: update(.capturing(.up), .holdStill(.right), hold: 0.2, t: 0.2)) == .poseReached)
        #expect(cues.cue(for: update(.capturing(.up), .holdStill(.right), hold: 0.6, t: 0.3)) == nil)
        #expect(cues.cue(for: update(.capturing(.right), .capturing(.right), hold: 1, t: 0.4)) == nil)
        // Vision flicker out of the tolerance and back within 2 s does not tick again.
        #expect(cues.cue(for: update(.capturing(.right), .turn(.right, yawDelta: -9, pitchDelta: 0), t: 0.5)) == nil)
        #expect(cues.cue(for: update(.capturing(.right), .holdStill(.right), hold: 0.1, t: 0.6)) == nil)
        #expect(cues.cue(for: update(.capturing(.up), .capturing(.right), hold: 1, completed: .right,
                                     progress: [.right: 1], t: 0.9)) == .poseRecognized)
        // A pose already captured does not tick.
        #expect(cues.cue(for: update(.capturing(.up), .turn(.up, yawDelta: 0, pitchDelta: 9), t: 3)) == nil)
        #expect(cues.cue(for: update(.capturing(.up), .holdStill(.right), hold: 0.5, progress: [.right: 1], t: 3.1)) == nil)
        #expect(cues.cue(for: update(.finished, .finished, completed: .up, progress: [.right: 1, .up: 1], t: 5)) == .scanComplete)
        // Finishing by hand completes no pose: no fanfare.
        cues.reset()
        #expect(cues.cue(for: update(.finished, .finished, t: 6)) == nil)
    }

    @Test("A recording player gets the cues only while the setting is on")
    func cuesReachThePlayerWhenEnabled() {
        final class Recorder: ScanSoundPlaying { var played: [ScanSound] = [] ; func play(_ sound: ScanSound) { played.append(sound) } }
        let recorder = Recorder()
        let model = AppModel(defaults: nil, soundPlayer: recorder)
        model.handleScanCue(update(.capturing(.up), .holdStill(.up), hold: 0.3, t: 0))
        model.settings.scanSounds = false
        model.handleScanCue(update(.capturing(.up), .capturing(.up), hold: 1, completed: .up, progress: [.up: 1], t: 1))
        #expect(recorder.played == [.poseReached])
        for sound in [ScanSound.poseReached, .poseRecognized, .scanComplete] {
            #expect(FileManager.default.fileExists(atPath: "/System/Library/Sounds/\(sound.systemName).aiff"), "\(sound)")
        }
        #expect(Set([ScanSound.poseReached, .poseRecognized, .scanComplete].map(\.systemName)).count == 3)
    }

    @Test("A turn beyond the tolerance says which way to correct it")
    func bannerUsesTheTurnHint() {
        var u = update(.capturing(.right), .turn(.right, yawDelta: 30, pitchDelta: 0), t: 0)
        #expect(S.instruction(for: u) == S.poseInstruction(.right), "no hint: the pose instruction")
        u.turnHint = .further
        #expect(S.instruction(for: u) == S.poseInstruction(.right))
        u.turnHint = .back
        #expect(S.instruction(for: u) == S.turnHint(.back))
        #expect(S.instruction(for: u) != S.poseInstruction(.right))
        for hint in TurnHint.allCases { #expect(!S.turnHint(hint).isEmpty) }
    }

    @Test("The scan sounds are on by default and survive older stored settings")
    func scanSoundsSetting() throws {
        #expect(AppSettings().scanSounds)
        let suite = "IntelFaceScanAppTests.sounds.\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suite))
        defer { defaults.removePersistentDomain(forName: suite) }
        var stored = try #require(try JSONSerialization.jsonObject(with: JSONEncoder().encode(AppSettings())) as? [String: Any])
        stored.removeValue(forKey: "scanSounds")
        defaults.set(try JSONSerialization.data(withJSONObject: stored), forKey: AppSettings.defaultsKey)
        #expect(AppSettings.load(from: defaults).scanSounds)
        var off = AppSettings()
        off.scanSounds = false
        off.save(to: defaults)
        #expect(!AppSettings.load(from: defaults).scanSounds)
    }

    @Test("The save panel's doubled extension is dropped, unless that would replace another file")
    func recordingNameKeepsOneExtension() {
        let folder = URL(fileURLWithPath: "/tmp/ifs-panel", isDirectory: true)
        let types = [FilePanels.db3]
        let doubled = folder.appendingPathComponent("head-20261104-105231.db3.db3")
        let single = folder.appendingPathComponent("head-20261104-105231.db3")
        #expect(FilePanels.normalizedSaveURL(doubled, contentTypes: types, exists: { _ in false }) == single)
        #expect(FilePanels.normalizedSaveURL(single, contentTypes: types, exists: { _ in false }) == single)
        #expect(FilePanels.normalizedSaveURL(doubled, contentTypes: types, exists: { $0 == single }) == doubled)
        let other = folder.appendingPathComponent("head.obj")
        #expect(FilePanels.normalizedSaveURL(other, contentTypes: types, exists: { _ in false }) == other)
        #expect(FilePanels.normalizedSaveURL(doubled, contentTypes: [], exists: { _ in false }) == doubled)
    }
}
