// License: Apache 2.0. See LICENSE file in root directory.
//
// The guided scan's sounds (when they play, without playing them), the banner and pose guide when the head is turned
// toward no pose, and the recording file name of the save panel.

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
        switch instruction {
        case .turn(let pose, _, _), .holdStill(let pose), .capturing(let pose): u.activePose = pose
        default: break
        }
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
        // The person turns right (the phase names up, the first unfinished pose): the hold on the right pose ticks
        // once.
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

    @Test("Order-free scan: one tick per reached pose, whatever the order and the frames between")
    func cuesTickOncePerReachedPose() {
        // What the controller reports when the person turns down, then left (Vision reads it out of the tolerance
        // once), then right, then up, going back near the front (turn in any direction) between the poses; the phase
        // names up, the first unfinished pose, all along.
        var cues = ScanCueTracker()
        var progress: [ScanPose: Float] = [.front: 1, .up: 0, .down: 0, .left: 0, .right: 0]
        var t = 10.0
        var sounds: [ScanSound] = []
        func feed(_ phase: ScanPhase, _ instruction: ScanInstruction, hold: Float = 0, completed: ScanPose? = nil) {
            if let completed { progress[completed] = 1 }
            let u = update(phase, instruction, hold: hold, completed: completed, progress: progress, t: t)
            if let sound = cues.cue(for: u) { sounds.append(sound) }
            t += 1.0 / 30
        }
        func make(_ pose: ScanPose, phase before: ScanPhase, next: ScanPhase, flicker: Bool = false) {
            for _ in 0 ..< 10 { feed(before, .turnAnyDirection) }
            for _ in 0 ..< 6 { feed(before, .turn(pose, yawDelta: 10, pitchDelta: 0)) }
            for i in 1 ... 9 { feed(before, .holdStill(pose), hold: Float(i) / 9) }
            if flicker {
                // Vision reads the head out of the tolerance and back within 2 s: no second tick.
                for _ in 0 ..< 3 { feed(before, .turn(pose, yawDelta: 9, pitchDelta: 0)) }
                for i in 1 ... 9 { feed(before, .holdStill(pose), hold: Float(i) / 9) }
            }
            feed(.capturing(pose), .capturing(pose), hold: 1)
            for _ in 0 ..< 4 { feed(.capturing(pose), .capturing(pose), hold: 1) }
            feed(next, .capturing(pose), hold: 1, completed: pose)
        }
        make(.down, phase: .capturing(.up), next: .capturing(.up))
        make(.left, phase: .capturing(.up), next: .capturing(.up), flicker: true)
        make(.right, phase: .capturing(.up), next: .capturing(.up))
        for _ in 0 ..< 10 { feed(.capturing(.up), .turnAnyDirection) }
        for i in 1 ... 9 { feed(.capturing(.up), .holdStill(.up), hold: Float(i) / 9) }
        feed(.capturing(.up), .capturing(.up), hold: 1)
        feed(.finished, .finished, completed: .up)
        #expect(sounds == [.poseReached, .poseRecognized, .poseReached, .poseRecognized, .poseReached, .poseRecognized,
                           .poseReached, .scanComplete])
    }

    @Test("With no active pose the banner lists the unfinished directions, the last after or, in English and Polish")
    func bannerListsTheOpenDirections() {
        func banner(_ progress: [ScanPose: Float], skipped: Set<ScanPose> = []) -> (en: String, pl: String) {
            var u = update(.capturing(.up), .turnAnyDirection, progress: progress, t: 0)
            u.skippedPoses = skipped
            return AppTest.inBothLanguages { S.instruction(for: u) }
        }
        // The sides last, person-relative, as one item when both are open (its "or" ends the list).
        let all = banner([.front: 1, .up: 0, .down: 0, .left: 0.34, .right: 0])
        #expect(all.en == "Turn your head up, down, to your left or right")
        #expect(all.pl == "Obróć głowę w górę, w dół, w swoją lewą lub prawą stronę")
        let three = banner([.front: 1, .up: 0.67, .down: 1, .left: 0, .right: 0])
        #expect(three.en == "Turn your head up, to your left or right")
        #expect(three.pl == "Obróć głowę w górę, w swoją lewą lub prawą stronę")
        let two = banner([.front: 1, .up: 1, .down: 0, .left: 1, .right: 0.34])
        #expect(two.en == "Turn your head down or to your right")
        #expect(two.pl == "Obróć głowę w dół lub w swoją prawą stronę")
        let sides = banner([.front: 1, .up: 1, .down: 1, .left: 0, .right: 0])
        #expect(sides.en == "Turn your head to your left or right")
        #expect(sides.pl == "Obróć głowę w swoją lewą lub prawą stronę")
        let vertical = banner([.front: 1, .up: 0, .down: 0, .left: 1, .right: 1])
        #expect(vertical.en == "Turn your head up or down")
        #expect(vertical.pl == "Obróć głowę w górę lub w dół")
        let upAndLeft = banner([.front: 1, .up: 0, .down: 1, .left: 0, .right: 1])
        #expect(upAndLeft.en == "Turn your head up or to your left")
        #expect(upAndLeft.pl == "Obróć głowę w górę lub w swoją lewą stronę")
        // A single pose left: its own instruction.
        let one = banner([.front: 1, .up: 1, .down: 1, .left: 1, .right: 0])
        #expect(one.en == "Turn your head to your right" && one.pl == "Obróć głowę w swoją prawą stronę")
        // Skipped poses are not offered; a diagonal is one direction of its own.
        let diagonals = banner([.front: 1, .up: 0, .down: 1, .left: 1, .right: 1, .upLeft: 0, .upRight: 0.34,
                                .downLeft: 0, .downRight: 0], skipped: [.downLeft, .upRight])
        #expect(diagonals.en == "Turn your head up, up-left or down-right")
        #expect(diagonals.pl == "Obróć głowę w górę, w lewo do góry lub w prawo w dół")
        let four = banner([.front: 1, .up: 1, .down: 1, .left: 0, .right: 0, .upLeft: 0, .upRight: 0,
                           .downLeft: 1, .downRight: 1])
        #expect(four.en == "Turn your head up-left, up-right, to your left or right")
        #expect(four.pl == "Obróć głowę w lewo do góry, w prawo do góry, w swoją lewą lub prawą stronę")
        let downDiagonals = banner([.front: 1, .up: 1, .down: 1, .left: 1, .right: 1, .upLeft: 1, .upRight: 1,
                                    .downLeft: 0, .downRight: 0.34])
        #expect(downDiagonals.pl == "Obróć głowę w lewo w dół lub w prawo w dół")
        // More than four: no list, the pose guide's arrows show which.
        let five = banner([.front: 1, .up: 0, .down: 0, .left: 0, .right: 0, .upLeft: 0, .upRight: 1,
                           .downLeft: 1, .downRight: 1])
        #expect(five.en == "Turn your head in any direction" && five.pl == "Obróć głowę w dowolną stronę")
        // Without an update (no list known), and with an active pose: the pose's instruction.
        #expect(AppTest.inBothLanguages { S.instruction(.turnAnyDirection) }.pl == "Obróć głowę w dowolną stronę")
        var active = update(.capturing(.up), .turn(.left, yawDelta: 20, pitchDelta: 0), t: 0)
        active.turnHint = .further
        #expect(AppTest.inBothLanguages { S.instruction(for: active) }.en == "Turn your head to your left")
    }

    @Test("Turning in any direction, the pose guide points to every unfinished direction; not on other frames")
    func poseGuideDirections() {
        let progress: [ScanPose: Float] = [.front: 1, .up: 1, .down: 0.34, .left: 0, .right: 0]
        var u = update(.capturing(.up), .turnAnyDirection, progress: progress, t: 0)
        u.skippedPoses = [.right]
        #expect(PoseGuideView.directions(u) == [.init(pose: .up, done: true), .init(pose: .down, done: false),
                                                .init(pose: .left, done: false)])
        #expect(PoseGuideView.directions(nil).isEmpty)
        // No active pose for other reasons: the face lost, too close or too far, the frame entering the front pose.
        for instruction in [ScanInstruction.showFace, .moveCloser, .moveBack, .lookAtCamera] {
            let other = update(.capturing(.up), instruction, progress: progress, t: 0)
            #expect(other.activePose == nil && PoseGuideView.directions(other).isEmpty, "\(instruction)")
        }
    }

    @Test("The skip button names what it skips, in English and Polish")
    func skipButtonNamesThePose() {
        let upLeft = AppTest.inBothLanguages { S.skipPose(.upLeft) }
        #expect(upLeft.en == "Skip Up-left" && upLeft.pl == "Pomiń: góra-lewo")
        #expect(AppTest.inBothLanguages { S.skipPose(.downRight) }.pl == "Pomiń: dół-prawo")
        let remaining = AppTest.inBothLanguages { S.skipRemainingOptionalPoses }
        #expect(remaining.en == "Skip the remaining optional poses")
        #expect(remaining.pl == "Pomiń pozostałe pozy opcjonalne")
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

    @Test("One-letter words stay with the next word in a wrapped banner")
    func shortWordsStayAttached() {
        let nbsp = "\u{00A0}"
        #expect(S.keepingShortWordsAttached("Obróć głowę w dół, w swoją lewą lub prawą stronę")
            == "Obróć głowę w\(nbsp)dół, w\(nbsp)swoją lewą lub prawą stronę")
        #expect(S.keepingShortWordsAttached("Turn your head up or to your left") == "Turn your head up or to your left")
        #expect(S.keepingShortWordsAttached("") == "")
    }
}
