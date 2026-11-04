// License: Apache 2.0. See LICENSE file in root directory.
//
// Recognizing the poses a person actually makes: turns far beyond the target, any order after the front pose (the
// scan suggests none: the active pose is the one the head is within tolerance of or turned toward), turns that stop
// short of the target, and the completion event the app plays its sound on. The head poses follow the first live D455
// session (2026-11-04), where the scan never got past the right pose: Vision read yaw -74 on the right turn (76 from
// the front) and +70 on the left one, and the person turned up, down, right, left in that order.

import CameraKit
import Foundation
@testable import ScanKit
import Testing

@Suite("Guidance pose recognition")
struct GuidancePoseRecognitionTests {
    /// Feeds scripted head poses at 30 fps to a controller with the offline test configuration.
    final class Session {
        let controller: ScanController
        let tracker = ScriptedTracker { _ in nil }
        let factory = GuidanceFrameFactory()
        var timeMs: Double = 1_000_000
        var sequence: UInt64 = 0
        var updates: [ScanUpdate] = []

        init(_ configuration: ScanConfiguration = .testOffline) {
            controller = ScanController(configuration: configuration, tracker: tracker)
            controller.start(device: nil, captureConfiguration: nil)
        }

        /// Holds the head at (yaw, pitch) for `seconds`, with a little Vision noise.
        func hold(yaw: Float, pitch: Float, seconds: Double) async {
            for i in 0 ..< Int(seconds * 30) {
                let wobble = Float(i % 5) * 0.2 - 0.4
                let observation = HeadObservation(pose: HeadPose(yaw: yaw + wobble, pitch: pitch - wobble, roll: 0, confidence: 0.8),
                                                  faceRect: factory.faceRect, landmarks: [], personMask: nil)
                tracker.setScript { _ in observation }
                updates.append(await controller.process(factory.frame(sequence: sequence, timestampMs: timeMs)))
                sequence += 1
                timeMs += 1000.0 / 30
            }
        }

        /// Turns from (yaw0, pitch0) to (yaw1, pitch1) over `seconds`.
        func turn(from start: (Float, Float), to end: (Float, Float), seconds: Double) async {
            let n = max(1, Int(seconds * 30))
            for i in 1 ... n {
                let f = Float(i) / Float(n)
                await hold(yaw: start.0 + (end.0 - start.0) * f, pitch: start.1 + (end.1 - start.1) * f, seconds: 1.0 / 30)
            }
        }

        func frames(_ pose: ScanPose) -> Int { controller.finish().keyframes.filter { $0.scanPose == pose }.count }
        var completed: [ScanPose] { updates.compactMap(\.completedPose) }
        /// The updates from index `from` on.
        func since(_ from: Int) -> ArraySlice<ScanUpdate> { updates[from...] }
    }

    @Test func rightTurnFarBeyondTheTargetIsAccepted() async {
        func run(overshoot: Float) async -> Session {
            var configuration = ScanConfiguration.testOffline
            configuration.overshootToleranceDeg = overshoot
            configuration.poses = [.front, .right]
            let s = Session(configuration)
            await s.hold(yaw: 0, pitch: 4, seconds: 2)
            await s.turn(from: (0, 4), to: (-74, 15), seconds: 1)
            await s.hold(yaw: -74, pitch: 15, seconds: 3)
            return s
        }
        let now = await run(overshoot: ScanConfiguration().overshootToleranceDeg)
        #expect(now.completed == [.front, .right])
        #expect(now.updates.last?.phase == .finished)
        // The former 15-degree window: the session stays on the right pose and asks to turn back.
        let before = await run(overshoot: 15)
        #expect(before.completed == [.front])
        #expect(before.updates.last?.phase == .capturing(.right))
        if case .turn(.right, let yawDelta, _) = before.updates.last?.instructionKind {
            #expect(yawDelta > 0, "turn back toward the front")
        } else {
            Issue.record("expected a turn instruction, got \(String(describing: before.updates.last?.instructionKind))")
        }
        #expect(before.updates.last?.turnHint == .back, "the banner says to turn back, not to turn right")
    }

    @Test func shortTurnCountsForThePoseTheHeadIsHeadingTo() async {
        // After the front pose nothing is suggested: a left turn of 30 degrees (67% of the target) held slowly for
        // 1.5 s is the left pose, and a right one the right pose, whatever pose the phase names.
        let s = Session()
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        #expect(s.updates.last?.phase == .capturing(.up))
        // Barely turned (10 degrees: 22%, below headingFraction): the front still, nothing counts.
        var mark = s.updates.count
        await s.hold(yaw: 10, pitch: 4, seconds: 4)
        #expect(s.completed == [.front])
        #expect(s.since(mark).allSatisfy { $0.activePose == nil && $0.instructionKind == .turnAnyDirection })
        // 20 degrees (44%): turned toward left, which the instruction names, but short of the short-turn fraction.
        mark = s.updates.count
        await s.hold(yaw: 20, pitch: 4, seconds: 4)
        #expect(s.completed == [.front])
        #expect(s.since(mark).dropFirst(5).allSatisfy {
            if case .turn(.left, _, _) = $0.instructionKind { return $0.activePose == .left }
            return false
        })
        // 30 degrees: a second is not enough, held longer it counts.
        mark = s.updates.count
        await s.hold(yaw: 30, pitch: 4, seconds: 1)
        #expect(s.completed == [.front])
        await s.hold(yaw: 30, pitch: 4, seconds: 2)
        #expect(s.completed == [.front, .left])
        #expect(s.since(mark).filter(\.acceptedKeyframe).allSatisfy { $0.activePose == .left })
        let left = s.controller.finish().keyframes.filter { $0.scanPose == .left }
        #expect(left.count == 3 && left.allSatisfy { abs($0.headPose.yaw - 30) < 1 })
        // The same on the other side; the phase names up all along.
        let s2 = Session()
        await s2.hold(yaw: 0, pitch: 4, seconds: 2)
        await s2.hold(yaw: -30, pitch: 4, seconds: 3)
        #expect(s2.completed == [.front, .right])
    }

    @Test func withoutPosesInAnyOrderTheShortTurnCountsOnlyForTheSuggestedPose() async {
        // posesInAnyOrder = false keeps the former fixed suggestion: the scan suggests up; the person holds a right
        // turn of 30 degrees (67% of the target): not the right pose, which needs its full tolerance.
        var configuration = ScanConfiguration.testOffline
        configuration.posesInAnyOrder = false
        let s = Session(configuration)
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        #expect(s.updates.last?.activePose == .up)
        if case .turn(.up, _, let pitchDelta) = s.updates.last?.instructionKind {
            #expect(pitchDelta > 20)
        } else {
            Issue.record("expected the suggested up pose, got \(String(describing: s.updates.last?.instructionKind))")
        }
        await s.turn(from: (0, 4), to: (-30, 4), seconds: 0.5)
        await s.hold(yaw: -30, pitch: 4, seconds: 3)
        #expect(s.completed == [.front])
        #expect(s.updates.allSatisfy { $0.instructionKind != .turnAnyDirection })
        await s.turn(from: (-30, 4), to: (-52, 6), seconds: 0.4)
        await s.hold(yaw: -52, pitch: 6, seconds: 2)
        #expect(s.completed == [.front, .right])
    }

    @Test func anyUnfinishedDirectionBecomesTheActivePose() async {
        // From the front, a turn toward any unfinished pose makes it the active pose and captures it; the phase (up,
        // the first unfinished pose) plays no part.
        let turns: [(ScanPose, Float, Float)] = [(.up, 0, 27), (.down, 1, -21), (.left, 50, 6), (.right, -52, 5)]
        for (pose, yaw, pitch) in turns {
            let s = Session()
            await s.hold(yaw: 0, pitch: 4, seconds: 2)
            #expect(s.updates.last?.phase == .capturing(.up))
            #expect(s.updates.last?.activePose == nil, "\(pose)")
            let mark = s.updates.count
            await s.turn(from: (0, 4), to: (yaw, pitch), seconds: 0.8)
            await s.hold(yaw: yaw, pitch: pitch, seconds: 2)
            #expect(s.completed == [.front, pose], "\(pose)")
            let after = s.since(mark)
            #expect(after.compactMap(\.activePose).allSatisfy { $0 == pose }, "\(pose): only \(pose) is active")
            // Active from about a third of the way, before the pose is within tolerance.
            let firstActive = after.firstIndex { $0.activePose == pose }
            let firstHold = after.firstIndex { $0.instructionKind == .holdStill(pose) }
            #expect(firstActive != nil && firstHold != nil && firstActive! < firstHold!, "\(pose)")
            if let firstActive, case .turn(pose, _, _) = after[firstActive].instructionKind {} else {
                Issue.record("\(pose): the instruction names the active pose")
            }
            // The phase follows the keyframes (CLI, trace); after the pose it is the first unfinished one in order.
            #expect(after.first { $0.acceptedKeyframe }?.phase == .capturing(pose))
            #expect(s.updates.last?.phase == .capturing(pose == .up ? .down : .up), "\(pose)")
            #expect(s.frames(pose) == 3)
        }
    }

    @Test func nearTheFrontTheInstructionIsToTurnAnyDirection() async {
        let s = Session()
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        let mark = s.updates.count
        // A few degrees off the front, and 10 degrees (22% of a side pose) on the other side: no pose is active.
        await s.hold(yaw: 3, pitch: 6, seconds: 1)
        await s.hold(yaw: -10, pitch: 3, seconds: 1)
        for u in s.since(mark) {
            #expect(u.instructionKind == .turnAnyDirection)
            #expect(u.activePose == nil && u.rejection == nil && u.targetDelta == nil && u.angularErrorDeg == nil)
            #expect(u.turnHint == nil && u.holdProgress == 0 && !u.acceptedKeyframe)
            #expect(u.unfinishedPoses == [.up, .down, .left, .right])
        }
        #expect(s.updates.last?.instruction == "Turn your head in any direction")
        // 9 degrees up (36% of the up target): up becomes the active pose, the instruction points up.
        await s.turn(from: (-10, 3), to: (0, 13), seconds: 0.3)
        await s.hold(yaw: 0, pitch: 13, seconds: 0.5)
        #expect(s.updates.last?.activePose == .up)
        if case .turn(.up, _, let pitchDelta) = s.updates.last?.instructionKind {
            #expect(pitchDelta > 10)
        } else {
            Issue.record("expected a turn toward up, got \(String(describing: s.updates.last?.instructionKind))")
        }
        // Back at the front after a pose: the finished pose is no longer listed.
        await s.hold(yaw: 0, pitch: 27, seconds: 2)
        #expect(s.completed == [.front, .up])
        await s.hold(yaw: 0, pitch: 4, seconds: 1)
        #expect(s.updates.last?.instructionKind == .turnAnyDirection)
        #expect(s.updates.last?.unfinishedPoses == [.down, .left, .right])
    }

    @Test func frontPoseCompletesTenToTwelveDegreesOff() async {
        func run(_ yaw: Float, _ pitch: Float, tolerance: Float = 12) async -> Session {
            var configuration = ScanConfiguration.testOffline
            configuration.frontToleranceDeg = tolerance
            let s = Session(configuration)
            await s.hold(yaw: yaw, pitch: pitch, seconds: 2)
            return s
        }
        #expect(ScanConfiguration().frontToleranceDeg == 12 && ScanConfiguration().angleTolerance == 8)
        for (yaw, pitch): (Float, Float) in [(0, 11), (7, 8.5), (-10.5, 0), (0, -10)] {
            let s = await run(yaw, pitch)
            #expect(s.completed == [.front], "front at (\(yaw), \(pitch))")
            #expect(s.frames(.front) == 3)
            #expect(s.updates.contains { $0.instructionKind == .capturing(.front) && $0.activePose == .front })
        }
        // Beyond 12 degrees it is not the front yet; the former 8-degree circle refused 11 degrees.
        let beyond = await run(0, 13.5)
        #expect(beyond.completed.isEmpty)
        #expect(beyond.updates.last?.activePose == .front)
        if case .turn(.front, _, let pitchDelta) = beyond.updates.last?.instructionKind {
            #expect(pitchDelta < -12)
        } else {
            Issue.record("expected a turn toward the front, got \(String(describing: beyond.updates.last))")
        }
        let eight = await run(0, 11, tolerance: 8)
        #expect(eight.completed.isEmpty)
    }

    @Test func diagnosticsAttributeFramesToTheActivePose() async {
        let s = Session()
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        // A left turn with the chin 12 degrees up (Vision reads large yaw partly as pitch).
        await s.turn(from: (0, 4), to: (50, 16), seconds: 0.6)
        await s.hold(yaw: 50, pitch: 16, seconds: 2)
        #expect(s.completed == [.front, .left])
        let d = s.controller.diagnostics
        let front = d.poses[.front], left = d.poses[.left]
        #expect(front?.outcomes["accepted"] == 3 && left?.outcomes["accepted"] == 3)
        #expect((d.poses[.up]?.frames ?? 0) == 0, "up was never the active pose")
        // The frames near the front after the front pose, and the start of the turn, worked on no pose.
        #expect(d.noActivePose.frames > 25)
        #expect(d.noActivePose.outcomes["turnAnyDirection"] == d.noActivePose.frames)
        #expect(abs(d.noActivePose.seconds - Double(d.noActivePose.frames) / 30) < 0.05)
        // Every capture frame counts once: the frame after one whose phase was capturing.
        let captureFrames = s.updates.indices.dropFirst().filter {
            if case .capturing = s.updates[$0 - 1].phase { return true }
            return false
        }.count
        #expect(d.poses.values.reduce(0) { $0 + $1.frames } + d.noActivePose.frames == captureFrames)
        // The furthest turn toward a pose counts on the frames it is the active pose, or the head within its sideways
        // tolerance: the pitch of the left turn is no turn up (up's 12 degrees there would read as half its target).
        #expect(abs((left?.furthestDeg ?? 0) - 50) < 1)
        let up = d.poses[.up]?.furthestDeg ?? .nan
        #expect(up > -1 && up < 5, "up measured only while the head was within 15 degrees of yaw: \(up)")
        #expect((d.poses[.right]?.furthestDeg ?? .infinity) < 1)
        #expect(d.noActivePose.summary.hasPrefix(String(format: "%.1f s, %d frames: turnAnyDirection 100%%",
                                                        d.noActivePose.seconds, d.noActivePose.frames)))
    }

    @Test func skipTargetsThePoseTheUpdateNames() async {
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.front, .up, .upLeft, .left]
        let s = Session(configuration)
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        // Near the front no pose is active: the update names none to skip, and required poses are still open.
        #expect(s.updates.last?.activePose == nil && s.updates.last?.skipPose == nil)
        #expect(s.updates.last?.canSkipRemainingPoses == false)
        // Without a pose, skipPose() falls back to the phase's pose (up, required): refused, like the required ones.
        #expect(!s.controller.canSkipCurrentPose && !s.controller.skipPose())
        #expect(!s.controller.skipPose(.left) && !s.controller.skipPose(.front) && !s.controller.skipPose(.downLeft))
        // Turned halfway toward up-left (optional): the update names it, and exactly that pose is skipped; the phase
        // stays on up.
        await s.hold(yaw: 15, pitch: 14, seconds: 0.5)
        #expect(s.updates.last?.activePose == .upLeft && s.updates.last?.skipPose == .upLeft)
        #expect(s.controller.canSkipCurrentPose)
        #expect(s.controller.skipPose(.upLeft))
        #expect(!s.controller.skipPose(.upLeft), "skipped already")
        #expect(s.controller.phase == .capturing(.up))
        await s.hold(yaw: 15, pitch: 14, seconds: 0.2)
        #expect(s.updates.last?.skippedPoses == [.upLeft])
        #expect(s.updates.last?.activePose != .upLeft && s.updates.last?.skipPose == nil)
        #expect(s.controller.finish().notes["skipped_poses"] == "upLeft")
    }

    @Test func onlyOptionalPosesLeftCanAllBeSkipped() async {
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.front, .up, .upLeft, .downRight]
        let s = Session(configuration)
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        #expect(s.updates.last?.canSkipRemainingPoses == false, "up (required) is open")
        await s.turn(from: (0, 4), to: (0, 30), seconds: 0.6)
        await s.hold(yaw: 0, pitch: 30, seconds: 2)
        #expect(s.completed == [.front, .up])
        // Back near the front, turned toward neither optional pose: no pose to name, the update offers to skip both.
        await s.hold(yaw: 0, pitch: 4, seconds: 0.5)
        #expect(s.updates.last?.instructionKind == .turnAnyDirection)
        #expect(s.updates.last?.skipPose == nil && s.updates.last?.canSkipRemainingPoses == true)
        #expect(s.controller.skipRemainingOptionalPoses())
        #expect(s.controller.phase == .finished)
        #expect(!s.controller.skipRemainingOptionalPoses())
        #expect(s.controller.finish().notes["skipped_poses"] == "upLeft,downRight")
    }

    @Test func turnedTowardAPoseBeyondItsSidewaysToleranceTheHintCorrectsIt() async {
        // Only up and left are open. The chin 20 degrees up with the head turned 17 degrees to a side is beyond the
        // sideways tolerance (15 degrees) of both up and left: neither is reached nor turned toward. The head is
        // clearly turned, though: the instruction names the closest pose, up, with the correction, not "turn your head
        // in any direction", and nothing counts until the head is within up's sideways tolerance.
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.front, .up, .left]
        for (yaw, hint): (Float, TurnHint) in [(17, .faceRight), (-17, .faceLeft)] {
            let s = Session(configuration)
            await s.hold(yaw: 0, pitch: 4, seconds: 2)
            await s.turn(from: (0, 4), to: (yaw, 24), seconds: 0.5)
            let mark = s.updates.count
            await s.hold(yaw: yaw, pitch: 24, seconds: 3)
            for u in s.since(mark).dropFirst(8) {
                #expect(u.activePose == .up && u.turnHint == hint, "yaw \(yaw)")
                if case .turn(.up, _, _) = u.instructionKind {} else {
                    Issue.record("yaw \(yaw): expected a turn toward up, got \(u.instructionKind)")
                }
            }
            // For the instruction only: no hold, and no short turn however long it is held (80 % of up's target).
            #expect(s.completed == [.front], "yaw \(yaw)")
            #expect(s.since(mark).allSatisfy { $0.holdProgress == 0 && !$0.acceptedKeyframe })
            #expect((s.controller.diagnostics.poses[.up]?.outcomes["offAxis"] ?? 0) >= 80)
            // Corrected as the hint says, the head is within up's tolerance: captured.
            await s.turn(from: (yaw, 24), to: (0, 24), seconds: 0.5)
            await s.hold(yaw: 0, pitch: 24, seconds: 3)
            #expect(s.completed == [.front, .up], "yaw \(yaw)")
        }
    }

    @Test func shortTurnTimeSurvivesJitterBetweenNeighbouringPoses() async {
        // With the diagonals on, a left turn of 35 degrees with the chin about 5 degrees up sits where up-left's
        // sideways tolerance ends: 1-degree steps of Vision's reading move the head in and out of it, and while in,
        // up-left is the pose turned toward most (90 % of its target against left's 78 %). Left qualifies for the
        // short turn all along and keeps its own start time: after 1.5 s it is reached, stays the active pose and is
        // captured. One start time for whichever pose was turned toward most restarted at every switch, for ever.
        var configuration = ScanConfiguration.testOffline
        configuration.poses = ScanPose.allCases
        let s = Session(configuration)
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        #expect(s.completed == [.front])
        await s.turn(from: (0, 4), to: (35, 8.8), seconds: 0.5)
        let mark = s.updates.count
        for step in 0 ..< 12 {
            await s.hold(yaw: 35, pitch: step % 2 == 0 ? 8.8 : 9.8, seconds: 0.4)
        }
        let jitter = s.since(mark)
        #expect(jitter.contains { $0.activePose == .upLeft }, "the jitter switches the pose turned toward most")
        #expect(s.completed == [.front, .left])
        #expect(s.frames(.left) == 3 && s.frames(.upLeft) == 0)
        // Once reached, left stayed the active pose until it was complete.
        if let reached = jitter.firstIndex(where: { $0.instructionKind == .holdStill(.left) }),
           let done = jitter.firstIndex(where: { $0.completedPose == .left }) {
            #expect(jitter[reached ... done].allSatisfy { $0.activePose == .left })
        } else {
            Issue.record("left was never held")
        }
    }

    @Test func turnHintSaysWhichWayToCorrect() {
        func hint(_ pose: ScanPose, _ yaw: Float, _ pitch: Float) -> TurnHint {
            TurnHint.hint(pose: pose, delta: SIMD2(yaw, pitch), along: 8, across: 15)
        }
        #expect(hint(.right, -20, 1) == .further)
        #expect(hint(.right, 16, -2) == .back)
        #expect(hint(.right, 2, 12) == .chinUp)
        #expect(hint(.left, -2, -12) == .chinDown)
        #expect(hint(.up, 10, 1) == .faceLeft)
        #expect(hint(.down, -10, -1) == .faceRight)
        #expect(hint(.front, 10, 10) == .further)
    }

    @Test func perpendicularToleranceGrowsBeyondTheTarget() {
        let offset = SIMD2<Float>(1.4, 4.6)
        #expect(ScanController.acrossTolerance(pose: .right, current: SIMD2(-40, 10), frontOffset: offset, base: 15, growth: 0.3) == 15)
        let far = ScanController.acrossTolerance(pose: .right, current: SIMD2(-74, 18), frontOffset: offset, base: 15, growth: 0.3)
        #expect(abs(far - (15 + 0.3 * (75.4 - 45))) < 1e-4)
        #expect(ScanController.acrossTolerance(pose: .front, current: SIMD2(-74, 18), frontOffset: offset, base: 15, growth: 0.3) == 15)
    }

    @Test func posesAreRecognizedInTheOrderThePersonTurns() async {
        let s = Session()
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        #expect(s.completed == [.front])
        #expect(s.updates.last?.phase == .capturing(.up), "the phase names the first unfinished pose")
        #expect(s.updates.last?.activePose == nil, "but the scan suggests none")
        // The person turns right: the instruction names the pose being held, and from its first keyframe on the
        // phase names it until it is done.
        await s.turn(from: (0, 4), to: (-70, 12), seconds: 1)
        await s.hold(yaw: -70, pitch: 12, seconds: 2)
        #expect(s.updates.contains { $0.instructionKind == .holdStill(.right) && $0.activePose == .right })
        let firstRight = s.updates.first { $0.acceptedKeyframe && $0.keyframeID == 3 }
        #expect(firstRight?.phase == .capturing(.right))
        #expect(s.completed.last == .right)
        for (pose, target) in [(ScanPose.down, (Float(1), Float(-22))), (.left, (62, 14)), (.up, (0, 27))] {
            await s.turn(from: (0, 4), to: target, seconds: 0.8)
            await s.hold(yaw: target.0, pitch: target.1, seconds: 2)
            #expect(s.completed.last == pose)
        }
        #expect(s.completed == [.front, .right, .down, .left, .up], "each pose once, in the order it was made")
        #expect(s.updates.last?.phase == .finished)
        for pose in ScanPose.required { #expect(s.frames(pose) == 3, "\(pose)") }
        // The keyframes carry the pose they show.
        let capture = s.controller.finish()
        #expect(capture.keyframes.filter { $0.scanPose == .right }.allSatisfy { $0.headPose.yaw < -60 })
    }

    @Test func shortTurnIsAcceptedOnlyWhenHeld() async {
        // Up at 16 degrees from the front (64% of the 25-degree target): beyond the 8-degree tolerance.
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.front, .up]
        let passing = Session(configuration)
        await passing.hold(yaw: 0, pitch: 4, seconds: 2)
        await passing.hold(yaw: 0, pitch: 20, seconds: 1.0)
        #expect(passing.completed == [.front], "a second at the short angle is not enough")
        #expect(passing.updates.contains { if case .turn(.up, _, _) = $0.instructionKind { return true }; return false })

        let held = Session(configuration)
        await held.hold(yaw: 0, pitch: 4, seconds: 2)
        await held.hold(yaw: 0, pitch: 20, seconds: 3)
        #expect(held.completed == [.front, .up])
        // 60% is the limit: 12 degrees (48%) never counts.
        let tooShort = Session(configuration)
        await tooShort.hold(yaw: 0, pitch: 4, seconds: 2)
        await tooShort.hold(yaw: 0, pitch: 16, seconds: 4)
        #expect(tooShort.completed == [.front])
    }

    @Test func turnFractionAlongTheTargetDirection() {
        let offset = SIMD2<Float>(1, 4)
        #expect(abs((ScanController.turnFraction(pose: .right, current: SIMD2(-44, 4), frontOffset: offset, across: 15) ?? 0) - 1) < 1e-5)
        #expect(abs((ScanController.turnFraction(pose: .up, current: SIMD2(1, 19), frontOffset: offset, across: 15) ?? 0) - 0.6) < 1e-5)
        #expect(ScanController.turnFraction(pose: .front, current: .zero, frontOffset: .zero, across: 15) == nil)
        // Turned up while the pose is a side one: beyond the perpendicular tolerance.
        #expect(ScanController.turnFraction(pose: .left, current: SIMD2(30, 30), frontOffset: .zero, across: 15) == nil)
    }
}
