// License: Apache 2.0. See LICENSE file in root directory.
//
// Recognizing the poses a person actually makes: turns far beyond the target, any order after the front pose,
// turns that stop short of the target, and the completion event the app plays its sound on. The head poses follow
// the first live D455 session (2026-11-04), where the scan never got past the right pose: Vision read yaw -74 on the
// right turn (76 from the front) and +70 on the left one, and the person turned up, down, right, left in that order.

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

    @Test func shortTurnCountsOnlyForTheSuggestedPose() async {
        // The scan suggests up; the person holds a right turn of 30 degrees (67% of the target): not the right pose.
        let s = Session()
        await s.hold(yaw: 0, pitch: 4, seconds: 2)
        await s.turn(from: (0, 4), to: (-30, 4), seconds: 0.5)
        await s.hold(yaw: -30, pitch: 4, seconds: 3)
        #expect(s.completed == [.front])
        await s.turn(from: (-30, 4), to: (-52, 6), seconds: 0.4)
        await s.hold(yaw: -52, pitch: 6, seconds: 2)
        #expect(s.completed == [.front, .right])
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
        #expect(s.updates.last?.phase == .capturing(.up), "the scan suggests the next pose in order")
        // The person turns right while the scan suggests up: the instruction names the pose being held, and from its
        // first keyframe on the scan suggests it until it is done.
        await s.turn(from: (0, 4), to: (-70, 12), seconds: 1)
        await s.hold(yaw: -70, pitch: 12, seconds: 2)
        #expect(s.updates.contains { $0.instructionKind == .holdStill(.right) })
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
