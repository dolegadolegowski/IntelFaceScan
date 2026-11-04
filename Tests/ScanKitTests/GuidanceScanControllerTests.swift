// License: Apache 2.0. See LICENSE file in root directory.
//
// The guided scan state machine driven by scripted observations and synthetic RGB-D frames.

import CameraKit
import CoreGraphics
import Foundation
import simd
import Testing
@testable import ScanKit

@Suite struct GuidanceScanControllerTests {
    private func makeController(_ configuration: ScanConfiguration = .testOffline) -> (ScanController, ScriptedTracker) {
        let tracker = ScriptedTracker { _ in nil }
        let controller = ScanController(configuration: configuration, tracker: tracker)
        controller.start(device: nil, captureConfiguration: nil)
        return (controller, tracker)
    }

    @Test func fullScanAcceptsFramesPerPoseInOrder() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 900)

        #expect(sim.updates.last?.phase == .finished)
        let capture = controller.finish()
        #expect(capture.keyframes.count == 15)
        for pose in ScanPose.required {
            let frames = capture.keyframes.filter { $0.scanPose == pose }
            #expect(frames.count == 3, "pose \(pose)")
            // Keyframes of a pose are spaced in time and taken at the target.
            for (a, b) in zip(frames, frames.dropFirst()) {
                #expect(b.frame.timestampMs - a.frame.timestampMs >= 200 - 1e-6)
            }
            for kf in frames {
                #expect(abs(kf.headPose.yaw - pose.target.yaw) <= 8)
                #expect(abs(kf.headPose.pitch - pose.target.pitch) <= 8)
            }
        }
        // Poses are captured in the configured order and keyframe ids are sequential.
        var order: [ScanPose] = []
        for kf in capture.keyframes where order.last != kf.scanPose { order.append(kf.scanPose) }
        #expect(order == ScanPose.required)
        #expect(capture.keyframes.map(\.id) == Array(0 ..< 15))
        let last = sim.updates.last
        #expect(last?.overallProgress == 1)
        #expect(ScanPose.required.allSatisfy { last?.poseProgress[$0] == 1 })
        #expect(last?.instructionKind == .finished)
    }

    @Test func keyframesCarryMaskQualityAndLandmarks() async throws {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 300) { $0.keyframeCount >= 1 }
        let capture = controller.finish()
        let kf = try #require(capture.keyframes.first)
        let mask = try #require(kf.mask)
        #expect(mask.count == sim.factory.width * sim.factory.height)
        #expect(HeadMaskBuilder.count(mask) > 500)
        #expect(kf.quality.validDepthPoints > 500)
        #expect(abs(kf.quality.faceDistanceM - 0.62) < 0.005)
        #expect(kf.quality.validDepthRatio > 0.95)
        #expect(kf.quality.sharpness > 100)
        #expect(kf.faceRect == sim.factory.faceRect)
        #expect(kf.landmarks.count == 1)
        #expect(kf.scanPose == .front)
        // Segmentation was asked for (the scripted tracker has none, so the ellipse fallback was used).
        #expect(tracker.segmentationCalls >= 1)
    }

    @Test func positioningWaitsForGoodDistance() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 60, options: { _ in .init(distance: 0.95) }, until: { _ in false })
        #expect(controller.phase == .positioning)
        #expect(sim.updates.last?.instructionKind == .moveCloser)
        #expect(sim.updates.last?.rejection == .tooFar)
        #expect(sim.updates.last?.distanceAdvice == .tooFar)

        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 30, options: { _ in .init(distance: 0.45) }, until: { _ in false })
        #expect(controller.phase == .positioning)
        #expect(sim.updates.last?.instructionKind == .moveBack)
        #expect(sim.updates.last?.rejection == .tooClose)

        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 30, until: { $0.phase != .positioning })
        #expect(controller.phase == .capturing(.front))
        // 0.5 s hold at 30 fps.
        #expect(sim.updates.count >= 15 && sim.updates.count <= 17)
    }

    @Test func noFaceKeepsPositioning() async {
        let (controller, tracker) = makeController()
        let factory = GuidanceFrameFactory()
        tracker.setScript { _ in nil }
        var last = ScanUpdate()
        for i in 0 ..< 40 {
            last = await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 1000 + Double(i) * 33.3))
        }
        #expect(last.phase == .positioning)
        #expect(last.instructionKind == .showFace)
        #expect(last.rejection == .noFace)
        #expect(last.observation == nil)
    }

    @Test func rejectsHeadMotion() async {
        let (controller, tracker) = makeController()
        let factory = GuidanceFrameFactory()
        var updates: [ScanUpdate] = []
        // Steady for positioning, then a fast oscillation that stays inside the angle tolerance.
        for i in 0 ..< 150 {
            let t = Double(i) / 30
            let yaw: Float = i < 20 ? 0 : 6 * Float(sin(2 * Double.pi * 2.5 * t))
            let obs = HeadObservation(pose: HeadPose(yaw: yaw, pitch: 0, roll: 0, confidence: 0.9),
                                      faceRect: factory.faceRect, landmarks: [], personMask: nil)
            tracker.setScript { _ in obs }
            updates.append(await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 5000 + t * 1000)))
        }
        #expect(controller.keyframeCount == 0)
        #expect(updates.contains { $0.rejection == .headMoving })
        #expect(updates.allSatisfy { !$0.acceptedKeyframe })
    }

    @Test func jitteryVisionOnAStillHeadDoesNotBlockThePoses() async {
        // Vision jumps by 4 degrees every 6-14 frames per angle and has single-frame outliers while the head is held:
        // the scan must take about as long as with exact readings. The motion gate used to read every jump as
        // 20-25 deg/s and restart the hold (seeds 1-3: not finished in 900 frames, 540-570 x headMoving).
        func run(_ jitter: VisionJitter?) async -> (ScanPhase?, keyframes: Int, frames: Int, moving: Int) {
            var configuration = ScanConfiguration.testOffline
            configuration.angleTolerance = 12   // the jumps must not push the pose out of tolerance instead
            let (controller, tracker) = makeController(configuration)
            var sim = ScanSimulation()
            sim.user.jitter = jitter
            await sim.run(controller, tracker: tracker, maxFrames: 900)
            return (sim.updates.last?.phase, controller.finish().keyframes.count, sim.updates.count,
                    sim.rejections.filter { $0 == .headMoving }.count)
        }
        let exact = await run(nil)
        #expect(exact.0 == .finished && exact.keyframes == 15)
        for seed: UInt64 in 1 ... 3 {
            let jittery = await run(VisionJitter(seed: seed))
            #expect(jittery.0 == .finished && jittery.keyframes == 15, "seed \(seed)")
            #expect(Double(jittery.frames) < 1.2 * Double(exact.frames),
                    "seed \(seed): \(jittery.frames) frames, exact readings \(exact.frames)")
            // headMoving while arriving at each pose (turning at 60 deg/s) is expected; jitter adds little to it.
            #expect(Double(jittery.moving) < 1.5 * Double(exact.moving) + 5,
                    "seed \(seed): \(jittery.moving) x headMoving, exact readings \(exact.moving)")
        }
    }

    @Test func rejectsHeadMotionThroughJitter() async {
        // The 2.5 Hz shake of rejectsHeadMotion with Vision's jumps on top: still no keyframe.
        let (controller, tracker) = makeController()
        let factory = GuidanceFrameFactory()
        var jitter = VisionJitter(seed: 7)
        var updates: [ScanUpdate] = []
        for i in 0 ..< 150 {
            let t = Double(i) / 30
            let e = jitter.next()
            let yaw: Float = i < 20 ? 0 : 6 * Float(sin(2 * Double.pi * 2.5 * t))
            let obs = HeadObservation(pose: HeadPose(yaw: yaw + (i < 20 ? 0 : e.x), pitch: i < 20 ? 0 : e.y, roll: 0,
                                                     confidence: 0.9),
                                      faceRect: factory.faceRect, landmarks: [], personMask: nil)
            tracker.setScript { _ in obs }
            updates.append(await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 5000 + t * 1000)))
        }
        #expect(controller.keyframeCount == 0)
        #expect(updates.contains { $0.rejection == .headMoving })
    }

    @Test func rejectsCameraMotion() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        // 0.2 rad/s = 11.5 deg/s, above the 4 deg/s gate.
        await sim.run(controller, tracker: tracker, maxFrames: 120, options: { _ in .init(gyroRadPerSec: 0.2) },
                      until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(sim.rejections.contains(.cameraMoving))
        #expect(sim.updates.last?.instructionKind == .holdStill(.front))

        // The camera comes to rest: frames are accepted again.
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 60, until: { $0.keyframeCount >= 1 })
        #expect(controller.keyframeCount == 1)
    }

    @Test func rejectsBlurAgainstRecentSharpness() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        // Reach the front pose hold with sharp frames.
        await sim.run(controller, tracker: tracker, maxFrames: 120, until: { $0.keyframeCount >= 1 })
        #expect(controller.keyframeCount == 1)
        // A burst of motion-blurred frames is rejected...
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 10, options: { _ in .init(texture: .blurry) },
                      until: { _ in false })
        #expect(sim.rejections.contains(.blurry))
        #expect(sim.accepted.isEmpty)
        #expect(controller.keyframeCount == 1)
        // ...and sharp frames are accepted again.
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 30, until: { $0.keyframeCount >= 2 })
        #expect(controller.keyframeCount == 2)
    }

    @Test func absoluteSharpnessGate() async {
        var configuration = ScanConfiguration.testOffline
        configuration.minSharpness = 1e9
        let (controller, tracker) = makeController(configuration)
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 120, until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(sim.rejections.contains(.blurry))
        #expect(sim.updates.last?.quality != nil)
    }

    @Test func rejectsOutOfRangeDuringCapture() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 120, until: { $0.keyframeCount >= 1 })
        sim.updates.removeAll()
        // Beyond the range + slack (0.75 + 0.03).
        await sim.run(controller, tracker: tracker, maxFrames: 30, options: { _ in .init(distance: 0.85) }, until: { _ in false })
        #expect(sim.accepted.isEmpty)
        #expect(sim.updates.last?.rejection == .tooFar)
        #expect(sim.updates.last?.instructionKind == .moveCloser)
        // Within the slack the scan continues.
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 60, options: { _ in .init(distance: 0.77) },
                      until: { $0.keyframeCount >= 2 })
        #expect(controller.keyframeCount == 2)
    }

    @Test func rejectsDepthHoles() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 150, options: { _ in .init(headDepthHoles: true) },
                      until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(sim.rejections.contains(.lowDepthCoverage))
    }

    @Test func rejectsTooFewDepthPoints() async {
        var configuration = ScanConfiguration.testOffline
        configuration.minValidDepthPoints = 10_000_000
        let (controller, tracker) = makeController(configuration)
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 120, until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(sim.rejections.contains(.tooFewDepthPoints))
        #expect((sim.updates.last { $0.rejection == .tooFewDepthPoints }?.quality?.validDepthPoints ?? 0) > 500)
    }

    @Test func segmentationThatMissesTheFaceDoesNotBlockThePose() async throws {
        // Vision's person segmentation of the synthetic head at 1280x800 returns little more than the neck; the scan
        // used to reject every frame as "too few depth points" until the segmentation happened to work.
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        let face = sim.factory.faceRect
        var neckOnly = Data(count: sim.factory.width * sim.factory.height)
        for y in Int(face.maxY) - 2 ..< sim.factory.height {
            for x in Int(face.midX) - 10 ..< Int(face.midX) + 10 { neckOnly[y * sim.factory.width + x] = 255 }
        }
        tracker.segmentation = neckOnly
        await sim.run(controller, tracker: tracker, maxFrames: 300) { $0.keyframeCount >= 3 }
        #expect(!sim.rejections.contains(.tooFewDepthPoints))
        let capture = controller.finish()
        #expect(capture.keyframes.count == 3)
        let mask = try #require(capture.keyframes.first?.mask)
        #expect(HeadMaskBuilder.count(mask) > 500)
        #expect(tracker.segmentationCalls >= 3)
    }

    @Test func keepShouldersWithoutSegmentationKeepsTheShoulders() async throws {
        // No person mask (the scripted tracker has no segmentation): the keyframe masks still contain the shoulders.
        var configuration = ScanConfiguration.testOffline
        configuration.keepShoulders = true
        let (controller, tracker) = makeController(configuration)
        var sim = ScanSimulation()
        sim.factory.height = 160
        await sim.run(controller, tracker: tracker, maxFrames: 300, options: { _ in .init(body: true) },
                      until: { $0.keyframeCount >= 1 })
        let mask = try #require(controller.finish().keyframes.first?.mask)
        var shoulders = 0
        for y in 0 ..< sim.factory.height {
            for x in 0 ..< sim.factory.width where sim.factory.inShoulders(x, y) && mask[y * sim.factory.width + x] != 0 {
                shoulders += 1
            }
        }
        #expect(shoulders > 3000)
        #expect(tracker.segmentationCalls >= 1)
    }

    @Test func offlineTimingUsesFrameTimestamps() async {
        // Frames 1 ms apart: 200 frames are only 0.2 s of scan time, less than positioning (0.5 s) + hold.
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        sim.fps = 1000
        await sim.run(controller, tracker: tracker, maxFrames: 200, until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(controller.phase == .positioning)

        // The same scan at 30 fps timestamps succeeds, independent of how fast frames are processed.
        let (controller2, tracker2) = makeController()
        var sim2 = ScanSimulation()
        await sim2.run(controller2, tracker: tracker2, maxFrames: 900)
        #expect(controller2.phase == .finished)
        // Positioning (0.5 s) + 5 poses x (turn + 0.4 s hold + 2 x 0.2 s spacing) at 30 fps.
        #expect(sim2.updates.count < 300)
    }

    @Test func offlineWithoutTimestampsAssumes30fps() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        sim.fps = 1e9   // timestamps stay (almost) constant; replace them with 0 below
        let factory = sim.factory
        var count = 0
        for i in 0 ..< 900 {
            sim.user.step(toward: controller.phase, dt: 1 / 30)
            let obs = sim.user.observation(faceRect: factory.faceRect)
            tracker.setScript { _ in obs }
            let update = await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 0))
            count += 1
            if update.phase == .finished { break }
        }
        #expect(controller.phase == .finished)
        #expect(count < 300)
    }

    @Test func timestampJumpBackResetsHold() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 120, until: { $0.keyframeCount >= 1 })
        // Playback loops: time goes back by 10 s. The hold must restart instead of accepting immediately.
        sim.timeMs -= 10_000
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 5, until: { _ in false })
        #expect(sim.accepted.isEmpty)
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 40, until: { $0.keyframeCount >= 2 })
        #expect(controller.keyframeCount == 2)
    }

    @Test func skipOnlyOptionalPoses() async {
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.front, .upLeft, .up]
        configuration.framesPerPose = 1
        let (controller, tracker) = makeController(configuration)
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 200, until: { $0.phase == .capturing(.front) })
        #expect(!controller.canSkipCurrentPose)
        #expect(controller.skipPose() == false)
        #expect(controller.phase == .capturing(.front))

        await sim.run(controller, tracker: tracker, maxFrames: 200, until: { $0.phase == .capturing(.upLeft) })
        #expect(controller.canSkipCurrentPose)
        #expect(controller.skipPose() == true)
        #expect(controller.phase == .capturing(.up))

        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 300)
        #expect(controller.phase == .finished)
        let last = sim.updates.last
        #expect(last?.skippedPoses == [.upLeft])
        #expect(last?.overallProgress == 1)
        let capture = controller.finish()
        #expect(capture.keyframes.map(\.scanPose) == [.front, .up])
        #expect(capture.notes["skipped_poses"] == "upLeft")
    }

    @Test func skipRequiredPoseWhenAllowed() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 200, until: { $0.phase == .capturing(.front) })
        #expect(controller.skipPose(allowRequired: true))
        #expect(controller.phase == .capturing(.up))
    }

    @Test func cancelDiscardsAndRestartWorks() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 200, until: { $0.keyframeCount >= 2 })
        #expect(controller.keyframeCount == 2)
        controller.cancel()
        #expect(controller.phase == .idle)
        #expect(controller.keyframeCount == 0)
        // In idle the tracker still reports the face and the distance for the preview.
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 3, until: { _ in false })
        #expect(sim.updates.last?.phase == .idle)
        #expect(sim.updates.last?.observation != nil)
        #expect(abs((sim.updates.last?.distanceM ?? 0) - 0.62) < 0.005)
        #expect(controller.finish().keyframes.isEmpty)

        controller.start(device: nil, captureConfiguration: nil)
        #expect(controller.phase == .positioning)
        await sim.run(controller, tracker: tracker, maxFrames: 900)
        #expect(controller.finish().keyframes.count == 15)
    }

    @Test func finishEndsScanEarlyWithPartialCapture() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 300, until: { $0.keyframeCount >= 4 })
        let capture = controller.finish()
        #expect(capture.keyframes.count == 4)
        #expect(controller.phase == .finished)
        // Frames after finish are not accepted.
        sim.updates.removeAll()
        await sim.run(controller, tracker: tracker, maxFrames: 60, until: { _ in false })
        #expect(sim.accepted.isEmpty)
        #expect(controller.keyframeCount == 4)
    }

    @Test func coverageGrowsAroundCapturedPoses() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 900)
        let coverage = try! #require(sim.updates.last?.coverage)
        let front = CoverageMap.bin(yaw: 0, pitch: 0)
        let left = CoverageMap.bin(yaw: 45, pitch: 0)
        let farBack = CoverageMap.bin(yaw: -89, pitch: -59)
        #expect(coverage.value(yawBin: front.yawBin, pitchBin: front.pitchBin) > 0.9)
        #expect(coverage.value(yawBin: left.yawBin, pitchBin: left.pitchBin) > 0.9)
        #expect(coverage.value(yawBin: farBack.yawBin, pitchBin: farBack.pitchBin) < 0.1)
        #expect(coverage.scanAreaCoverage > 0.6)
        // Coverage only grows during the scan.
        var previous: Float = 0
        for u in sim.updates {
            let total = u.coverage.values.reduce(0, +)
            #expect(total >= previous - 1e-4)
            previous = total
        }
    }

    @Test func turnInstructionPointsToTarget() async {
        var configuration = ScanConfiguration.testOffline
        configuration.poses = [.left]
        let (controller, tracker) = makeController(configuration)
        var sim = ScanSimulation()
        sim.user.speedDegPerSec = 0   // the user does not move: stays frontal
        await sim.run(controller, tracker: tracker, maxFrames: 30, until: { _ in false })
        guard case .turn(let pose, let yawDelta, let pitchDelta) = sim.updates.last?.instructionKind else {
            Issue.record("expected a turn instruction, got \(String(describing: sim.updates.last?.instructionKind))")
            return
        }
        #expect(pose == .left)
        #expect(abs(yawDelta - 45) < 0.5)
        #expect(abs(pitchDelta) < 0.5)
        #expect(sim.updates.last?.angularErrorDeg ?? 0 > 40)
        #expect(sim.updates.last?.poseIndex == 0)
    }

    @Test func poseDeltaUsesFrontOffsetAndOvershootWindow() {
        func delta(_ pose: ScanPose, _ yaw: Float, _ pitch: Float, offset: SIMD2<Float> = .zero) -> SIMD2<Float> {
            ScanController.poseDelta(pose: pose, current: SIMD2(yaw, pitch), frontOffset: offset, overshootDeg: 15)
        }
        func near(_ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Bool { simd_length(a - b) < 1e-4 }
        // The front target ignores the offset (it defines it).
        #expect(near(delta(.front, -4, 2, offset: SIMD2(-4, 2)), SIMD2(4, -2)))
        // Vision reads this person's straight-ahead face as yaw -4: left at 38 is 42 degrees of turn.
        #expect(near(delta(.left, 38, 0, offset: SIMD2(-4, 0)), SIMD2(3, 0)))
        // Up to 15 degrees beyond the target counts as reached; the perpendicular error still counts.
        #expect(near(delta(.right, -50, 0, offset: SIMD2(-4, 0)), .zero))
        #expect(near(delta(.up, 5, 30), SIMD2(-5, 0)))
        // Further than the window: turn back toward the front.
        #expect(near(delta(.right, -70, 0), SIMD2(10, 0)))
        // Not reached yet: the full remaining turn.
        #expect(near(delta(.down, 0, -10), SIMD2(0, -15)))
    }

    @Test func biasedVisionAndShortTurnsStillCompleteTheScan() async {
        // Vision reads 6 degrees to the right, and the user turns only 90% of each target: the left pose is 34.5
        // degrees by Vision, 10.5 from the absolute target but 4.5 from the front-relative one.
        func run(relative: Bool) async -> (ScanPhase?, Int) {
            var configuration = ScanConfiguration.testOffline
            configuration.targetsRelativeToFront = relative
            let (controller, tracker) = makeController(configuration)
            var sim = ScanSimulation()
            sim.user.reach = 0.9
            sim.user.visionYawBias = -6
            await sim.run(controller, tracker: tracker, maxFrames: 900)
            return (sim.updates.last?.phase, controller.finish().keyframes.count)
        }
        let relative = await run(relative: true)
        #expect(relative.0 == .finished && relative.1 == 15)
        let absolute = await run(relative: false)
        #expect(absolute.0 == .capturing(.left), "the absolute target is out of reach")
    }

    @Test func overestimatedTurnsAreAccepted() async {
        // Vision overestimates yaw by 20%: left reads 54 degrees, beyond the 8 degree tolerance but inside the
        // overshoot window.
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        sim.user.visionYawScale = 1.2
        await sim.run(controller, tracker: tracker, maxFrames: 900)
        #expect(sim.updates.last?.phase == .finished)
        #expect(controller.finish().keyframes.count == 15)
    }

    @Test func lowConfidenceCountsAsNoFace() async {
        let (controller, tracker) = makeController()
        let factory = GuidanceFrameFactory()
        let obs = HeadObservation(pose: HeadPose(yaw: 0, pitch: 0, roll: 0, confidence: 0.2),
                                  faceRect: factory.faceRect, landmarks: [], personMask: nil)
        tracker.setScript { _ in obs }
        var last = ScanUpdate()
        for i in 0 ..< 30 { last = await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 100 + Double(i) * 33)) }
        #expect(last.rejection == .lowConfidence)
        #expect(controller.phase == .positioning)
    }

    @Test func rollTooLargeAsksToStraighten() async {
        let (controller, tracker) = makeController()
        var sim = ScanSimulation()
        sim.user.roll = 35
        await sim.run(controller, tracker: tracker, maxFrames: 60, until: { _ in false })
        #expect(controller.keyframeCount == 0)
        #expect(sim.updates.last?.instructionKind == .straightenHead)
        #expect(sim.updates.last?.rejection == .rollTooLarge)
    }

    @Test func persistentTrackerErrorsFailTheScan() async {
        var configuration = ScanConfiguration.testOffline
        configuration.maxConsecutiveTrackerErrors = 5
        let (controller, tracker) = makeController(configuration)
        tracker.setScript { _ in throw ScriptError() }
        let factory = GuidanceFrameFactory()
        var last = ScanUpdate()
        for i in 0 ..< 6 { last = await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 100 + Double(i) * 33)) }
        guard case .failed = controller.phase else {
            Issue.record("expected failed, got \(controller.phase)")
            return
        }
        #expect(last.rejection == .trackingFailed)
    }

    @Test func autoDistanceRangeFromCaptureConfiguration() {
        var configuration = ScanConfiguration.offline
        configuration.autoDistanceRange = true
        let controller = ScanController(configuration: configuration, tracker: ScriptedTracker { _ in nil })
        var capture = CaptureConfiguration.faceScanD455
        capture.disparityShift = 60
        controller.start(device: nil, captureConfiguration: capture)
        let expected = WorkingDistance.recommendedRange(device: nil, configuration: capture)
        #expect(controller.distanceRange == expected)
        #expect(controller.distanceRange.upperBound < 0.55)

        configuration.autoDistanceRange = false
        configuration.distanceRangeM = 0.4 ... 0.9
        let manual = ScanController(configuration: configuration, tracker: ScriptedTracker { _ in nil })
        manual.start(device: nil, captureConfiguration: capture)
        #expect(manual.distanceRange == 0.4 ... 0.9)
    }

    @Test func preparedIdleControllerReportsDistanceWithoutScanning() async {
        var configuration = ScanConfiguration.offline
        configuration.autoDistanceRange = true
        let tracker = ScriptedTracker { _ in nil }
        let controller = ScanController(configuration: configuration, tracker: tracker)
        var capture = CaptureConfiguration.faceScanD455
        capture.disparityShift = 100   // recommended range ~0.33-0.48 m
        controller.prepare(device: GuidanceDevice.make(), captureConfiguration: capture)
        #expect(controller.phase == .idle)
        var sim = ScanSimulation()
        await sim.run(controller, tracker: tracker, maxFrames: 40, until: { _ in false })
        let last = sim.updates.last
        #expect(last?.phase == .idle)
        #expect(last?.distanceAdvice == .tooFar)          // 0.62 m is beyond the close-range band
        #expect(last?.distanceRangeM == controller.distanceRange)
        #expect(controller.keyframeCount == 0)
        // start() keeps using the prepared camera description.
        controller.start(device: GuidanceDevice.make(), captureConfiguration: capture)
        #expect(controller.distanceRange == WorkingDistance.recommendedRange(device: GuidanceDevice.make(), configuration: capture))
    }

    @Test func concurrentControlCallsAreSafe() async {
        let (controller, tracker) = makeController()
        let factory = GuidanceFrameFactory()
        let obs = HeadObservation(pose: .frontal, faceRect: factory.faceRect, landmarks: [], personMask: nil)
        tracker.setScript { _ in obs }
        await withTaskGroup(of: Void.self) { group in
            group.addTask {
                for i in 0 ..< 200 {
                    _ = await controller.process(factory.frame(sequence: UInt64(i), timestampMs: 1000 + Double(i) * 33.3))
                }
            }
            group.addTask {
                for i in 0 ..< 200 {
                    if i % 50 == 0 { controller.start(device: nil, captureConfiguration: nil) }
                    _ = controller.skipPose()
                    _ = controller.phase
                    _ = controller.keyframeCount
                    await Task.yield()
                }
            }
        }
        // Whatever interleaving happened, the keyframes stay consistent with the per-pose limit.
        let capture = controller.finish()
        for pose in ScanPose.allCases {
            #expect(capture.keyframes.filter { $0.scanPose == pose }.count <= 3)
        }
    }
}
