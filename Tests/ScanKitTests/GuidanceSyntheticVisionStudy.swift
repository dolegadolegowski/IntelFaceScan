// License: Apache 2.0. See LICENSE file in root directory.
//
// Measurements (run by hand, IFS_VISION_STUDY=1): what Apple Vision reads on the rendered synthetic head at the two
// resolutions the app uses, how often a held pose with the animation's sway lies in the guided scan's tolerance, the
// motion Vision's angles show on a held head, landmark motion, and how often person segmentation covers the face.
// The numbers behind SyntheticCameraSource's animation, PoseHistory, ScanConfiguration.perpendicularToleranceDeg and
// docs/TESTING.md ("lighting").
//
//   IFS_VISION_STUDY=1 scripts/test.sh --filter GuidanceSyntheticVisionStudy
//   IFS_VISION_STUDY=1 IFS_VISION_STUDY_GRID=1 scripts/test.sh --filter acceptanceAroundTargets   (pitch x yaw grid)

import CameraKit
import Foundation
import simd
import SyntheticData
import Testing
@testable import ScanKit

@Suite("GuidanceSyntheticVisionStudy", .serialized,
       .enabled(if: ProcessInfo.processInfo.environment["IFS_VISION_STUDY"] == "1", "set IFS_VISION_STUDY=1"))
struct GuidanceSyntheticVisionStudy {
    static func scene(width: Int, height: Int) -> SyntheticScene {
        var options = SyntheticCameraSource.defaultOptions
        options.width = width
        options.height = height
        return SyntheticScene(options: options)
    }

    static func format(_ v: Float?) -> String { v.map { String(format: "%6.1f", $0) } ?? "     -" }

    /// Mean and spread of Vision's yaw (pitch) over a few noise realizations per true angle.
    @Test(arguments: [(640, 400), (1280, 800)])
    func responseCurves(width: Int, height: Int) throws {
        let scene = Self.scene(width: width, height: height)
        let tracker = HeadTracker(convention: .documented)
        var index: UInt64 = 0
        func read(_ pose: HeadPose) -> [HeadPose] {
            (0 ..< 4).compactMap { _ in
                index += 1
                let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose), frameIndex: index,
                                        sequence: index, timestampMs: Double(index))
                return (try? tracker.analyze(view.frame, segmentation: false)).flatMap { $0 }?.pose
            }
        }
        print("\(width)x\(height) yaw sweep (pitch 0): true -> Vision yaw mean [min, max] (detections of 4), pitch mean")
        for yaw in stride(from: -65, through: 65, by: 5) {
            let r = read(HeadPose(yaw: Float(yaw), pitch: 0, roll: 0))
            let yaws = r.map(\.yaw), pitches = r.map(\.pitch)
            print("  yaw \(yaw): \(Self.format(yaws.mean)) [\(Self.format(yaws.min())), \(Self.format(yaws.max()))] "
                + "(\(r.count)) pitch \(Self.format(pitches.mean))")
        }
        print("\(width)x\(height) pitch sweep (yaw 0): true -> Vision pitch mean [min, max] (detections of 4), yaw mean")
        for pitch in stride(from: -45, through: 45, by: 5) {
            let r = read(HeadPose(yaw: 0, pitch: Float(pitch), roll: 0))
            let pitches = r.map(\.pitch), yaws = r.map(\.yaw)
            print("  pitch \(pitch): \(Self.format(pitches.mean)) [\(Self.format(pitches.min())), \(Self.format(pitches.max()))] "
                + "(\(r.count)) yaw \(Self.format(yaws.mean))")
        }
    }

    /// Fine sweep around the outer targets with the animation's sway: how often the reading lies in the guided scan's
    /// tolerance (front-relative target, 15 degrees of overshoot): the 8-degree circle and the default ellipse
    /// (8 along the target direction, ScanConfiguration.perpendicularToleranceDeg across).
    @Test(arguments: [(640, 400), (1280, 800)])
    func acceptanceAroundTargets(width: Int, height: Int) throws {
        let scene = Self.scene(width: width, height: height)
        let tracker = HeadTracker(convention: .documented)
        var index: UInt64 = 20_000
        func read(_ pose: HeadPose) -> HeadPose? {
            index += 1
            let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose), frameIndex: index,
                                    sequence: index, timestampMs: Double(index))
            return (try? tracker.analyze(view.frame, segmentation: false)).flatMap { $0 }?.pose
        }
        // The front reading defines the offset, as in ScanController.
        let fronts = (0 ..< 6).compactMap { _ in read(.frontal) }
        let offset = SIMD2(fronts.map(\.yaw).reduce(0, +), fronts.map(\.pitch).reduce(0, +)) / Float(max(1, fronts.count))
        print("\(width)x\(height) front offset \(offset)")
        let sway: [(Float, Float, Float)] = [(0, 0, 0), (1.5, 0, 0), (-1.5, 0, 0), (0, 1, 0), (0, -1, 0), (1, 0.7, 0.8),
                                             (-1, -0.7, -0.8), (0.7, -0.7, 0.5)]
        var stops: [(ScanPose, Float, Float)] = []
        if ProcessInfo.processInfo.environment["IFS_VISION_STUDY_GRID"] == "1" {
            // Side poses with a pitch offset, down with a yaw offset.
            for yaw in stride(from: 44, through: 58, by: 2) {
                for pitch in stride(from: -12, through: 4, by: 2) { stops.append((.left, Float(yaw), Float(pitch))) }
            }
            for pitch in stride(from: -32, through: -20, by: 2) {
                for yaw in stride(from: -6, through: 6, by: 3) { stops.append((.down, Float(yaw), Float(pitch))) }
            }
        } else {
            for (pose, amplitudes) in [(ScanPose.left, Array(stride(from: 40, through: 64, by: 2))),
                                       (.right, Array(stride(from: 40, through: 64, by: 2))),
                                       (.down, Array(stride(from: 18, through: 36, by: 1))),
                                       (.up, Array(stride(from: 18, through: 36, by: 2)))] {
                for a in amplitudes {
                    let scale = Float(a) / simd_length(SIMD2(pose.target.yaw, pose.target.pitch))
                    stops.append((pose, pose.target.yaw * scale, pose.target.pitch * scale))
                }
            }
        }
        let c = ScanConfiguration()
        for (pose, yaw, pitch) in stops {
            var inCircle = 0, inEllipse = 0, detected = 0
            var readings: [String] = []
            for s in sway {
                guard let r = read(HeadPose(yaw: yaw + s.0, pitch: pitch + s.1, roll: s.2)) else { readings.append("-"); continue }
                detected += 1
                let delta = ScanController.poseDelta(pose: pose, current: SIMD2(r.yaw, r.pitch), frontOffset: offset,
                                                     overshootDeg: c.overshootToleranceDeg)
                if simd_length(delta) <= c.angleTolerance { inCircle += 1 }
                if ScanController.withinTolerance(delta, pose: pose, along: c.angleTolerance, across: c.perpendicularToleranceDeg) {
                    inEllipse += 1
                }
                readings.append(String(format: "%.0f/%.0f", r.yaw, r.pitch))
            }
            print("  \(width)x\(height) \(pose.rawValue) " + String(format: "%.0f/%.0f", yaw, pitch)
                + ": in tolerance \(inEllipse)/\(sway.count) (8-degree circle \(inCircle)), detected \(detected): "
                + readings.joined(separator: " "))
        }
    }

    /// A held pose with the animation's sway: frame-to-frame changes of Vision's angles.
    @Test(arguments: [(640, 400), (1280, 800)])
    func heldPoseJitter(width: Int, height: Int) throws {
        let scene = Self.scene(width: width, height: height)
        let tracker = HeadTracker(convention: .documented)
        for (name, base) in [("front", HeadPose(yaw: 0, pitch: 0, roll: 0)), ("down", HeadPose(yaw: 0, pitch: -25, roll: 0)),
                             ("down-32", HeadPose(yaw: 0, pitch: -32, roll: 0)), ("up", HeadPose(yaw: 0, pitch: 25, roll: 0)),
                             ("left", HeadPose(yaw: 45, pitch: 0, roll: 0)), ("right", HeadPose(yaw: -45, pitch: 0, roll: 0))] {
            var series: [HeadPose] = []
            var history = PoseHistory(), plainHistory = PoseHistory()
            plainHistory.minJumpRate = .infinity
            var speeds: [Float] = [], plainSpeeds: [Float] = []
            for i in 0 ..< 60 {
                let t = Double(i) / 30
                let pose = HeadPose(yaw: base.yaw + 1.5 * Float(sin(t * 1.7)), pitch: base.pitch + Float(sin(t * 1.3 + 0.5)),
                                    roll: 0.8 * Float(sin(t * 0.9)))
                let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose), frameIndex: UInt64(1000 + i),
                                        sequence: UInt64(i), timestampMs: 1000 + t * 1000)
                guard let o = try tracker.analyze(view.frame, segmentation: false) else { continue }
                series.append(o.pose)
                history.add(o.pose, at: t)
                plainHistory.add(o.pose, at: t)
                if let s = plainHistory.angularSpeed() { plainSpeeds.append(s) }
                if let s = history.angularSpeed() { speeds.append(s) }
            }
            let dYaw = zip(series, series.dropFirst()).map { abs($1.yaw - $0.yaw) }
            let dPitch = zip(series, series.dropFirst()).map { abs($1.pitch - $0.pitch) }
            let dRoll = zip(series, series.dropFirst()).map { abs($1.roll - $0.roll) }
            print("\(width)x\(height) \(name): \(series.count)/60 detected, yaw \(Self.format(series.map(\.yaw).mean)) "
                + "sd \(Self.format(series.map(\.yaw).sd)), pitch \(Self.format(series.map(\.pitch).mean)) "
                + "sd \(Self.format(series.map(\.pitch).sd)), roll sd \(Self.format(series.map(\.roll).sd)); "
                + "|frame diff| max yaw \(Self.format(dYaw.max())) pitch \(Self.format(dPitch.max())) roll \(Self.format(dRoll.max())); "
                + "speed > 20: plain least squares \(plainSpeeds.filter { $0 > 20 }.count)/\(plainSpeeds.count), "
                + "without jumps \(speeds.filter { $0 > 20 }.count) (median \(Self.format(speeds.median)))")
            print("   pitch series: " + series.prefix(30).map { String(format: "%.1f", $0.pitch) }.joined(separator: " "))
            print("   yaw series:   " + series.prefix(30).map { String(format: "%.1f", $0.yaw) }.joined(separator: " "))
        }
    }

    /// Landmark motion as a cue for head motion that does not depend on Vision's angles: held poses (with the sway)
    /// against real turns and shakes.
    @Test(arguments: [(640, 400), (1280, 800)])
    func landmarkMotion(width: Int, height: Int) throws {
        let scene = Self.scene(width: width, height: height)
        let tracker = HeadTracker(convention: .documented)
        let sway: (Double) -> HeadPose = { t in
            HeadPose(yaw: 1.5 * Float(sin(t * 1.7)), pitch: Float(sin(t * 1.3 + 0.5)), roll: 0.8 * Float(sin(t * 0.9)))
        }
        func plus(_ a: HeadPose, _ b: HeadPose) -> HeadPose { HeadPose(yaw: a.yaw + b.yaw, pitch: a.pitch + b.pitch, roll: a.roll + b.roll) }
        let scenarios: [(String, (Double) -> HeadPose)] = [
            ("held front", { sway($0) }),
            ("held down", { plus(HeadPose(yaw: 0, pitch: -25, roll: 0), sway($0)) }),
            ("held down-32", { plus(HeadPose(yaw: 0, pitch: -32, roll: 0), sway($0)) }),
            ("held right", { plus(HeadPose(yaw: -45, pitch: 0, roll: 0), sway($0)) }),
            ("held left-52", { plus(HeadPose(yaw: 52, pitch: 0, roll: 0), sway($0)) }),
            ("yaw 10 deg/s", { HeadPose(yaw: Float(10 * $0) - 10, pitch: 0, roll: 0) }),
            ("yaw 20 deg/s", { HeadPose(yaw: Float(20 * $0) - 20, pitch: 0, roll: 0) }),
            ("yaw 40 deg/s", { HeadPose(yaw: Float(40 * $0) - 40, pitch: 0, roll: 0) }),
            ("pitch 20 deg/s at down", { HeadPose(yaw: 0, pitch: Float(20 * $0) - 45, roll: 0) }),
            ("shake 2.5 Hz 6 deg", { HeadPose(yaw: 6 * Float(sin(2 * .pi * 2.5 * $0)), pitch: 0, roll: 0) }),
        ]
        for (name, poseAt) in scenarios {
            var previous: HeadObservation?
            var history = PoseHistory()
            var poseSpeeds: [Float] = [], landmarkSpeeds: [Float] = [], rectSpeeds: [Float] = []
            for i in 0 ..< 60 {
                let t = Double(i) / 30
                let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: poseAt(t)), frameIndex: UInt64(3000 + i),
                                        sequence: UInt64(i), timestampMs: 1000 + t * 1000)
                guard let o = try tracker.analyze(view.frame, segmentation: false) else { previous = nil; continue }
                history.add(o.pose, at: t)
                if let s = history.angularSpeed() { poseSpeeds.append(s) }
                if let p = previous, p.landmarks.count == o.landmarks.count, !o.landmarks.isEmpty {
                    let d = zip(p.landmarks, o.landmarks).map { simd_length($1 - $0) }.sorted()
                    landmarkSpeeds.append(d[d.count / 2] * 30 / Float(o.faceRect.width))
                    let c0 = SIMD2(Float(p.faceRect.midX), Float(p.faceRect.midY)), c1 = SIMD2(Float(o.faceRect.midX), Float(o.faceRect.midY))
                    rectSpeeds.append(simd_length(c1 - c0) * 30 / Float(o.faceRect.width))
                }
                previous = o
            }
            func stats(_ v: [Float]) -> String {
                let s = v.sorted()
                guard !s.isEmpty else { return "-" }
                return String(format: "median %5.2f p90 %5.2f max %5.2f", s[s.count / 2], s[min(s.count - 1, s.count * 9 / 10)], s.last!)
            }
            print("\(width)x\(height) \(name): pose speed \(stats(poseSpeeds)) | landmarks (face widths/s) \(stats(landmarkSpeeds)) "
                + "| face rect \(stats(rectSpeeds))")
        }
    }

    /// Person segmentation per pose: does it cover the central face region (HeadMaskBuilder's check)?
    @Test(arguments: [(640, 400), (1280, 800)])
    func segmentationCoverage(width: Int, height: Int) throws {
        let scene = Self.scene(width: width, height: height)
        let tracker = HeadTracker(convention: .documented)
        var index: UInt64 = 5000
        for pose in ScanPose.required {
            var covered = 0, total = 0, faces = 0
            for k in 0 ..< 8 {
                index += 1
                let jitter = Float(k % 4) - 1.5
                let head = HeadPose(yaw: pose.target.yaw + jitter, pitch: pose.target.pitch - jitter, roll: 0)
                let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: head), frameIndex: index,
                                        sequence: index, timestampMs: Double(index))
                total += 1
                guard let o = try tracker.analyze(view.frame, segmentation: false) else { continue }
                faces += 1
                guard let mask = try tracker.segmentPerson(in: view.frame) else { continue }
                if HeadMaskBuilder.personMaskCoversFace(mask, width: width, height: height, face: o.faceRect, minCoverage: 0.5) {
                    covered += 1
                }
            }
            print("\(width)x\(height) \(pose.rawValue): face \(faces)/\(total), segmentation covers the face \(covered)/\(faces)")
        }
    }
}

private extension Array where Element == Float {
    var mean: Float? { isEmpty ? nil : reduce(0, +) / Float(count) }
    var sd: Float? {
        guard let m = mean, count > 1 else { return nil }
        return (map { ($0 - m) * ($0 - m) }.reduce(0, +) / Float(count - 1)).squareRoot()
    }
    var median: Float? {
        guard !isEmpty else { return nil }
        let s = sorted()
        return s[s.count / 2]
    }
}
