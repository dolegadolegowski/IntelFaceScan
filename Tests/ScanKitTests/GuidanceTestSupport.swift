// License: Apache 2.0. See LICENSE file in root directory.
//
// Synthetic frames and scripted head observations for the guided-scan tests.

import CameraKit
import CoreGraphics
import Foundation
@testable import ScanKit

/// Builds small RGB-D frames: a textured "head" ellipse at a given distance in front of a far background.
struct GuidanceFrameFactory {
    var width = 160
    var height = 100
    var faceRect = CGRect(x: 60, y: 30, width: 40, height: 46)
    var depthScale: Float = 0.0001
    var backgroundDistance: Float = 1.6

    var intrinsics: Intrinsics {
        Intrinsics(width: width, height: height, fx: 80, fy: 80, ppx: Float(width) / 2, ppy: Float(height) / 2)
    }

    enum Texture { case sharp, blurry, flat }

    struct Options {
        var distance: Float = 0.62
        var texture: Texture = .sharp
        /// Gyro rate in rad/s around the camera's vertical axis.
        var gyroRadPerSec: Float = 0
        /// Head pixels without depth (holes), as a fraction of the head ellipse; the face center keeps depth.
        var headDepthHoles: Bool = false
    }

    func frame(sequence: UInt64, timestampMs: Double, _ options: Options = Options()) -> CameraFrame {
        let color = ColorImage(width: width, height: height, bytesPerRow: width * 3,
                               pixels: colorPixels(options.texture), intrinsics: intrinsics,
                               frameNumber: sequence, timestampMs: timestampMs)
        let depth = DepthImage(width: width, height: height, pixels: depthPixels(options), depthScale: depthScale,
                               intrinsics: intrinsics, frameNumber: sequence, timestampMs: timestampMs)
        let imu = IMUSample(accel: [0, -9.81, 0], gyro: [0, options.gyroRadPerSec, 0], timestampMs: timestampMs)
        return CameraFrame(sequence: sequence, timestampMs: timestampMs, color: color, alignedDepth: depth, imu: imu)
    }

    /// True inside the head ellipse around the face rectangle.
    func inHead(_ x: Int, _ y: Int) -> Bool {
        let cx = Double(faceRect.midX), cy = Double(faceRect.midY) - 0.3 * Double(faceRect.height)
        let ax = 0.75 * Double(faceRect.width), ay = 0.95 * Double(faceRect.height)
        let dx = (Double(x) + 0.5 - cx) / ax, dy = (Double(y) + 0.5 - cy) / ay
        return dx * dx + dy * dy <= 1
    }

    func colorPixels(_ texture: Texture) -> Data {
        var checker = [UInt8](repeating: 0, count: width * height)
        for y in 0 ..< height {
            for x in 0 ..< width {
                checker[y * width + x] = ((x / 3) + (y / 3)) % 2 == 0 ? 40 : 215
            }
        }
        var gray = checker
        switch texture {
        case .sharp: break
        case .flat: gray = [UInt8](repeating: 128, count: width * height)
        case .blurry:
            // 7x7 box blur: the 3-pixel checkerboard nearly disappears.
            let r = 3
            for y in 0 ..< height {
                for x in 0 ..< width {
                    var sum = 0, n = 0
                    for yy in max(0, y - r) ... min(height - 1, y + r) {
                        for xx in max(0, x - r) ... min(width - 1, x + r) { sum += Int(checker[yy * width + xx]); n += 1 }
                    }
                    gray[y * width + x] = UInt8(sum / n)
                }
            }
        }
        var rgb = Data(count: width * height * 3)
        rgb.withUnsafeMutableBytes { raw in
            let p = raw.bindMemory(to: UInt8.self)
            for i in 0 ..< width * height {
                p[3 * i] = gray[i]; p[3 * i + 1] = gray[i]; p[3 * i + 2] = gray[i]
            }
        }
        return rgb
    }

    func depthPixels(_ options: Options) -> Data {
        var values = [UInt16](repeating: 0, count: width * height)
        let head = UInt16(options.distance / depthScale)
        let background = UInt16(backgroundDistance / depthScale)
        let center = SIMD2(Float(faceRect.midX), Float(faceRect.midY))
        for y in 0 ..< height {
            for x in 0 ..< width {
                let i = y * width + x
                if inHead(x, y) {
                    let nearCenter = abs(Float(x) - center.x) < Float(faceRect.width) * 0.2
                        && abs(Float(y) - center.y) < Float(faceRect.height) * 0.25
                    values[i] = options.headDepthHoles && !nearCenter && (x + y) % 3 != 0 ? 0 : head
                } else {
                    values[i] = background
                }
            }
        }
        return values.withUnsafeBytes { Data($0) }
    }
}

/// Head tracker driven by a closure (the scripted user).
final class ScriptedTracker: HeadTracking, @unchecked Sendable {
    private let lock = NSLock()
    private var script: (CameraFrame) throws -> HeadObservation?
    private(set) var segmentationCalls = 0

    init(_ script: @escaping (CameraFrame) throws -> HeadObservation?) { self.script = script }

    func setScript(_ script: @escaping (CameraFrame) throws -> HeadObservation?) {
        lock.withLock { self.script = script }
    }

    func analyze(_ frame: CameraFrame, segmentation: Bool) throws -> HeadObservation? {
        let s = lock.withLock { script }
        return try s(frame)
    }

    func segmentPerson(in frame: CameraFrame) throws -> Data? {
        lock.withLock { segmentationCalls += 1 }
        return nil
    }
}

struct ScriptError: Error {}

/// A user who turns toward the current pose target at a limited speed and then holds still.
struct SimulatedUser {
    var yaw: Float = 0
    var pitch: Float = 0
    var roll: Float = 0
    var speedDegPerSec: Float = 60

    mutating func step(toward phase: ScanPhase, dt: Float) {
        guard case .capturing(let pose) = phase else { return }
        let target = pose.target
        let dy = target.yaw - yaw, dp = target.pitch - pitch
        let distance = (dy * dy + dp * dp).squareRoot()
        let maxStep = speedDegPerSec * dt
        if distance <= maxStep {
            yaw = target.yaw; pitch = target.pitch
        } else {
            yaw += dy / distance * maxStep; pitch += dp / distance * maxStep
        }
    }

    func observation(faceRect: CGRect, confidence: Float = 0.95) -> HeadObservation {
        HeadObservation(pose: HeadPose(yaw: yaw, pitch: pitch, roll: roll, confidence: confidence),
                        faceRect: faceRect, landmarks: [SIMD2(Float(faceRect.midX), Float(faceRect.midY))],
                        personMask: nil)
    }
}

/// Runs a controller over simulated frames at 30 fps until it finishes or `maxFrames` frames were processed.
struct ScanSimulation {
    var factory = GuidanceFrameFactory()
    var user = SimulatedUser()
    var fps: Double = 30
    var sequence: UInt64 = 0
    var timeMs: Double = 1_000_000
    var updates: [ScanUpdate] = []

    mutating func run(_ controller: ScanController, tracker: ScriptedTracker, maxFrames: Int,
                      options: (Double) -> GuidanceFrameFactory.Options = { _ in .init() },
                      until stop: (ScanUpdate) -> Bool = { $0.phase == .finished }) async {
        for _ in 0 ..< maxFrames {
            user.step(toward: controller.phase, dt: Float(1 / fps))
            let observation = user.observation(faceRect: factory.faceRect)
            tracker.setScript { _ in observation }
            let frame = factory.frame(sequence: sequence, timestampMs: timeMs, options(timeMs))
            let update = await controller.process(frame)
            updates.append(update)
            sequence += 1
            timeMs += 1000 / fps
            if stop(update) { break }
        }
    }

    var accepted: [ScanUpdate] { updates.filter(\.acceptedKeyframe) }
    var rejections: [ScanRejection] { updates.compactMap(\.rejection) }
}

extension ScanConfiguration {
    /// Offline configuration for the 160x100 test frames.
    static var testOffline: ScanConfiguration {
        var c = ScanConfiguration.offline
        c.autoDistanceRange = false
        c.distanceRangeM = 0.55 ... 0.75
        return c
    }
}

enum GuidanceDevice {
    /// DeviceInfo as the helper sends it (it has no public memberwise initializer).
    static func make(name: String = "Intel RealSense D455", baselineMm: Float? = 95) -> DeviceInfo {
        var object: [String: Any] = [
            "name": name, "serial": "123456789012", "firmware": "5.16.0.1", "usb_type": "3.2",
            "product_id": "0B5C", "product_line": "D400", "advanced_mode": true, "depth_scale": 0.0001,
            "sensors": [Any](), "profiles": [Any](), "presets": ["High Accuracy"], "imu": true, "source": "live",
        ]
        if let baselineMm { object["baseline_mm"] = baselineMm }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        // The JSON is built right here; a failure is a bug in this helper.
        let data = try! JSONSerialization.data(withJSONObject: object)
        return try! decoder.decode(DeviceInfo.self, from: data)
    }
}
