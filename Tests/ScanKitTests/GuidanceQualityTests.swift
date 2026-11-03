// License: Apache 2.0. See LICENSE file in root directory.
//
// Quality metrics on synthetic images: sharpness, depth statistics, motion estimators, distance advice.

import CameraKit
import CoreGraphics
import Foundation
import Testing
@testable import ScanKit

@Suite struct GuidanceQualityTests {
    private func grayImage(width: Int, height: Int, padding: Int = 0, _ value: (Int, Int) -> UInt8) -> ColorImage {
        let stride = width * 3 + padding
        var data = Data(count: stride * height)
        data.withUnsafeMutableBytes { raw in
            let p = raw.bindMemory(to: UInt8.self)
            for y in 0 ..< height {
                for x in 0 ..< width {
                    let v = value(x, y)
                    p[y * stride + 3 * x] = v; p[y * stride + 3 * x + 1] = v; p[y * stride + 3 * x + 2] = v
                }
            }
        }
        return ColorImage(width: width, height: height, bytesPerRow: stride, pixels: data,
                          intrinsics: Intrinsics(width: width, height: height, fx: 100, fy: 100, ppx: 50, ppy: 50))
    }

    private func depthImage(width: Int, height: Int, scale: Float = 0.0001, _ meters: (Int, Int) -> Float) -> DepthImage {
        var values = [UInt16](repeating: 0, count: width * height)
        for y in 0 ..< height { for x in 0 ..< width { values[y * width + x] = UInt16((meters(x, y) / scale).rounded()) } }
        return DepthImage(width: width, height: height, pixels: values.withUnsafeBytes { Data($0) }, depthScale: scale,
                          intrinsics: Intrinsics(width: width, height: height, fx: 100, fy: 100, ppx: 50, ppy: 50))
    }

    // MARK: Sharpness

    @Test func flatImageHasZeroSharpness() {
        let image = grayImage(width: 64, height: 48) { _, _ in 128 }
        #expect(SharpnessMetric.varianceOfLaplacian(image, in: CGRect(x: 0, y: 0, width: 64, height: 48)) < 1e-3)
    }

    @Test func pixelCheckerboardMatchesAnalyticVariance() {
        // Alternating 0/255: the Laplacian is +-4*255 everywhere (edge extension keeps it at the border, too,
        // except that the border pixels see one equal neighbor).
        let image = grayImage(width: 64, height: 64) { x, y in (x + y) % 2 == 0 ? 0 : 255 }
        let v = SharpnessMetric.varianceOfLaplacian(image, in: CGRect(x: 16, y: 16, width: 32, height: 32))
        let expected: Float = 1020 * 1020
        #expect(abs(v - expected) / expected < 0.15, "variance \(v)")
    }

    @Test func blurLowersSharpness() {
        let factory = GuidanceFrameFactory()
        let rect = CGRect(x: 40, y: 20, width: 80, height: 60)
        let sharp = factory.frame(sequence: 0, timestampMs: 0, .init(texture: .sharp)).color
        let blurry = factory.frame(sequence: 0, timestampMs: 0, .init(texture: .blurry)).color
        let s = SharpnessMetric.varianceOfLaplacian(sharp, in: rect)
        let b = SharpnessMetric.varianceOfLaplacian(blurry, in: rect)
        #expect(s > 1000)
        #expect(b < 0.2 * s, "sharp \(s) blurry \(b)")
    }

    @Test func sharpnessHandlesPaddedRowsAndClipping() {
        let padded = grayImage(width: 50, height: 40, padding: 7) { x, y in (x / 2 + y / 2) % 2 == 0 ? 10 : 240 }
        let tight = grayImage(width: 50, height: 40) { x, y in (x / 2 + y / 2) % 2 == 0 ? 10 : 240 }
        let rect = CGRect(x: 5, y: 5, width: 30, height: 20)
        #expect(SharpnessMetric.varianceOfLaplacian(padded, in: rect) == SharpnessMetric.varianceOfLaplacian(tight, in: rect))
        // Partly outside: clipped to the image; completely outside or degenerate: 0.
        #expect(SharpnessMetric.varianceOfLaplacian(tight, in: CGRect(x: 40, y: 30, width: 50, height: 50)) > 0)
        #expect(SharpnessMetric.varianceOfLaplacian(tight, in: CGRect(x: 60, y: 60, width: 10, height: 10)) == 0)
        #expect(SharpnessMetric.varianceOfLaplacian(tight, in: CGRect(x: 1, y: 1, width: 2, height: 2)) == 0)
        #expect(SharpnessMetric.varianceOfLaplacian(tight, in: CGRect(x: CGFloat.nan, y: 0, width: 5, height: 5)) == 0)
    }

    @Test func sharpnessRejectsTruncatedImage() {
        var image = grayImage(width: 20, height: 20) { x, _ in UInt8(x * 10) }
        image.pixels = image.pixels.prefix(100)
        #expect(!image.isValidRGB8)
        #expect(SharpnessMetric.varianceOfLaplacian(image, in: CGRect(x: 0, y: 0, width: 20, height: 20)) == 0)
    }

    // MARK: Depth

    @Test func medianDistanceIgnoresInvalidPixels() throws {
        // Left half 0.60 m, right half 0.70 m, top rows invalid.
        let depth = depthImage(width: 40, height: 40) { x, y in y < 10 ? 0 : (x < 20 ? 0.60 : 0.70) }
        let all = try #require(DepthStatistics.medianDistance(depth, in: CGRect(x: 0, y: 0, width: 40, height: 40)))
        #expect(all >= 0.60 && all <= 0.701)
        let left = try #require(DepthStatistics.medianDistance(depth, in: CGRect(x: 0, y: 0, width: 15, height: 40)))
        #expect(abs(left - 0.600) < 0.0005)
        let mostlyRight = try #require(DepthStatistics.medianDistance(depth, in: CGRect(x: 15, y: 0, width: 25, height: 40)))
        #expect(abs(mostlyRight - 0.700) < 0.0005)
        #expect(DepthStatistics.medianDistance(depth, in: CGRect(x: 0, y: 0, width: 40, height: 9)) == nil)
    }

    @Test func medianDistanceScalesColorRectToDepthSize() throws {
        // Depth at half the color resolution.
        let depth = depthImage(width: 20, height: 20) { x, _ in x < 10 ? 0.5 : 0.9 }
        let d = try #require(DepthStatistics.medianDistance(depth, in: CGRect(x: 0, y: 0, width: 16, height: 40),
                                                             colorSize: CGSize(width: 40, height: 40)))
        #expect(abs(d - 0.500) < 0.0005)
    }

    @Test func validRatioAndPoints() {
        let depth = depthImage(width: 10, height: 10) { x, _ in x < 5 ? 0 : 0.6 }
        #expect(DepthStatistics.validRatio(depth, in: CGRect(x: 0, y: 0, width: 10, height: 10)) == 0.5)
        #expect(DepthStatistics.validRatio(depth, in: CGRect(x: 5, y: 0, width: 5, height: 10)) == 1)
        var mask = Data(count: 100)
        for i in 0 ..< 100 where i % 10 >= 3 && i / 10 < 4 { mask[i] = 255 }   // columns 3...9, rows 0...3
        #expect(DepthStatistics.validPoints(depth, mask: mask) == 5 * 4)
        #expect(DepthStatistics.validPoints(depth, mask: Data(count: 10)) == 0)
    }

    @Test func invalidDepthIsHandled() {
        let bad = DepthImage(width: 10, height: 10, pixels: Data(count: 20), depthScale: 0.001,
                             intrinsics: Intrinsics(width: 10, height: 10, fx: 1, fy: 1, ppx: 0, ppy: 0))
        #expect(DepthStatistics.medianDistance(bad, in: CGRect(x: 0, y: 0, width: 10, height: 10)) == nil)
        #expect(DepthStatistics.validRatio(bad, in: CGRect(x: 0, y: 0, width: 10, height: 10)) == 0)
    }

    // MARK: Motion

    @Test func cameraMotionFromGyro() {
        let tenDegPerSec = Float(10 * Double.pi / 180)
        #expect(abs(CameraMotion.degreesPerSecond(IMUSample(accel: nil, gyro: [0, 0, tenDegPerSec], timestampMs: 0)) - 10) < 1e-3)
        #expect(abs(CameraMotion.degreesPerSecond(IMUSample(accel: nil, gyro: [3, 4, 0].map { $0 * tenDegPerSec / 5 }, timestampMs: 0)) - 10) < 1e-3)
        #expect(CameraMotion.degreesPerSecond(nil) == 0)
        #expect(CameraMotion.degreesPerSecond(IMUSample(accel: [1, 2, 3], gyro: nil, timestampMs: 0)) == 0)
        #expect(CameraMotion.degreesPerSecond(IMUSample(accel: nil, gyro: [.nan, 0, 0], timestampMs: 0)) == 0)
    }

    @Test func angularSpeedFromPoseRamp() throws {
        var history = PoseHistory(window: 0.25)
        for i in 0 ..< 30 {
            let t = Double(i) / 30
            history.add(HeadPose(yaw: Float(30 * t), pitch: 0, roll: 0), at: t)
        }
        let speed = try #require(history.angularSpeed())
        #expect(abs(speed - 30) < 0.5)
        let smoothed = try #require(history.smoothed)
        // Mean over the last 0.25 s of a 30 deg/s ramp lags the newest value by ~half the window.
        #expect(abs(smoothed.yaw - Float(30 * (29.0 / 30) - 30 * 0.125)) < 0.6)
    }

    @Test func angularSpeedIsRobustToJitter() throws {
        // +-1 degree of frame-to-frame jitter would be ~60 deg/s with plain differences at 30 fps.
        var history = PoseHistory(window: 0.25)
        var rng = SystemRandomNumberGenerator()
        for i in 0 ..< 60 {
            let jitter = { Float.random(in: -1 ... 1, using: &rng) }
            history.add(HeadPose(yaw: jitter(), pitch: jitter(), roll: jitter()), at: Double(i) / 30)
        }
        let speed = try #require(history.angularSpeed())
        #expect(speed < 20, "speed \(speed)")
    }

    @Test func poseHistoryResetsOnGapsAndBackwardTime() {
        var history = PoseHistory(window: 0.25, maxGap: 0.5)
        history.add(HeadPose(yaw: 0, pitch: 0, roll: 0), at: 1.0)
        history.add(HeadPose(yaw: 10, pitch: 0, roll: 0), at: 1.03)
        #expect(history.angularSpeed(minSpan: 0.01) != nil)
        history.add(HeadPose(yaw: 40, pitch: 0, roll: 0), at: 2.0)   // gap
        #expect(history.angularSpeed() == nil)
        #expect(history.smoothed?.yaw == 40)
        history.add(HeadPose(yaw: 20, pitch: 0, roll: 0), at: 1.5)   // backwards
        #expect(history.smoothed?.yaw == 20)
        history.reset()
        #expect(history.isEmpty)
    }

    @Test func gyroHistoryKeepsWindowMaximum() {
        var gyro = GyroHistory(window: 0.3)
        gyro.add(1, at: 0.0)
        gyro.add(9, at: 0.1)
        gyro.add(2, at: 0.2)
        #expect(gyro.maximum == 9)
        gyro.add(1, at: 0.45)
        #expect(gyro.maximum == 2)
        gyro.add(1, at: 0.9)
        #expect(gyro.maximum == 1)
    }

    // MARK: Distance and blur reference

    @Test func distanceAdviceHasHysteresis() {
        var c = DistanceClassifier(range: 0.55 ... 0.75, hysteresis: 0.015)
        #expect(c.update(nil) == .unknown)
        #expect(c.update(0.50) == .tooClose)
        #expect(c.update(0.545) == .tooClose)
        #expect(c.update(0.56) == .ok)
        #expect(c.update(0.54) == .ok)          // inside the hysteresis
        #expect(c.update(0.53) == .tooClose)
        #expect(c.update(0.70) == .ok)
        #expect(c.update(0.76) == .ok)
        #expect(c.update(0.77) == .tooFar)
        #expect(c.update(0.76) == .tooFar)
        #expect(c.update(.nan) == .unknown)
    }

    @Test func sharpnessReferenceIsUpperQuartile() {
        var r = SharpnessReference(capacity: 8, minSamples: 4)
        r.add(10); r.add(20); r.add(30)
        #expect(r.reference == nil)
        r.add(40)
        #expect(r.reference == 30)
        for v: Float in [100, 100, 100, 100, 100, 100, 100, 100] { r.add(v) }
        #expect(r.count == 8)
        #expect(r.reference == 100)
        r.add(.infinity)
        #expect(r.count == 8)
    }

    @Test func distanceFromFaceSizeEstimate() throws {
        let k = Intrinsics(width: 1280, height: 800, fx: 640, fy: 640, ppx: 640, ppy: 400)
        let d = try #require(ScanController.distanceFromFaceSize(CGRect(x: 0, y: 0, width: 160, height: 180), intrinsics: k))
        #expect(abs(d - 0.64) < 1e-4)
        #expect(ScanController.distanceFromFaceSize(.zero, intrinsics: k) == nil)
    }
}
