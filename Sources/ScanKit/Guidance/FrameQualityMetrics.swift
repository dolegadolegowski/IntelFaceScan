// License: Apache 2.0. See LICENSE file in root directory.
//
// Per-frame quality measures used by the guided scan gates: sharpness of the face, depth coverage and distance,
// head angular speed and camera motion.

import Accelerate
import CameraKit
import CoreGraphics
import Foundation

public enum SharpnessMetric {
    /// Variance of the Laplacian of the gray (BT.601 luma, [0, 255]) image inside `rect` (color pixels).
    /// Returns 0 when the region is smaller than 3x3 pixels or outside the image.
    public static func varianceOfLaplacian(_ image: ColorImage, in rect: CGRect) -> Float {
        guard let region = PixelRect(rect, clippedTo: image.width, image.height),
              region.width >= 3, region.height >= 3,
              var gray = GrayImage.lumaFloats(image, in: region) else { return 0 }
        return varianceOfLaplacian(gray: &gray, width: region.width, height: region.height)
    }

    /// Variance of the Laplacian of a row-major float gray image.
    static func varianceOfLaplacian(gray: inout [Float], width: Int, height: Int) -> Float {
        guard width >= 3, height >= 3, gray.count >= width * height else { return 0 }
        var laplacian = [Float](repeating: 0, count: width * height)
        let kernel: [Float] = [0, 1, 0,
                               1, -4, 1,
                               0, 1, 0]
        let error: vImage_Error = gray.withUnsafeMutableBytes { srcRaw in
            laplacian.withUnsafeMutableBytes { dstRaw in
                var src = vImage_Buffer(data: srcRaw.baseAddress!, height: vImagePixelCount(height),
                                        width: vImagePixelCount(width), rowBytes: width * 4)
                var dst = vImage_Buffer(data: dstRaw.baseAddress!, height: vImagePixelCount(height),
                                        width: vImagePixelCount(width), rowBytes: width * 4)
                // Edge extension: a zero border would add a fake edge around every crop.
                return vImageConvolve_PlanarF(&src, &dst, nil, 0, 0, kernel, 3, 3, 0, vImage_Flags(kvImageEdgeExtend))
            }
        }
        guard error == kvImageNoError else { return 0 }
        var mean: Float = 0, meanSquare: Float = 0
        vDSP_meanv(laplacian, 1, &mean, vDSP_Length(laplacian.count))
        vDSP_measqv(laplacian, 1, &meanSquare, vDSP_Length(laplacian.count))
        return max(0, meanSquare - mean * mean)
    }

    /// The face region whose sharpness matters: the inner part of the face rectangle (hair and background
    /// edges around the face would dominate a full crop).
    public static func faceRegion(_ faceRect: CGRect) -> CGRect {
        faceRect.scaledAboutCenter(0.8, 0.8)
    }
}

/// Depth statistics of the aligned depth image.
public enum DepthStatistics {
    /// Region of the face used for its distance: the center of the face rectangle (nose and cheeks), away from
    /// the silhouette where the aligned depth mixes face and background.
    public static func distanceRegion(_ faceRect: CGRect) -> CGRect {
        faceRect.scaledAboutCenter(0.5, 0.6)
    }

    /// Region of the face whose depth coverage is checked: most of the face rectangle, but not its corners,
    /// which show background in profile views (often beyond the threshold filter, i.e. without depth).
    public static func coverageRegion(_ faceRect: CGRect) -> CGRect {
        faceRect.scaledAboutCenter(0.7, 0.75)
    }

    /// Median depth (meters) of the valid pixels in `rect` (color pixels; scaled when the depth image has a
    /// different size), with 1 mm resolution. nil when fewer than `minSamples` pixels are valid.
    public static func medianDistance(_ depth: DepthImage, in rect: CGRect, colorSize: CGSize? = nil,
                                      minSamples: Int = 16) -> Float? {
        guard depth.isValidZ16, let region = depthRect(rect, depth: depth, colorSize: colorSize) else { return nil }
        let binCount = 8192   // 1 mm bins up to 8.19 m, beyond any D4xx face distance
        var histogram = [UInt32](repeating: 0, count: binCount)
        let toMM = depth.depthScale * 1000
        var samples = 0
        depth.pixels.withUnsafeBytes { raw in
            let values = raw.bindMemory(to: UInt16.self)
            for y in region.y ..< region.y + region.height {
                let row = y * depth.width
                for x in region.x ..< region.x + region.width {
                    let v = UInt16(littleEndian: values[row + x])
                    if v == 0 { continue }
                    let bin = Int((Float(v) * toMM).rounded())
                    if bin <= 0 || bin >= binCount { continue }
                    histogram[bin] += 1
                    samples += 1
                }
            }
        }
        guard samples >= max(1, minSamples) else { return nil }
        let half = (samples + 1) / 2
        var cumulative = 0
        for (bin, count) in histogram.enumerated() where count > 0 {
            cumulative += Int(count)
            if cumulative >= half { return Float(bin) / 1000 }
        }
        return nil
    }

    /// Fraction of the pixels in `rect` with valid (non-zero) depth.
    public static func validRatio(_ depth: DepthImage, in rect: CGRect, colorSize: CGSize? = nil) -> Float {
        guard depth.isValidZ16, let region = depthRect(rect, depth: depth, colorSize: colorSize) else { return 0 }
        var valid = 0
        depth.pixels.withUnsafeBytes { raw in
            let values = raw.bindMemory(to: UInt16.self)
            for y in region.y ..< region.y + region.height {
                let row = y * depth.width
                for x in region.x ..< region.x + region.width where values[row + x] != 0 { valid += 1 }
            }
        }
        return Float(valid) / Float(region.area)
    }

    /// Number of mask pixels (mask at depth resolution, > 0 = inside) with valid depth.
    public static func validPoints(_ depth: DepthImage, mask: Data) -> Int {
        guard depth.isValidZ16, mask.count >= depth.width * depth.height else { return 0 }
        var count = 0
        depth.pixels.withUnsafeBytes { raw in
            mask.withUnsafeBytes { maskRaw in
                let values = raw.bindMemory(to: UInt16.self)
                let m = maskRaw.bindMemory(to: UInt8.self)
                for i in 0 ..< depth.width * depth.height where m[i] != 0 && values[i] != 0 { count += 1 }
            }
        }
        return count
    }

    /// Maps a color-pixel rectangle onto the depth image (the aligned depth normally has the color size).
    static func depthRect(_ rect: CGRect, depth: DepthImage, colorSize: CGSize?) -> PixelRect? {
        var r = rect
        if let colorSize, colorSize.width > 0, colorSize.height > 0,
           Int(colorSize.width) != depth.width || Int(colorSize.height) != depth.height {
            let sx = CGFloat(depth.width) / colorSize.width, sy = CGFloat(depth.height) / colorSize.height
            r = CGRect(x: rect.minX * sx, y: rect.minY * sy, width: rect.width * sx, height: rect.height * sy)
        }
        return PixelRect(r, clippedTo: depth.width, depth.height)
    }
}

public enum CameraMotion {
    /// Gyro magnitude in degrees per second (0 when the sample has no gyro).
    public static func degreesPerSecond(_ imu: IMUSample?) -> Float {
        guard let g = imu?.gyro, g.count >= 3, g[0].isFinite, g[1].isFinite, g[2].isFinite else { return 0 }
        return (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).squareRoot() * 180 / .pi
    }
}

/// Recent head poses: a smoothed pose and the angular speed over a short time window.
///
/// Vision's angles jitter by about a degree from frame to frame; differencing consecutive frames at 30 fps would
/// turn that into ~30 deg/s of fake motion, so the speed is the least-squares slope over the window instead.
/// Vision's readings also jump while the head is still: a reading switches by several degrees within one frame and
/// stays there, or is off for a single frame (on the held synthetic head: 2-4 degrees in pitch on "down", up to 12
/// degrees in yaw at +45 at 1280x800). One step in the 0.25 s window is a least-squares slope of up to 5.7 deg/s
/// per degree, so a 4-degree step alone exceeded the 20 deg/s gate for several frames. A head cannot move like that
/// (a step is a turn at hundreds of deg/s that stops within a frame), so single-frame outliers and isolated steps
/// are removed before the fit; real motion changes the pose over several frames and is measured as before.
public struct PoseHistory: Sendable {
    public var window: Double
    /// A gap longer than this (seconds) or a timestamp going backwards starts a new history.
    public var maxGap: Double
    /// A frame-to-frame change faster than this (deg/s; 2 degrees at 30 fps) can be a jump (see withoutJumps).
    public var minJumpRate: Double = 60
    public var jumpNeighborRatio: Double = 0.5
    private var samples: [(t: Double, pose: HeadPose)] = []

    public init(window: Double = 0.25, maxGap: Double = 0.5) {
        self.window = window; self.maxGap = maxGap
    }

    public var isEmpty: Bool { samples.isEmpty }

    public mutating func add(_ pose: HeadPose, at t: Double) {
        if let last = samples.last, t <= last.t || t - last.t > maxGap { samples.removeAll() }
        samples.append((t, pose))
        let cutoff = t - window
        if let firstKept = samples.firstIndex(where: { $0.t >= cutoff }), firstKept > 0 {
            samples.removeFirst(firstKept)
        }
    }

    public mutating func reset() { samples.removeAll() }

    /// Mean pose over the window.
    public var smoothed: HeadPose? {
        guard !samples.isEmpty else { return nil }
        var yaw: Float = 0, pitch: Float = 0, roll: Float = 0, confidence: Float = 0
        for s in samples {
            yaw += s.pose.yaw; pitch += s.pose.pitch; roll += s.pose.roll; confidence += s.pose.confidence
        }
        let n = Float(samples.count)
        return HeadPose(yaw: yaw / n, pitch: pitch / n, roll: roll / n, confidence: confidence / n)
    }

    /// Angular speed (deg/s) from the least-squares slopes of yaw, pitch and roll after removing Vision's jumps;
    /// nil until the window spans at least `minSpan` seconds.
    public func angularSpeed(minSpan: Double = 0.06) -> Float? {
        guard samples.count >= 2, let first = samples.first, let last = samples.last,
              last.t - first.t >= minSpan else { return nil }
        let t = samples.map(\.t)
        let tMean = t.reduce(0, +) / Double(t.count)
        let stt = t.reduce(0) { $0 + ($1 - tMean) * ($1 - tMean) }
        guard stt > 0 else { return nil }
        var squared = 0.0
        for axis in [samples.map { Double($0.pose.yaw) }, samples.map { Double($0.pose.pitch) },
                     samples.map { Double($0.pose.roll) }] {
            let values = Self.withoutJumps(axis, t: t, minJumpRate: minJumpRate, neighborRatio: jumpNeighborRatio)
            var sty = 0.0
            for (ti, v) in zip(t, values) { sty += (ti - tMean) * v }
            squared += (sty / stt) * (sty / stt)
        }
        return Float(squared.squareRoot())
    }

    /// One angle over time with Vision's jumps removed. A jump is a change faster than `minJumpRate`:
    ///   * a single-frame outlier is reached and left by jumps in opposite directions: it takes the value of the
    ///     nearer neighbor (the median of the three), which leaves at most a step when it sat on one;
    ///   * an isolated step has changes at most `neighborRatio` times as fast before and after it: it is replaced by
    ///     the slower of those changes, so a turn through a step still counts.
    /// An outlier of two frames is two steps. The newest change has no successor yet and is kept, so a fresh jump
    /// counts as motion for a frame rather than hiding the start of a real turn.
    static func withoutJumps(_ x: [Double], t: [Double], minJumpRate: Double, neighborRatio: Double) -> [Double] {
        let n = x.count
        guard n >= 3, t.count == n else { return x }
        func fast(_ change: Double, _ i: Int) -> Bool { abs(change) >= minJumpRate * (t[i] - t[i - 1]) }
        var y = x
        for i in 1 ..< n - 1 {
            let a = y[i - 1], b = x[i], c = x[i + 1]
            if fast(b - a, i) && fast(c - b, i + 1) && (b - a) * (c - b) < 0 {
                y[i] = max(min(a, b), min(max(a, b), c))
            }
        }
        var rates = [Double](repeating: 0, count: n)   // rates[i]: from sample i - 1 to i
        for i in 1 ..< n {
            let dt = t[i] - t[i - 1]
            rates[i] = dt > 0 ? (y[i] - y[i - 1]) / dt : 0
        }
        var out = y
        for i in 1 ..< n {
            var change = y[i] - y[i - 1]
            if i < n - 1, abs(rates[i]) >= minJumpRate {
                let limit = neighborRatio * abs(rates[i])
                if abs(rates[i + 1]) <= limit && (i == 1 || abs(rates[i - 1]) <= limit) {
                    let neighbor = i == 1 || abs(rates[i + 1]) < abs(rates[i - 1]) ? rates[i + 1] : rates[i - 1]
                    change = neighbor * (t[i] - t[i - 1])
                }
            }
            out[i] = out[i - 1] + change
        }
        return out
    }
}

/// Camera motion over a short window: the maximum gyro magnitude, so that a short bump is not missed between
/// the (single) IMU samples attached to the framesets.
public struct GyroHistory: Sendable {
    public var window: Double
    private var samples: [(t: Double, degPerSec: Float)] = []

    public init(window: Double = 0.3) { self.window = window }

    public mutating func add(_ degPerSec: Float, at t: Double) {
        if let last = samples.last, t < last.t { samples.removeAll() }
        samples.append((t, degPerSec))
        let cutoff = t - window
        samples.removeAll { $0.t < cutoff }
    }

    public mutating func reset() { samples.removeAll() }

    public var maximum: Float { samples.map(\.degPerSec).max() ?? 0 }
}

/// Distance advice with hysteresis, so that the advice does not flicker at the edges of the range.
public struct DistanceClassifier: Sendable {
    public var range: ClosedRange<Float>
    public var hysteresis: Float
    public private(set) var advice: DistanceAdvice = .unknown

    public init(range: ClosedRange<Float>, hysteresis: Float = 0.015) {
        self.range = range; self.hysteresis = hysteresis
    }

    @discardableResult
    public mutating func update(_ distance: Float?) -> DistanceAdvice {
        guard let d = distance, d.isFinite, d > 0 else { advice = .unknown; return advice }
        switch advice {
        case .ok:
            if d < range.lowerBound - hysteresis { advice = .tooClose }
            else if d > range.upperBound + hysteresis { advice = .tooFar }
        default:
            if d < range.lowerBound { advice = .tooClose }
            else if d > range.upperBound { advice = .tooFar }
            else { advice = .ok }
        }
        return advice
    }

    public mutating func reset() { advice = .unknown }
}

/// Recent sharpness values of a pose: the blur gate compares a candidate with what this person, light and camera
/// achieve when still, which works without a scene-specific absolute threshold.
public struct SharpnessReference: Sendable {
    public var capacity: Int
    public var minSamples: Int
    private var values: [Float] = []

    public init(capacity: Int = 30, minSamples: Int = 5) {
        self.capacity = capacity; self.minSamples = minSamples
    }

    public mutating func add(_ value: Float) {
        guard value.isFinite else { return }
        values.append(value)
        if values.count > capacity { values.removeFirst(values.count - capacity) }
    }

    public mutating func reset() { values.removeAll() }

    public var count: Int { values.count }

    /// 75th percentile of the recent values; nil until `minSamples` values were seen.
    public var reference: Float? {
        guard values.count >= minSamples else { return nil }
        let sorted = values.sorted()
        return sorted[min(sorted.count - 1, Int(Float(sorted.count - 1) * 0.75 + 0.5))]
    }
}
