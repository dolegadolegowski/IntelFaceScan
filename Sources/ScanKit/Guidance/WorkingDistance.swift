// License: Apache 2.0. See LICENSE file in root directory.
//
// Recommended face distance for a camera configuration. D4xx stereo depth searches 126 disparities starting at
// the disparity shift, so the nearest measurable distance is Min-Z = f_px * baseline / (126 + shift) and the
// farthest (with a shift) Max-Z = f_px * baseline / shift. Depth error grows with z^2, so the face should be as
// close as possible while staying safely beyond Min-Z (the depth image empties below it).

import CameraKit
import Foundation

public enum WorkingDistance {
    /// Disparity search range of the D4xx stereo matcher, in pixels of the depth resolution.
    public static let disparitySearchRange: Float = 126
    /// Margin beyond Min-Z where the recommended range starts / ends (meters).
    public static let nearMarginM: Float = 0.05
    public static let farMarginM: Float = 0.20
    /// Default depth stream width when nothing is configured (D455 face scan profile).
    public static let defaultDepthWidth = 1280

    /// Nearest measurable distance in meters.
    public static func minZ(depthFocalPx: Float, baselineMm: Float, disparityShift: Int) -> Float {
        let denominator = disparitySearchRange + Float(max(0, disparityShift))
        return depthFocalPx * baselineMm / denominator / 1000
    }

    /// Farthest measurable distance in meters with a disparity shift (nil = unbounded, shift 0).
    public static func maxZ(depthFocalPx: Float, baselineMm: Float, disparityShift: Int) -> Float? {
        guard disparityShift > 0 else { return nil }
        return depthFocalPx * baselineMm / Float(disparityShift) / 1000
    }

    /// Horizontal field of view of the depth stream (degrees) by model; the D455/D435/D405 have ~87 deg.
    public static func depthHorizontalFOVDegrees(deviceName: String?) -> Float {
        let name = deviceName?.uppercased() ?? ""
        if name.contains("D415") { return 65 }
        return 87
    }

    /// Stereo baseline (mm) by model, used when the device does not report it.
    public static func defaultBaselineMm(deviceName: String?) -> Float {
        let name = deviceName?.uppercased() ?? ""
        if name.contains("D405") { return 18 }
        if name.contains("D415") { return 55 }
        if name.contains("D435") || name.contains("D436") { return 50 }
        return 95   // D455 / D456 / D457
    }

    /// Focal length in pixels of a stream with the given width and horizontal field of view.
    public static func focalPx(width: Int, horizontalFOVDegrees: Float) -> Float {
        Float(width) / 2 / tan(horizontalFOVDegrees / 2 * .pi / 180)
    }

    /// Recommended face distance (meters) for a device and capture configuration: [Min-Z + 5 cm, Min-Z + 20 cm],
    /// limited by Max-Z (disparity shift) and the threshold filter, at least 5 cm wide when the limits allow,
    /// rounded to centimeters.
    /// - Parameter depthIntrinsics: intrinsics of the depth stream itself (not of the aligned depth), when known.
    public static func recommendedRange(device: DeviceInfo?, configuration: CaptureConfiguration?,
                                        depthIntrinsics: Intrinsics? = nil) -> ClosedRange<Float> {
        let depthWidth = configuration?.depth.width ?? defaultDepthWidth
        let focal: Float
        if let k = depthIntrinsics, k.fx > 0, k.width > 0 {
            focal = k.fx * Float(depthWidth) / Float(k.width)
        } else {
            focal = focalPx(width: depthWidth, horizontalFOVDegrees: depthHorizontalFOVDegrees(deviceName: device?.name))
        }
        let baseline = device?.baselineMm.flatMap { $0 > 0 && $0.isFinite ? $0 : nil }
            ?? defaultBaselineMm(deviceName: device?.name)
        let shift = configuration?.disparityShift ?? 0
        let near = minZ(depthFocalPx: focal, baselineMm: baseline, disparityShift: shift)
        let far = maxZ(depthFocalPx: focal, baselineMm: baseline, disparityShift: shift)

        // Preferred range, then hard limits: just beyond Min-Z, inside Max-Z and inside the threshold filter.
        var lower = near + nearMarginM
        var upper = near + farMarginM
        var hardLower = near + 0.02
        var hardUpper = Float.infinity
        if let far {
            // The far end of a shifted range is noisy; keep the face well inside it.
            upper = min(upper, near + 0.8 * (far - near))
            hardUpper = near + 0.9 * (far - near)
        }
        if let threshold = configuration?.filters.threshold, threshold.enabled, threshold.maxM > threshold.minM {
            hardLower = max(hardLower, threshold.minM + 0.02)
            hardUpper = min(hardUpper, threshold.maxM - 0.05)
        }
        lower = max(lower, hardLower)
        upper = min(upper, hardUpper)
        if upper - lower < 0.05 {
            upper = min(hardUpper, lower + 0.05)
            lower = max(hardLower, upper - 0.05)
        }
        if !(lower < upper) {
            // Contradictory configuration (e.g. a threshold filter beyond Max-Z): report what the stereo
            // geometry allows, the middle of the measurable range.
            let end = far ?? (near + 2 * farMarginM)
            lower = near + 0.25 * (end - near)
            upper = near + 0.75 * (end - near)
        }
        return round(lower) ... max(round(lower), round(upper))
    }

    private static func round(_ v: Float) -> Float { (v * 100).rounded() / 100 }
}

extension ScanConfiguration {
    /// A copy whose distance range is the recommended one for this device and capture configuration.
    public func withRecommendedDistance(device: DeviceInfo?, captureConfiguration: CaptureConfiguration?) -> ScanConfiguration {
        var c = self
        c.distanceRangeM = WorkingDistance.recommendedRange(device: device, configuration: captureConfiguration)
        return c
    }
}
