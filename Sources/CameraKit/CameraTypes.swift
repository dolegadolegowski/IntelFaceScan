// License: Apache 2.0. See LICENSE file in root directory.
//
// Shared camera data types. These mirror the JSON of docs/PROTOCOL.md (snake_case on the wire, converted with
// JSONDecoder.keyDecodingStrategy = .convertFromSnakeCase / JSONEncoder.keyEncodingStrategy = .convertToSnakeCase).
// Changing a type here changes the protocol: update docs/PROTOCOL.md and the helper together.

import Foundation
import simd

// MARK: - Calibration

/// Pinhole intrinsics of a stream, as reported by librealsense (factory calibration).
public struct Intrinsics: Codable, Sendable, Equatable, Hashable {
    public var width: Int
    public var height: Int
    public var fx: Float
    public var fy: Float
    public var ppx: Float
    public var ppy: Float
    /// librealsense distortion model name, e.g. "none", "brown_conrady", "inverse_brown_conrady".
    public var model: String
    public var coeffs: [Float]

    public init(width: Int, height: Int, fx: Float, fy: Float, ppx: Float, ppy: Float,
                model: String = "none", coeffs: [Float] = [0, 0, 0, 0, 0]) {
        self.width = width; self.height = height
        self.fx = fx; self.fy = fy; self.ppx = ppx; self.ppy = ppy
        self.model = model; self.coeffs = coeffs
    }

    /// Projects a camera-frame point (meters, +Z forward) to pixel coordinates.
    public func project(_ p: SIMD3<Float>) -> SIMD2<Float> {
        SIMD2(fx * p.x / p.z + ppx, fy * p.y / p.z + ppy)
    }

    /// Back-projects a pixel at depth z (meters) to a camera-frame point.
    public func deproject(_ pixel: SIMD2<Float>, depth z: Float) -> SIMD3<Float> {
        SIMD3((pixel.x - ppx) / fx * z, (pixel.y - ppy) / fy * z, z)
    }

    /// Scales the intrinsics to another resolution of the same stream (e.g. preview downsampling).
    public func scaled(toWidth w: Int, height h: Int) -> Intrinsics {
        let sx = Float(w) / Float(width), sy = Float(h) / Float(height)
        return Intrinsics(width: w, height: h, fx: fx * sx, fy: fy * sy, ppx: ppx * sx, ppy: ppy * sy,
                          model: model, coeffs: coeffs)
    }
}

/// Rigid transform between two sensors (librealsense rs2_extrinsics): rotation is a 3x3 matrix in column-major
/// order, translation in meters. Maps points from the source sensor frame to the target sensor frame.
public struct Extrinsics: Codable, Sendable, Equatable, Hashable {
    public var rotation: [Float]
    public var translation: [Float]

    public init(rotation: [Float], translation: [Float]) {
        self.rotation = rotation
        self.translation = translation
    }

    public static let identity = Extrinsics(rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1], translation: [0, 0, 0])

    /// The same transform as a 4x4 matrix.
    public var matrix: simd_float4x4 {
        guard rotation.count == 9, translation.count == 3 else { return matrix_identity_float4x4 }
        return simd_float4x4(columns: (
            SIMD4(rotation[0], rotation[1], rotation[2], 0),
            SIMD4(rotation[3], rotation[4], rotation[5], 0),
            SIMD4(rotation[6], rotation[7], rotation[8], 0),
            SIMD4(translation[0], translation[1], translation[2], 1)))
    }
}

// MARK: - Device description

public enum SensorKind: String, Codable, Sendable, CaseIterable {
    case depth
    case color
    case motion
}

public struct StreamProfile: Codable, Sendable, Hashable {
    /// "depth", "color", "infrared", "accel", "gyro"
    public var stream: String
    public var index: Int
    /// "z16", "rgb8", "y8", "motion_xyz32f", ...
    public var format: String
    public var width: Int
    public var height: Int
    public var fps: Int

    public init(stream: String, index: Int, format: String, width: Int, height: Int, fps: Int) {
        self.stream = stream; self.index = index; self.format = format
        self.width = width; self.height = height; self.fps = fps
    }
}

public struct SensorOption: Codable, Sendable, Hashable {
    /// librealsense option name (rs2_option_to_string), e.g. "Laser Power", "Visual Preset".
    public var name: String
    public var value: Float
    public var min: Float
    public var max: Float
    public var step: Float
    public var `default`: Float
    public var readOnly: Bool
    /// Value descriptions for enumerated options (e.g. visual presets), keyed by the value printed as integer.
    public var valueDescriptions: [String: String]?
}

public struct SensorInfo: Codable, Sendable, Hashable {
    /// e.g. "Stereo Module", "RGB Camera", "Motion Module"
    public var name: String
    public var kind: SensorKind
    public var options: [SensorOption]
}

public struct DeviceInfo: Codable, Sendable, Hashable {
    public var name: String
    public var serial: String
    public var firmware: String
    public var recommendedFirmware: String?
    /// USB descriptor, e.g. "3.2"; anything below 3 limits the available profiles.
    public var usbType: String
    public var productId: String
    public var productLine: String
    public var advancedMode: Bool
    /// Meters per Z16 unit currently configured on the depth sensor.
    public var depthScale: Float
    /// Stereo baseline in millimeters (from the depth sensor's extrinsics).
    public var baselineMm: Float?
    public var sensors: [SensorInfo]
    public var profiles: [StreamProfile]
    /// Visual preset names supported by the depth sensor.
    public var presets: [String]
    public var imu: Bool
    /// "live" for a camera, "playback" for a recorded file.
    public var source: String
}

// MARK: - Capture configuration

public struct StreamSettings: Codable, Sendable, Hashable {
    public var width: Int
    public var height: Int
    public var fps: Int
    public init(width: Int, height: Int, fps: Int) { self.width = width; self.height = height; self.fps = fps }
}

/// librealsense post-processing applied by the helper, in this order:
/// threshold -> depth-to-disparity -> spatial -> temporal -> disparity-to-depth -> hole filling -> align to color.
/// Decimation is never applied to frames used for reconstruction (the app downsamples for preview itself).
public struct FilterConfiguration: Codable, Sendable, Hashable {
    public struct Threshold: Codable, Sendable, Hashable {
        public var enabled: Bool
        public var minM: Float
        public var maxM: Float
        public init(enabled: Bool, minM: Float, maxM: Float) { self.enabled = enabled; self.minM = minM; self.maxM = maxM }
    }
    public struct Spatial: Codable, Sendable, Hashable {
        public var enabled: Bool
        public var magnitude: Int     // 1...5 iterations
        public var alpha: Float       // 0.25...1
        public var delta: Int         // 1...50 (step size boundary, disparity units)
        public var holesFill: Int     // 0...5
        public init(enabled: Bool, magnitude: Int, alpha: Float, delta: Int, holesFill: Int) {
            self.enabled = enabled; self.magnitude = magnitude; self.alpha = alpha; self.delta = delta; self.holesFill = holesFill
        }
    }
    public struct Temporal: Codable, Sendable, Hashable {
        public var enabled: Bool
        public var alpha: Float       // 0...1
        public var delta: Int         // 1...100
        public var persistence: Int   // 0...8 (librealsense persistency index)
        public init(enabled: Bool, alpha: Float, delta: Int, persistence: Int) {
            self.enabled = enabled; self.alpha = alpha; self.delta = delta; self.persistence = persistence
        }
    }
    public struct HoleFilling: Codable, Sendable, Hashable {
        public var enabled: Bool
        public var mode: Int          // 0 fill from left, 1 farthest from around, 2 nearest from around
        public init(enabled: Bool, mode: Int) { self.enabled = enabled; self.mode = mode }
    }

    public var threshold: Threshold
    /// Run spatial/temporal filters in the disparity domain (recommended for stereo depth).
    public var disparityDomain: Bool
    public var spatial: Spatial
    public var temporal: Temporal
    public var holeFilling: HoleFilling

    public init(threshold: Threshold, disparityDomain: Bool, spatial: Spatial, temporal: Temporal, holeFilling: HoleFilling) {
        self.threshold = threshold; self.disparityDomain = disparityDomain
        self.spatial = spatial; self.temporal = temporal; self.holeFilling = holeFilling
    }

    /// Starting point for face scanning; the capture module tunes these values by measurement (docs/CAMERA.md).
    public static let faceScan = FilterConfiguration(
        threshold: Threshold(enabled: true, minM: 0.2, maxM: 1.2),
        disparityDomain: true,
        spatial: Spatial(enabled: true, magnitude: 2, alpha: 0.5, delta: 20, holesFill: 0),
        temporal: Temporal(enabled: true, alpha: 0.4, delta: 20, persistence: 3),
        holeFilling: HoleFilling(enabled: false, mode: 1))
}

public struct CaptureConfiguration: Codable, Sendable, Hashable {
    public var depth: StreamSettings
    public var color: StreamSettings
    public var enableIr: Bool
    public var enableImu: Bool
    /// Visual preset name, e.g. "High Accuracy", "High Density", "Default".
    public var preset: String
    /// Laser power in mW; nil = maximum supported by the device.
    public var laserPower: Float?
    public var emitterEnabled: Bool
    /// Meters per Z16 unit (D4xx default 0.001; 0.0001 gives 0.1 mm quantization up to 6.5 m).
    public var depthUnits: Float
    /// Advanced-mode disparity shift (0 = off). Moves the working range closer (smaller Min-Z, smaller Max-Z).
    public var disparityShift: Int
    public var depthAutoExposure: Bool
    public var colorAutoExposure: Bool
    /// Also send the filtered depth in the depth sensor's own frame (not only aligned to color).
    public var sendDepth: Bool
    public var sendIr: Bool
    /// Align depth to color (the reconstruction input). Should stay true.
    public var alignToColor: Bool
    public var filters: FilterConfiguration

    public init(depth: StreamSettings, color: StreamSettings, enableIr: Bool, enableImu: Bool, preset: String,
                laserPower: Float?, emitterEnabled: Bool, depthUnits: Float, disparityShift: Int,
                depthAutoExposure: Bool, colorAutoExposure: Bool, sendDepth: Bool, sendIr: Bool,
                alignToColor: Bool, filters: FilterConfiguration) {
        self.depth = depth; self.color = color; self.enableIr = enableIr; self.enableImu = enableImu
        self.preset = preset; self.laserPower = laserPower; self.emitterEnabled = emitterEnabled
        self.depthUnits = depthUnits; self.disparityShift = disparityShift
        self.depthAutoExposure = depthAutoExposure; self.colorAutoExposure = colorAutoExposure
        self.sendDepth = sendDepth; self.sendIr = sendIr; self.alignToColor = alignToColor; self.filters = filters
    }

    /// Default face-scan configuration for a D455 (refined by measurement, see docs/CAMERA.md).
    public static let faceScanD455 = CaptureConfiguration(
        depth: StreamSettings(width: 1280, height: 720, fps: 30),
        color: StreamSettings(width: 1280, height: 800, fps: 30),
        enableIr: false, enableImu: true, preset: "High Accuracy", laserPower: nil, emitterEnabled: true,
        depthUnits: 0.0001, disparityShift: 0, depthAutoExposure: true, colorAutoExposure: true,
        sendDepth: false, sendIr: false, alignToColor: true, filters: .faceScan)
}

// MARK: - Frames

/// 8-bit RGB image (3 bytes per pixel, rows of bytesPerRow bytes).
public struct ColorImage: Sendable {
    public var width: Int
    public var height: Int
    public var bytesPerRow: Int
    public var pixels: Data
    public var intrinsics: Intrinsics
    public var frameNumber: UInt64
    public var timestampMs: Double

    public init(width: Int, height: Int, bytesPerRow: Int, pixels: Data, intrinsics: Intrinsics,
                frameNumber: UInt64 = 0, timestampMs: Double = 0) {
        self.width = width; self.height = height; self.bytesPerRow = bytesPerRow; self.pixels = pixels
        self.intrinsics = intrinsics; self.frameNumber = frameNumber; self.timestampMs = timestampMs
    }
}

/// 16-bit depth image (little-endian UInt16 per pixel, tightly packed rows, 0 = no data).
public struct DepthImage: Sendable {
    public var width: Int
    public var height: Int
    public var pixels: Data
    /// Meters per unit.
    public var depthScale: Float
    public var intrinsics: Intrinsics
    public var frameNumber: UInt64
    public var timestampMs: Double

    public init(width: Int, height: Int, pixels: Data, depthScale: Float, intrinsics: Intrinsics,
                frameNumber: UInt64 = 0, timestampMs: Double = 0) {
        self.width = width; self.height = height; self.pixels = pixels; self.depthScale = depthScale
        self.intrinsics = intrinsics; self.frameNumber = frameNumber; self.timestampMs = timestampMs
    }

    /// Depth in meters at a pixel (0 when invalid or out of bounds).
    public func meters(x: Int, y: Int) -> Float {
        guard x >= 0, y >= 0, x < width, y < height else { return 0 }
        let raw: UInt16 = pixels.withUnsafeBytes { $0.load(fromByteOffset: (y * width + x) * 2, as: UInt16.self) }
        return Float(UInt16(littleEndian: raw)) * depthScale
    }
}

public struct IMUSample: Codable, Sendable, Hashable {
    /// m/s^2 in the motion module frame
    public var accel: [Float]?
    /// rad/s in the motion module frame
    public var gyro: [Float]?
    public var timestampMs: Double
    public init(accel: [Float]?, gyro: [Float]?, timestampMs: Double) {
        self.accel = accel; self.gyro = gyro; self.timestampMs = timestampMs
    }
}

/// One synchronized frameset from the camera (or a recording / synthetic source).
public struct CameraFrame: Sendable {
    public var sequence: UInt64
    public var timestampMs: Double
    public var color: ColorImage
    /// Filtered depth re-projected onto the color image: same size and intrinsics as `color`.
    public var alignedDepth: DepthImage
    /// Filtered depth in the depth sensor frame (only when CaptureConfiguration.sendDepth).
    public var depth: DepthImage?
    public var depthToColor: Extrinsics
    public var imu: IMUSample?
    /// Per-frame metadata, e.g. "color_exposure_us", "depth_exposure_us", "laser_power".
    public var metadata: [String: Double]

    public init(sequence: UInt64, timestampMs: Double, color: ColorImage, alignedDepth: DepthImage,
                depth: DepthImage? = nil, depthToColor: Extrinsics = .identity, imu: IMUSample? = nil,
                metadata: [String: Double] = [:]) {
        self.sequence = sequence; self.timestampMs = timestampMs; self.color = color
        self.alignedDepth = alignedDepth; self.depth = depth; self.depthToColor = depthToColor
        self.imu = imu; self.metadata = metadata
    }
}

// MARK: - Events and errors

public enum LogLevel: String, Codable, Sendable {
    case debug, info, warning, error
}

public enum CameraEvent: Sendable {
    case connected(DeviceInfo)
    case disconnected(reason: String)
    case streamingStarted
    case streamingStopped
    /// The firmware restarted a stream (e.g. RGB toggled while depth streams); frame counters reset.
    case streamRestarted(String)
    case recordingStarted
    case recordingFinished(bytes: Int64)
    case playbackFinished
    case log(LogLevel, String)
    case error(String)
    /// The helper process exited (after handing the camera back to macOS when it was live).
    case helperExited(status: Int32)
}

public enum CameraError: Error, LocalizedError, Sendable, Equatable {
    case helperNotFound(String)
    case authorizationCancelled
    case launchFailed(String)
    case connectionFailed(String)
    case protocolError(String)
    case commandFailed(String)
    case notConnected
    case unsupported(String)
    case timeout(String)

    public var errorDescription: String? {
        switch self {
        case .helperNotFound(let path): return "Camera helper not found: \(path)"
        case .authorizationCancelled: return "Administrator authorization was cancelled."
        case .launchFailed(let m): return "Could not start the camera helper: \(m)"
        case .connectionFailed(let m): return "Could not connect to the camera helper: \(m)"
        case .protocolError(let m): return "Camera helper protocol error: \(m)"
        case .commandFailed(let m): return m
        case .notConnected: return "The camera is not connected."
        case .unsupported(let m): return "Not supported: \(m)"
        case .timeout(let m): return "Timed out: \(m)"
        }
    }
}
