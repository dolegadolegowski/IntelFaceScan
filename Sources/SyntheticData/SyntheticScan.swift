// License: Apache 2.0. See LICENSE file in root directory.
//
// Procedurally rendered head scans with ground truth: the keyframes a guided scan would accept (front, up, down,
// left, right, ...), with the true head poses, a Vision-like (noisy) head pose estimate, landmarks, head mask and
// frame quality, plus the ground-truth surface. World frame = camera frame of the first keyframe (frontal pose),
// meters, as in the reconstruction (fr_types.h).

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd

public struct SyntheticScanResult: Sendable {
    public var capture: ScanCapture
    /// Ground-truth world_from_camera per keyframe (keyframe 0 is the identity).
    public var groundTruthPoses: [simd_float4x4]
    /// Ground-truth head surface in the world frame (meters), closed, with normals and vertex colors.
    public var groundTruthMesh: ReconMesh
    /// The exact head orientation of every keyframe (Keyframe.headPose carries the simulated Vision estimate).
    public var groundTruthHeadPoses: [HeadPose] = []
    /// Head center and rotation pivot (near the neck) in the world frame.
    public var headCenter: SIMD3<Float> = .zero
    public var pivot: SIMD3<Float> = .zero
}

public enum SyntheticScan {
    public struct Options: Sendable, Hashable {
        /// Color (= aligned depth) resolution; intrinsics are those of a D455 at 1280x800, scaled.
        public var width = 640
        public var height = 400
        /// Distance from the camera to the face (meters).
        public var distanceM: Float = 0.6
        /// Depth noise sigma at 1 m (scaled with z^2), meters.
        public var depthNoiseAt1m: Float = 0.002
        public var poses: [ScanPose] = ScanPose.required
        public var framesPerPose = 2
        public var seed: UInt64 = 1
        /// Wall behind the head (distance from the camera in meters), nil = no background.
        public var backgroundDistanceM: Float? = 1.1
        /// Standard deviation of the simulated Vision head pose error per angle (degrees).
        public var headPoseErrorDeg: Float = 2
        /// Uniform variation of the true head orientation around each pose target (degrees, +-).
        public var poseVariationDeg: Float = 3
        /// Uniform head translation jitter per axis (meters, +-).
        public var translationJitterM: Float = 0.002
        /// Surfaces seen at a larger angle than this get no depth.
        public var maxIncidenceDeg: Float = 80
        /// Fraction of random invalid depth pixels.
        public var dropoutRate: Float = 0.01
        /// Color sensor noise (8-bit levels, standard deviation).
        public var colorNoise: Float = 1.5
        /// Shading strength of the camera-mounted light (0 = flat albedo).
        public var lighting: Float = 0.25
        /// Landmark detection noise (pixels at 1280x800, scaled with the resolution).
        public var landmarkNoisePx: Float = 1
        /// Cube-sphere tessellation of the head: 12 * meshResolution^2 triangles (128: ~197k).
        public var meshResolution = 128
        /// Lens distortion of the color camera (e.g. LensDistortion.d455Color), nil = ideal pinhole. Color, head mask,
        /// landmarks and face rectangle are rendered through it; the aligned depth follows it only where
        /// librealsense's align does (LensDistortion.alignedDepthFollowsDistortion: modified Brown-Conrady) and stays
        /// in the pinhole geometry otherwise, as the helper delivers a D455's inverse Brown-Conrady color stream.
        /// Both images report the model and coefficients in their intrinsics, as librealsense does.
        public var lensDistortion: LensDistortion? = nil
        /// Head position relative to the default (nose near the image center), camera frame, meters (+X right,
        /// +Y down): a person sitting off the optical axis.
        public var headOffsetM: SIMD2<Float> = .zero
        /// Line-of-sight effect of the simulated Vision head pose: Apple Vision measures the face relative to the line
        /// of sight, so a face seen at angle a right of (below) the optical axis reads gain * a more yaw (less pitch).
        /// Measured with Vision on the rendered head (docs/RECONSTRUCTION.md, "Initial poses"): yaw 0.75, pitch
        /// 0.67, the medians of slopes from 0.2 to 1.5. (0, 0) = Vision without this effect.
        public var visionLineOfSightGain = SIMD2<Float>(0.75, 0.67)

        public init() {}
    }

    /// Simulated session start: a fixed instant, so the same options always produce identical data.
    public static let referenceDate = Date(timeIntervalSince1970: 1_793_739_600)   // 2026-11-03T21:00:00Z

    public static func make(_ options: Options = Options()) -> SyntheticScanResult {
        let scene = SyntheticScene(options: options)
        var random = SplitMix64(seed: options.seed)

        // Plan: true head orientation and translation jitter per keyframe. Keyframe 0 is exactly frontal so that
        // the world frame (its camera frame) is the model frame.
        struct Planned { var scanPose: ScanPose; var headPose: HeadPose; var offset: SIMD3<Float> }
        var plan: [Planned] = []
        for scanPose in options.poses {
            for _ in 0 ..< max(1, options.framesPerPose) {
                let target = scanPose.target
                if plan.isEmpty {
                    plan.append(Planned(scanPose: scanPose, headPose: HeadPose(yaw: target.yaw, pitch: target.pitch, roll: 0),
                                        offset: .zero))
                    continue
                }
                let v = options.poseVariationDeg, j = options.translationJitterM
                let pose = HeadPose(yaw: target.yaw + v * random.symmetric(), pitch: target.pitch + v * random.symmetric(),
                                    roll: 0.6 * v * random.symmetric())
                plan.append(Planned(scanPose: scanPose, headPose: pose,
                                    offset: SIMD3(j * random.symmetric(), j * random.symmetric(), j * random.symmetric())))
            }
        }

        let cameraFromModel = plan.map { scene.cameraFromWorld(headPose: $0.headPose, translation: $0.offset) }
        // world = camera frame of keyframe 0; identical to the model frame unless the first pose is not frontal.
        let worldFromModel = cameraFromModel.first ?? matrix_identity_float4x4
        let modelFromWorld = worldFromModel.inverse

        var keyframes: [Keyframe] = []
        var poses: [simd_float4x4] = []
        let landmarkSigma = options.landmarkNoisePx * Float(options.width) / 1280
        for (i, planned) in plan.enumerated() {
            let sequence = UInt64(120 + 45 * i)
            let timestamp = referenceDate.timeIntervalSince1970 * 1000 + 1500 * Double(i)
            let view = scene.render(cameraFromWorld: cameraFromModel[i], frameIndex: UInt64(i), sequence: sequence,
                                    timestampMs: timestamp)
            poses.append(worldFromModel * cameraFromModel[i].inverse)

            let landmarks = scene.landmarks(cameraFromWorld: cameraFromModel[i]).map {
                $0 + SIMD2(landmarkSigma * random.gaussian(), landmarkSigma * random.gaussian())
            }
            let faceRect = SyntheticScene.faceRect(landmarks: landmarks, width: options.width, height: options.height)
            // Vision reads the face relative to its line of sight, through the center of its face rectangle.
            let lineOfSight = scene.lineOfSightDegrees(cameraFromWorld: cameraFromModel[i])
            let e = options.headPoseErrorDeg, gain = options.visionLineOfSightGain
            let estimate = HeadPose(yaw: planned.headPose.yaw + gain.x * lineOfSight.x + e * random.gaussian(),
                                    pitch: planned.headPose.pitch - gain.y * lineOfSight.y + e * random.gaussian(),
                                    roll: planned.headPose.roll + e * random.gaussian(),
                                    confidence: 0.9 + 0.09 * random.uniform())
            var quality = SyntheticScene.quality(view: view, faceRect: faceRect)
            quality.angularSpeedDegPerSec = 2 + 6 * random.uniform()
            quality.cameraMotionDegPerSec = 0.1 + 0.3 * random.uniform()
            keyframes.append(Keyframe(id: i, frame: view.frame, scanPose: planned.scanPose, headPose: estimate,
                                      faceRect: faceRect, landmarks: landmarks, mask: view.mask, quality: quality))
        }

        var configuration = CaptureConfiguration.faceScanD455
        configuration.color = StreamSettings(width: options.width, height: options.height, fps: 30)
        configuration.depth = StreamSettings(width: options.width, height: options.height, fps: 30)
        let notes = [
            "source": "synthetic",
            "seed": String(options.seed),
            "depth_noise_at_1m_m": String(options.depthNoiseAt1m),
            "distance_m": String(options.distanceM),
        ]
        let capture = ScanCapture(keyframes: keyframes, device: SyntheticScene.deviceInfo(width: options.width, height: options.height),
                                  configuration: configuration, createdAt: referenceDate, notes: notes)
        let center = modelFromWorld.inverse * SIMD4(scene.headCenter, 1)
        let pivot = modelFromWorld.inverse * SIMD4(scene.pivot, 1)
        return SyntheticScanResult(capture: capture, groundTruthPoses: poses,
                                   groundTruthMesh: scene.groundTruthMesh(worldFromModel: worldFromModel),
                                   groundTruthHeadPoses: plan.map { $0.headPose },
                                   headCenter: SIMD3(center.x, center.y, center.z), pivot: SIMD3(pivot.x, pivot.y, pivot.z))
    }
}

/// One rendered view of the synthetic head.
public struct SyntheticView: Sendable {
    public var frame: CameraFrame
    /// Head mask (255 = head above the neck cut), color resolution.
    public var mask: Data
    /// Noise-free depth of the visible surface in meters (0 = nothing), before any invalid-pixel simulation.
    public var cleanDepth: [Float]
    public var cameraFromWorld: simd_float4x4
}

/// The synthetic head placed in front of a D455-like camera; renders arbitrary head poses.
public final class SyntheticScene: @unchecked Sendable {
    public let options: SyntheticScan.Options
    /// Color (and aligned depth) intrinsics, with the lens distortion model when there is one.
    public let intrinsics: Intrinsics
    /// Head center and rotation pivot (near the base of the neck), world frame.
    public let headCenter: SIMD3<Float>
    public let pivot: SIMD3<Float>
    let model: HeadModel
    let center: Vec3
    /// The color lens (nil = pinhole) and its pixel rays.
    let lens: LensDistortion?
    let rays: RayTable?

    public init(options: SyntheticScan.Options = SyntheticScan.Options()) {
        self.options = options
        let pinhole = SyntheticScene.intrinsics(width: options.width, height: options.height)
        lens = options.lensDistortion.flatMap { $0.isIdentity ? nil : $0 }
        if let lens {
            var k = pinhole
            k.model = lens.model.rawValue
            k.coeffs = lens.coeffs
            intrinsics = k
            rays = RayTable(intrinsics: pinhole, distortion: lens)
        } else {
            intrinsics = pinhole
            rays = nil
        }
        model = HeadModel.shared(resolution: options.meshResolution)
        // The face surface is ~8 cm in front of the head center; the nose sits near the image center.
        center = Vec3(options.headOffsetM.x, -0.03 + options.headOffsetM.y, options.distanceM + 0.08)
        headCenter = center.simd
        pivot = SIMD3(center.x, center.y + 0.11, center.z + 0.02)
    }

    /// Projects a camera-frame point into the color image, through the lens (rs2_project_point_to_pixel).
    public func project(_ p: SIMD3<Float>) -> SIMD2<Float> {
        lens.map { $0.project(p, intrinsics) } ?? intrinsics.project(p)
    }

    /// Angles (degrees; x toward the image right, y toward the bottom) of the line of sight through the center of
    /// the noise-free face rectangle, in the ideal pinhole camera: what Apple Vision measures the head pose against.
    public func lineOfSightDegrees(cameraFromWorld: simd_float4x4) -> SIMD2<Float> {
        let pose = Rigid(cameraFromWorld) * worldFromLocal
        let pinhole = model.landmarks.map { intrinsics.project(pose.apply($0).simd) }
        let rect = SyntheticScene.faceRect(landmarks: pinhole, width: 1 << 20, height: 1 << 20)
        let toDegrees: Float = 180 / .pi
        return SIMD2(atan((Float(rect.midX) - intrinsics.ppx) / intrinsics.fx) * toDegrees,
                     atan((Float(rect.midY) - intrinsics.ppy) / intrinsics.fy) * toDegrees)
    }

    /// D455 color intrinsics at 1280x800 scaled to the requested resolution (the principal point is slightly off
    /// center, as on real units).
    public static func intrinsics(width: Int, height: Int) -> Intrinsics {
        Intrinsics(width: 1280, height: 800, fx: 640.4, fy: 639.9, ppx: 641.2, ppy: 401.3)
            .scaled(toWidth: max(width, 16), height: max(height, 16))
    }

    /// camera_from_world for a head orientation: the head turns about the pivot (p_cam = R (p - pivot) + pivot + t).
    public func cameraFromWorld(headPose: HeadPose, translation: SIMD3<Float> = .zero) -> simd_float4x4 {
        let r = simd_float3x3(headPose.cameraFrameRotation)
        let t = pivot - r * pivot + translation
        return simd_float4x4(columns: (SIMD4(r.columns.0, 0), SIMD4(r.columns.1, 0), SIMD4(r.columns.2, 0), SIMD4(t, 1)))
    }

    /// world_from_local (local = head frame of HeadShape).
    var worldFromLocal: Rigid {
        Rigid(r: Mat3(m00: 1, m01: 0, m02: 0, m10: 0, m11: -1, m12: 0, m20: 0, m21: 0, m22: -1), t: center)
    }

    /// Renders one view. With a lens distortion, color and mask come from a pass through the lens; the aligned depth
    /// (and cleanDepth) from the same pass when librealsense's align follows the distortion, otherwise from a
    /// pinhole pass, which equals the depth of the same scan without distortion.
    public func render(cameraFromWorld: simd_float4x4, frameIndex: UInt64, sequence: UInt64 = 0,
                       timestampMs: Double = 0) -> SyntheticView {
        var settings = RenderSettings(intrinsics: intrinsics)
        settings.depthNoiseAt1m = options.depthNoiseAt1m
        settings.maxIncidenceDeg = options.maxIncidenceDeg
        settings.dropoutRate = options.dropoutRate
        settings.colorNoise = options.colorNoise
        settings.lighting = options.lighting
        settings.backgroundDistance = options.backgroundDistanceM
        settings.seed = options.seed
        let pose = Rigid(cameraFromWorld) * worldFromLocal
        var output = SyntheticRenderer.render(model: model, cameraFromLocal: pose, settings: settings, frameKey: frameIndex)
        if let lens {
            var distorted = settings
            distorted.distortion = lens
            distorted.rays = rays
            let view = SyntheticRenderer.render(model: model, cameraFromLocal: pose, settings: distorted, frameKey: frameIndex)
            if lens.alignedDepthFollowsDistortion {
                output = view
            } else {
                output.color = view.color
                output.mask = view.mask
            }
        }
        let w = intrinsics.width, h = intrinsics.height
        let color = ColorImage(width: w, height: h, bytesPerRow: w * 3, pixels: output.color, intrinsics: intrinsics,
                               frameNumber: sequence, timestampMs: timestampMs)
        let depth = DepthImage(width: w, height: h, pixels: output.depth, depthScale: settings.depthScale,
                               intrinsics: intrinsics, frameNumber: sequence, timestampMs: timestampMs - 1.2)
        var random = SplitMix64(seed: SplitMix64.mix(options.seed ^ frameIndex ^ 0x1_4D0))
        let imu = IMUSample(accel: [0.02 * random.gaussian(), -9.81 + 0.02 * random.gaussian(), 0.02 * random.gaussian()],
                            gyro: [0.001 * random.gaussian(), 0.001 * random.gaussian(), 0.001 * random.gaussian()],
                            timestampMs: timestampMs - 2.5)
        let frame = CameraFrame(sequence: sequence, timestampMs: timestampMs, color: color, alignedDepth: depth,
                                depthToColor: SyntheticScene.depthToColor, imu: imu,
                                metadata: ["color_exposure_us": 8000, "depth_exposure_us": 8500, "laser_power": 360])
        return SyntheticView(frame: frame, mask: output.mask, cleanDepth: output.cleanDepth, cameraFromWorld: cameraFromWorld)
    }

    /// The 76 landmarks projected into the color image, through the lens (no detection noise).
    public func landmarks(cameraFromWorld: simd_float4x4) -> [SIMD2<Float>] {
        let pose = Rigid(cameraFromWorld) * worldFromLocal
        return model.landmarks.map { project(pose.apply($0).simd) }
    }

    /// Landmarks in the world frame.
    public var landmarkPositions: [SIMD3<Float>] { model.landmarks.map { worldFromLocal.apply($0).simd } }

    /// The ground-truth surface in the world frame.
    public func groundTruthMesh(worldFromModel: simd_float4x4 = matrix_identity_float4x4) -> ReconMesh {
        let data = model.worldMesh(center: center, transform: Rigid(worldFromModel))
        return ReconMesh(positions: data.positions, normals: data.normals, colors: data.colors, indices: data.indices)
    }

    static let depthToColor = Extrinsics(rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1], translation: [-0.0590, 0.0001, 0.0004])

    /// Face rectangle like Vision's: landmark bounds extended up to the forehead.
    static func faceRect(landmarks: [SIMD2<Float>], width: Int, height: Int) -> CGRect {
        guard var lo = landmarks.first, var hi = landmarks.first else { return .zero }
        for p in landmarks {
            lo = simd_min(lo, p)
            hi = simd_max(hi, p)
        }
        let w = hi.x - lo.x, h = hi.y - lo.y
        let x0 = max(0, lo.x - 0.05 * w), x1 = min(Float(width), hi.x + 0.05 * w)
        let y0 = max(0, lo.y - 0.25 * h), y1 = min(Float(height), hi.y + 0.03 * h)
        guard x1 > x0, y1 > y0 else { return .zero }
        return CGRect(x: CGFloat(x0), y: CGFloat(y0), width: CGFloat(x1 - x0), height: CGFloat(y1 - y0))
    }

    /// Frame quality measured on the rendered images (sharpness, depth coverage and distance of the face).
    static func quality(view: SyntheticView, faceRect: CGRect) -> FrameQuality {
        let frame = view.frame
        let w = frame.color.width, h = frame.color.height
        let x0 = max(1, Int(faceRect.minX)), x1 = min(w - 2, Int(faceRect.maxX))
        let y0 = max(1, Int(faceRect.minY)), y1 = min(h - 2, Int(faceRect.maxY))
        var sum: Double = 0, sum2: Double = 0, count = 0
        var faceDepths: [Float] = []
        var facePixels = 0, faceValid = 0, headValid = 0
        frame.color.pixels.withUnsafeBytes { colorBytes in
            frame.alignedDepth.pixels.withUnsafeBytes { depthBytes in
                view.mask.withUnsafeBytes { maskBytes in
                    let rgb = colorBytes.bindMemory(to: UInt8.self)
                    let depth = depthBytes.bindMemory(to: UInt16.self)
                    let mask = maskBytes.bindMemory(to: UInt8.self)
                    func gray(_ x: Int, _ y: Int) -> Double {
                        let i = 3 * (y * w + x)
                        return 0.299 * Double(rgb[i]) + 0.587 * Double(rgb[i + 1]) + 0.114 * Double(rgb[i + 2])
                    }
                    if x1 > x0 && y1 > y0 {
                        for y in y0 ... y1 {
                            for x in x0 ... x1 {
                                let laplacian = gray(x - 1, y) + gray(x + 1, y) + gray(x, y - 1) + gray(x, y + 1) - 4 * gray(x, y)
                                sum += laplacian
                                sum2 += laplacian * laplacian
                                count += 1
                                if mask[y * w + x] != 0 {
                                    facePixels += 1
                                    let d = UInt16(littleEndian: depth[y * w + x])
                                    if d != 0 {
                                        faceValid += 1
                                        faceDepths.append(Float(d) * frame.alignedDepth.depthScale)
                                    }
                                }
                            }
                        }
                    }
                    for i in 0 ..< w * h where mask[i] != 0 && depth[i] != 0 { headValid += 1 }
                }
            }
        }
        let mean = count > 0 ? sum / Double(count) : 0
        let variance = count > 0 ? max(0, sum2 / Double(count) - mean * mean) : 0
        faceDepths.sort()
        let median = faceDepths.isEmpty ? 0 : faceDepths[faceDepths.count / 2]
        return FrameQuality(sharpness: Float(variance), angularSpeedDegPerSec: 0,
                            validDepthRatio: facePixels > 0 ? Float(faceValid) / Float(facePixels) : 0,
                            validDepthPoints: headValid, faceDistanceM: median, cameraMotionDegPerSec: 0)
    }

    /// A plausible D455 description for synthetic sources. DeviceInfo has no public memberwise initializer, so it is
    /// decoded from its protocol JSON (docs/PROTOCOL.md, QUERY_DEVICE).
    public static func deviceInfo(width: Int, height: Int, fps: Int = 30, source: String = "synthetic") -> DeviceInfo {
        let json: [String: Any] = [
            "name": "Synthetic D455", "serial": "SYN000000455", "firmware": "5.17.0.10",
            "recommended_firmware": "5.17.0.10", "usb_type": "3.2", "product_id": "0B5C", "product_line": "D400",
            "advanced_mode": true, "depth_scale": 0.0001, "baseline_mm": 95.0, "imu": true, "source": source,
            "presets": ["Custom", "Default", "Hand", "High Accuracy", "High Density", "Medium Density"],
            "sensors": [
                ["name": "Stereo Module", "kind": "depth", "options": [
                    ["name": "Laser Power", "value": 360, "min": 0, "max": 360, "step": 30, "default": 150,
                     "read_only": false],
                    ["name": "Emitter Enabled", "value": 1, "min": 0, "max": 2, "step": 1, "default": 1,
                     "read_only": false, "value_descriptions": ["0": "Off", "1": "Laser", "2": "Laser Auto"]],
                    ["name": "Enable Auto Exposure", "value": 1, "min": 0, "max": 1, "step": 1, "default": 1,
                     "read_only": false],
                ]],
                ["name": "RGB Camera", "kind": "color", "options": [
                    ["name": "Enable Auto Exposure", "value": 1, "min": 0, "max": 1, "step": 1, "default": 1,
                     "read_only": false],
                ]],
                ["name": "Motion Module", "kind": "motion", "options": []],
            ],
            "profiles": [
                ["stream": "depth", "index": 0, "format": "z16", "width": width, "height": height, "fps": fps],
                ["stream": "color", "index": 0, "format": "rgb8", "width": width, "height": height, "fps": fps],
                ["stream": "accel", "index": 0, "format": "motion_xyz32f", "width": 0, "height": 0, "fps": 200],
                ["stream": "gyro", "index": 0, "format": "motion_xyz32f", "width": 0, "height": 0, "fps": 200],
            ],
        ]
        do {
            let data = try JSONSerialization.data(withJSONObject: json)
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            return try decoder.decode(DeviceInfo.self, from: data)
        } catch {
            preconditionFailure("synthetic DeviceInfo JSON does not match CameraTypes.swift: \(error)")
        }
    }
}
