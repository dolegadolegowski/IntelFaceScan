// License: Apache 2.0. See LICENSE file in root directory.
//
// User settings, persisted in UserDefaults as one JSON object, and their mapping onto the camera, scan and
// reconstruction configurations.

import CameraKit
import Foundation
import ReconKit
import ScanKit

struct AppSettings: Codable, Equatable {
    enum DepthResolution: String, Codable, CaseIterable, Identifiable {
        case hd720 = "1280x720"
        case wvga480 = "848x480"
        var id: String { rawValue }
        var size: (width: Int, height: Int) { self == .hd720 ? (1280, 720) : (848, 480) }
        var label: String { self == .hd720 ? "1280 × 720" : "848 × 480" }
    }

    enum DepthPreset: String, Codable, CaseIterable, Identifiable {
        case highAccuracy = "High Accuracy"
        case highDensity = "High Density"
        var id: String { rawValue }
        var label: String { self == .highAccuracy ? S.highAccuracy : S.highDensity }
    }

    enum PreviewMode: String, Codable, CaseIterable, Identifiable {
        case color, depth, blend
        var id: String { rawValue }
        var label: String {
            switch self {
            case .color: return S.previewColor
            case .depth: return S.previewDepth
            case .blend: return S.previewBlend
            }
        }
    }

    // Camera
    var depthResolution: DepthResolution = .hd720
    var preset: DepthPreset = .highAccuracy
    var closeRange = false
    var disparityShift = 50
    var laserPowerMax = true
    var laserPower: Float = 150
    var spatialFilter = true
    var temporalFilter = true
    var holeFilling = false
    var maxDistanceM: Float = 1.2
    var autoExposureOnFace = true
    // Scan
    var keepShoulders = false
    var diagonalPoses = false
    var framesPerPose = 3
    // Reconstruction
    var voxelSizeMM: Float = 0.75
    var method: SurfaceMethod = .tsdf
    var closeHead = true
    // Display
    var mirrored = true
    var previewMode: PreviewMode = .color
    var showLandmarks = true

    static let defaultsKey = "IntelFaceScan.settings.v1"

    // MARK: Persistence

    /// Loads the stored settings; keys missing from an older version keep their defaults, unreadable data is
    /// ignored.
    static func load(from defaults: UserDefaults = .standard) -> AppSettings {
        guard let stored = defaults.data(forKey: defaultsKey),
              let storedObject = (try? JSONSerialization.jsonObject(with: stored)) as? [String: Any],
              let defaultData = try? JSONEncoder().encode(AppSettings()),
              var merged = (try? JSONSerialization.jsonObject(with: defaultData)) as? [String: Any] else {
            return AppSettings()
        }
        merged.merge(storedObject) { _, new in new }
        guard let data = try? JSONSerialization.data(withJSONObject: merged),
              let settings = try? JSONDecoder().decode(AppSettings.self, from: data) else { return AppSettings() }
        return settings.sanitized()
    }

    func save(to defaults: UserDefaults = .standard) {
        if let data = try? JSONEncoder().encode(self) { defaults.set(data, forKey: Self.defaultsKey) }
    }

    /// Values clamped to what the camera and the pipeline accept.
    func sanitized() -> AppSettings {
        var s = self
        s.disparityShift = min(max(s.disparityShift, 0), 300)
        s.laserPower = min(max(s.laserPower, 0), 360)
        s.maxDistanceM = min(max(s.maxDistanceM, 0.5), 3)
        s.framesPerPose = min(max(s.framesPerPose, 1), 10)
        s.voxelSizeMM = min(max(s.voxelSizeMM, 0.3), 3)
        return s
    }

    // MARK: Mapping

    var filterConfiguration: FilterConfiguration {
        var f = FilterConfiguration.faceScan
        f.threshold.maxM = maxDistanceM
        f.spatial.enabled = spatialFilter
        f.temporal.enabled = temporalFilter
        f.holeFilling.enabled = holeFilling
        return f
    }

    var captureConfiguration: CaptureConfiguration {
        var c = CaptureConfiguration.faceScanD455
        let size = depthResolution.size
        c.depth = StreamSettings(width: size.width, height: size.height, fps: 30)
        c.preset = preset.rawValue
        c.laserPower = laserPowerMax ? nil : laserPower
        c.disparityShift = closeRange ? disparityShift : 0
        c.filters = filterConfiguration
        return c
    }

    func scanConfiguration(realTime: Bool) -> ScanConfiguration {
        var c = ScanConfiguration()
        c.realTime = realTime
        c.poses = diagonalPoses ? ScanPose.required + ScanPose.diagonals : ScanPose.required
        c.framesPerPose = framesPerPose
        c.keepShoulders = keepShoulders
        return c
    }

    var reconstructionOptions: ReconstructionOptions {
        var o = ReconstructionOptions()
        o.voxelSizeMM = voxelSizeMM
        o.method = method
        o.keepShoulders = keepShoulders
        o.closeHead = closeHead
        return o
    }

    /// Settings whose change needs CameraSource.configure (stream profiles, preset, depth table, laser).
    func needsReconfigure(comparedTo other: AppSettings) -> Bool {
        depthResolution != other.depthResolution || preset != other.preset || closeRange != other.closeRange
            || (closeRange && disparityShift != other.disparityShift) || laserPowerMax != other.laserPowerMax
            || (!laserPowerMax && laserPower != other.laserPower)
    }

    /// Settings whose change needs only CameraSource.setFilters.
    func needsFilterUpdate(comparedTo other: AppSettings) -> Bool {
        filterConfiguration != other.filterConfiguration
    }
}
