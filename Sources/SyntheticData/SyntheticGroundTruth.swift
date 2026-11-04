// License: Apache 2.0. See LICENSE file in root directory.
//
// Ground truth stored next to a synthetic dataset (ignored by ScanDataset readers):
//   <name>.ifsdataset/ground_truth.json   true world_from_camera and head pose per frame, scene parameters
//   <name>.ifsdataset/ground_truth.obj    true head surface, millimeters, world frame
// so that `ifscan reconstruct` can report pose and surface errors for any synthetic dataset.

import Foundation
import ReconKit
import ScanKit
import simd

public struct SyntheticGroundTruth: Codable, Sendable {
    public static let fileName = "ground_truth.json"
    public static let meshFileName = "ground_truth.obj"
    public static let meshComments = [
        "IntelFaceScan synthetic ground truth",
        "units: millimeters; frame: camera frame of keyframe 0 (+X right, +Y down, +Z forward)",
    ]

    public struct Frame: Codable, Sendable {
        public var id: Int
        /// Column-major 4x4, meters.
        public var worldFromCamera: [Float]
        /// True head orientation (the dataset's head_pose is the simulated Vision estimate).
        public var headPose: HeadPose
    }

    public struct Scene: Codable, Sendable {
        public var width: Int
        public var height: Int
        public var distanceM: Float
        public var depthNoiseAtOneMeterM: Float
        public var seed: UInt64
        public var framesPerPose: Int
        public var poses: [String]
        public var backgroundDistanceM: Float?
        public var meshResolution: Int
    }

    public var format = "ifsdataset-ground-truth"
    public var version = 1
    public var frame = "world = camera frame of keyframe 0 (+X right, +Y down, +Z forward), meters"
    public var mesh = SyntheticGroundTruth.meshFileName
    public var meshUnits = "millimeters"
    public var headCenter: [Float]
    public var pivot: [Float]
    public var frames: [Frame]
    public var scene: Scene

    public init(result: SyntheticScanResult, options: SyntheticScan.Options) {
        headCenter = [result.headCenter.x, result.headCenter.y, result.headCenter.z]
        pivot = [result.pivot.x, result.pivot.y, result.pivot.z]
        frames = result.capture.keyframes.indices.map { i in
            Frame(id: result.capture.keyframes[i].id, worldFromCamera: result.groundTruthPoses[i].columnMajor,
                         headPose: i < result.groundTruthHeadPoses.count ? result.groundTruthHeadPoses[i] : .frontal)
        }
        scene = Scene(width: options.width, height: options.height, distanceM: options.distanceM,
                      depthNoiseAtOneMeterM: options.depthNoiseAt1m, seed: options.seed, framesPerPose: options.framesPerPose,
                      poses: options.poses.map(\.rawValue), backgroundDistanceM: options.backgroundDistanceM,
                      meshResolution: options.meshResolution)
    }

    public var worldFromCamera: [simd_float4x4] {
        frames.map { simd_float4x4(columnMajor: $0.worldFromCamera) ?? matrix_identity_float4x4 }
    }

    /// Writes ground_truth.json and ground_truth.obj into an existing dataset folder.
    public func write(toDataset directory: URL, mesh groundTruthMesh: ReconMesh) throws {
        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        try encoder.encode(self).write(to: directory.appendingPathComponent(Self.fileName), options: .atomic)
        try WavefrontOBJ.write(groundTruthMesh, to: directory.appendingPathComponent(mesh), scale: 1000,
                               comments: Self.meshComments)
    }

    /// The ground truth of a dataset folder, nil when it has none (not synthetic).
    public static func load(fromDataset directory: URL) throws -> SyntheticGroundTruth? {
        let url = directory.appendingPathComponent(fileName)
        guard FileManager.default.fileExists(atPath: url.path) else { return nil }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let truth = try decoder.decode(SyntheticGroundTruth.self, from: Data(contentsOf: url))
        guard truth.format == "ifsdataset-ground-truth", truth.version == 1 else {
            throw ScanDatasetError.invalidManifest("\(fileName): unsupported format \(truth.format) v\(truth.version)")
        }
        return truth
    }

    /// The ground-truth mesh of a dataset in meters (world frame), nil when absent.
    public func loadMesh(fromDataset directory: URL) throws -> ReconMesh? {
        guard let path = try? ScanDatasetPath.resolve(mesh, in: directory),
              FileManager.default.fileExists(atPath: path.path) else { return nil }
        var mesh = try WavefrontOBJ.read(path)
        let scale: Float = meshUnits == "millimeters" ? 0.001 : 1
        mesh.positions = mesh.positions.map { $0 * scale }
        return mesh
    }
}

/// Plain relative paths inside a dataset folder only (mirrors ScanDataset's manifest path rules).
enum ScanDatasetPath {
    static func resolve(_ relative: String, in root: URL) throws -> URL {
        let parts = relative.split(separator: "/", omittingEmptySubsequences: false)
        guard !relative.isEmpty, !relative.hasPrefix("/"),
              parts.allSatisfy({ !$0.isEmpty && $0 != "." && $0 != ".." }) else {
            throw ScanDatasetError.unsafePath(relative)
        }
        return root.appendingPathComponent(relative)
    }
}

extension simd_float4x4 {
    /// Column-major floats (the C API's float[16] layout).
    var columnMajor: [Float] {
        [columns.0.x, columns.0.y, columns.0.z, columns.0.w, columns.1.x, columns.1.y, columns.1.z, columns.1.w,
         columns.2.x, columns.2.y, columns.2.z, columns.2.w, columns.3.x, columns.3.y, columns.3.z, columns.3.w]
    }

    init?(columnMajor m: [Float]) {
        guard m.count == 16, m.allSatisfy({ $0.isFinite }) else { return nil }
        self.init(columns: (SIMD4(m[0], m[1], m[2], m[3]), SIMD4(m[4], m[5], m[6], m[7]),
                            SIMD4(m[8], m[9], m[10], m[11]), SIMD4(m[12], m[13], m[14], m[15])))
    }
}
