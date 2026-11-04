// License: Apache 2.0. See LICENSE file in root directory.
//
// Saving and loading a ScanCapture as a dataset folder (format: docs/ARCHITECTURE.md "ScanDataset"):
//
//   <name>.ifsdataset/manifest.json        snake_case JSON, see Manifest
//   <name>.ifsdataset/color/000000.png     8-bit RGB
//   <name>.ifsdataset/depth/000000.png     16-bit grayscale, aligned to color, units = frames[i].depth_scale meters
//   <name>.ifsdataset/mask/000000.png      8-bit head mask (when the keyframe has one)
//
// Pixels and metadata round-trip exactly (created_at to the millisecond). Beyond the documented keys, frames may
// carry "imu", "depth_intrinsics" (only when the aligned depth reports other intrinsics than color) and the color /
// depth frame numbers and timestamps; readers ignore keys and files they do not know (synthetic datasets add
// ground_truth.json and ground_truth.obj, see docs/TESTING.md).

import CameraKit
import CoreGraphics
import Dispatch
import Foundation

public enum ScanDatasetError: Error, LocalizedError, Equatable {
    case notADataset(String)
    case unsupportedVersion(Int)
    case invalidManifest(String)
    case invalidCapture(String)
    case unsafePath(String)
    case destinationExists(String)

    public var errorDescription: String? {
        switch self {
        case .notADataset(let m): return "Not an IntelFaceScan dataset: \(m)"
        case .unsupportedVersion(let v): return "Unsupported dataset version \(v) (this build reads version \(ScanDataset.formatVersion))."
        case .invalidManifest(let m): return "Invalid dataset manifest: \(m)"
        case .invalidCapture(let m): return "Cannot save the scan: \(m)"
        case .unsafePath(let p): return "Dataset file path leaves the dataset folder: \(p)"
        case .destinationExists(let p): return "\(p) exists and is not a dataset; refusing to replace it."
        }
    }
}

public enum ScanDataset {
    public static let fileExtension = "ifsdataset"
    public static let formatName = "ifsdataset"
    public static let formatVersion = 1
    public static let manifestFileName = "manifest.json"

    // MARK: Manifest

    /// manifest.json. Property names are converted to snake_case on disk; dictionary keys (notes, metadata) are
    /// stored verbatim.
    public struct Manifest: Codable, Sendable {
        public var format: String
        public var version: Int
        /// ISO 8601 with milliseconds, UTC.
        public var createdAt: String
        public var device: DeviceInfo?
        public var configuration: CaptureConfiguration?
        public var notes: [String: String]
        public var frames: [Frame]
    }

    public struct Frame: Codable, Sendable {
        public var id: Int
        public var sequence: UInt64
        public var timestampMs: Double
        /// ScanPose raw value.
        public var scanPose: String
        /// Paths relative to the dataset folder.
        public var color: String
        public var depth: String
        public var mask: String?
        public var depthScale: Float
        /// Color intrinsics (= aligned depth intrinsics).
        public var intrinsics: Intrinsics
        public var depthIntrinsics: Intrinsics?
        public var depthToColor: Extrinsics
        public var headPose: HeadPose
        /// [x, y, width, height] in color pixels, top-left origin.
        public var faceRect: [Double]
        /// [[x, y], ...] in color pixels.
        public var landmarks: [[Float]]
        public var quality: FrameQuality
        public var metadata: [String: Double]
        public var imu: IMUSample?
        public var colorFrameNumber: UInt64?
        public var colorTimestampMs: Double?
        public var depthFrameNumber: UInt64?
        public var depthTimestampMs: Double?
    }

    // MARK: Writing

    /// Writes the capture as a dataset folder. An existing dataset (or empty folder) at `directory` is replaced; the
    /// new dataset is assembled next to it first, so a failure never leaves a half-written dataset behind.
    public static func write(_ capture: ScanCapture, to directory: URL) throws {
        let fm = FileManager.default
        let destination = directory.standardizedFileURL
        var isDirectory: ObjCBool = false
        let exists = fm.fileExists(atPath: destination.path, isDirectory: &isDirectory)
        if exists {
            let contents = (try? fm.contentsOfDirectory(atPath: destination.path)) ?? []
            guard isDirectory.boolValue,
                  contents.isEmpty || contents.contains(manifestFileName) else {
                throw ScanDatasetError.destinationExists(destination.path)
            }
        }
        let frames = try validate(capture)

        let parent = destination.deletingLastPathComponent()
        try fm.createDirectory(at: parent, withIntermediateDirectories: true)
        let staging = parent.appendingPathComponent(".\(destination.lastPathComponent).tmp-\(UUID().uuidString)")
        do {
            for folder in ["color", "depth", "mask"] {
                try fm.createDirectory(at: staging.appendingPathComponent(folder), withIntermediateDirectories: true)
            }
            try writeImages(capture.keyframes, frames: frames, to: staging)
            let manifest = Manifest(format: formatName, version: formatVersion, createdAt: formatDate(capture.createdAt),
                                    device: capture.device, configuration: capture.configuration,
                                    notes: capture.notes, frames: frames)
            let encoder = JSONEncoder()
            encoder.keyEncodingStrategy = .convertToSnakeCase
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes]
            encoder.nonConformingFloatEncodingStrategy = .throw
            try encoder.encode(manifest).write(to: staging.appendingPathComponent(manifestFileName))
            if exists {
                _ = try fm.replaceItemAt(destination, withItemAt: staging)
            } else {
                try fm.moveItem(at: staging, to: destination)
            }
        } catch {
            try? fm.removeItem(at: staging)
            if error is ScanDatasetError || error is PNGImageIOError { throw error }
            throw ScanDatasetError.invalidCapture("\(destination.path): \(error.localizedDescription)")
        }
    }

    private static func validate(_ capture: ScanCapture) throws -> [Frame] {
        var frames: [Frame] = []
        frames.reserveCapacity(capture.keyframes.count)
        for (i, kf) in capture.keyframes.enumerated() {
            let color = kf.frame.color, depth = kf.frame.alignedDepth
            let w = color.width, h = color.height
            guard w > 0, h > 0, color.bytesPerRow >= 3 * w, color.pixels.count >= color.bytesPerRow * (h - 1) + 3 * w else {
                throw ScanDatasetError.invalidCapture("keyframe \(i): color image \(w)x\(h) with \(color.pixels.count) bytes")
            }
            guard depth.width == w, depth.height == h, depth.pixels.count == 2 * w * h else {
                throw ScanDatasetError.invalidCapture("keyframe \(i): aligned depth must be \(w)x\(h) UInt16")
            }
            guard depth.depthScale > 0, depth.depthScale.isFinite else {
                throw ScanDatasetError.invalidCapture("keyframe \(i): depth scale \(depth.depthScale)")
            }
            if let mask = kf.mask, mask.count != w * h {
                throw ScanDatasetError.invalidCapture("keyframe \(i): mask has \(mask.count) bytes, expected \(w * h)")
            }
            let name = String(format: "%06d.png", i)
            let rect = kf.faceRect
            frames.append(Frame(
                id: kf.id, sequence: kf.frame.sequence, timestampMs: kf.frame.timestampMs,
                scanPose: kf.scanPose.rawValue, color: "color/\(name)", depth: "depth/\(name)",
                mask: kf.mask == nil ? nil : "mask/\(name)", depthScale: depth.depthScale,
                intrinsics: color.intrinsics, depthIntrinsics: depth.intrinsics == color.intrinsics ? nil : depth.intrinsics,
                depthToColor: kf.frame.depthToColor, headPose: kf.headPose,
                faceRect: [Double(rect.origin.x), Double(rect.origin.y), Double(rect.size.width), Double(rect.size.height)],
                landmarks: kf.landmarks.map { [$0.x, $0.y] }, quality: kf.quality, metadata: kf.frame.metadata,
                imu: kf.frame.imu,
                colorFrameNumber: color.frameNumber == kf.frame.sequence ? nil : color.frameNumber,
                colorTimestampMs: color.timestampMs == kf.frame.timestampMs ? nil : color.timestampMs,
                depthFrameNumber: depth.frameNumber == kf.frame.sequence ? nil : depth.frameNumber,
                depthTimestampMs: depth.timestampMs == kf.frame.timestampMs ? nil : depth.timestampMs))
        }
        return frames
    }

    private static func writeImages(_ keyframes: [Keyframe], frames: [Frame], to folder: URL) throws {
        let errors = ErrorBox()
        DispatchQueue.concurrentPerform(iterations: keyframes.count) { i in
            let kf = keyframes[i], frame = frames[i]
            let color = kf.frame.color
            do {
                try PNGImageIO.writeRGB8(color.pixels, width: color.width, height: color.height,
                                         bytesPerRow: color.bytesPerRow, to: folder.appendingPathComponent(frame.color))
                try PNGImageIO.writeGray16(kf.frame.alignedDepth.pixels, width: color.width, height: color.height,
                                           to: folder.appendingPathComponent(frame.depth))
                if let mask = kf.mask, let path = frame.mask {
                    try PNGImageIO.writeGray8(mask, width: color.width, height: color.height,
                                              to: folder.appendingPathComponent(path))
                }
            } catch {
                errors.record(error)
            }
        }
        if let error = errors.first { throw error }
    }

    // MARK: Reading

    /// Reads only manifest.json (fast; for listing and `ifscan info`).
    public static func readManifest(from directory: URL) throws -> Manifest {
        let url = directory.appendingPathComponent(manifestFileName)
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw ScanDatasetError.notADataset("\(directory.path) has no \(manifestFileName)")
        }
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw ScanDatasetError.notADataset("\(url.path): \(error.localizedDescription)")
        }
        let manifest: Manifest
        do {
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            manifest = try decoder.decode(Manifest.self, from: data)
        } catch let DecodingError.keyNotFound(key, context) {
            throw ScanDatasetError.invalidManifest("missing \"\(key.stringValue)\" at \(path(context.codingPath))")
        } catch let DecodingError.typeMismatch(_, context), let DecodingError.valueNotFound(_, context),
                let DecodingError.dataCorrupted(context) {
            throw ScanDatasetError.invalidManifest("\(context.debugDescription) at \(path(context.codingPath))")
        } catch {
            throw ScanDatasetError.invalidManifest(error.localizedDescription)
        }
        guard manifest.format == formatName else {
            throw ScanDatasetError.invalidManifest("format is \"\(manifest.format)\", expected \"\(formatName)\"")
        }
        guard manifest.version >= 1 else { throw ScanDatasetError.invalidManifest("version \(manifest.version)") }
        guard manifest.version <= formatVersion else { throw ScanDatasetError.unsupportedVersion(manifest.version) }
        return manifest
    }

    public static func read(from directory: URL) throws -> ScanCapture {
        let root = directory.standardizedFileURL
        let manifest = try readManifest(from: root)
        guard let createdAt = parseDate(manifest.createdAt) else {
            throw ScanDatasetError.invalidManifest("created_at \"\(manifest.createdAt)\" is not an ISO 8601 date")
        }

        let results = ResultBox<Keyframe>(count: manifest.frames.count)
        DispatchQueue.concurrentPerform(iterations: manifest.frames.count) { i in
            do {
                results.set(i, try keyframe(manifest.frames[i], index: i, root: root))
            } catch {
                results.fail(error)
            }
        }
        if let error = results.error { throw error }
        return ScanCapture(keyframes: results.values.compactMap { $0 }, device: manifest.device,
                           configuration: manifest.configuration, createdAt: createdAt, notes: manifest.notes)
    }

    private static func keyframe(_ f: Frame, index i: Int, root: URL) throws -> Keyframe {
        guard let scanPose = ScanPose(rawValue: f.scanPose) else {
            throw ScanDatasetError.invalidManifest("frames[\(i)]: unknown scan_pose \"\(f.scanPose)\"")
        }
        guard f.faceRect.count == 4, f.landmarks.allSatisfy({ $0.count == 2 }) else {
            throw ScanDatasetError.invalidManifest("frames[\(i)]: face_rect needs 4 numbers and landmarks [x, y] pairs")
        }
        guard f.depthScale > 0, f.depthScale.isFinite else {
            throw ScanDatasetError.invalidManifest("frames[\(i)]: depth_scale \(f.depthScale)")
        }
        let color = try PNGImageIO.readRGB8(from: resolve(f.color, in: root))
        let depth = try PNGImageIO.readGray16(from: resolve(f.depth, in: root))
        let w = color.width, h = color.height
        guard depth.width == w, depth.height == h else {
            throw ScanDatasetError.invalidManifest("frames[\(i)]: depth is \(depth.width)x\(depth.height), color \(w)x\(h)")
        }
        guard f.intrinsics.width == w, f.intrinsics.height == h else {
            throw ScanDatasetError.invalidManifest("frames[\(i)]: intrinsics are for \(f.intrinsics.width)x\(f.intrinsics.height), image is \(w)x\(h)")
        }
        var mask: Data?
        if let path = f.mask {
            let m = try PNGImageIO.readGray8(from: resolve(path, in: root))
            guard m.width == w, m.height == h else {
                throw ScanDatasetError.invalidManifest("frames[\(i)]: mask is \(m.width)x\(m.height), color \(w)x\(h)")
            }
            mask = m.pixels
        }
        let colorImage = ColorImage(width: w, height: h, bytesPerRow: 3 * w, pixels: color.pixels, intrinsics: f.intrinsics,
                                    frameNumber: f.colorFrameNumber ?? f.sequence,
                                    timestampMs: f.colorTimestampMs ?? f.timestampMs)
        let depthImage = DepthImage(width: w, height: h, pixels: depth.pixels, depthScale: f.depthScale,
                                    intrinsics: f.depthIntrinsics ?? f.intrinsics,
                                    frameNumber: f.depthFrameNumber ?? f.sequence,
                                    timestampMs: f.depthTimestampMs ?? f.timestampMs)
        let frame = CameraFrame(sequence: f.sequence, timestampMs: f.timestampMs, color: colorImage, alignedDepth: depthImage,
                                depthToColor: f.depthToColor, imu: f.imu, metadata: f.metadata)
        let rect = CGRect(x: f.faceRect[0], y: f.faceRect[1], width: f.faceRect[2], height: f.faceRect[3])
        return Keyframe(id: f.id, frame: frame, scanPose: scanPose, headPose: f.headPose, faceRect: rect,
                        landmarks: f.landmarks.map { SIMD2($0[0], $0[1]) }, mask: mask, quality: f.quality)
    }

    /// Resolves a manifest path; only plain relative paths inside the dataset folder are accepted, so a crafted
    /// manifest cannot make the reader open arbitrary files.
    static func resolve(_ relative: String, in root: URL) throws -> URL {
        let components = relative.split(separator: "/", omittingEmptySubsequences: false)
        guard !relative.isEmpty, !relative.hasPrefix("/"), !relative.contains("\\"),
              components.allSatisfy({ !$0.isEmpty && $0 != "." && $0 != ".." }) else {
            throw ScanDatasetError.unsafePath(relative)
        }
        let url = root.appendingPathComponent(relative).standardizedFileURL
        guard url.path.hasPrefix(root.path + "/") else { throw ScanDatasetError.unsafePath(relative) }
        // Symbolic links inside the dataset (a linked "color" folder or file) must not lead out of it either. A file
        // that does not exist is left to the reader, which reports it as missing.
        if let real = realPath(url.path), let realRoot = realPath(root.path), !real.hasPrefix(realRoot + "/") {
            throw ScanDatasetError.unsafePath(relative)
        }
        return url
    }

    /// realpath(3): the absolute path with every symbolic link resolved, nil when it does not exist.
    private static func realPath(_ path: String) -> String? {
        guard let resolved = realpath(path, nil) else { return nil }
        defer { free(resolved) }
        return String(cString: resolved)
    }

    // MARK: Dates

    static func formatDate(_ date: Date) -> String {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        // Round to the millisecond explicitly; the formatter truncates.
        let rounded = Date(timeIntervalSinceReferenceDate: (date.timeIntervalSinceReferenceDate * 1000).rounded() / 1000)
        return formatter.string(from: rounded)
    }

    static func parseDate(_ string: String) -> Date? {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        if let date = formatter.date(from: string) { return date }
        formatter.formatOptions = [.withInternetDateTime]
        return formatter.date(from: string)
    }

    private static func path(_ codingPath: [CodingKey]) -> String {
        codingPath.map { $0.intValue.map { "[\($0)]" } ?? ".\($0.stringValue)" }.joined()
    }
}

// MARK: - Thread-safe collection of parallel results

final class ErrorBox: @unchecked Sendable {
    private let lock = NSLock()
    private var errors: [Error] = []
    func record(_ error: Error) { lock.lock(); errors.append(error); lock.unlock() }
    var first: Error? { lock.lock(); defer { lock.unlock() }; return errors.first }
}

final class ResultBox<T>: @unchecked Sendable {
    private let lock = NSLock()
    private var storage: [T?]
    private var firstError: Error?
    init(count: Int) { storage = Array(repeating: nil, count: count) }
    func set(_ i: Int, _ value: T) { lock.lock(); storage[i] = value; lock.unlock() }
    func fail(_ error: Error) { lock.lock(); if firstError == nil { firstError = error }; lock.unlock() }
    var values: [T?] { lock.lock(); defer { lock.unlock() }; return storage }
    var error: Error? { lock.lock(); defer { lock.unlock() }; return firstError }
}
