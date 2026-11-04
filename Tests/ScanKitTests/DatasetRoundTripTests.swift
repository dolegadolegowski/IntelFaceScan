// License: Apache 2.0. See LICENSE file in root directory.
//
// ScanDataset: exact round trip of pixels and metadata, the on-disk format, error handling, and the replay source.

import CameraKit
import CoreGraphics
import Foundation
import ImageIO
@testable import ScanKit
import SyntheticData
import Testing

// MARK: - Fixtures

private func temporaryDirectory(_ name: String = #function) throws -> URL {
    let url = FileManager.default.temporaryDirectory
        .appendingPathComponent("ifs-tests-\(name.filter { $0.isLetter })-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    return url
}

private func randomData(_ count: Int, _ random: inout SplitMix64) -> Data {
    var data = Data(count: count)
    data.withUnsafeMutableBytes { raw in
        for i in 0 ..< count { raw[i] = UInt8(truncatingIfNeeded: random.next()) }
    }
    return data
}

/// Three keyframes covering every optional field: padded color rows, no mask, no IMU, distinct depth
/// intrinsics, frame numbers and timestamps that differ from the frameset's, depth values 0 and 65535.
private func makeCapture() -> ScanCapture {
    var random = SplitMix64(seed: 42)
    let w = 64, h = 40
    let intrinsics = Intrinsics(width: w, height: h, fx: 32.1, fy: 31.9, ppx: 32.4, ppy: 20.2,
                                model: "inverse_brown_conrady", coeffs: [0.1, -0.02, 0.001, 0, 0.003])
    var keyframes: [Keyframe] = []
    for i in 0 ..< 3 {
        let stride = i == 1 ? 3 * w + 7 : 3 * w
        let color = ColorImage(width: w, height: h, bytesPerRow: stride, pixels: randomData(stride * h, &random),
                               intrinsics: intrinsics, frameNumber: UInt64(1000 + i), timestampMs: 1_730_000_000_100.25 + Double(i))
        var depthPixels = randomData(2 * w * h, &random)
        depthPixels.withUnsafeMutableBytes { raw in
            let v = raw.bindMemory(to: UInt16.self)
            v[0] = 0; v[1] = 65535; v[2] = 1; v[3] = UInt16(6000).littleEndian
        }
        let depthIntrinsics = i == 2 ? Intrinsics(width: w, height: h, fx: 30, fy: 30, ppx: 31, ppy: 19) : intrinsics
        let depth = DepthImage(width: w, height: h, pixels: depthPixels, depthScale: i == 0 ? 0.0001 : 0.001,
                               intrinsics: depthIntrinsics, frameNumber: i == 0 ? 77 : UInt64(10 + i),
                               timestampMs: i == 0 ? 1_730_000_000_098.5 : 1_730_000_000_100.25 + Double(i))
        let frame = CameraFrame(
            sequence: UInt64(10 + i), timestampMs: 1_730_000_000_100.25 + Double(i), color: color, alignedDepth: depth,
            depthToColor: Extrinsics(rotation: [0.9999, 0.01, 0, -0.01, 0.9999, 0, 0, 0, 1], translation: [-0.059, 0.0001, 0.0004]),
            imu: i == 1 ? nil : IMUSample(accel: [0.1, -9.8, 0.2], gyro: i == 0 ? [0.001, 0, -0.002] : nil, timestampMs: 1_730_000_000_099),
            metadata: ["color_exposure_us": 8000, "depth_exposure_us": 8500.5, "laserPower": 360, "Mixed_Key x": -1.25])
        let mask = i == 1 ? nil : randomData(w * h, &random)
        keyframes.append(Keyframe(
            id: 5 * i + 3, frame: frame, scanPose: [ScanPose.front, .upLeft, .downRight][i],
            headPose: HeadPose(yaw: 0.4 + Float(i), pitch: -1.2, roll: 0.3, confidence: 0.98),
            faceRect: CGRect(x: 10.5, y: 4.25, width: 30.125, height: 28),
            landmarks: (0 ..< 76).map { SIMD2(Float($0) * 0.5 + 0.1, Float(i) + 0.333) }, mask: mask,
            quality: FrameQuality(sharpness: 123.456, angularSpeedDegPerSec: 4.5, validDepthRatio: 0.875,
                                  validDepthPoints: 21_000, faceDistanceM: 0.6123, cameraMotionDegPerSec: 0.25)))
    }
    var configuration = CaptureConfiguration.faceScanD455
    configuration.laserPower = 240
    return ScanCapture(keyframes: keyframes, device: SyntheticScene.deviceInfo(width: w, height: h),
                       configuration: configuration, createdAt: Date(timeIntervalSince1970: 1_793_739_600.123),
                       notes: ["source": "head-1280x720.db3", "operator note": "äöü / \"quoted\""])
}

private func rows(_ image: ColorImage) -> [Data] {
    (0 ..< image.height).map { image.pixels.subdata(in: $0 * image.bytesPerRow ..< $0 * image.bytesPerRow + 3 * image.width) }
}

private func expectEqual(_ a: ScanCapture, _ b: ScanCapture) {
    #expect(a.keyframes.count == b.keyframes.count)
    #expect(a.device == b.device)
    #expect(a.configuration == b.configuration)
    #expect(a.notes == b.notes)
    #expect(abs(a.createdAt.timeIntervalSince(b.createdAt)) < 0.0005)
    for (x, y) in zip(a.keyframes, b.keyframes) {
        #expect(x.id == y.id && x.scanPose == y.scanPose && x.headPose == y.headPose)
        #expect(x.faceRect == y.faceRect && x.landmarks == y.landmarks && x.quality == y.quality)
        #expect(x.mask == y.mask)
        let f = x.frame, g = y.frame
        #expect(f.sequence == g.sequence && f.timestampMs == g.timestampMs)
        #expect(f.depthToColor == g.depthToColor && f.imu == g.imu && f.metadata == g.metadata)
        #expect(rows(f.color) == rows(g.color))
        #expect(g.color.bytesPerRow == 3 * g.color.width)
        #expect(f.color.intrinsics == g.color.intrinsics)
        #expect(f.color.frameNumber == g.color.frameNumber && f.color.timestampMs == g.color.timestampMs)
        #expect(f.alignedDepth.pixels == g.alignedDepth.pixels)
        #expect(f.alignedDepth.depthScale == g.alignedDepth.depthScale)
        #expect(f.alignedDepth.intrinsics == g.alignedDepth.intrinsics)
        #expect(f.alignedDepth.frameNumber == g.alignedDepth.frameNumber)
        #expect(f.alignedDepth.timestampMs == g.alignedDepth.timestampMs)
    }
}

// MARK: - Round trip and format

@Test func datasetRoundTripIsExact() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let capture = makeCapture()
    let url = root.appendingPathComponent("session.ifsdataset")
    try ScanDataset.write(capture, to: url)
    expectEqual(capture, try ScanDataset.read(from: url))

    // Overwriting an existing dataset replaces it completely.
    var smaller = capture
    smaller.keyframes.removeLast()
    try ScanDataset.write(smaller, to: url)
    let reread = try ScanDataset.read(from: url)
    expectEqual(smaller, reread)
    #expect(!FileManager.default.fileExists(atPath: url.appendingPathComponent("color/000002.png").path))
    let leftovers = try FileManager.default.contentsOfDirectory(atPath: root.path).filter { $0.contains(".tmp-") }
    #expect(leftovers.isEmpty)
}

@Test func datasetRoundTripOfASyntheticScan() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    var options = SyntheticScan.Options()
    options.width = 320
    options.height = 200
    options.framesPerPose = 1
    options.meshResolution = 48
    let capture = SyntheticScan.make(options).capture
    let url = root.appendingPathComponent("synthetic.ifsdataset")
    try ScanDataset.write(capture, to: url)
    expectEqual(capture, try ScanDataset.read(from: url))
}

@Test func datasetManifestUsesTheDocumentedLayout() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let url = root.appendingPathComponent("layout.ifsdataset")
    try ScanDataset.write(makeCapture(), to: url)

    let json = try #require(try JSONSerialization.jsonObject(with: Data(contentsOf: url.appendingPathComponent("manifest.json"))) as? [String: Any])
    #expect(json["format"] as? String == "ifsdataset")
    #expect(json["version"] as? Int == 1)
    #expect((json["created_at"] as? String)?.hasPrefix("2026-11-03T21:00:00.123") == true)
    #expect((json["device"] as? [String: Any])?["product_line"] as? String == "D400")
    #expect((json["configuration"] as? [String: Any])?["depth_units"] != nil)
    #expect((json["notes"] as? [String: String])?["source"] == "head-1280x720.db3")
    let frames = try #require(json["frames"] as? [[String: Any]])
    #expect(frames.count == 3)
    let f = frames[0]
    for key in ["id", "sequence", "timestamp_ms", "scan_pose", "color", "depth", "mask", "depth_scale", "intrinsics",
                "depth_to_color", "head_pose", "face_rect", "landmarks", "quality", "metadata"] {
        #expect(f[key] != nil, "missing \(key)")
    }
    #expect(f["color"] as? String == "color/000000.png")
    #expect(f["depth"] as? String == "depth/000000.png")
    #expect(f["mask"] as? String == "mask/000000.png")
    #expect(frames[1]["mask"] == nil)
    #expect(frames[1]["scan_pose"] as? String == "upLeft")
    #expect((f["quality"] as? [String: Any])?["valid_depth_ratio"] as? Double == 0.875)
    #expect((f["head_pose"] as? [String: Any])?["confidence"] != nil)
    // metadata keys are stored verbatim
    #expect((f["metadata"] as? [String: Double])?["color_exposure_us"] == 8000)
    #expect((f["metadata"] as? [String: Double])?["laserPower"] == 360)

    // Depth is a real 16-bit grayscale PNG (readable by OpenCV/MeshLab tools), color 8-bit RGB, mask 8-bit gray.
    func properties(_ path: String) throws -> [CFString: Any] {
        let source = try #require(CGImageSourceCreateWithURL(url.appendingPathComponent(path) as CFURL, nil))
        return try #require(CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any])
    }
    let depth = try properties("depth/000000.png")
    #expect(depth[kCGImagePropertyDepth] as? Int == 16)
    #expect(depth[kCGImagePropertyColorModel] as? String == (kCGImagePropertyColorModelGray as String))
    let color = try properties("color/000000.png")
    #expect(color[kCGImagePropertyDepth] as? Int == 8)
    #expect(color[kCGImagePropertyColorModel] as? String == (kCGImagePropertyColorModelRGB as String))
    #expect(color[kCGImagePropertyHasAlpha] as? Bool != true)
    let mask = try properties("mask/000000.png")
    #expect(mask[kCGImagePropertyColorModel] as? String == (kCGImagePropertyColorModelGray as String))
}

@Test func pngCodecsPassValuesThroughBitExactly() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    // Every 16-bit value once (256 x 256), including both bytes' extremes.
    var values = Data(count: 65536 * 2)
    values.withUnsafeMutableBytes { raw in
        let v = raw.bindMemory(to: UInt16.self)
        for i in 0 ..< 65536 { v[i] = UInt16(i).littleEndian }
    }
    let depthURL = root.appendingPathComponent("depth.png")
    try PNGImageIO.writeGray16(values, width: 256, height: 256, to: depthURL)
    #expect(try PNGImageIO.readGray16(from: depthURL).pixels == values)

    var gray = Data(count: 256 * 3)
    for i in 0 ..< gray.count { gray[i] = UInt8(i % 256) }
    let grayURL = root.appendingPathComponent("gray.png")
    try PNGImageIO.writeGray8(gray, width: 256, height: 3, to: grayURL)
    #expect(try PNGImageIO.readGray8(from: grayURL).pixels == gray)
    // 8-bit images are not depth: refusing them avoids silently reinterpreting values.
    #expect(throws: PNGImageIOError.self) { try PNGImageIO.readGray16(from: grayURL) }

    var rgb = Data(count: 256 * 256 * 3)
    for i in 0 ..< rgb.count { rgb[i] = UInt8(truncatingIfNeeded: i * 7 + i / 256) }
    let rgbURL = root.appendingPathComponent("rgb.png")
    try PNGImageIO.writeRGB8(rgb, width: 256, height: 256, to: rgbURL)
    #expect(try PNGImageIO.readRGB8(from: rgbURL).pixels == rgb)
    #expect(throws: PNGImageIOError.self) { try PNGImageIO.writeRGB8(rgb, width: 300, height: 300, to: rgbURL) }
}

// MARK: - Errors

@Test func datasetReadRejectsBrokenDatasets() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let url = root.appendingPathComponent("broken.ifsdataset")
    #expect(throws: ScanDatasetError.self) { try ScanDataset.read(from: url) }   // no folder

    try ScanDataset.write(makeCapture(), to: url)
    let manifestURL = url.appendingPathComponent("manifest.json")
    let original = try String(contentsOf: manifestURL, encoding: .utf8)
    func expectFailure(_ manifest: String, _ check: (ScanDatasetError) -> Bool) throws {
        try manifest.write(to: manifestURL, atomically: true, encoding: .utf8)
        do {
            _ = try ScanDataset.read(from: url)
            Issue.record("read succeeded")
        } catch let error as ScanDatasetError {
            #expect(check(error), "\(error)")
        }
    }
    try expectFailure(original.replacingOccurrences(of: "\"version\" : 1", with: "\"version\" : 2")) {
        $0 == .unsupportedVersion(2)
    }
    try expectFailure(original.replacingOccurrences(of: "\"format\" : \"ifsdataset\"", with: "\"format\" : \"other\"")) {
        if case .invalidManifest = $0 { return true } else { return false }
    }
    try expectFailure(original.replacingOccurrences(of: "\"color/000001.png\"", with: "\"../../etc/passwd\"")) {
        $0 == .unsafePath("../../etc/passwd")
    }
    try expectFailure(original.replacingOccurrences(of: "\"color/000001.png\"", with: "\"/etc/hosts\"")) {
        $0 == .unsafePath("/etc/hosts")
    }
    try expectFailure(original.replacingOccurrences(of: "\"scan_pose\" : \"upLeft\"", with: "\"scan_pose\" : \"sideways\"")) {
        if case .invalidManifest = $0 { return true } else { return false }
    }
    try expectFailure("{\"format\": \"ifsdataset\"}") {
        if case .invalidManifest(let message) = $0 { return message.contains("version") } else { return false }
    }
    try original.write(to: manifestURL, atomically: true, encoding: .utf8)
    try FileManager.default.removeItem(at: url.appendingPathComponent("depth/000001.png"))
    #expect(throws: PNGImageIOError.self) { try ScanDataset.read(from: url) }
}

@Test func datasetReadRefusesSymlinksOutOfTheDataset() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let fm = FileManager.default
    let url = root.appendingPathComponent("linked.ifsdataset")
    try ScanDataset.write(makeCapture(), to: url)
    // The manifest only names "color/000000.png", but "color" now points at a folder outside the dataset.
    func expectRefused() {
        do {
            _ = try ScanDataset.read(from: url)
            Issue.record("a dataset with links out of its folder was read")
        } catch {
            // Frames are read in parallel: any of the linked color files may be reported first.
            guard case .unsafePath(let path)? = error as? ScanDatasetError else {
                Issue.record("unexpected error \(error)")
                return
            }
            #expect(path.hasPrefix("color/0000"), "\(path)")
        }
    }
    let outside = root.appendingPathComponent("outside")
    try fm.moveItem(at: url.appendingPathComponent("color"), to: outside)
    try fm.createSymbolicLink(at: url.appendingPathComponent("color"), withDestinationURL: outside)
    expectRefused()

    // A link to a single file outside is refused as well.
    try fm.removeItem(at: url.appendingPathComponent("color"))
    try fm.createDirectory(at: url.appendingPathComponent("color"), withIntermediateDirectories: false)
    for name in try fm.contentsOfDirectory(atPath: outside.path) {
        try fm.createSymbolicLink(at: url.appendingPathComponent("color/\(name)"),
                                  withDestinationURL: outside.appendingPathComponent(name))
    }
    expectRefused()

    // Links that stay inside the dataset, and a dataset reached through a linked parent folder, still work.
    for name in try fm.contentsOfDirectory(atPath: outside.path) {
        try fm.removeItem(at: url.appendingPathComponent("color/\(name)"))
        try fm.moveItem(at: outside.appendingPathComponent(name), to: url.appendingPathComponent("color/\(name)"))
    }
    try fm.moveItem(at: url.appendingPathComponent("color/000002.png"), to: url.appendingPathComponent("frame2.png"))
    try fm.createSymbolicLink(atPath: url.appendingPathComponent("color/000002.png").path, withDestinationPath: "../frame2.png")
    let alias = root.appendingPathComponent("alias")
    try fm.createSymbolicLink(at: alias, withDestinationURL: root)
    expectEqual(makeCapture(), try ScanDataset.read(from: alias.appendingPathComponent("linked.ifsdataset")))
}

@Test func datasetWriteRefusesToReplaceOtherFolders() throws {
    let root = try temporaryDirectory()
    defer { try? FileManager.default.removeItem(at: root) }
    let precious = root.appendingPathComponent("Documents")
    try FileManager.default.createDirectory(at: precious, withIntermediateDirectories: true)
    try "keep me".write(to: precious.appendingPathComponent("thesis.txt"), atomically: true, encoding: .utf8)
    #expect(throws: ScanDatasetError.destinationExists(precious.standardizedFileURL.path)) {
        try ScanDataset.write(makeCapture(), to: precious)
    }
    #expect(FileManager.default.fileExists(atPath: precious.appendingPathComponent("thesis.txt").path))

    var bad = makeCapture()
    bad.keyframes[0].mask = Data(count: 5)
    #expect(throws: ScanDatasetError.self) { try ScanDataset.write(bad, to: root.appendingPathComponent("bad.ifsdataset")) }
    #expect(!FileManager.default.fileExists(atPath: root.appendingPathComponent("bad.ifsdataset").path))
}

// MARK: - Replay

private func writeTimedDataset(spacingMs: Double) throws -> (URL, ScanCapture) {
    var capture = makeCapture()
    for i in capture.keyframes.indices {
        capture.keyframes[i].frame.timestampMs = 1_000 + spacingMs * Double(i)
    }
    let url = try temporaryDirectory("replay").appendingPathComponent("timed.ifsdataset")
    try ScanDataset.write(capture, to: url)
    return (url, capture)
}

@Test func datasetSourceReplaysEveryFrameInOrderOffline() async throws {
    let (url, capture) = try writeTimedDataset(spacingMs: 5_000)
    defer { try? FileManager.default.removeItem(at: url.deletingLastPathComponent()) }
    let source = DatasetCameraSource(directory: url, realTime: false)
    let events = source.events()
    let device = try await source.connect()
    #expect(device == capture.device)
    #expect(source.kind == .dataset(url))
    let frames = source.frames(bufferingNewest: 1)
    try await source.start()
    let start = Date()
    var sequences: [UInt64] = []
    for await frame in frames { sequences.append(frame.sequence) }
    #expect(sequences == capture.keyframes.map(\.frame.sequence))
    #expect(Date().timeIntervalSince(start) < 2)   // offline ignores the 5 s gaps
    await source.shutdown()
    var sawFinished = false
    for await event in events {
        if case .playbackFinished = event { sawFinished = true }
    }
    #expect(sawFinished)
}

@Test func datasetSourceRespectsTimestampsInRealTime() async throws {
    let (url, capture) = try writeTimedDataset(spacingMs: 150)
    defer { try? FileManager.default.removeItem(at: url.deletingLastPathComponent()) }
    let source = DatasetCameraSource(directory: url, realTime: true)
    _ = try await source.connect()
    let frames = source.frames(bufferingNewest: 4)
    let startedAt = ProcessInfo.processInfo.systemUptime
    try await source.start()
    var arrivals: [Double] = []
    var count = 0
    for await frame in frames {
        #expect(frame.sequence == capture.keyframes[count].frame.sequence)
        arrivals.append(ProcessInfo.processInfo.systemUptime)
        count += 1
    }
    #expect(count == 3)
    // A paced source never delivers frame k before k * 150 ms. Pairwise gaps are not checked: on a loaded machine
    // (the reconstruction suites run in parallel) the consumer may wake late for one frame and early for the next.
    #expect(arrivals[2] - startedAt > 0.27, "the last frame arrived \(arrivals[2] - startedAt) s after start")
    #expect(arrivals[2] - startedAt < 3.0)

    // Restarting replays from the first frame; looping continues past the end.
    await source.shutdown()
    let looping = DatasetCameraSource(directory: url, realTime: false, loop: true)
    _ = try await looping.connect()
    try await looping.start()
    var looped: [UInt64] = []
    for await frame in looping.frames(bufferingNewest: 1) {
        looped.append(frame.sequence)
        if looped.count == 7 { break }
    }
    #expect(looped == [10, 11, 12, 10, 11, 12, 10])
    try await looping.stop()
    try await looping.start()
    var restarted: UInt64?
    for await frame in looping.frames(bufferingNewest: 1) {
        restarted = frame.sequence
        break
    }
    #expect(restarted == 10)
    await looping.shutdown()
    var afterShutdown = 0
    for await _ in looping.frames(bufferingNewest: 1) { afterShutdown += 1 }
    #expect(afterShutdown == 0)
}

@Test func datasetSourceNeedsAConnection() async throws {
    let source = DatasetCameraSource(directory: URL(fileURLWithPath: "/nonexistent/x.ifsdataset"))
    await #expect(throws: CameraError.notConnected) { try await source.start() }
    await #expect(throws: ScanDatasetError.self) { _ = try await source.connect() }
}
