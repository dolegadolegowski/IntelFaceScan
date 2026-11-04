// License: Apache 2.0. See LICENSE file in root directory.
//
// Debug builds only: renders the main screens with fixture data to PNG files (`--render-snapshots <dir>`), the
// Command Line Tools replacement for SwiftUI previews. The Metal preview is an AppKit view and stays empty in
// these images.

#if DEBUG
import AppKit
import CameraKit
import ImageIO
import ReconKit
import ScanKit
import simd
import SwiftUI
import UniformTypeIdentifiers

@MainActor
enum DeveloperSnapshots {
    /// Renders every screen into `directory`; returns the process exit status.
    static func render(to directory: URL) -> Int32 {
        do {
            try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        } catch {
            FileHandle.standardError.write(Data("cannot create \(directory.path): \(error)\n".utf8))
            return 1
        }
        let fixture = Fixture()
        var failures = 0
        let screens: [(String, AppModel.Screen, CGSize)] = [
            ("welcome", .welcome, CGSize(width: 1100, height: 700)),
            ("preview", .preview, CGSize(width: 1100, height: 760)),
            ("scan", .scanning, CGSize(width: 1240, height: 820)),
            ("summary", .scanComplete, CGSize(width: 1000, height: 640)),
            ("reconstruction", .reconstructing, CGSize(width: 1000, height: 640)),
            ("result", .result, CGSize(width: 1240, height: 820)),
        ]
        for (name, screen, size) in screens {
            let model = fixture.model(screen: screen)
            let view = DetailScreenForSnapshot(screen: screen)
                .padding(16)
                .frame(width: size.width, height: size.height)
                .background(Color(nsColor: .windowBackgroundColor))
                .environment(model)
            if !write(view, to: directory.appendingPathComponent("\(name).png")) { failures += 1 }
        }
        // The narrowest detail area (window at its minimum width, sidebar at its maximum): the busiest control rows.
        let narrow: [(String, AppModel.Screen, CameraSourceKind, Bool, Bool)] = [
            ("preview-live-recording-narrow", .preview, .live, true, false),
            ("preview-dataset-narrow", .preview, .dataset(URL(fileURLWithPath: "/tmp/anna-0001.ifsdataset")), false, true),
            ("scan-narrow", .scanning, .live, true, false),
            ("summary-diagonals-narrow", .scanComplete, .live, false, false),
        ]
        for (name, screen, kind, recording, dataset) in narrow {
            let model = fixture.model(screen: screen, sourceKind: kind, recording: recording, dataset: dataset,
                                      allPoses: name.contains("diagonals"))
            let view = DetailScreenForSnapshot(screen: screen)
                .padding(16)
                .frame(width: 640, height: 620)
                .background(Color(nsColor: .windowBackgroundColor))
                .environment(model)
            if !write(view, to: directory.appendingPathComponent("\(name).png")) { failures += 1 }
        }
        // The whole window through AppKit (sidebar form, pickers, scroll views draw; ImageRenderer cannot draw them).
        let windows: [(String, AppModel.Screen, CGSize)] = [
            ("window-preview-min", .preview, CGSize(width: 1040, height: 680)),
            ("window-scan-min", .scanning, CGSize(width: 1040, height: 680)),
            ("window-result", .result, CGSize(width: 1280, height: 820)),
            ("window-result-min", .result, CGSize(width: 1040, height: 680)),
            ("window-result-min-settings-changed", .result, CGSize(width: 1040, height: 680)),
            ("window-summary-min", .scanComplete, CGSize(width: 1040, height: 680)),
            ("window-sidebar-tall", .welcome, CGSize(width: 1040, height: 1640)),
        ]
        for (name, screen, size) in windows {
            let model = fixture.model(screen: screen, sourceKind: .live, recording: screen == .preview, dataset: false)
            // The voxel size changed after the reconstruction: the result panel offers to reconstruct again.
            if name.hasSuffix("-settings-changed") { model.settings.voxelSizeMM += 0.25 }
            let view = ContentView().environment(model).environment(\.poseGuideAnimates, false)
            if !writeHosted(view, size: size, to: directory.appendingPathComponent("\(name).png")) { failures += 1 }
        }
        // The scan panel scrolls in the app; ImageRenderer cannot draw AppKit scroll views, so render its content.
        let panel = ScanPanelContent()
            .padding(12)
            .frame(width: 354, height: 760, alignment: .top)
            .background(Color(nsColor: .windowBackgroundColor))
            .environment(fixture.model(screen: .scanning))
        if !write(panel, to: directory.appendingPathComponent("scan-panel.png")) { failures += 1 }
        // The result panel's report scrolls too.
        if let report = fixture.model(screen: .result).result?.report {
            let view = ReportView(report: report)
                .frame(width: 320)
                .padding(12)
                .background(Color(nsColor: .windowBackgroundColor))
            if !write(view, to: directory.appendingPathComponent("report.png")) { failures += 1 }
        }
        // The sidebar is a grouped Form (an AppKit scroll view): ImageRenderer draws it blank, AppKit draws it.
        let sidebar = SidebarView().environment(fixture.model(screen: .preview))
        if !writeHosted(sidebar, size: CGSize(width: 330, height: 1500),
                        to: directory.appendingPathComponent("sidebar.png")) { failures += 1 }
        failures += checkPreviewShaders(fixture.model(screen: .preview).renderer, directory: directory)
        print("snapshots written to \(directory.path)\(failures > 0 ? " (\(failures) failed)" : "")")
        return failures == 0 ? 0 : 1
    }

    /// Renders a synthetic frame through the preview shaders and checks orientation, mirroring and the depth
    /// colors: left half red / right half blue, a green band at the top; depth 0.4 m left, 1.0 m right, no depth
    /// in the bottom-right quadrant. Returns the number of failed checks.
    private static func checkPreviewShaders(_ renderer: PreviewRenderer?, directory: URL) -> Int {
        guard let renderer else {
            FileHandle.standardError.write(Data("no Metal renderer\n".utf8))
            return 1
        }
        let w = 160, h = 100
        var rgb = [UInt8](repeating: 0, count: w * h * 3)
        var depth = [UInt16](repeating: 0, count: w * h)
        for y in 0 ..< h {
            for x in 0 ..< w {
                let i = y * w + x
                let c: (UInt8, UInt8, UInt8) = y < 10 ? (0, 255, 0) : (x < w / 2 ? (255, 0, 0) : (0, 0, 255))
                rgb[3 * i] = c.0; rgb[3 * i + 1] = c.1; rgb[3 * i + 2] = c.2
                depth[i] = x < w / 2 ? 4000 : (y < h / 2 ? 10000 : 0)
            }
        }
        let k = Intrinsics(width: w, height: h, fx: 80, fy: 80, ppx: 80, ppy: 50)
        let frame = CameraFrame(sequence: 1, timestampMs: 1,
                                color: ColorImage(width: w, height: h, bytesPerRow: w * 3, pixels: Data(rgb), intrinsics: k),
                                alignedDepth: DepthImage(width: w, height: h, pixels: depth.withUnsafeBytes { Data($0) },
                                                         depthScale: 0.0001, intrinsics: k))
        var failures = 0
        func check(_ name: String, _ condition: Bool) {
            if !condition {
                failures += 1
                FileHandle.standardError.write(Data("preview shader check failed: \(name)\n".utf8))
            }
        }
        func render(_ name: String, _ display: PreviewDisplay) -> (Int, Int) -> (r: Int, g: Int, b: Int) {
            guard let image = renderer.renderOffscreen(frame, display: display, width: w, height: h),
                  let data = image.dataProvider?.data as Data? else {
                check("\(name) rendered", false)
                return { _, _ in (0, 0, 0) }
            }
            if let destination = CGImageDestinationCreateWithURL(directory.appendingPathComponent("metal-\(name).png") as CFURL,
                                                                 UTType.png.identifier as CFString, 1, nil) {
                CGImageDestinationAddImage(destination, image, nil)
                CGImageDestinationFinalize(destination)
            }
            let rowBytes = image.bytesPerRow
            return { x, y in
                let o = y * rowBytes + x * 4   // BGRA in memory
                return (Int(data[o + 2]), Int(data[o + 1]), Int(data[o]))
            }
        }
        let color = render("color", PreviewDisplay(mode: .color, mirrored: false))
        check("color left is red", color(20, 60).r > 240 && color(20, 60).b < 15)
        check("color right is blue", color(140, 60).b > 240 && color(140, 60).r < 15)
        check("color top is green", color(80, 3).g > 240)
        let mirrored = render("mirrored", PreviewDisplay(mode: .color, mirrored: true))
        check("mirrored left is blue", mirrored(20, 60).b > 240)
        check("mirrored right is red", mirrored(140, 60).r > 240)
        check("mirrored top is green", mirrored(80, 3).g > 240)
        let depthView = render("depth", PreviewDisplay(mode: .depth, mirrored: false, nearM: 0.3, farM: 1.2))
        let near = depthView(20, 60), far = depthView(140, 25), none = depthView(140, 80)
        check("near depth is cool", near.b > near.r)
        check("far depth is warm", far.r > far.b)
        check("missing depth is dark", none.r < 30 && none.g < 30 && none.b < 30)
        let blend = render("blend", PreviewDisplay(mode: .blend, mirrored: false, nearM: 0.3, farM: 1.2, blend: 0.5))
        check("blend mixes color and depth", blend(20, 60).r < 250 && blend(20, 60).r > 100)
        check("blend keeps color without depth", blend(140, 80).b > 240)
        return failures
    }

    /// Renders a view hosted in an offscreen window (never shown) with AppKit drawing.
    private static func writeHosted<V: View>(_ view: V, size: CGSize, to url: URL) -> Bool {
        _ = NSApplication.shared
        let hosting = NSHostingView(rootView: view)
        hosting.frame = CGRect(origin: .zero, size: size)
        let window = NSWindow(contentRect: hosting.frame, styleMask: [.titled, .resizable], backing: .buffered, defer: false)
        window.isReleasedWhenClosed = false
        window.isRestorable = false
        window.appearance = NSAppearance(named: .aqua)
        window.contentView = hosting
        hosting.layoutSubtreeIfNeeded()
        RunLoop.main.run(until: Date(timeIntervalSinceNow: 0.5))   // SwiftUI updates the AppKit controls
        hosting.layoutSubtreeIfNeeded()
        defer { window.close() }
        guard let rep = hosting.bitmapImageRepForCachingDisplay(in: hosting.bounds) else {
            FileHandle.standardError.write(Data("render failed: \(url.lastPathComponent)\n".utf8))
            return false
        }
        hosting.cacheDisplay(in: hosting.bounds, to: rep)
        guard let image = rep.cgImage,
              let destination = CGImageDestinationCreateWithURL(url as CFURL, UTType.png.identifier as CFString, 1, nil) else {
            return false
        }
        CGImageDestinationAddImage(destination, image, nil)
        return CGImageDestinationFinalize(destination)
    }

    private static func write<V: View>(_ view: V, to url: URL) -> Bool {
        let renderer = ImageRenderer(content: view)
        renderer.scale = 2
        guard let image = renderer.cgImage,
              let destination = CGImageDestinationCreateWithURL(url as CFURL, UTType.png.identifier as CFString, 1, nil) else {
            FileHandle.standardError.write(Data("render failed: \(url.lastPathComponent)\n".utf8))
            return false
        }
        CGImageDestinationAddImage(destination, image, nil)
        return CGImageDestinationFinalize(destination)
    }
}

private struct DetailScreenForSnapshot: View {
    let screen: AppModel.Screen

    var body: some View {
        switch screen {
        case .welcome: WelcomeView()
        case .preview: PreviewScreen()
        case .scanning: ScanScreen()
        case .scanComplete: ScanSummaryView()
        case .reconstructing: ReconstructionView()
        case .result: ResultView()
        }
    }
}

/// A plausible mid-scan state: front/up done, down partly, now turning left.
@MainActor
private struct Fixture {
    let device: DeviceInfo? = {
        let json = """
        {"name": "Intel RealSense D455", "serial": "123456789012", "firmware": "5.16.0.1",
         "recommended_firmware": "5.17.0.10", "usb_type": "3.2", "product_id": "0B5C", "product_line": "D400",
         "advanced_mode": true, "depth_scale": 0.0001, "baseline_mm": 95.0, "imu": true, "source": "live",
         "presets": ["High Accuracy", "High Density"], "sensors": [],
         "profiles": [{"stream": "depth", "index": 0, "format": "z16", "width": 1280, "height": 720, "fps": 30},
                      {"stream": "depth", "index": 0, "format": "z16", "width": 848, "height": 480, "fps": 30},
                      {"stream": "color", "index": 0, "format": "rgb8", "width": 1280, "height": 800, "fps": 30}]}
        """
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try? decoder.decode(DeviceInfo.self, from: Data(json.utf8))
    }()

    func model(screen: AppModel.Screen, sourceKind: CameraSourceKind = .synthetic, recording: Bool = false,
               dataset: Bool = false, allPoses: Bool = false) -> AppModel {
        let model = AppModel(defaults: nil)
        let configuration = ScanConfiguration()
        let face = CGRect(x: 520, y: 220, width: 230, height: 280)
        var landmarks: [SIMD2<Float>] = []
        for i in 0 ..< 40 {
            let a = Float(i) / 40 * 2 * .pi
            landmarks.append(SIMD2(635 + 100 * cos(a), 370 + 125 * sin(a)))
        }
        let tracking = TrackingState(imageSize: CGSize(width: 1280, height: 800), faceRect: face, landmarks: landmarks,
                                     pose: HeadPose(yaw: 24, pitch: 1, roll: 2), distance: 0.63,
                                     distanceIsEstimate: false, advice: .ok, range: 0.56 ... 0.71)
        var update = ScanUpdate()
        update.phase = .capturing(.left)
        update.instructionKind = .turn(.left, yawDelta: 21, pitchDelta: -1)
        update.rejection = .headMoving
        update.poseProgress = [.front: 1, .up: 1, .down: 1, .left: 0.34, .right: 0]
        update.overallProgress = 0.67
        update.keyframeCount = 10
        update.targetDelta = SIMD2(21, -1)
        update.angularErrorDeg = 21
        update.poseIndex = 3
        let poses = [HeadPose](repeating: HeadPose(yaw: 0, pitch: 0, roll: 0), count: 3)
            + [HeadPose](repeating: HeadPose(yaw: 0, pitch: 25, roll: 0), count: 3)
            + [HeadPose](repeating: HeadPose(yaw: 0, pitch: -25, roll: 0), count: 3)
            + [HeadPose(yaw: 44, pitch: 0, roll: 0)]
        update.coverage = CoverageMap(keyframePoses: poses, framesPerPose: 3)

        let k = Intrinsics(width: 2, height: 2, fx: 1, fy: 1, ppx: 1, ppy: 1)
        let frame = CameraFrame(sequence: 0, timestampMs: 0,
                                color: ColorImage(width: 2, height: 2, bytesPerRow: 6, pixels: Data(count: 12), intrinsics: k),
                                alignedDepth: DepthImage(width: 2, height: 2, pixels: Data(count: 8), depthScale: 0.0001, intrinsics: k))
        let scanPoses: [ScanPose] = allPoses
            ? ScanPose.allCases.flatMap { [$0, $0, $0] }
            : poses.indices.map { [ScanPose.front, .up, .down, .left][min(3, $0 / 3)] }
        let keyframes = scanPoses.enumerated().map { i, scanPose in
            Keyframe(id: i, frame: frame, scanPose: scanPose, headPose: poses[min(i, poses.count - 1)],
                     faceRect: face, landmarks: [], mask: nil,
                     quality: FrameQuality(sharpness: 120, angularSpeedDegPerSec: 3, validDepthRatio: 0.97,
                                           validDepthPoints: 41_000, faceDistanceM: 0.63))
        }
        var report = ReconstructionReport()
        report.keyframes = 15; report.usedFrames = 15; report.odometryEdges = 14; report.loopClosures = 6
        report.meanRegistrationRMSEmm = 0.62; report.maxRegistrationRMSEmm = 1.10; report.meanFitness = 0.91
        report.voxelSizeMM = 0.75; report.usedGPU = true; report.vertexCount = 412_118; report.triangleCount = 823_904
        report.durations = ["segmentation": 0.6, "registration": 2.4, "registration check": 0.2, "bounds": 0.1,
                            "fusion": 1.1, "cleanup": 0.8, "texturing": 3.2, "total": 8.5]
        // The notes of a real run (they are as long as this).
        report.notes = ["initial poses: 14 landmark fits, 0 Vision rotation + landmark translation, 0 Vision rotation "
                        + "+ head centroid",
                        "keyframes excluded after registration (surface does not fit): 7, 12"]
        let mesh = ReconMesh(positions: [SIMD3(0, 0, 0), SIMD3(0.1, 0, 0), SIMD3(0, 0.1, 0)], indices: [0, 1, 2])
        let result = ReconstructionResult(mesh: mesh, texture: nil, cameraPoses: [], exportTransform: matrix_identity_float4x4,
                                          report: report)
        let capture = ScanCapture(keyframes: keyframes, device: device)
        model.applyFixture(screen: screen, sourceKind: sourceKind, device: device, tracking: tracking,
                           scanUpdate: update, scanConfiguration: configuration, capture: capture,
                           progress: ReconstructionProgress(stage: .fusion, fraction: 0.46,
                                                            message: "integrating frame 7 of 15"),
                           result: result, stats: FrameStats(displayFPS: 30, analysisFPS: 24),
                           recordingStartedAt: recording ? Date(timeIntervalSinceNow: -83) : nil,
                           datasetCapture: dataset ? capture : nil)
        return model
    }
}
#endif
