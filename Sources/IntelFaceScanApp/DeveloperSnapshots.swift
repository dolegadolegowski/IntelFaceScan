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
        // The scan panel scrolls in the app; ImageRenderer cannot draw AppKit scroll views, so render its content.
        let panel = ScanPanelContent()
            .padding(12)
            .frame(width: 354, height: 760, alignment: .top)
            .background(Color(nsColor: .windowBackgroundColor))
            .environment(fixture.model(screen: .scanning))
        if !write(panel, to: directory.appendingPathComponent("scan-panel.png")) { failures += 1 }
        let sidebar = SidebarView()
            .frame(width: 330, height: 1500)
            .background(Color(nsColor: .windowBackgroundColor))
            .environment(fixture.model(screen: .preview))
        if !write(sidebar, to: directory.appendingPathComponent("sidebar.png")) { failures += 1 }
        print("snapshots written to \(directory.path)\(failures > 0 ? " (\(failures) failed)" : "")")
        return failures == 0 ? 0 : 1
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

    func model(screen: AppModel.Screen) -> AppModel {
        let model = AppModel()
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
        let keyframes = poses.enumerated().map { i, pose in
            Keyframe(id: i, frame: frame, scanPose: [ScanPose.front, .up, .down, .left][min(3, i / 3)], headPose: pose,
                     faceRect: face, landmarks: [], mask: nil,
                     quality: FrameQuality(sharpness: 120, angularSpeedDegPerSec: 3, validDepthRatio: 0.97,
                                           validDepthPoints: 41_000, faceDistanceM: 0.63))
        }
        var report = ReconstructionReport()
        report.keyframes = 15; report.usedFrames = 15; report.odometryEdges = 14; report.loopClosures = 6
        report.meanRegistrationRMSEmm = 0.62; report.maxRegistrationRMSEmm = 1.10; report.meanFitness = 0.91
        report.voxelSizeMM = 0.75; report.usedGPU = true; report.vertexCount = 412_118; report.triangleCount = 823_904
        report.durations = ["registration": 2.4, "fusion": 1.1, "meshing": 0.8, "texturing": 3.2]
        report.notes = ["Synthetic fixture"]
        let mesh = ReconMesh(positions: [SIMD3(0, 0, 0), SIMD3(0.1, 0, 0), SIMD3(0, 0.1, 0)], indices: [0, 1, 2])
        let result = ReconstructionResult(mesh: mesh, texture: nil, cameraPoses: [], exportTransform: matrix_identity_float4x4,
                                          report: report)
        model.applyFixture(screen: screen, sourceKind: .synthetic, device: device, tracking: tracking,
                           scanUpdate: update, scanConfiguration: configuration,
                           capture: ScanCapture(keyframes: keyframes, device: device),
                           progress: ReconstructionProgress(stage: .fusion, fraction: 0.46, message: "TSDF 412 / 900"),
                           result: result, stats: FrameStats(displayFPS: 30, analysisFPS: 24))
        return model
    }
}
#endif
