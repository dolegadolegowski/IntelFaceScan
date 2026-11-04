// License: Apache 2.0. See LICENSE file in root directory.
//
// HeadTracker: the Vision sign convention, coordinate conversions, landmark cues and Vision on synthetic images
// (no face available without a camera: the tests check that the pipeline runs and fails cleanly).

import CameraKit
import CoreGraphics
import CoreVideo
import Foundation
import SyntheticData
import Testing
@testable import ScanKit

/// Apple Vision on the rendered synthetic head (lighting 0.6: Vision needs the shading cues), as HeadPose through
/// HeadTracker with the documented convention. The head is rendered with HeadPose.cameraFrameRotation, so these
/// tests tie Vision's angles, VisionPoseConvention and the HeadPose convention together.
@Suite(.serialized) struct GuidanceVisionSignTests {
    static let scene: SyntheticScene = {
        var options = SyntheticCameraSource.defaultOptions
        options.width = 640
        options.height = 400
        return SyntheticScene(options: options)
    }()

    private func measure(_ pose: HeadPose, index: UInt64) throws -> HeadPose {
        let scene = Self.scene
        let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: pose), frameIndex: index,
                                sequence: index, timestampMs: 1_000 + Double(index))
        let tracker = HeadTracker(convention: .documented)
        let observation = try #require(try tracker.analyze(view.frame, segmentation: false), "no face at \(pose)")
        return observation.pose
    }

    /// Readings at -angle, 0 and +angle on one axis: they must increase with the true angle, with the right sign at
    /// both ends and a clear spread (Vision on the synthetic face: e.g. yaw +-20 -> ~+20 / -27).
    private func expectAxis(_ angle: Float, _ make: (Float) -> HeadPose, _ read: (HeadPose) -> Float,
                            _ name: String) throws {
        let minus = read(try measure(make(-angle), index: 1))
        let zero = read(try measure(make(0), index: 2))
        let plus = read(try measure(make(angle), index: 3))
        #expect(minus < zero && zero < plus, "\(name): \(minus), \(zero), \(plus)")
        #expect(plus > 0 && minus < 0, "\(name): \(minus), \(plus)")
        #expect(plus - minus > angle, "\(name): \(minus), \(plus) for +-\(angle)")
    }

    @Test func yawSignMatchesHeadPose() throws {
        // yaw > 0: the nose moves toward the image right.
        try expectAxis(20, { HeadPose(yaw: $0, pitch: 0, roll: 0) }, \.yaw, "yaw")
    }

    @Test func pitchSignMatchesHeadPose() throws {
        // pitch > 0: the face turns up.
        try expectAxis(20, { HeadPose(yaw: 0, pitch: $0, roll: 0) }, \.pitch, "pitch")
    }

    @Test func rollSignMatchesHeadPose() throws {
        // roll > 0: the head tilts clockwise in the image; the eye line from the landmarks agrees.
        try expectAxis(15, { HeadPose(yaw: 0, pitch: 0, roll: $0) }, \.roll, "roll")
        let scene = Self.scene
        let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: HeadPose(yaw: 0, pitch: 0, roll: 15)),
                                frameIndex: 4, sequence: 4, timestampMs: 1_004)
        let observation = try #require(try HeadTracker(convention: .documented).analyze(view.frame, segmentation: false))
        let cues = try #require(observation.regions.flatMap { LandmarkPoseCues(regions: $0) })
        #expect(cues.rollDegrees > 5, "eye line \(cues.rollDegrees)")
    }

    @Test func framesWithEqualNumbersButOtherPixelsAreAnalyzedAgain() throws {
        // The converted-image cache must not hand Vision the previous frame when a source repeats sequence numbers
        // and timestamps (datasets without them, two synthetic datasets with the same numbering).
        let scene = Self.scene
        let face = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: .frontal), frameIndex: 5,
                                sequence: 9, timestampMs: 9).frame
        var blank = face
        blank.color.pixels = Data(repeating: 128, count: face.color.pixels.count)
        let tracker = HeadTracker(convention: .documented)
        #expect(try tracker.analyze(face, segmentation: false) != nil)
        #expect(try tracker.analyze(blank, segmentation: false) == nil)
        #expect(try tracker.analyze(face, segmentation: false) != nil)
    }
}

@Suite struct GuidanceHeadTrackerTests {
    @Test func documentedConventionMapsVisionSigns() {
        let c = VisionPoseConvention.documented
        let rad = Double.pi / 180
        // Vision: nose toward image right is +yaw; nodding down is +pitch; counterclockwise is +roll.
        let pose = c.headPose(visionYaw: 30 * rad, visionPitch: 10 * rad, visionRoll: 5 * rad, confidence: 0.8)
        #expect(abs(pose.yaw - 30) < 1e-4)
        #expect(abs(pose.pitch + 10) < 1e-4)   // HeadPose: face up is positive
        #expect(abs(pose.roll + 5) < 1e-4)     // HeadPose: clockwise is positive
        #expect(pose.confidence == 0.8)
        // Missing angles are 0; roll is wrapped around an upright face.
        let partial = c.headPose(visionYaw: nil, visionPitch: nil, visionRoll: 3 * Double.pi / 2, confidence: 1)
        #expect(partial.yaw == 0 && partial.pitch == 0)
        #expect(abs(partial.roll - 90) < 1e-3)
    }

    @Test func conventionFromEnvironmentString() {
        #expect(VisionPoseConvention(environmentValue: "-1, -1, -1") == VisionPoseConvention(yawSign: -1, pitchSign: -1, rollSign: -1))
        #expect(VisionPoseConvention(environmentValue: "1,-1,1") == VisionPoseConvention(yawSign: 1, pitchSign: -1, rollSign: 1))
        #expect(VisionPoseConvention(environmentValue: "1,-1") == nil)
        #expect(VisionPoseConvention(environmentValue: "a,b,c") == nil)
        #expect(VisionPoseConvention(environmentValue: nil) == nil)
    }

    @Test func pixelRectFlipsVisionOrigin() {
        let r = HeadTracker.pixelRect(CGRect(x: 0.25, y: 0.5, width: 0.25, height: 0.25),
                                      imageSize: CGSize(width: 1280, height: 800))
        #expect(r == CGRect(x: 320, y: 200, width: 320, height: 200))
    }

    @Test func landmarkCuesMeasureRollAndYaw() throws {
        // Eyes 40 px apart, the right one 40 * tan(20 deg) lower: a clockwise tilt in a y-down image.
        let dy = Float(40 * tan(20 * Double.pi / 180))
        var regions = FaceLandmarkRegions(leftEye: [SIMD2(100, 100)], rightEye: [SIMD2(140, 100 + dy)],
                                          nose: [SIMD2(120, 100 + dy / 2 + 20)])
        var cues = try #require(LandmarkPoseCues(regions: regions))
        #expect(abs(cues.rollDegrees - 20) < 1e-3)
        // Nose below the eye midpoint (perpendicular to the eye line? approximately): small yaw cue.
        #expect(abs(cues.yawCue) < 0.2)
        // Vision's left/right naming does not matter: the cue sorts eyes by x.
        regions = FaceLandmarkRegions(leftEye: [SIMD2(140, 100)], rightEye: [SIMD2(100, 100)], nose: [SIMD2(130, 120)])
        cues = try #require(LandmarkPoseCues(regions: regions))
        #expect(abs(cues.rollDegrees) < 1e-4)
        #expect(abs(cues.yawCue - 0.25) < 1e-4)   // nose 10 px right of the midpoint, eyes 40 px apart
        #expect(LandmarkPoseCues(regions: FaceLandmarkRegions()) == nil)
    }

    @Test func chinPointUsesContourAlongHeadDown() {
        let contour: [SIMD2<Float>] = [SIMD2(100, 150), SIMD2(120, 190), SIMD2(140, 185), SIMD2(160, 150)]
        var obs = HeadObservation(pose: .frontal, faceRect: CGRect(x: 100, y: 100, width: 60, height: 80),
                                  landmarks: [], personMask: nil, regions: FaceLandmarkRegions(faceContour: contour))
        #expect(obs.chinPoint == SIMD2(120, 190))
        // Rolled 80 deg clockwise: "down" points almost to the image left, so the left contour point wins.
        obs.pose.roll = 80
        #expect(obs.chinPoint == SIMD2(100, 150))
    }

    private func frame(width: Int, height: Int, fill: (Int, Int) -> (UInt8, UInt8, UInt8)) -> CameraFrame {
        var data = Data(count: width * height * 3)
        data.withUnsafeMutableBytes { raw in
            let p = raw.bindMemory(to: UInt8.self)
            for y in 0 ..< height {
                for x in 0 ..< width {
                    let (r, g, b) = fill(x, y)
                    p[(y * width + x) * 3] = r; p[(y * width + x) * 3 + 1] = g; p[(y * width + x) * 3 + 2] = b
                }
            }
        }
        let k = Intrinsics(width: width, height: height, fx: Float(width) / 2, fy: Float(width) / 2,
                           ppx: Float(width) / 2, ppy: Float(height) / 2)
        let color = ColorImage(width: width, height: height, bytesPerRow: width * 3, pixels: data, intrinsics: k)
        let depth = DepthImage(width: width, height: height, pixels: Data(count: width * height * 2), depthScale: 0.0001,
                               intrinsics: k)
        return CameraFrame(sequence: 1, timestampMs: 1, color: color, alignedDepth: depth)
    }

    @Test func visionFindsNoFaceInSyntheticScene() throws {
        let tracker = HeadTracker(convention: .documented)
        let f = frame(width: 320, height: 200) { x, y in (UInt8((x * 7) & 255), UInt8((y * 5) & 255), 90) }
        #expect(try tracker.analyze(f, segmentation: false) == nil)
        // Segmentation runs on its own and returns a binary mask at color resolution (or nil).
        if let mask = try tracker.segmentPerson(in: f) {
            #expect(mask.count == 320 * 200)
            #expect(mask.allSatisfy { $0 == 0 || $0 == 255 })
        }
    }

    @Test func invalidImagesThrow() {
        let tracker = HeadTracker()
        var f = frame(width: 64, height: 48) { _, _ in (0, 0, 0) }
        f.color.pixels = f.color.pixels.prefix(100)
        #expect(throws: HeadTrackerError.self) { try tracker.analyze(f, segmentation: false) }
        f.color.width = 0
        #expect(throws: HeadTrackerError.self) { try tracker.analyze(f, segmentation: true) }
    }

    @Test func pixelBufferConversionKeepsColors() throws {
        let f = frame(width: 33, height: 7) { x, y in (UInt8(x), UInt8(y * 10), 200) }
        let buffer = try PixelBufferConverter().makeBGRA(from: f.color)
        #expect(CVPixelBufferGetWidth(buffer) == 33)
        #expect(CVPixelBufferGetHeight(buffer) == 7)
        #expect(CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_32BGRA)
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        let base = try #require(CVPixelBufferGetBaseAddress(buffer)).assumingMemoryBound(to: UInt8.self)
        let rowBytes = CVPixelBufferGetBytesPerRow(buffer)
        let p = base + 5 * rowBytes + 12 * 4
        #expect(p[0] == 200 && p[1] == 50 && p[2] == 12 && p[3] == 255)   // B, G, R, A
    }

    @Test func maskResamplerThresholds() throws {
        var pb: CVPixelBuffer?
        #expect(CVPixelBufferCreate(nil, 4, 2, kCVPixelFormatType_OneComponent8, nil, &pb) == kCVReturnSuccess)
        let buffer = try #require(pb)
        CVPixelBufferLockBaseAddress(buffer, [])
        let base = try #require(CVPixelBufferGetBaseAddress(buffer)).assumingMemoryBound(to: UInt8.self)
        let rowBytes = CVPixelBufferGetBytesPerRow(buffer)
        for y in 0 ..< 2 { for x in 0 ..< 4 { base[y * rowBytes + x] = x < 2 ? 20 : 240 } }
        CVPixelBufferUnlockBaseAddress(buffer, [])
        let mask = try MaskResampler.binaryMask(buffer, width: 40, height: 20)
        #expect(mask.count == 800)
        #expect(mask.allSatisfy { $0 == 0 || $0 == 255 })
        #expect(mask[10 * 40 + 2] == 0)
        #expect(mask[10 * 40 + 37] == 255)
    }
}
