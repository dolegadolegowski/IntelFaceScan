// License: Apache 2.0. See LICENSE file in root directory.
//
// Face detection, landmarks, head pose and person segmentation with Apple Vision on the RGB8 color image.
// ScanController talks to the tracker through the HeadTracking protocol so that it can be driven by scripted
// observations in tests.

import CameraKit
import CoreGraphics
import CoreVideo
import Foundation
import os
import simd
import Vision

public struct HeadObservation: Sendable {
    public var pose: HeadPose
    /// Face rectangle in color image pixels (top-left origin).
    public var faceRect: CGRect
    /// Landmarks in color image pixels (top-left origin).
    public var landmarks: [SIMD2<Float>]
    /// Person/head segmentation at color resolution (255 = person), nil when not computed.
    public var personMask: Data?
    /// Named landmark regions in color image pixels (top-left origin), nil when landmarks were not detected.
    public var regions: FaceLandmarkRegions?

    public init(pose: HeadPose, faceRect: CGRect, landmarks: [SIMD2<Float>], personMask: Data?) {
        self.pose = pose; self.faceRect = faceRect; self.landmarks = landmarks; self.personMask = personMask
        self.regions = nil
    }

    public init(pose: HeadPose, faceRect: CGRect, landmarks: [SIMD2<Float>], personMask: Data?,
                regions: FaceLandmarkRegions?) {
        self.pose = pose; self.faceRect = faceRect; self.landmarks = landmarks; self.personMask = personMask
        self.regions = regions
    }

    /// Lowest point of the chin along the head's own "down" direction (pixels), from the face contour when
    /// available, else the bottom center of the face rectangle.
    public var chinPoint: SIMD2<Float> {
        let down = HeadGeometry.downDirection(rollDegrees: pose.roll)
        if let contour = regions?.faceContour, !contour.isEmpty {
            var best = contour[0]
            var bestDot = simd_dot(best, down)
            for p in contour.dropFirst() {
                let d = simd_dot(p, down)
                if d > bestDot { best = p; bestDot = d }
            }
            return best
        }
        let center = SIMD2(Float(faceRect.midX), Float(faceRect.midY))
        return center + down * Float(faceRect.height) * 0.5
    }
}

/// Vision landmark regions (76-point constellation), in color image pixels with a top-left origin.
/// "left"/"right" follow Vision's naming; code that needs image-left/image-right sorts by x instead.
public struct FaceLandmarkRegions: Sendable, Equatable {
    public var faceContour: [SIMD2<Float>]
    public var leftEye: [SIMD2<Float>]
    public var rightEye: [SIMD2<Float>]
    public var leftPupil: [SIMD2<Float>]
    public var rightPupil: [SIMD2<Float>]
    public var nose: [SIMD2<Float>]
    public var noseCrest: [SIMD2<Float>]
    public var medianLine: [SIMD2<Float>]
    public var outerLips: [SIMD2<Float>]

    public init(faceContour: [SIMD2<Float>] = [], leftEye: [SIMD2<Float>] = [], rightEye: [SIMD2<Float>] = [],
                leftPupil: [SIMD2<Float>] = [], rightPupil: [SIMD2<Float>] = [], nose: [SIMD2<Float>] = [],
                noseCrest: [SIMD2<Float>] = [], medianLine: [SIMD2<Float>] = [], outerLips: [SIMD2<Float>] = []) {
        self.faceContour = faceContour; self.leftEye = leftEye; self.rightEye = rightEye
        self.leftPupil = leftPupil; self.rightPupil = rightPupil; self.nose = nose; self.noseCrest = noseCrest
        self.medianLine = medianLine; self.outerLips = outerLips
    }
}

/// Head pose cues measured directly on the landmarks. They have no sign ambiguity, so they are used to check the
/// Vision sign convention on real data (see VisionPoseConvention).
public struct LandmarkPoseCues: Sendable, Equatable {
    /// In-plane rotation of the eye line, degrees, HeadPose convention (clockwise in the image > 0).
    public var rollDegrees: Float
    /// Horizontal offset of the nose from the eye midpoint, in inter-ocular distances; > 0 when the nose is
    /// toward the image right (HeadPose yaw > 0).
    public var yawCue: Float

    public init(rollDegrees: Float, yawCue: Float) {
        self.rollDegrees = rollDegrees; self.yawCue = yawCue
    }

    public init?(regions: FaceLandmarkRegions) {
        guard let a = Self.centroid(regions.leftEye), let b = Self.centroid(regions.rightEye),
              let nose = Self.centroid(regions.nose.isEmpty ? regions.noseCrest : regions.nose) else { return nil }
        let (l, r) = a.x <= b.x ? (a, b) : (b, a)
        let eyeLine = r - l
        let distance = simd_length(eyeLine)
        guard distance > 1 else { return nil }
        // Image y grows downward, so atan2 of the eye line is already clockwise-positive.
        rollDegrees = atan2(eyeLine.y, eyeLine.x) * 180 / .pi
        // Measure the nose offset along the eye line so that roll does not leak into the yaw cue.
        yawCue = simd_dot(nose - (l + r) * 0.5, eyeLine / distance) / distance
    }

    static func centroid(_ points: [SIMD2<Float>]) -> SIMD2<Float>? {
        guard !points.isEmpty else { return nil }
        return points.reduce(SIMD2<Float>(repeating: 0), +) / Float(points.count)
    }
}

/// The ONE place where Vision's face angles are mapped onto HeadPose (ScanTypes.swift).
///
/// Vision (VNFaceObservation, VNDetectFaceRectanglesRequestRevision3), per the SDK header documentation:
///   * roll: radians, positive = counterclockwise in the image,
///   * yaw: radians, positive = counterclockwise about the vertical axis seen from above, i.e. the nose moves
///     toward the image right (right-hand rule in Vision's x-right / y-up / z-toward-viewer frame),
///   * pitch: radians, positive = nodding the head down.
/// HeadPose: yaw > 0 nose toward the image right, pitch > 0 face up, roll > 0 clockwise in the image.
///
/// After a check with the real camera, flip a sign here (or, without rebuilding, set the environment variable
/// IFS_VISION_POSE_SIGNS="yaw,pitch,roll", e.g. "-1,-1,-1"). HeadTracker logs a warning when the Vision roll/yaw
/// signs disagree with the landmark geometry (LandmarkPoseCues) over many frames.
public struct VisionPoseConvention: Sendable, Hashable {
    public var yawSign: Float
    public var pitchSign: Float
    public var rollSign: Float

    public init(yawSign: Float, pitchSign: Float, rollSign: Float) {
        self.yawSign = yawSign; self.pitchSign = pitchSign; self.rollSign = rollSign
    }

    /// Mapping derived from the Vision header documentation.
    public static let documented = VisionPoseConvention(yawSign: 1, pitchSign: -1, rollSign: -1)

    /// `documented`, unless IFS_VISION_POSE_SIGNS overrides it.
    public static let current: VisionPoseConvention =
        VisionPoseConvention(environmentValue: ProcessInfo.processInfo.environment["IFS_VISION_POSE_SIGNS"]) ?? .documented

    /// Parses "yaw,pitch,roll" signs such as "1,-1,-1".
    public init?(environmentValue: String?) {
        guard let parts = environmentValue?.split(separator: ",").map({ $0.trimmingCharacters(in: .whitespaces) }),
              parts.count == 3 else { return nil }
        let signs = parts.compactMap { Float($0) }.map { $0 < 0 ? Float(-1) : Float(1) }
        guard signs.count == 3 else { return nil }
        self.init(yawSign: signs[0], pitchSign: signs[1], rollSign: signs[2])
    }

    /// Converts Vision angles (radians, nil when not computed) to a HeadPose in degrees.
    public func headPose(visionYaw: Double?, visionPitch: Double?, visionRoll: Double?, confidence: Float) -> HeadPose {
        let deg = 180 / Double.pi
        return HeadPose(yaw: yawSign * Float((visionYaw ?? 0) * deg),
                        pitch: pitchSign * Float((visionPitch ?? 0) * deg),
                        roll: rollSign * Float(Self.wrapRoll(visionRoll ?? 0) * deg),
                        confidence: confidence)
    }

    /// Vision reports roll in [-pi, pi); keep it continuous around an upright face.
    static func wrapRoll(_ r: Double) -> Double {
        var v = r
        while v >= .pi { v -= 2 * .pi }
        while v < -.pi { v += 2 * .pi }
        return v
    }
}

public enum HeadGeometry {
    /// Unit vector of the head's "down" direction in image pixels (top-left origin) for a HeadPose roll.
    public static func downDirection(rollDegrees: Float) -> SIMD2<Float> {
        let r = rollDegrees * .pi / 180
        // A clockwise roll in a y-down image rotates (0, 1) to (-sin r, cos r).
        return SIMD2(-sin(r), cos(r))
    }
}

/// Analyzes color frames. Implementations must be safe to call from any thread.
public protocol HeadTracking: AnyObject, Sendable {
    /// Detects the most prominent face. Returns nil when no face is visible.
    func analyze(_ frame: CameraFrame, segmentation: Bool) throws -> HeadObservation?
    /// Person segmentation of the frame at color resolution (255 = person), nil when unavailable.
    func segmentPerson(in frame: CameraFrame) throws -> Data?
}

extension HeadTracking {
    public func segmentPerson(in frame: CameraFrame) throws -> Data? { nil }
}

public enum HeadTrackerError: Error, LocalizedError, Sendable, Equatable {
    case invalidImage(String)
    case pixelBuffer(Int32)
    case vision(String)

    public var errorDescription: String? {
        switch self {
        case .invalidImage(let m): return "Invalid color image: \(m)"
        case .pixelBuffer(let status): return "Could not create a pixel buffer (CVReturn \(status))."
        case .vision(let m): return "Vision failed: \(m)"
        }
    }
}

public final class HeadTracker: HeadTracking, @unchecked Sendable {
    public let convention: VisionPoseConvention
    public let segmentationQuality: VNGeneratePersonSegmentationRequest.QualityLevel

    private let converter = PixelBufferConverter()
    private let signCheck = ConventionCheck()
    private let lock = NSLock()
    // The last converted frame, so that a segmentation request right after analyze() reuses the buffer.
    private var lastBuffer: (sequence: UInt64, timestampMs: Double, buffer: CVPixelBuffer)?

    public init(convention: VisionPoseConvention,
                segmentationQuality: VNGeneratePersonSegmentationRequest.QualityLevel = .balanced) {
        self.convention = convention
        self.segmentationQuality = segmentationQuality
    }

    public convenience init() {
        self.init(convention: .current, segmentationQuality: .balanced)
    }

    /// Detects the most prominent face. Returns nil when no face is visible.
    public func analyze(_ frame: CameraFrame, segmentation: Bool = false) throws -> HeadObservation? {
        let color = frame.color
        let buffer = try pixelBuffer(for: frame)
        let size = CGSize(width: color.width, height: color.height)
        let handler = VNImageRequestHandler(cvPixelBuffer: buffer, orientation: .up, options: [:])

        let faces = VNDetectFaceRectanglesRequest()
        faces.revision = VNDetectFaceRectanglesRequestRevision3
        do { try handler.perform([faces]) } catch { throw HeadTrackerError.vision(error.localizedDescription) }
        guard let face = Self.mostProminent(faces.results ?? []) else { return nil }

        let landmarksRequest = VNDetectFaceLandmarksRequest()
        landmarksRequest.revision = VNDetectFaceLandmarksRequestRevision3
        landmarksRequest.constellation = .constellation76Points
        landmarksRequest.inputFaceObservations = [face]
        do { try handler.perform([landmarksRequest]) } catch { throw HeadTrackerError.vision(error.localizedDescription) }

        let pose = convention.headPose(visionYaw: face.yaw?.doubleValue, visionPitch: face.pitch?.doubleValue,
                                       visionRoll: face.roll?.doubleValue, confidence: face.confidence)
        let faceRect = Self.pixelRect(face.boundingBox, imageSize: size)
        var landmarks: [SIMD2<Float>] = []
        var regions: FaceLandmarkRegions?
        if let lm = landmarksRequest.results?.first?.landmarks {
            landmarks = Self.points(lm.allPoints, imageSize: size)
            regions = FaceLandmarkRegions(
                faceContour: Self.points(lm.faceContour, imageSize: size),
                leftEye: Self.points(lm.leftEye, imageSize: size),
                rightEye: Self.points(lm.rightEye, imageSize: size),
                leftPupil: Self.points(lm.leftPupil, imageSize: size),
                rightPupil: Self.points(lm.rightPupil, imageSize: size),
                nose: Self.points(lm.nose, imageSize: size),
                noseCrest: Self.points(lm.noseCrest, imageSize: size),
                medianLine: Self.points(lm.medianLine, imageSize: size),
                outerLips: Self.points(lm.outerLips, imageSize: size))
            if let regions, let cues = LandmarkPoseCues(regions: regions) {
                signCheck.record(pose: pose, cues: cues)
            }
        }
        // The mask is optional (the head mask falls back to an ellipse): a segmentation failure must not cost the
        // face observation.
        let mask = segmentation ? try? personMask(handler: handler, width: color.width, height: color.height) : nil
        return HeadObservation(pose: pose, faceRect: faceRect, landmarks: landmarks, personMask: mask, regions: regions)
    }

    public func segmentPerson(in frame: CameraFrame) throws -> Data? {
        let buffer = try pixelBuffer(for: frame)
        let handler = VNImageRequestHandler(cvPixelBuffer: buffer, orientation: .up, options: [:])
        return try personMask(handler: handler, width: frame.color.width, height: frame.color.height)
    }

    // MARK: - Helpers

    private func personMask(handler: VNImageRequestHandler, width: Int, height: Int) throws -> Data? {
        let request = VNGeneratePersonSegmentationRequest()
        request.revision = VNGeneratePersonSegmentationRequestRevision1
        request.qualityLevel = segmentationQuality
        request.outputPixelFormat = kCVPixelFormatType_OneComponent8
        do { try handler.perform([request]) } catch { throw HeadTrackerError.vision(error.localizedDescription) }
        guard let result = request.results?.first else { return nil }
        return try MaskResampler.binaryMask(result.pixelBuffer, width: width, height: height)
    }

    private func pixelBuffer(for frame: CameraFrame) throws -> CVPixelBuffer {
        let cached: CVPixelBuffer? = lock.withLock {
            guard let last = lastBuffer, last.sequence == frame.sequence,
                  last.timestampMs == frame.timestampMs else { return nil }
            return last.buffer
        }
        if let cached { return cached }
        let buffer = try converter.makeBGRA(from: frame.color)
        lock.withLock { lastBuffer = (frame.sequence, frame.timestampMs, buffer) }
        return buffer
    }

    /// Largest face weighted by confidence: the person being scanned is the closest one.
    static func mostProminent(_ faces: [VNFaceObservation]) -> VNFaceObservation? {
        faces.max { a, b in
            a.boundingBox.width * a.boundingBox.height * CGFloat(a.confidence)
                < b.boundingBox.width * b.boundingBox.height * CGFloat(b.confidence)
        }
    }

    /// Vision normalized rectangle (bottom-left origin) -> pixels with a top-left origin.
    static func pixelRect(_ normalized: CGRect, imageSize: CGSize) -> CGRect {
        CGRect(x: normalized.minX * imageSize.width,
               y: (1 - normalized.maxY) * imageSize.height,
               width: normalized.width * imageSize.width,
               height: normalized.height * imageSize.height)
    }

    static func points(_ region: VNFaceLandmarkRegion2D?, imageSize: CGSize) -> [SIMD2<Float>] {
        guard let region else { return [] }
        let h = Float(imageSize.height)
        return region.pointsInImage(imageSize: imageSize).map { SIMD2(Float($0.x), h - Float($0.y)) }
    }
}

/// Counts how often the Vision angles agree in sign with the landmark geometry and logs once when they clearly
/// disagree: the fastest way to notice a wrong VisionPoseConvention on the real camera.
final class ConventionCheck: @unchecked Sendable {
    private let lock = NSLock()
    private var rollAgree = 0, rollDisagree = 0
    private var yawAgree = 0, yawDisagree = 0
    private var warned = false
    private static let log = Logger(subsystem: "IntelFaceScan", category: "HeadTracker")

    func record(pose: HeadPose, cues: LandmarkPoseCues) {
        lock.lock(); defer { lock.unlock() }
        // Only clear cases count: small angles are dominated by noise.
        if abs(pose.roll) > 8 && abs(cues.rollDegrees) > 8 {
            if (pose.roll > 0) == (cues.rollDegrees > 0) { rollAgree += 1 } else { rollDisagree += 1 }
        }
        if abs(pose.yaw) > 15 && abs(cues.yawCue) > 0.12 {
            if (pose.yaw > 0) == (cues.yawCue > 0) { yawAgree += 1 } else { yawDisagree += 1 }
        }
        guard !warned else { return }
        if rollDisagree >= 30 && rollDisagree > 4 * rollAgree {
            warned = true
            Self.log.warning("Vision roll sign disagrees with the eye line: flip VisionPoseConvention.rollSign")
        } else if yawDisagree >= 30 && yawDisagree > 4 * yawAgree {
            warned = true
            Self.log.warning("Vision yaw sign disagrees with the nose position: flip VisionPoseConvention.yawSign")
        }
    }
}
