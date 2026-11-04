// License: Apache 2.0. See LICENSE file in root directory.
//
// Head mask construction, recommended working distance, coverage map, exposure ROI policy and screen directions.

import CameraKit
import CoreGraphics
import Foundation
import simd
import Testing
@testable import ScanKit

@Suite struct GuidanceHeadMaskTests {
    let factory = GuidanceFrameFactory()

    private func observation(roll: Float = 0, personMask: Data? = nil, chinY: Float? = nil) -> HeadObservation {
        var regions: FaceLandmarkRegions?
        if let chinY {
            let r = factory.faceRect
            regions = FaceLandmarkRegions(faceContour: [SIMD2(Float(r.minX), Float(r.midY)),
                                                        SIMD2(Float(r.midX), chinY),
                                                        SIMD2(Float(r.maxX), Float(r.midY))])
        }
        return HeadObservation(pose: HeadPose(yaw: 0, pitch: 0, roll: roll), faceRect: factory.faceRect,
                               landmarks: [], personMask: personMask, regions: regions)
    }

    private func pixel(_ mask: Data, _ x: Int, _ y: Int) -> UInt8 { mask[y * factory.width + x] }

    @Test func ellipseFallbackWithDepthBand() {
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        var parameters = HeadMaskParameters()
        parameters.erosionRadius = 0
        let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                         depth: frame.alignedDepth, faceDistanceM: 0.62, parameters: parameters)
        #expect(mask.count == factory.width * factory.height)
        let face = factory.faceRect
        #expect(pixel(mask, Int(face.midX), Int(face.midY)) == 255)            // face center
        #expect(pixel(mask, Int(face.midX), Int(face.minY) - 5) == 255)        // forehead above the face rect
        #expect(pixel(mask, 2, 2) == 0)                                          // background corner
        // Every mask pixel has depth inside the band.
        for y in 0 ..< factory.height {
            for x in 0 ..< factory.width where pixel(mask, x, y) != 0 {
                let d = frame.alignedDepth.meters(x: x, y: y)
                #expect(d >= 0.50 && d <= 0.82)
            }
        }
    }

    @Test func depthBandExcludesBackgroundInsidePersonMask() {
        // A person mask covering everything: only the depth band separates head from background.
        let everything = Data(repeating: 255, count: factory.width * factory.height)
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                         observation: observation(personMask: everything),
                                         depth: frame.alignedDepth, faceDistanceM: 0.62)
        var headPixels = 0, backgroundPixels = 0
        for y in 0 ..< factory.height {
            for x in 0 ..< factory.width where pixel(mask, x, y) != 0 {
                if factory.inHead(x, y) { headPixels += 1 } else { backgroundPixels += 1 }
            }
        }
        #expect(headPixels > 1000)
        #expect(backgroundPixels == 0)
        // Without depth the person mask (within the region of interest, above the neck cut) is used as is.
        let noDepth = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                            observation: observation(personMask: everything), depth: nil, faceDistanceM: nil)
        #expect(HeadMaskBuilder.count(noDepth) > HeadMaskBuilder.count(mask))
    }

    @Test func neckCutBelowChinUnlessShoulders() {
        let everything = Data(repeating: 255, count: factory.width * factory.height)
        let chinY: Float = 70
        var parameters = HeadMaskParameters()
        parameters.erosionRadius = 0
        let cut = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                        observation: observation(personMask: everything, chinY: chinY),
                                        depth: nil, faceDistanceM: nil, parameters: parameters)
        let cutLine = Int(chinY + Float(factory.faceRect.height) * parameters.neckMarginFraction)
        let x = Int(factory.faceRect.midX)
        #expect(pixel(cut, x, cutLine - 2) == 255)
        #expect(pixel(cut, x, cutLine + 2) == 0)
        #expect(pixel(cut, x, factory.height - 1) == 0)

        parameters.keepShoulders = true
        let shoulders = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                              observation: observation(personMask: everything, chinY: chinY),
                                              depth: nil, faceDistanceM: nil, parameters: parameters)
        #expect(pixel(shoulders, x, cutLine + 2) == 255)
        #expect(pixel(shoulders, x, factory.height - 1) == 255)
    }

    @Test func shouldersWithoutPersonMaskGrowFromTheHead() {
        // Segmentation off, failed or rejected: the ellipse covers only the head. With keepShoulders the neck and the
        // shoulders are grown from it through the depth band, but not a desk, a chair back or a headrest.
        var tall = factory
        tall.height = 160
        var depth = tall.frame(sequence: 0, timestampMs: 0, .init(body: true)).alignedDepth
        let face = tall.faceRect
        let deskRows = (tall.shoulderTop + 52) ..< 160                 // in the band, below a gap of background
        let chairBack = (110 ..< 140, 70 ..< tall.shoulderTop)        // in the band, touching the shoulders, 12 cm behind
        let headrest = (20 ..< 45, 10 ..< 60)                          // in the band, beside the head, above the chin
        let scale = depth.depthScale
        depth.pixels.withUnsafeMutableBytes { raw in
            let v = raw.bindMemory(to: UInt16.self)
            func set(_ x: Int, _ y: Int, _ meters: Float) { v[y * tall.width + x] = UInt16(meters / scale) }
            for y in deskRows { for x in 20 ..< 140 { set(x, y, 0.67) } }
            for y in chairBack.1 { for x in chairBack.0 where !tall.inShoulders(x, y) { set(x, y, 0.79) } }
            for y in headrest.1 { for x in headrest.0 { set(x, y, 0.68) } }
        }
        let obs = HeadObservation(pose: .frontal, faceRect: face, landmarks: [], personMask: nil)
        var parameters = HeadMaskParameters()
        parameters.erosionRadius = 0
        func build(_ keepShoulders: Bool, depth: DepthImage?) -> Data {
            parameters.keepShoulders = keepShoulders
            return HeadMaskBuilder.build(width: tall.width, height: tall.height, observation: obs, depth: depth,
                                         faceDistanceM: depth == nil ? nil : 0.62, parameters: parameters)
        }
        func count(_ mask: Data, _ inside: (Int, Int) -> Bool) -> (inMask: Int, total: Int) {
            var n = 0, total = 0
            for y in 0 ..< tall.height {
                for x in 0 ..< tall.width where inside(x, y) {
                    total += 1
                    if mask[y * tall.width + x] != 0 { n += 1 }
                }
            }
            return (n, total)
        }
        let below = { (_: Int, y: Int) in y > Int(face.maxY) + 6 }
        let neck = { (x: Int, y: Int) in tall.inNeck(x, y) && below(x, y) }

        let cut = build(false, depth: depth)
        #expect(count(cut, below).inMask == 0)

        let shoulders = build(true, depth: depth)
        let neckPixels = count(shoulders, neck), shoulderPixels = count(shoulders, tall.inShoulders)
        #expect(neckPixels.inMask == neckPixels.total && neckPixels.total > 300)
        #expect(shoulderPixels.inMask == shoulderPixels.total && shoulderPixels.total > 4000)
        #expect(count(shoulders, { _, y in deskRows.contains(y) }).inMask == 0)
        #expect(count(shoulders, { x, y in chairBack.0.contains(x) && chairBack.1.contains(y) && !tall.inShoulders(x, y) }).inMask == 0)
        #expect(count(shoulders, { x, y in headrest.0.contains(x) && headrest.1.contains(y) }).inMask == 0)
        // Everything below the neck cut that was added is neck or shoulders, and nothing changed above the chin.
        #expect(count(shoulders, { x, y in below(x, y) && !tall.inNeck(x, y) && !tall.inShoulders(x, y) }).inMask == 0)
        let aboveChin = { (_: Int, y: Int) in y < Int(face.maxY) }
        #expect(count(shoulders, aboveChin).inMask == count(cut, aboveChin).inMask)

        // Without depth nothing tells the body from the background: the head ellipse only.
        #expect(count(build(true, depth: nil), below).inMask == 0)
        // The default erosion keeps the shoulders.
        parameters.erosionRadius = 2
        #expect(count(build(true, depth: depth), tall.inShoulders).inMask > 3000)
    }

    @Test func neckCutFollowsRoll() {
        // Head tilted clockwise by 30 deg: the chin direction rotates toward -x, so points straight below the
        // chin but far to the right of the tilted down-axis stay, while points along the axis are cut.
        let down = HeadGeometry.downDirection(rollDegrees: 30)
        #expect(abs(down.x + 0.5) < 1e-5 && abs(down.y - 0.866) < 1e-3)
        let up = HeadGeometry.downDirection(rollDegrees: 0)
        #expect(up == SIMD2(0, 1))
        let obs = observation(roll: 30)
        let chin = obs.chinPoint
        let center = SIMD2(Float(factory.faceRect.midX), Float(factory.faceRect.midY))
        #expect(simd_distance(chin, center + down * Float(factory.faceRect.height) / 2) < 1e-3)
    }

    @Test func erosionShrinksMask() {
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        var parameters = HeadMaskParameters()
        parameters.erosionRadius = 0
        let raw = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                        depth: frame.alignedDepth, faceDistanceM: 0.62, parameters: parameters)
        parameters.erosionRadius = 2
        let eroded = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                           depth: frame.alignedDepth, faceDistanceM: 0.62, parameters: parameters)
        #expect(HeadMaskBuilder.count(eroded) < HeadMaskBuilder.count(raw))
        #expect(HeadMaskBuilder.count(eroded) > HeadMaskBuilder.count(raw) / 2)
        for i in 0 ..< eroded.count where eroded[i] != 0 { #expect(raw[i] != 0) }
    }

    @Test func erosionDoesNotWidenHolesInsideTheHead() {
        // Single pixels without depth on the face (stereo speckle): each must cost one mask pixel, not 25.
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        var holed = frame.alignedDepth
        var holes = Set<Int>()
        let face = factory.faceRect
        for y in stride(from: Int(face.minY) + 8, to: Int(face.maxY) - 8, by: 6) {
            for x in stride(from: Int(face.minX) + 8, to: Int(face.maxX) - 8, by: 6) { holes.insert(y * factory.width + x) }
        }
        holed.pixels.withUnsafeMutableBytes { raw in
            let v = raw.bindMemory(to: UInt16.self)
            for i in holes { v[i] = 0 }
        }
        #expect(holes.count >= 16)
        let intact = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                           depth: frame.alignedDepth, faceDistanceM: 0.62)
        let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                         depth: holed, faceDistanceM: 0.62)
        let differing = (0 ..< mask.count).filter { mask[$0] != (holes.contains($0) ? 0 : intact[$0]) }
        #expect(differing.isEmpty, "\(differing.count) pixels differ from the intact mask minus the holes")
        #expect(HeadMaskBuilder.count(mask) == HeadMaskBuilder.count(intact) - holes.count)
    }

    @Test func personMaskThatMissesTheFaceIsIgnored() {
        // Vision's person segmentation of a head without a body can return only the neck: the head mask must not
        // collapse to the few overlapping pixels (every frame would then fail the depth-point gate).
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        var neckOnly = Data(count: factory.width * factory.height)
        let face = factory.faceRect
        for y in Int(face.maxY) - 2 ..< factory.height {
            for x in Int(face.midX) - 10 ..< Int(face.midX) + 10 { neckOnly[y * factory.width + x] = 255 }
        }
        let ellipse = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                            depth: frame.alignedDepth, faceDistanceM: 0.62)
        let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                         observation: observation(personMask: neckOnly),
                                         depth: frame.alignedDepth, faceDistanceM: 0.62)
        #expect(mask == ellipse)
        #expect(HeadMaskBuilder.count(mask) > 1000)

        // A person mask that covers the face is used as is (here: a box around the head, smaller than the ellipse).
        var box = Data(count: factory.width * factory.height)
        for y in Int(face.minY) - 4 ..< Int(face.maxY) {
            for x in Int(face.minX) + 2 ..< Int(face.maxX) - 2 { box[y * factory.width + x] = 255 }
        }
        let boxed = HeadMaskBuilder.build(width: factory.width, height: factory.height,
                                          observation: observation(personMask: box),
                                          depth: frame.alignedDepth, faceDistanceM: 0.62)
        #expect(HeadMaskBuilder.count(boxed) > 500)
        #expect(HeadMaskBuilder.count(boxed) < HeadMaskBuilder.count(ellipse))
        for i in 0 ..< boxed.count where boxed[i] != 0 { #expect(box[i] != 0) }
    }

    @Test func depthBandBeyondTheDepthUnitDoesNotTrap() {
        // 7 m is beyond what Z16 at 0.1 mm can hold (6.55 m): the band is clamped, the mask is just empty.
        let frame = factory.frame(sequence: 0, timestampMs: 0)
        for distance: Float in [7, 1000, .greatestFiniteMagnitude] {
            let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: observation(),
                                             depth: frame.alignedDepth, faceDistanceM: distance)
            #expect(HeadMaskBuilder.count(mask) == 0)
        }
    }

    @Test func degenerateInputsGiveEmptyMasks() {
        let empty = HeadObservation(pose: .frontal, faceRect: .zero, landmarks: [], personMask: nil)
        #expect(HeadMaskBuilder.count(HeadMaskBuilder.build(width: 10, height: 10, observation: empty, depth: nil,
                                                            faceDistanceM: nil)) == 0)
        #expect(HeadMaskBuilder.build(width: 0, height: 10, observation: empty, depth: nil, faceDistanceM: nil).isEmpty)
        // A person mask of the wrong size is ignored (ellipse fallback).
        let wrongSize = observation(personMask: Data(repeating: 255, count: 7))
        let mask = HeadMaskBuilder.build(width: factory.width, height: factory.height, observation: wrongSize,
                                         depth: nil, faceDistanceM: nil)
        #expect(pixel(mask, 2, 2) == 0)
        #expect(pixel(mask, Int(factory.faceRect.midX), Int(factory.faceRect.midY)) == 255)
    }
}

@Suite struct GuidanceWorkingDistanceTests {
    private func device(name: String = "Intel RealSense D455", baseline: Float? = 95) -> DeviceInfo {
        GuidanceDevice.make(name: name, baselineMm: baseline)
    }

    @Test func minZFormula() {
        #expect(abs(WorkingDistance.minZ(depthFocalPx: 640, baselineMm: 95, disparityShift: 0) - 640 * 95 / 126 / 1000) < 1e-6)
        #expect(abs(WorkingDistance.minZ(depthFocalPx: 640, baselineMm: 95, disparityShift: 74) - 640 * 95 / 200 / 1000) < 1e-6)
        #expect(WorkingDistance.maxZ(depthFocalPx: 640, baselineMm: 95, disparityShift: 0) == nil)
        #expect(abs((WorkingDistance.maxZ(depthFocalPx: 640, baselineMm: 95, disparityShift: 50) ?? 0) - 1.216) < 1e-4)
        // D455 1280x720: 87 deg HFOV -> ~674 px.
        let f = WorkingDistance.focalPx(width: 1280, horizontalFOVDegrees: 87)
        #expect(abs(f - 674.4) < 0.5)
    }

    @Test func d455DefaultRange() {
        let range = WorkingDistance.recommendedRange(device: device(), configuration: .faceScanD455)
        // Min-Z ~0.51 m -> ~0.56 ... 0.71 m.
        #expect(range.lowerBound >= 0.53 && range.lowerBound <= 0.58)
        #expect(range.upperBound >= 0.68 && range.upperBound <= 0.75)
        // Without any information the same defaults apply.
        #expect(WorkingDistance.recommendedRange(device: nil, configuration: nil) == range)
    }

    @Test func baselineInTheWrongUnitFallsBackToTheModel() {
        // librealsense 2.58 played a D455 recording back with a baseline of 0.095 "mm" (meters): the range came out
        // as 0.22-0.27 m and the offline scan never left positioning at 0.64 m.
        let range = WorkingDistance.recommendedRange(device: device(), configuration: .faceScanD455)
        #expect(WorkingDistance.recommendedRange(device: device(baseline: 0.095), configuration: .faceScanD455) == range)
        #expect(WorkingDistance.recommendedRange(device: device(baseline: 95_000), configuration: .faceScanD455) == range)
        #expect(WorkingDistance.recommendedRange(device: device(baseline: 94.95), configuration: .faceScanD455) == range)
    }

    @Test func lowerResolutionAndDisparityShiftMoveCloser() {
        var vga = CaptureConfiguration.faceScanD455
        vga.depth = StreamSettings(width: 848, height: 480, fps: 30)
        let base = WorkingDistance.recommendedRange(device: device(), configuration: .faceScanD455)
        let small = WorkingDistance.recommendedRange(device: device(), configuration: vga)
        #expect(small.lowerBound < base.lowerBound - 0.1)

        var shifted = CaptureConfiguration.faceScanD455
        shifted.disparityShift = 100
        let close = WorkingDistance.recommendedRange(device: device(), configuration: shifted)
        let f = WorkingDistance.focalPx(width: 1280, horizontalFOVDegrees: 87)
        let minZ = WorkingDistance.minZ(depthFocalPx: f, baselineMm: 95, disparityShift: 100)
        #expect(abs(close.lowerBound - (minZ + 0.05)) < 0.006)
        #expect(close.upperBound < base.lowerBound)

        // A large shift caps the range inside the measurable [Min-Z, Max-Z].
        shifted.disparityShift = 400
        shifted.filters.threshold.enabled = false
        let veryClose = WorkingDistance.recommendedRange(device: device(), configuration: shifted)
        let nearZ = WorkingDistance.minZ(depthFocalPx: f, baselineMm: 95, disparityShift: 400)
        let maxZ = WorkingDistance.maxZ(depthFocalPx: f, baselineMm: 95, disparityShift: 400) ?? 0
        #expect(veryClose.upperBound < maxZ)
        #expect(veryClose.lowerBound > nearZ)
        #expect(veryClose.upperBound > veryClose.lowerBound)

        // Contradictory: the threshold filter starts beyond Max-Z. Still a valid range, inside the geometry.
        shifted.filters.threshold = .init(enabled: true, minM: 0.5, maxM: 1.2)
        let contradictory = WorkingDistance.recommendedRange(device: device(), configuration: shifted)
        #expect(contradictory.lowerBound >= nearZ - 0.01 && contradictory.upperBound <= maxZ + 0.01)
    }

    @Test func thresholdFilterAndDeviceDataLimitTheRange() {
        var capture = CaptureConfiguration.faceScanD455
        capture.filters.threshold = .init(enabled: true, minM: 0.2, maxM: 0.65)
        let limited = WorkingDistance.recommendedRange(device: device(), configuration: capture)
        #expect(limited.upperBound <= 0.60 + 1e-6)

        // D435 (50 mm baseline) works much closer.
        let d435 = WorkingDistance.recommendedRange(device: device(name: "Intel RealSense D435", baseline: nil),
                                                    configuration: .faceScanD455)
        #expect(d435.lowerBound < 0.35)
        // A reported baseline wins over the model default.
        let measured = WorkingDistance.recommendedRange(device: device(baseline: 50), configuration: .faceScanD455)
        #expect(measured == d435)
        // Real depth intrinsics win over the nominal field of view.
        let k = Intrinsics(width: 640, height: 360, fx: 320, fy: 320, ppx: 320, ppy: 180)
        let fromIntrinsics = WorkingDistance.recommendedRange(device: device(), configuration: .faceScanD455, depthIntrinsics: k)
        let minZ = WorkingDistance.minZ(depthFocalPx: 640, baselineMm: 95, disparityShift: 0)
        #expect(abs(fromIntrinsics.lowerBound - (minZ + 0.05)) < 0.006)
    }

    @Test func configurationHelper() {
        var capture = CaptureConfiguration.faceScanD455
        capture.disparityShift = 50
        let c = ScanConfiguration().withRecommendedDistance(device: device(), captureConfiguration: capture)
        #expect(c.distanceRangeM == WorkingDistance.recommendedRange(device: device(), configuration: capture))
        #expect(ScanConfiguration().minValidDepthPoints(width: 640, height: 400) == 3000)
        #expect(ScanConfiguration().minValidDepthPoints(width: 1280, height: 800) == 12_000)
    }
}

@Suite struct GuidanceTypesTests {
    @Test func screenDirectionHonorsMirroring() {
        // Pose "left" (person turns to THEIR left): the nose moves toward the image right.
        let raw = ScanGuidance.screenDirection(yawDelta: 45, pitchDelta: 0, mirrored: false)
        #expect(raw == SIMD2(1, 0))
        let mirrored = ScanGuidance.screenDirection(yawDelta: 45, pitchDelta: 0, mirrored: true)
        #expect(mirrored == SIMD2(-1, 0))
        let up = ScanGuidance.screenDirection(yawDelta: 0, pitchDelta: 25, mirrored: true)
        #expect(up == SIMD2(0, -1))
        #expect(ScanGuidance.screenDirection(yawDelta: 0, pitchDelta: 0, mirrored: false) == .zero)
    }

    @Test func coveredRegionIsOppositeToTurn() {
        let p = ScanGuidance.coveredRegionPosition(yaw: 45, pitch: 0, mirrored: false)
        #expect(p.x < 0)
        #expect(ScanGuidance.coveredRegionPosition(yaw: 45, pitch: 0, mirrored: true).x > 0)
        #expect(ScanGuidance.coveredRegionPosition(yaw: 0, pitch: 25, mirrored: false).y > 0)
    }

    @Test func coverageBins() {
        #expect(CoverageMap.bin(yaw: -90, pitch: -60) == (0, 0))
        #expect(CoverageMap.bin(yaw: 90, pitch: 60) == (CoverageMap.yawBins - 1, CoverageMap.pitchBins - 1))
        #expect(CoverageMap.bin(yaw: 0.1, pitch: 0.1) == (6, 4))
        let c = CoverageMap.binCenter(yawBin: 6, pitchBin: 4)
        #expect(c.yaw == 7.5 && c.pitch == 7.5)
        #expect(CoverageMap().values.allSatisfy { $0 == 0 })
        let one = CoverageMap(keyframePoses: [HeadPose(yaw: 7.5, pitch: 7.5, roll: 0)], framesPerPose: 1)
        #expect(abs(one.value(yawBin: 6, pitchBin: 4) - (1 - exp(-3))) < 1e-4)
        #expect(one.value(yawBin: 0, pitchBin: 0) < 1e-3)
        #expect(one.values.allSatisfy { $0 >= 0 && $0 <= 1 })
    }

    @Test func exposureROIIsThrottled() throws {
        var policy = ExposureROIPolicy(minInterval: 1.0, maxOverlap: 0.7, expansion: 1.2)
        let size = CGSize(width: 1280, height: 800)
        let face = CGRect(x: 500, y: 200, width: 200, height: 250)
        let firstROI = policy.roi(for: face, imageSize: size, now: 10)
        let first = try #require(firstROI)
        #expect(first.contains(face))
        #expect(policy.roi(for: face.offsetBy(dx: 150, dy: 0), imageSize: size, now: 10.5) == nil)   // too soon
        #expect(policy.roi(for: face.offsetBy(dx: 5, dy: 0), imageSize: size, now: 12) == nil)       // barely moved
        #expect(policy.roi(for: face.offsetBy(dx: 150, dy: 0), imageSize: size, now: 12) != nil)
        #expect(policy.roi(for: nil, imageSize: size, now: 20) == nil)
        // Clipped to the image.
        policy.reset()
        let edgeROI = policy.roi(for: CGRect(x: 1200, y: 700, width: 200, height: 200), imageSize: size, now: 30)
        let edge = try #require(edgeROI)
        #expect(edge.maxX <= 1280 && edge.maxY <= 800)
    }

    @Test func instructionTextsExist() {
        #expect(!ScanInstruction.turn(.left, yawDelta: 10, pitchDelta: 0).englishText.isEmpty)
        #expect(ScanInstruction.turnAnyDirection.englishText == "Turn your head in any direction")
        for r in ScanRejection.allCases { #expect(!r.englishText.isEmpty) }
        for p in ScanPose.allCases { #expect(!p.englishName.isEmpty) }
    }
}
