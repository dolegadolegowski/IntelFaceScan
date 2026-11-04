// License: Apache 2.0. See LICENSE file in root directory.
//
// Lens distortion: the librealsense projection math (golden values from librealsense 2.58.4 itself), the one-time
// keyframe undistortion against a pinhole render of the same view, and the pipeline on an off-center synthetic
// scan rendered through the D455 color distortion (docs/RECONSTRUCTION.md, "Lens distortion").

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

@Suite("Recon lens distortion")
struct ReconLensDistortionTests {
    /// rs2_project_point_to_pixel and rs2_deproject_pixel_to_point of the helper's SDK build (librealsense 2.58.4,
    /// -ffp-contract=off) for fx 640.1, fy 639.8, ppx 641.2, ppy 401.3 and the D455 coefficients.
    static let k = Intrinsics(width: 1280, height: 800, fx: 640.1, fy: 639.8, ppx: 641.2, ppy: 401.3)
    static let points: [SIMD3<Float>] = [SIMD3(0.55, 0.02, 1), SIMD3(0.2, -0.3, 0.8), SIMD3(-0.4, 0.31, 0.6), SIMD3(0, 0, 1)]
    static let golden: [LensDistortion.Model: [SIMD2<Float>]] = [
        .inverseBrownConrady: [SIMD2(989.617554, 413.843262), SIMD2(800.026855, 163.210342), SIMD2(221.010773, 726.732544),
                               SIMD2(641.200012, 401.299988)],
        .brownConrady: [SIMD2(989.622559, 413.843445), SIMD2(800.028931, 163.207214), SIMD2(221.026855, 726.720154),
                        SIMD2(641.200012, 401.299988)],
        .modifiedBrownConrady: [SIMD2(989.617554, 413.843262), SIMD2(800.026855, 163.210342), SIMD2(221.010773, 726.732544),
                                SIMD2(641.200012, 401.299988)],
    ]

    @Test func projectionMatchesLibrealsense() {
        for (model, pixels) in Self.golden {
            let lens = LensDistortion(model: model, coeffs: LensDistortion.d455Color.coeffs)
            for (p, expected) in zip(Self.points, pixels) {
                let pixel = lens.project(p, Self.k)
                #expect(simd_length(pixel - expected) < 2e-4, "\(model) \(p): \(pixel) vs \(expected)")
                // librealsense deprojects with 10 fixed-point iterations; modified Brown-Conrady has no inverse there
                // (it returns the pinhole ray), ours inverts the same forward formula.
                let back = lens.deproject(pixel, depth: p.z, Self.k)
                #expect(simd_length(back - p) < 1e-6, "\(model) \(p): back to \(back)")
            }
        }
        #expect(LensDistortion(model: .inverseBrownConrady, coeffs: [0, 0, 0, 0, 0]).isIdentity)
        #expect(!LensDistortion.d455Color.isIdentity)
        #expect(LensDistortion(Intrinsics(width: 4, height: 4, fx: 1, fy: 1, ppx: 2, ppy: 2, model: "kb5")) == nil)
        // librealsense's SIMD align (NEON on Apple Silicon) follows the color distortion for modified Brown-Conrady only.
        #expect(!LensDistortion.d455Color.alignedDepthFollowsDistortion)
        #expect(LensDistortion(model: .modifiedBrownConrady, coeffs: LensDistortion.d455Color.coeffs).alignedDepthFollowsDistortion)
    }

    /// One off-center view rendered through a lens and as a pinhole, noise-free.
    static func views(_ lens: LensDistortion, offset: SIMD2<Float> = SIMD2(0.16, 0.08))
        -> (distorted: Keyframe, pinhole: Keyframe, scene: SyntheticScene) {
        var o = SyntheticScan.Options()
        o.width = 1280
        o.height = 800
        o.depthNoiseAt1m = 0
        o.dropoutRate = 0
        o.colorNoise = 0
        o.meshResolution = 96
        o.headOffsetM = offset
        var d = o
        d.lensDistortion = lens
        let pinholeScene = SyntheticScene(options: o), scene = SyntheticScene(options: d)
        let pose = scene.cameraFromWorld(headPose: HeadPose(yaw: 20, pitch: 5, roll: 0))
        func keyframe(_ s: SyntheticScene) -> Keyframe {
            let view = s.render(cameraFromWorld: pose, frameIndex: 1)
            let landmarks = s.landmarks(cameraFromWorld: pose)
            return Keyframe(id: 0, frame: view.frame, scanPose: .front, headPose: .frontal,
                            faceRect: CGRect(x: 700, y: 300, width: 300, height: 360), landmarks: landmarks, mask: view.mask,
                            quality: FrameQuality(sharpness: 1, angularSpeedDegPerSec: 0, validDepthRatio: 1, validDepthPoints: 1,
                                                  faceDistanceM: 0.6))
        }
        return (keyframe(scene), keyframe(pinholeScene), scene)
    }

    static func depthValues(_ kf: Keyframe) -> [UInt16] {
        kf.frame.alignedDepth.pixels.withUnsafeBytes { Array($0.bindMemory(to: UInt16.self)) }
    }

    /// Mean absolute color difference over pixels where both views see the head (mask) and the face is textured.
    static func colorDifference(_ a: Keyframe, _ b: Keyframe) -> Double {
        let ca = [UInt8](a.frame.color.pixels), cb = [UInt8](b.frame.color.pixels)
        let ma = [UInt8](a.mask!), mb = [UInt8](b.mask!)
        var sum = 0.0, n = 0
        for i in 0 ..< ma.count where ma[i] != 0 && mb[i] != 0 {
            for c in 0 ..< 3 { sum += abs(Double(ca[3 * i + c]) - Double(cb[3 * i + c])) }
            n += 3
        }
        return sum / Double(max(n, 1))
    }

    @Test func undistortedKeyframeMatchesThePinholeRender() throws {
        // The D455 case: color, mask and landmarks distorted, aligned depth in the pinhole geometry (NEON align).
        let (distorted, pinhole, _) = Self.views(.d455Color)
        let k = distorted.frame.color.intrinsics
        #expect(k.model == "inverse_brown_conrady" && distorted.frame.alignedDepth.intrinsics.model == "inverse_brown_conrady")
        #expect(Self.depthValues(distorted) == Self.depthValues(pinhole))
        let map = KeyframeUndistortion.Map(intrinsics: k, lens: .d455Color)
        let fixed = KeyframeUndistortion.undistort(distorted, map: map, lens: .d455Color, depthFollowsDistortion: false)
        #expect(fixed.frame.color.intrinsics == pinhole.frame.color.intrinsics.pinhole)
        #expect(fixed.frame.alignedDepth.intrinsics.model == "none")
        #expect(Self.depthValues(fixed) == Self.depthValues(pinhole))

        // Landmarks: the distortion moves this off-center face's landmarks by up to ~2 px; undistorted, they are the
        // pinhole projections.
        let before = zip(distorted.landmarks, pinhole.landmarks).map { simd_length($0.0 - $0.1) }
        let after = zip(fixed.landmarks, pinhole.landmarks).map { simd_length($0.0 - $0.1) }
        print("lens: landmark error before \(before.max()!) px, after \(after.max()!) px")
        #expect(before.max()! > 1)
        #expect(after.max()! < 0.01)

        // Color and mask: bilinear resampling blurs slightly, but the image now lines up with the depth.
        let colorBefore = Self.colorDifference(distorted, pinhole), colorAfter = Self.colorDifference(fixed, pinhole)
        let maskA = [UInt8](fixed.mask!), maskB = [UInt8](pinhole.mask!), maskC = [UInt8](distorted.mask!)
        let maskMismatchAfter = zip(maskA, maskB).filter { $0.0 != $0.1 }.count
        let maskMismatchBefore = zip(maskC, maskB).filter { $0.0 != $0.1 }.count
        print("lens: color difference before \(colorBefore), after \(colorAfter); mask mismatch before \(maskMismatchBefore), after \(maskMismatchAfter)")
        #expect(colorAfter < 0.6 * colorBefore)
        #expect(maskMismatchAfter * 3 < maskMismatchBefore)
    }

    @Test func distortedAlignedDepthIsRemappedWithoutMixingDepths() throws {
        // Modified Brown-Conrady (same forward formula): librealsense's align follows it, so the aligned depth is
        // distorted like the color image and is remapped with nearest neighbors.
        let lens = LensDistortion(model: .modifiedBrownConrady, coeffs: LensDistortion.d455Color.coeffs)
        let (distorted, pinhole, _) = Self.views(lens)
        let original = Self.depthValues(distorted), truth = Self.depthValues(pinhole)
        let w = distorted.frame.color.width, h = distorted.frame.color.height
        let map = KeyframeUndistortion.Map(intrinsics: distorted.frame.color.intrinsics, lens: lens)
        let fixed = Self.depthValues(KeyframeUndistortion.undistort(distorted, map: map, lens: lens, depthFollowsDistortion: true))
        // Where the source neighborhood spans two surfaces (head and wall, > 2 cm apart), the output is one of the
        // source depths, never a value between the surfaces ...
        var silhouettes = 0, mixed = 0
        for i in fixed.indices {
            let u = map.sourceX[i].rounded(.down), v = map.sourceY[i].rounded(.down)
            guard u >= 0, v >= 0, u + 1 < Float(w), v + 1 < Float(h) else { continue }
            let j = Int(v) * w + Int(u)
            let neighbors = [original[j], original[j + 1], original[j + w], original[j + w + 1]]
            guard neighbors.max()! - neighbors.min()! > 200 else { continue }
            silhouettes += 1
            if !neighbors.contains(fixed[i]) { mixed += 1 }
        }
        #expect(silhouettes > 500 && mixed == 0, "\(mixed) of \(silhouettes) silhouette pixels mix depths")
        // ... and elsewhere it agrees with the pinhole render.
        func agreement(_ a: [UInt16]) -> (Double, Double) {
            var close = 0, valid = 0, sum = 0.0
            for i in truth.indices where truth[i] != 0 && a[i] != 0 && truth[i] < 9000 {
                valid += 1
                let d = abs(Int(a[i]) - Int(truth[i]))
                if d <= 10 { close += 1 }   // 1 mm
                sum += Double(d) * 0.1
            }
            return (Double(close) / Double(max(valid, 1)), sum / Double(max(valid, 1)))
        }
        let (closeBefore, meanBefore) = agreement(original), (closeAfter, meanAfter) = agreement(fixed)
        print("lens: depth within 1 mm of the pinhole render before \(closeBefore) (mean \(meanBefore) mm), after \(closeAfter) "
              + "(mean \(meanAfter) mm); \(silhouettes) silhouette pixels")
        #expect(closeAfter > 0.97)
        #expect(meanAfter < 0.5 * meanBefore)
    }
}

@Suite("Recon lens distortion pipeline", .enabled(if: CoreAvailability.reconstruction, "the core is a placeholder"),
       .timeLimit(.minutes(30)), .heavyReconstruction)
struct ReconLensDistortionPipelineTests {
    static let offCenter = SIMD2<Float>(0.15, 0.06)

    static func scenario(_ name: String, _ lens: LensDistortion?, offset: SIMD2<Float> = offCenter) -> ScanImperfections {
        var p = ScanImperfections()
        p.name = name
        p.lensDistortion = lens
        p.headOffsetM = offset
        return p
    }

    /// A head 15 cm right and 6 cm below the optical axis whose aligned depth is distorted like the color image
    /// (modified Brown-Conrady, which librealsense's align follows; same coefficients as the D455 inverse
    /// Brown-Conrady): undistorted, the reconstruction is as accurate as from a pinhole camera; read as a pinhole, the
    /// views are warped differently and do not fit together.
    @Test func distortedScanOffCenterReconstructsLikeAPinholeScan() async throws {
        var p = Self.scenario("depth distorted too, off-center", LensDistortion(model: .modifiedBrownConrady,
                                                                                 coeffs: LensDistortion.d455Color.coeffs))
        p.width = 640
        p.height = 400
        let undistorted = try await RobustnessStudy.run(p)
        let ignored = ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1" ? try await RobustnessStudy.run(p, undistortLens: false) : nil
        print("lens pipeline: undistorted " + RobustnessStudy.row(p, "point-to-plane", undistorted)
              + (ignored.map { " ignored " + RobustnessStudy.row(p, "point-to-plane", $0) } ?? "") + " notes \(undistorted.notes)")
        #expect(undistorted.notes.contains("lens distortion removed: modified_brown_conrady"))
        #expect(undistorted.usedFrames == undistorted.keyframes)
        // 640x400, seed 1: undistorted 0.08 / 0.20 deg, head misalignment 0.41 mm, surface 0.18 / 0.52 mm; ignored
        // 0.19 / 0.54 deg, 1.81 mm, 0.54 / 1.66 mm.
        #expect(undistorted.maxRotationDeg < 0.35)
        #expect(undistorted.maxMisalignmentMm < 1)
        #expect(undistorted.surfaceMeanMm < 0.3)
    }

    /// IFS_WRITE_LENS_DATASETS=<folder>: off-center synthetic datasets rendered through the lens, with ground truth, for
    /// `ifscan reconstruct` (docs/RECONSTRUCTION.md, "Lens distortion": the end-to-end comparison).
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_WRITE_LENS_DATASETS"] != nil, "set IFS_WRITE_LENS_DATASETS"))
    func writeLensDatasets() throws {
        let folder = URL(fileURLWithPath: try #require(ProcessInfo.processInfo.environment["IFS_WRITE_LENS_DATASETS"]))
        for (name, lens, offset, poses) in [
            ("d455-offcenter", LensDistortion.d455Color, SIMD2<Float>(0.10, 0.05), ScanPose.allCases),
            ("modified-offcenter", LensDistortion(model: .modifiedBrownConrady, coeffs: LensDistortion.d455Color.coeffs),
             SIMD2<Float>(0.15, 0.06), ScanPose.required),
        ] {
            var o = SyntheticScan.Options()
            o.width = 1280
            o.height = 800
            o.lensDistortion = lens
            o.headOffsetM = offset
            o.poses = poses
            let result = SyntheticScan.make(o)
            let url = folder.appendingPathComponent("\(name).ifsdataset")
            try ScanDataset.write(result.capture, to: url)
            try SyntheticGroundTruth(result: result, options: o).write(toDataset: url, mesh: result.groundTruthMesh)
        }
    }

    /// IFS_RECON_STUDY=1: the D455 distortion with and without undistortion, centered and off-center.
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func lensDistortionTable() async throws {
        let modified = LensDistortion(model: .modifiedBrownConrady, coeffs: LensDistortion.d455Color.coeffs)
        let runs: [(ScanImperfections, Bool, String)] = [
            (Self.scenario("pinhole, centered", nil, offset: .zero), true, "-"),
            (Self.scenario("pinhole, off-center", nil), true, "-"),
            (Self.scenario("D455 color distortion, centered", .d455Color, offset: .zero), true, "yes"),
            (Self.scenario("D455 color distortion, centered", .d455Color, offset: .zero), false, "no"),
            (Self.scenario("D455 color distortion, off-center", .d455Color), true, "yes"),
            (Self.scenario("D455 color distortion, off-center", .d455Color), false, "no"),
            (Self.scenario("depth distorted too, off-center", modified), true, "yes"),
            (Self.scenario("depth distorted too, off-center", modified), false, "no"),
        ]
        var rows = ["| scan | undistorted | rotation error mean / max | head misalignment max | surface mean / p95 | frames |",
                    "|---|---|---|---|---|---|"]
        for (p, undistort, label) in runs {
            let o = try await RobustnessStudy.run(p, undistortLens: undistort)
            rows.append(String(format: "| %@ | %@ | %.3f° / %.3f° | %.2f mm | %.3f / %.3f mm | %d/%d |", p.name, label, o.meanRotationDeg,
                               o.maxRotationDeg, o.maxMisalignmentMm, o.surfaceMeanMm, o.surfaceP95Mm, o.usedFrames, o.keyframes))
        }
        print("lens table:\n" + rows.joined(separator: "\n"))
    }
}
