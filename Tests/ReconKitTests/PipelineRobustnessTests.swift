// License: Apache 2.0. See LICENSE file in root directory.
//
// The pipeline on the textured SyntheticData head with the imperfections real D455 scans have and the synthetic
// renderer does not model: spatially correlated depth noise, missing depth on hair and at silhouettes, larger Vision
// pose errors, auto-exposure changes between keyframes, and a capture whose first keyframe is not frontal.
// The default test runs the combined scenario; IFS_RECON_STUDY=1 prints the whole table (docs/RECONSTRUCTION.md,
// "Robustness").

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

struct ScanImperfections: Sendable {
    var name = "clean"
    /// Correlated depth noise: sigma at 1 m (scaled with z^2) and correlation radius (pixels).
    var correlatedNoiseAt1m: Float = 0
    var correlationPx = 4
    /// Low-frequency depth waviness, sigma at 1 m (correlation radius 24 px).
    var wavinessAt1m: Float = 0
    /// Dark pixels (hair, brows, pupils) lose depth with this probability; the rest gets 2 mm of extra noise.
    var hairDropout: Float = 0
    /// Head pixels within this many pixels of the silhouette (no depth) lose their depth.
    var silhouetteErosionPx = 0
    /// Per-keyframe exposure gain drawn from [1 - j, 1 + j].
    var gainJitter: Float = 0
    /// Isolated invalid depth pixels (SyntheticScan default 1%).
    var speckleRate: Float = 0.01
    /// Moves the frontal keyframes to the end of the capture.
    var frontalLast = false
    /// Vision head pose error (degrees, standard deviation per angle; SyntheticScan default 2).
    var visionErrorDeg: Float = 2
    var lighting: Float = 0.25
    /// Color (= aligned depth) resolution.
    var width = 1280
    var height = 800
    /// Face distance and white depth noise (sigma at 1 m) of the renderer (SyntheticScan defaults 0.6 m, 2 mm).
    var distanceM: Float = 0.6
    var depthNoiseAt1m: Float = 0.002
    /// Color lens distortion (SyntheticScan.Options.lensDistortion) and head position off the optical axis.
    var lensDistortion: LensDistortion?
    var headOffsetM: SIMD2<Float> = .zero
    /// Line-of-sight gain of the simulated Vision pose (nil = SyntheticScan's default, measured on Apple Vision).
    var visionLineOfSightGain: SIMD2<Float>?

    static let realistic: ScanImperfections = {
        var p = ScanImperfections()
        p.name = "all combined"
        p.correlatedNoiseAt1m = 0.0014   // 0.5 mm at 0.6 m on top of the renderer's 0.72 mm white noise
        p.wavinessAt1m = 0.0008          // 0.3 mm at 0.6 m
        p.hairDropout = 0.8
        p.silhouetteErosionPx = 6
        p.gainJitter = 0.2
        p.frontalLast = true
        p.visionErrorDeg = 3
        p.speckleRate = 0.05
        return p
    }()
}

struct RobustnessOutcome: Sendable {
    var meanRotationDeg: Float
    var maxRotationDeg: Float
    var maxMisalignmentMm: Float
    var surfaceMeanMm: Double
    var surfaceP95Mm: Double
    var usedFrames: Int
    var keyframes: Int
    var notes: [String]
}

enum RobustnessStudy {
    /// Deterministic standard normal from a counter (no shared RNG state between pixels).
    static func gaussian(_ seed: UInt64, _ index: UInt64) -> Float {
        func mix(_ x: UInt64) -> UInt64 {
            var z = x &+ 0x9E37_79B9_7F4A_7C15
            z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
            z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
            return z ^ (z >> 31)
        }
        let a = mix(seed ^ (index &* 2)), b = mix(seed ^ (index &* 2 &+ 1))
        let u1 = max(Double(a >> 11) / 9_007_199_254_740_992.0, 1e-300), u2 = Double(b >> 11) / 9_007_199_254_740_992.0
        return Float((-2 * log(u1)).squareRoot() * cos(2 * .pi * u2))
    }

    /// Deterministic uniform in [0, 1).
    static func uniform(_ seed: UInt64, _ index: UInt64) -> Float {
        var z = (seed ^ 0x5DEE_CE66) &+ index &* 0x9E37_79B9_7F4A_7C15
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return Float((z ^ (z >> 31)) >> 40) / Float(1 << 24)
    }

    /// Unit-variance noise field with the given correlation radius (three box blurs ~ Gaussian).
    static func correlatedField(width w: Int, height h: Int, radius: Int, seed: UInt64) -> [Float] {
        var field = (0 ..< w * h).map { gaussian(seed, UInt64($0)) }
        guard radius > 0 else { return field }
        var tmp = field
        for _ in 0 ..< 3 {
            for y in 0 ..< h {   // horizontal running mean
                var sum: Float = 0
                for x in -radius ... radius { sum += field[y * w + min(max(x, 0), w - 1)] }
                for x in 0 ..< w {
                    tmp[y * w + x] = sum / Float(2 * radius + 1)
                    sum += field[y * w + min(x + radius + 1, w - 1)] - field[y * w + max(x - radius, 0)]
                }
            }
            for x in 0 ..< w {   // vertical running mean
                var sum: Float = 0
                for y in -radius ... radius { sum += tmp[min(max(y, 0), h - 1) * w + x] }
                for y in 0 ..< h {
                    field[y * w + x] = sum / Float(2 * radius + 1)
                    sum += tmp[min(y + radius + 1, h - 1) * w + x] - tmp[max(y - radius, 0) * w + x]
                }
            }
        }
        let variance = field.reduce(0) { $0 + $1 * $1 } / Float(field.count)
        let s = 1 / max(variance.squareRoot(), 1e-12)
        return field.map { $0 * s }
    }

    static func perturb(_ kf: Keyframe, _ p: ScanImperfections, seed: UInt64) -> Keyframe {
        var kf = kf
        let w = kf.frame.alignedDepth.width, h = kf.frame.alignedDepth.height
        let scale = kf.frame.alignedDepth.depthScale
        var depth: [UInt16] = kf.frame.alignedDepth.pixels.withUnsafeBytes { raw in
            (0 ..< w * h).map { UInt16(littleEndian: raw.loadUnaligned(fromByteOffset: 2 * $0, as: UInt16.self)) }
        }
        var rgb = [UInt8](kf.frame.color.pixels)
        let original = depth
        let fine = p.correlatedNoiseAt1m > 0 ? correlatedField(width: w, height: h, radius: p.correlationPx, seed: seed &+ 1) : []
        let coarse = p.wavinessAt1m > 0 ? correlatedField(width: w, height: h, radius: 24, seed: seed &+ 2) : []
        for i in 0 ..< w * h where original[i] != 0 {
            var z = Float(original[i]) * scale
            let z2 = z * z
            if !fine.isEmpty { z += p.correlatedNoiseAt1m * z2 * fine[i] }
            if !coarse.isEmpty { z += p.wavinessAt1m * z2 * coarse[i] }
            if p.hairDropout > 0 {
                let luminance = (0.299 * Float(rgb[3 * i]) + 0.587 * Float(rgb[3 * i + 1]) + 0.114 * Float(rgb[3 * i + 2])) / 255
                if luminance < 0.3 {
                    let u = uniform(seed &+ 3, UInt64(i))
                    if u < p.hairDropout { depth[i] = 0; continue }
                    z += 0.002 * gaussian(seed &+ 4, UInt64(i))
                }
            }
            depth[i] = UInt16(clamping: Int((z / scale).rounded()))
        }
        if p.silhouetteErosionPx > 0 {
            // Distance (in pixels, square neighborhood) to the nearest pixel outside the head (the renderer's head mask,
            // or no depth in a whole 5-pixel run: not the isolated speckle holes).
            let r = p.silhouetteErosionPx
            let mask = kf.mask.map { [UInt8]($0) }
            func outside(_ x: Int, _ y: Int) -> Bool {
                if let mask, mask[y * w + x] < 128 { return true }
                for dx in -2 ... 2 where x + dx >= 0 && x + dx < w && original[y * w + x + dx] != 0 { return false }
                return true
            }
            var near = [Bool](repeating: false, count: w * h)
            for y in 0 ..< h {
                var last = -1_000_000
                for x in 0 ..< w { if outside(x, y) { last = x }; near[y * w + x] = x - last <= r }
                var next = 1_000_000
                for x in stride(from: w - 1, through: 0, by: -1) { if outside(x, y) { next = x }; if next - x <= r { near[y * w + x] = true } }
            }
            var near2 = near
            for x in 0 ..< w {
                var last = -1_000_000
                for y in 0 ..< h { if near[y * w + x] { last = y }; if y - last <= r { near2[y * w + x] = true } }
                var next = 1_000_000
                for y in stride(from: h - 1, through: 0, by: -1) { if near[y * w + x] { next = y }; if next - y <= r { near2[y * w + x] = true } }
            }
            for i in 0 ..< w * h where near2[i] { depth[i] = 0 }
        }
        if p.gainJitter > 0 {
            let u = uniform(seed &+ 5, 0)
            let gain = 1 + p.gainJitter * (2 * u - 1)
            for i in rgb.indices { rgb[i] = UInt8(clamping: Int((Float(rgb[i]) * gain).rounded())) }
        }
        kf.frame.alignedDepth.pixels = depth.withUnsafeBufferPointer { Data(buffer: $0) }
        kf.frame.color.pixels = Data(rgb)
        return kf
    }

    static func scan(_ p: ScanImperfections, seed: UInt64 = 1) -> SyntheticScanResult {
        var options = SyntheticScan.Options()
        options.width = p.width
        options.height = p.height
        options.seed = seed
        options.headPoseErrorDeg = p.visionErrorDeg
        options.lighting = p.lighting
        options.dropoutRate = p.speckleRate
        options.distanceM = p.distanceM
        options.depthNoiseAt1m = p.depthNoiseAt1m
        options.lensDistortion = p.lensDistortion
        options.headOffsetM = p.headOffsetM
        if let gain = p.visionLineOfSightGain { options.visionLineOfSightGain = gain }
        var result = SyntheticScan.make(options)
        var keyframes = result.capture.keyframes.enumerated().map { perturb($0.element, p, seed: seed &* 1000 &+ UInt64($0.offset)) }
        var truth = result.groundTruthPoses
        if p.frontalLast {
            let order = keyframes.indices.filter { keyframes[$0].scanPose != .front } + keyframes.indices.filter { keyframes[$0].scanPose == .front }
            keyframes = order.map { keyframes[$0] }
            truth = order.map { truth[$0] }
        }
        result.capture.keyframes = keyframes
        result.groundTruthPoses = truth
        return result
    }

    static var options: ReconstructionOptions {
        var options = ReconstructionOptions()
        options.targetTriangles = 60_000
        options.textureSize = 512
        return options
    }

    /// - Parameter undistortLens: false skips the pipeline's lens undistortion (to measure what ignoring it costs).
    static func run(_ p: ScanImperfections, coloredICP: Bool = false, seed: UInt64 = 1, undistortLens: Bool = true,
                    configure: (inout ReconstructionOptions) -> Void = { _ in }) async throws -> RobustnessOutcome {
        let scan = scan(p, seed: seed)
        var options = options
        options.useColoredICP = coloredICP
        configure(&options)
        let result = try await ReconstructionPipeline(options: options, undistortLens: undistortLens).run(scan.capture)
        // Used keyframes, in capture order (ReconstructionResult.cameraPoses); excluded ones are named in the notes.
        let excluded = Set(result.report.notes.filter { $0.hasPrefix("keyframes excluded") }
            .flatMap { $0.split(separator: "#").dropFirst().compactMap { Int($0.prefix { $0.isNumber }) } })
        let used = scan.capture.keyframes.indices.filter { !excluded.contains(scan.capture.keyframes[$0].id) }
        let truth = used.map { scan.groundTruthPoses[$0] }
        // Anchor: the reference keyframe (identity estimate). Errors are relative to it; the head center goes into
        // its true camera frame (SyntheticScan's world is the camera frame of the original keyframe 0).
        let anchor = result.cameraPoses.indices.min { distanceToIdentity(result.cameraPoses[$0]) < distanceToIdentity(result.cameraPoses[$1]) } ?? 0
        let estimated = [result.cameraPoses[anchor]] + result.cameraPoses
        let groundTruth = [truth[anchor]] + truth
        let errors = MeshEvaluation.relativePoseErrors(estimated: estimated, groundTruth: groundTruth).dropFirst().map(\.rotationDeg)
        let c = truth[anchor].inverse * SIMD4(scan.headCenter, 1)
        let misalignment = MeshEvaluation.relativeHeadMisalignment(estimated: estimated, groundTruth: groundTruth,
                                                                   center: SIMD3(c.x, c.y, c.z)).dropFirst()
        // Surface: the mesh lives in the reference camera frame; map it into the ground truth's world.
        var mesh = result.mesh
        let toTruth = truth[anchor] * result.cameraPoses[anchor].inverse
        mesh.positions = mesh.positions.map { let q = toTruth * SIMD4($0, 1); return SIMD3(q.x, q.y, q.z) }
        let distance = try MeshEvaluation.distance(from: mesh, to: scan.groundTruthMesh, samples: 100_000)
        return RobustnessOutcome(meanRotationDeg: errors.reduce(0, +) / Float(max(errors.count, 1)), maxRotationDeg: errors.max() ?? .nan,
                                 maxMisalignmentMm: (misalignment.max() ?? .nan) * 1000, surfaceMeanMm: distance.meanMm,
                                 surfaceP95Mm: distance.p95Mm, usedFrames: result.report.usedFrames,
                                 keyframes: scan.capture.keyframes.count, notes: result.report.notes)
    }

    static func distanceToIdentity(_ m: simd_float4x4) -> Float {
        let d = m - matrix_identity_float4x4
        return simd_length(d.columns.0) + simd_length(d.columns.1) + simd_length(d.columns.2) + simd_length(d.columns.3)
    }

    static func row(_ p: ScanImperfections, _ icp: String, _ o: RobustnessOutcome) -> String {
        String(format: "| %@ | %@ | %.2f° / %.2f° | %.2f mm | %.2f / %.2f mm | %d/%d |", p.name, icp, o.meanRotationDeg, o.maxRotationDeg,
               o.maxMisalignmentMm, o.surfaceMeanMm, o.surfaceP95Mm, o.usedFrames, o.keyframes)
    }
}

@Suite("Pipeline robustness", .enabled(if: CoreAvailability.reconstruction, "fr_register_frames / fr_tsdf_create are placeholders"),
       .timeLimit(.minutes(30)))
struct PipelineRobustnessTests {
    /// All imperfections at once, point-to-plane ICP (the default): the end-to-end requirements still hold.
    @Test func realisticImperfectionsCombined() async throws {
        let o = try await RobustnessStudy.run(.realistic)
        print("robustness: " + RobustnessStudy.row(.realistic, "point-to-plane", o) + " notes \(o.notes)")
        #expect(o.usedFrames == o.keyframes)
        #expect(o.meanRotationDeg < 0.75)
        #expect(o.maxRotationDeg < 1.25)
        #expect(o.maxMisalignmentMm < 4)
        #expect(o.surfaceMeanMm < 0.75)
        #expect(o.surfaceP95Mm < 2)
    }

    /// IFS_RECON_STUDY=1: each imperfection alone, point-to-plane and colored ICP (docs/RECONSTRUCTION.md).
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func imperfectionTable() async throws {
        var scenarios: [ScanImperfections] = [ScanImperfections()]
        var s = ScanImperfections(); s.name = "correlated noise 0.5 mm + waviness 0.3 mm"; s.correlatedNoiseAt1m = 0.0014; s.wavinessAt1m = 0.0008
        scenarios.append(s)
        s = ScanImperfections(); s.name = "no depth on 80% of hair/brows/pupils"; s.hairDropout = 0.8; scenarios.append(s)
        s = ScanImperfections(); s.name = "silhouettes eroded 6 px"; s.silhouetteErosionPx = 6; scenarios.append(s)
        s = ScanImperfections(); s.name = "Vision error 4° per angle"; s.visionErrorDeg = 4; scenarios.append(s)
        s = ScanImperfections(); s.name = "exposure ±20%"; s.gainJitter = 0.2; scenarios.append(s)
        s = ScanImperfections(); s.name = "frontal keyframes last"; s.frontalLast = true; scenarios.append(s)
        s = ScanImperfections(); s.name = "8% speckle holes"; s.speckleRate = 0.08; scenarios.append(s)
        s = ScanImperfections(); s.name = "D455 color distortion, head 15 cm right, 6 cm down"; s.lensDistortion = .d455Color
        s.headOffsetM = SIMD2(0.15, 0.06); scenarios.append(s)
        scenarios.append(.realistic)
        s = .realistic; s.name = "all combined, D455 color distortion, head 8 cm right, 4 cm down"; s.lensDistortion = .d455Color
        s.headOffsetM = SIMD2(0.08, 0.04); scenarios.append(s)
        if let only = ProcessInfo.processInfo.environment["IFS_RECON_STUDY_ONLY"] {
            scenarios = scenarios.filter { $0.name.contains(only) }
        }
        var rows = ["| scenario | ICP | rotation error mean / max | head misalignment max | surface mean / p95 | frames |",
                    "|---|---|---|---|---|---|"]
        for p in scenarios {
            for colored in [false, true] {
                do {
                    let o = try await RobustnessStudy.run(p, coloredICP: colored)
                    rows.append(RobustnessStudy.row(p, colored ? "colored" : "point-to-plane", o))
                } catch {
                    rows.append("| \(p.name) | \(colored ? "colored" : "point-to-plane") | failed: \(error.localizedDescription) | | | |")
                }
            }
        }
        print("robustness table:\n" + rows.joined(separator: "\n"))
    }

    /// IFS_RECON_STUDY=1: TSDF voxel and truncation against the D455 noise at the recommended 0.47 m (theory:
    /// 1.25 mm at 1 m for 1280x720, docs/CAMERA.md) and at 0.6 m (the renderer's default 2 mm at 1 m), with
    /// correlated noise and waviness on top (docs/RECONSTRUCTION.md, "Voxel size and truncation").
    @Test(.enabled(if: ProcessInfo.processInfo.environment["IFS_RECON_STUDY"] == "1", "set IFS_RECON_STUDY=1"))
    func tsdfParameterTable() async throws {
        var near = ScanImperfections()
        near.name = "0.47 m, 1.25 mm at 1 m + correlated"
        near.distanceM = 0.47
        near.depthNoiseAt1m = 0.00125
        near.correlatedNoiseAt1m = 0.0014
        near.wavinessAt1m = 0.0008
        var far = ScanImperfections()
        far.name = "0.6 m, 2 mm at 1 m + correlated"
        far.correlatedNoiseAt1m = 0.0014
        far.wavinessAt1m = 0.0008
        let settings: [(Float, Float)] = [(0.5, 2.5), (0.5, 4), (0.75, 2.5), (0.75, 4), (0.75, 6), (1.0, 4)]
        var rows = ["| scan | voxel | truncation | surface mean / p95 | rotation error mean / max |", "|---|---|---|---|---|"]
        for scan in [near, far] {
            for (voxel, truncation) in settings {
                let o = try await RobustnessStudy.run(scan) { $0.voxelSizeMM = voxel; $0.truncationMM = truncation }
                rows.append(String(format: "| %@ | %.2f mm | %.1f mm | %.3f / %.3f mm | %.2f° / %.2f° |", scan.name, voxel, truncation,
                                   o.surfaceMeanMm, o.surfaceP95Mm, o.meanRotationDeg, o.maxRotationDeg))
            }
        }
        print("tsdf table:\n" + rows.joined(separator: "\n"))
    }
}
