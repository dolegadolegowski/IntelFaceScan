// License: Apache 2.0. See LICENSE file in root directory.

import CameraKit
import Foundation
@testable import ReconKit
import ScanKit
import simd
import Testing

/// Collects progress events from any thread.
final class ProgressLog: @unchecked Sendable {
    private let lock = NSLock()
    private var events: [ReconstructionProgress] = []
    func add(_ p: ReconstructionProgress) { lock.lock(); events.append(p); lock.unlock() }
    var all: [ReconstructionProgress] { lock.lock(); defer { lock.unlock() }; return events }
}

@Suite("Pipeline")
struct PipelineTests {
    static let scan = makeSyntheticScan()

    static func misalignment(_ a: simd_float4x4, _ b: simd_float4x4, center: P3) -> Double {
        // Largest displacement of points on a head-sized sphere.
        var worst = 0.0
        for k in 0 ..< 32 {
            let phi = Double(k) * 2.4, y = 1 - 2 * (Double(k) + 0.5) / 32, r = (1 - y * y).squareRoot()
            let p = SIMD4<Float>(Float(center.x + 0.09 * r * cos(phi)), Float(center.y + 0.09 * y), Float(center.z + 0.09 * r * sin(phi)), 1)
            let d = a * p - b * p
            worst = max(worst, Double(simd_length(SIMD3(d.x, d.y, d.z))))
        }
        return worst
    }

    @Test func reconstructsSyntheticHead() async throws {
        var options = ReconstructionOptions()
        options.voxelSizeMM = 1.0
        let log = ProgressLog()
        let result = try await ReconstructionPipeline(options: options).run(Self.scan.capture) { log.add($0) }
        let report = result.report
        print("pipeline: \(report.usedFrames)/\(report.keyframes) frames, odometry \(report.odometryEdges), loops \(report.loopClosures), "
            + "rejected \(report.rejectedEdges), RMSE \(report.meanRegistrationRMSEmm) mm, GPU \(report.usedGPU), "
            + "\(report.vertexCount) vertices, durations \(report.durations), notes \(report.notes)")

        #expect(report.usedFrames == Self.scan.capture.keyframes.count)
        #expect(result.cameraPoses.count == report.usedFrames)
        #expect(report.odometryEdges >= 8)
        #expect(report.meanRegistrationRMSEmm > 0 && report.meanRegistrationRMSEmm < 1.5)
        #expect(report.voxelSizeMM == 1.0)
        #expect(report.method == .tsdf)
        #expect(report.vertexCount == result.mesh.vertexCount && report.vertexCount > 20000)
        #expect(report.durations["registration"] != nil && report.durations["fusion"] != nil)

        // Poses (all keyframes used, original order) against the ground truth.
        var worst = 0.0
        for (pose, truth) in zip(result.cameraPoses, Self.scan.truth) {
            worst = max(worst, Self.misalignment(pose, truth, center: Self.scan.center))
        }
        #expect(worst < 0.0015, "worst head misalignment \(worst * 1000) mm")

        // Surface accuracy: vertices against the true head along their normals.
        let mesh = result.mesh
        var distances: [Double] = []
        for i in stride(from: 0, to: mesh.vertexCount, by: 7) {
            if let d = distanceToHead(P3(mesh.positions[i]), normal: P3(mesh.normals[i]), center: Self.scan.center) {
                distances.append(d)
            }
        }
        distances.sort()
        #expect(distances.count > mesh.vertexCount / 8)
        let mean = distances.reduce(0, +) / Double(distances.count)
        let p90 = distances[distances.count * 9 / 10]
        print("surface error: mean \(mean * 1000) mm, p90 \(p90 * 1000) mm over \(distances.count) vertices")
        #expect(mean < 0.0006, "mean surface error \(mean * 1000) mm")
        #expect(p90 < 0.0012, "p90 surface error \(p90 * 1000) mm")

        // The static neck is cut away: nothing far below the chin (canonical chin at y = +0.098).
        let lowest = mesh.positions.map { Double($0.y) - Self.scan.center.y }.max() ?? 0
        #expect(lowest < 0.125, "lowest point \(lowest * 1000) mm below the head center")

        // Export frame: rigid, head centered, upright, face toward +Z.
        let t = result.exportTransform
        let r = simd_float3x3(SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z), SIMD3(t.columns.1.x, t.columns.1.y, t.columns.1.z),
                              SIMD3(t.columns.2.x, t.columns.2.y, t.columns.2.z))
        #expect(abs(r.determinant - 1) < 1e-4)
        let gram = r.transpose * r
        let offIdentity = [gram.columns.0 - SIMD3(1, 0, 0), gram.columns.1 - SIMD3(0, 1, 0), gram.columns.2 - SIMD3(0, 0, 1)]
        #expect(offIdentity.allSatisfy { simd_length($0) < 1e-4 }, "orthonormal rotation")
        let exported = mesh.positions.map { p -> SIMD3<Float> in let q = t * SIMD4(p, 1); return SIMD3(q.x, q.y, q.z) }
        let lo = exported.reduce(SIMD3<Float>(repeating: .infinity)) { simd_min($0, $1) }
        let hi = exported.reduce(SIMD3<Float>(repeating: -.infinity)) { simd_max($0, $1) }
        #expect(simd_length((lo + hi) / 2) < 1e-4, "bounding box centered at the origin")
        let nose = t * SIMD4(Self.scan.center.simd + SIMD3(0, 0.012, -0.112), 1)
        #expect(nose.z > 0.05, "nose toward +Z")
        let crown = t * SIMD4(Self.scan.center.simd + SIMD3(0, -0.1, 0), 1)
        #expect(crown.y > 0.03, "top of the head toward +Y")
        #expect(abs(nose.x) < 0.005, "no yaw/roll offset of the nose")

        // Progress: every main stage, monotonic overall fraction, finished at 1.
        let events = log.all
        let stages = Set(events.map(\.stage))
        for stage in [ReconstructionProgress.Stage.preparing, .segmentation, .registration, .globalOptimization, .fusion, .meshing,
                      .cleanup, .texturing, .finished] {
            #expect(stages.contains(stage), "stage \(stage) reported")
        }
        #expect(events.last?.stage == .finished && events.last?.fraction == 1)
        let fractions = events.map(\.fraction)
        #expect(zip(fractions, fractions.dropFirst()).allSatisfy { $0 <= $1 + 1e-9 }, "monotonic progress")
    }

    @Test func keepShouldersKeepsTheNeckFromFrontalViews() async throws {
        var options = ReconstructionOptions()
        options.voxelSizeMM = 1.5
        options.keepShoulders = true
        let result = try await ReconstructionPipeline(options: options).run(Self.scan.capture)
        let lowest = result.mesh.positions.map { Double($0.y) - Self.scan.center.y }.max() ?? 0
        #expect(lowest > 0.15, "neck kept (lowest point \(lowest * 1000) mm below the head center)")
    }

    @Test func poissonMethodProducesAMesh() async throws {
        var options = ReconstructionOptions()
        options.method = .poisson
        options.poissonDepth = 8
        options.voxelSizeMM = 1.5
        let result = try await ReconstructionPipeline(options: options).run(Self.scan.capture)
        #expect(result.report.method == .poisson)
        #expect(result.mesh.triangleCount > 10000)
        #expect(result.report.durations["poisson"] != nil)
    }

    @Test func tooFewKeyframesAreRejected() async throws {
        var capture = Self.scan.capture
        capture.keyframes = Array(capture.keyframes.prefix(4))
        await #expect(throws: ReconstructionError.self) {
            _ = try await ReconstructionPipeline().run(capture)
        }
        do {
            _ = try await ReconstructionPipeline().run(capture)
        } catch ReconstructionError.notEnoughKeyframes(let n) {
            #expect(n == 4)
        }
    }

    @Test func missingFrontalKeyframeIsRejected() async throws {
        var capture = Self.scan.capture
        capture.keyframes = capture.keyframes.filter { $0.scanPose != .front }
        do {
            _ = try await ReconstructionPipeline().run(capture)
            Issue.record("expected an error")
        } catch ReconstructionError.core(let stage, _, let message) {
            #expect(stage == "validation")
            #expect(message.contains("frontal"))
        }
    }

    @Test func unusableKeyframesAreSkipped() async throws {
        var capture = Self.scan.capture
        // A keyframe without depth and one with inconsistent image sizes.
        var empty = capture.keyframes[1]
        empty.frame.alignedDepth.pixels = Data(count: empty.frame.alignedDepth.pixels.count)
        var broken = capture.keyframes[2]
        broken.frame.color.width = 10
        capture.keyframes[1] = empty
        capture.keyframes[2] = broken
        var options = ReconstructionOptions()
        options.voxelSizeMM = 2
        let result = try await ReconstructionPipeline(options: options).run(capture)
        #expect(result.report.usedFrames == capture.keyframes.count - 2)
        #expect(result.report.notes.contains { $0.contains("skipped") })
    }

    @Test func cancellationStopsTheRun() async throws {
        let task = Task {
            try await ReconstructionPipeline().run(Self.scan.capture)
        }
        try await Task.sleep(nanoseconds: 50_000_000)
        task.cancel()
        do {
            _ = try await task.value
            Issue.record("expected cancellation")
        } catch ReconstructionError.cancelled {
        }
    }
}

/// Performance of the full pipeline on a scan the size of a real one (40 keyframes at 1280x800, 0.75 mm voxels).
/// Opt-in: IFS_BENCH=1 scripts/test.sh -c release --filter PipelineBenchmark
@Suite("PipelineBenchmark", .enabled(if: ProcessInfo.processInfo.environment["IFS_BENCH"] == "1"))
struct PipelineBenchmark {
    @Test func fortyKeyframesAt1280x800() async throws {
        var o = SynthScanOptions()
        o.width = 1280
        o.height = 800
        let base: [(ScanPose, Double, Double)] = [(.front, 0, 0), (.up, 0, 25), (.down, 0, -25), (.left, 45, 0), (.right, -45, 0),
                                                   (.upLeft, 30, 20), (.upRight, -30, 20), (.downLeft, 30, -20), (.downRight, -30, -20)]
        var views: [(ScanPose, Double, Double)] = []
        var rng = Rng(s: 99)
        for (pose, yaw, pitch) in base {
            let count = pose == .front ? 8 : 4
            for _ in 0 ..< count { views.append((pose, yaw + 1.5 * rng.gaussian(), pitch + 1.5 * rng.gaussian())) }
        }
        o.views = views
        let renderStart = Date()
        let scan = makeSyntheticScan(o)
        let renderTime = Date().timeIntervalSince(renderStart)
        var options = ReconstructionOptions()
        options.voxelSizeMM = 0.75
        let start = Date()
        let result = try await ReconstructionPipeline(options: options).run(scan.capture)
        let elapsed = Date().timeIntervalSince(start)
        var worst = 0.0
        for (pose, truth) in zip(result.cameraPoses, scan.truth) { worst = max(worst, PipelineTests.misalignment(pose, truth, center: scan.center)) }
        print("benchmark: \(views.count) keyframes 1280x800 rendered in \(renderTime) s; pipeline \(elapsed) s; "
            + "durations \(result.report.durations.sorted { $0.key < $1.key }); GPU \(result.report.usedGPU); "
            + "\(result.report.vertexCount) vertices; worst misalignment \(worst * 1000) mm; notes \(result.report.notes)")
        #expect(elapsed < 60)
        #expect(result.report.usedFrames == views.count)
    }
}
