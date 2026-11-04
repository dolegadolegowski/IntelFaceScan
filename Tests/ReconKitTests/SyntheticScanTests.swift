// License: Apache 2.0. See LICENSE file in root directory.
//
// The synthetic scans must be exact ground truth: depth equals an independent ray cast of the ground-truth mesh at
// the ground-truth pose, the noise has the documented statistics, poses follow HeadPose's conventions.

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd
import SyntheticData
import Testing

private func noiseFree(_ configure: (inout SyntheticScan.Options) -> Void = { _ in }) -> SyntheticScan.Options {
    var options = SyntheticScan.Options()
    options.depthNoiseAt1m = 0
    options.dropoutRate = 0
    options.meshResolution = 48
    configure(&options)
    return options
}

private func depthValue(_ frame: CameraFrame, _ x: Int, _ y: Int) -> UInt16 {
    frame.alignedDepth.pixels.withUnsafeBytes { UInt16(littleEndian: $0.load(fromByteOffset: 2 * (y * frame.alignedDepth.width + x), as: UInt16.self)) }
}

/// Independent brute-force ray caster (scalar Möller-Trumbore over every triangle): returns the camera-frame z of
/// the first hit of the ray through pixel (x, y), or nil.
private struct BruteForceRayCaster {
    var xs: [Float] = [], ys: [Float] = [], zs: [Float] = []
    let indices: [UInt32]

    init(mesh: ReconMesh, cameraFromWorld m: simd_float4x4) {
        indices = mesh.indices
        for p in mesh.positions {
            let q = m * SIMD4(p, 1)
            xs.append(q.x); ys.append(q.y); zs.append(q.z)
        }
    }

    func cast(_ dx: Float, _ dy: Float) -> Float? {
        var best = Float.infinity
        var t = 0
        while t < indices.count {
            let a = Int(indices[t]), b = Int(indices[t + 1]), c = Int(indices[t + 2])
            t += 3
            let e1x = xs[b] - xs[a], e1y = ys[b] - ys[a], e1z = zs[b] - zs[a]
            let e2x = xs[c] - xs[a], e2y = ys[c] - ys[a], e2z = zs[c] - zs[a]
            let px = dy * e2z - e2y, py = e2x - dx * e2z, pz = dx * e2y - dy * e2x   // dir x e2, dir = (dx, dy, 1)
            let det = e1x * px + e1y * py + e1z * pz
            if abs(det) < 1e-12 { continue }
            let inv = 1 / det
            let tx = -xs[a], ty = -ys[a], tz = -zs[a]
            let u = (tx * px + ty * py + tz * pz) * inv
            if u < 0 || u > 1 { continue }
            let qx = ty * e1z - tz * e1y, qy = tz * e1x - tx * e1z, qz = tx * e1y - ty * e1x
            let v = (dx * qx + dy * qy + qz) * inv
            if v < 0 || u + v > 1 { continue }
            let hit = (e2x * qx + e2y * qy + e2z * qz) * inv
            if hit > 0.01 && hit < best { best = hit }
        }
        return best.isFinite ? best : nil
    }
}

/// Valid depth pixels triangulated into a mesh in the world frame (neighbors closer than 5 mm in depth).
private func depthMesh(_ frame: CameraFrame, worldFromCamera: simd_float4x4) -> ReconMesh {
    let w = frame.alignedDepth.width, h = frame.alignedDepth.height
    let intrinsics = frame.alignedDepth.intrinsics, scale = frame.alignedDepth.depthScale
    var index = [Int32](repeating: -1, count: w * h)
    var positions: [SIMD3<Float>] = []
    var depth = [Float](repeating: 0, count: w * h)
    for y in 0 ..< h {
        for x in 0 ..< w {
            let d = depthValue(frame, x, y)
            guard d != 0 else { continue }
            let z = Float(d) * scale
            depth[y * w + x] = z
            let p = intrinsics.deproject(SIMD2(Float(x), Float(y)), depth: z)
            let q = worldFromCamera * SIMD4(p, 1)
            index[y * w + x] = Int32(positions.count)
            positions.append(SIMD3(q.x, q.y, q.z))
        }
    }
    var indices: [UInt32] = []
    for y in 0 ..< h - 1 {
        for x in 0 ..< w - 1 {
            let i00 = y * w + x, i10 = i00 + 1, i01 = i00 + w, i11 = i01 + 1
            let quad = [i00, i10, i01, i11]
            guard quad.allSatisfy({ index[$0] >= 0 }) else { continue }
            let zs = quad.map { depth[$0] }
            guard zs.max()! - zs.min()! < 0.005 else { continue }
            indices += [UInt32(index[i00]), UInt32(index[i01]), UInt32(index[i10]),
                        UInt32(index[i10]), UInt32(index[i01]), UInt32(index[i11])]
        }
    }
    return ReconMesh(positions: positions, indices: indices)
}

// MARK: - Ground truth consistency

@Test func syntheticDepthMatchesARayCastOfTheGroundTruthMesh() throws {
    let options = noiseFree()
    let result = SyntheticScan.make(options)
    #expect(result.capture.keyframes.count == 10)
    var random = SplitMix64(seed: 7)
    var checked = 0, background = 0
    for (kf, pose) in zip(result.capture.keyframes, result.groundTruthPoses) {
        let caster = BruteForceRayCaster(mesh: result.groundTruthMesh, cameraFromWorld: pose.inverse)
        let frame = kf.frame, intrinsics = frame.alignedDepth.intrinsics
        let mask = try #require(kf.mask)
        // Random head pixels with depth, plus background pixels.
        var tries = 0, headSamples = 0
        while headSamples < 25 && tries < 20_000 {
            tries += 1
            let x = Int(random.next() % UInt64(frame.color.width)), y = Int(random.next() % UInt64(frame.color.height))
            let raw = depthValue(frame, x, y)
            let isHead = mask[y * frame.color.width + x] == 255
            guard raw != 0, isHead || background < 40 else { continue }
            let dx = (Float(x) - intrinsics.ppx) / intrinsics.fx, dy = (Float(y) - intrinsics.ppy) / intrinsics.fy
            let z = Float(raw) * frame.alignedDepth.depthScale
            if let hit = caster.cast(dx, dy) {
                #expect(abs(hit - z) <= 0.00005 + 2e-6, "pixel (\(x), \(y)): depth \(z) vs ray cast \(hit)")
                #expect(isHead || hit < 1.1)
                headSamples += isHead ? 1 : 0
            } else {
                #expect(!isHead, "head pixel (\(x), \(y)) has no ray hit")
                #expect(raw == 11_000)   // the wall at 1.1 m
                background += 1
            }
            checked += 1
        }
    }
    #expect(checked >= 250)
    #expect(background >= 30)
}

@Test func syntheticDepthAtTheGroundTruthPosesLiesOnTheGroundTruthSurface() throws {
    let result = SyntheticScan.make(noiseFree())
    for (kf, pose) in zip(result.capture.keyframes, result.groundTruthPoses) {
        var frame = kf.frame
        // Keep the head only (the wall is not part of the ground truth).
        let mask = try #require(kf.mask)
        frame.alignedDepth.pixels.withUnsafeMutableBytes { raw in
            let v = raw.bindMemory(to: UInt16.self)
            for i in 0 ..< v.count where mask[i] == 0 { v[i] = 0 }
        }
        let cloud = depthMesh(frame, worldFromCamera: pose)
        #expect(cloud.triangleCount > 3_000)
        let d = try MeshEvaluation.distance(from: cloud, to: result.groundTruthMesh, samples: 30_000)
        #expect(d.meanMm < 0.06, "\(kf.scanPose): \(d)")
        #expect(d.p95Mm < 0.15, "\(kf.scanPose): \(d)")
    }
}

@Test func syntheticNoiseHasTheDocumentedStatistics() throws {
    var options = noiseFree()
    options.depthNoiseAt1m = 0.002
    let scene = SyntheticScene(options: options)
    let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: .frontal), frameIndex: 3)
    var sum: Double = 0, sum2: Double = 0, count = 0
    var headSum2: Double = 0, headCount = 0
    view.frame.alignedDepth.pixels.withUnsafeBytes { raw in
        let v = raw.bindMemory(to: UInt16.self)
        for i in 0 ..< v.count where v[i] != 0 {
            let clean = Double(view.cleanDepth[i])
            let sigma = 0.002 * clean * clean
            let normalized = (Double(UInt16(littleEndian: v[i])) * 0.0001 - clean) / sigma
            sum += normalized; sum2 += normalized * normalized; count += 1
            if view.mask[i] == 255 { headSum2 += normalized * normalized; headCount += 1 }
        }
    }
    let mean = sum / Double(count), std = (sum2 / Double(count) - mean * mean).squareRoot()
    #expect(count > 200_000)
    #expect(abs(mean) < 0.01)
    #expect(abs(std - 1) < 0.02)            // quantization (0.1 mm) adds < 0.1% at these depths
    #expect(abs((headSum2 / Double(headCount)).squareRoot() - 1) < 0.05)

    // A different seed gives different noise, the same seed the same pixels.
    let again = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: .frontal), frameIndex: 3)
    #expect(again.frame.alignedDepth.pixels == view.frame.alignedDepth.pixels)
    #expect(again.frame.color.pixels == view.frame.color.pixels)
    let nextFrame = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: .frontal), frameIndex: 4)
    #expect(nextFrame.frame.alignedDepth.pixels != view.frame.alignedDepth.pixels)
}

@Test func syntheticInvalidDepthAppearsAtGrazingAnglesAndSilhouettes() throws {
    let scene = SyntheticScene(options: noiseFree())
    let view = scene.render(cameraFromWorld: scene.cameraFromWorld(headPose: HeadPose(yaw: 45, pitch: 0, roll: 0)), frameIndex: 0)
    let w = view.frame.color.width, h = view.frame.color.height
    var headPixels = 0, invalidHead = 0, silhouetteInvalid = 0, silhouette = 0
    view.frame.alignedDepth.pixels.withUnsafeBytes { raw in
        let v = raw.bindMemory(to: UInt16.self)
        for y in 1 ..< h - 1 {
            for x in 1 ..< w - 1 {
                let i = y * w + x
                guard view.cleanDepth[i] > 0, view.cleanDepth[i] < 1 else { continue }
                headPixels += 1
                if v[i] == 0 { invalidHead += 1 }
                // next to the wall: a depth jump of ~40 cm
                if view.cleanDepth[i - 1] > 1 || view.cleanDepth[i + 1] > 1 {
                    silhouette += 1
                    if v[i] == 0 { silhouetteInvalid += 1 }
                }
            }
        }
    }
    #expect(silhouette > 100 && silhouetteInvalid == silhouette)
    let ratio = Double(invalidHead) / Double(headPixels)
    #expect(ratio > 0.02 && ratio < 0.25, "invalid head ratio \(ratio)")
}

// MARK: - Poses and metadata

@Test func syntheticPosesFollowTheHeadPoseConventions() throws {
    let options = noiseFree { $0.framesPerPose = 1; $0.poses = [.front, .left, .right, .up, .down] }
    let result = SyntheticScan.make(options)
    let scene = SyntheticScene(options: options)
    #expect(result.groundTruthPoses[0] == matrix_identity_float4x4)
    for (i, kf) in result.capture.keyframes.enumerated() {
        let truth = result.groundTruthHeadPoses[i]
        // world_from_camera rotation = R^T with R = HeadPose.cameraFrameRotation
        let rotation = simd_quatf(simd_float3x3(columns: (SIMD3(result.groundTruthPoses[i].columns.0.x, result.groundTruthPoses[i].columns.0.y, result.groundTruthPoses[i].columns.0.z),
                                                         SIMD3(result.groundTruthPoses[i].columns.1.x, result.groundTruthPoses[i].columns.1.y, result.groundTruthPoses[i].columns.1.z),
                                                         SIMD3(result.groundTruthPoses[i].columns.2.x, result.groundTruthPoses[i].columns.2.y, result.groundTruthPoses[i].columns.2.z))))
        let expected = truth.cameraFrameRotation.inverse
        #expect(abs((rotation * expected.inverse).angle) < 1e-4 || abs((rotation * expected.inverse).angle - 2 * .pi) < 1e-4)
        let target = kf.scanPose.target
        #expect(abs(truth.yaw - target.yaw) <= options.poseVariationDeg + 1e-4)
        #expect(abs(truth.pitch - target.pitch) <= options.poseVariationDeg + 1e-4)
        // the simulated Vision estimate is close to, but not equal to, the truth
        #expect(abs(kf.headPose.yaw - truth.yaw) < 5 * options.headPoseErrorDeg)
        #expect(kf.headPose.confidence > 0.85 && kf.headPose.confidence <= 1)
    }
    // yaw > 0 ("left"): the nose moves toward the image's right edge; pitch > 0: toward the top.
    let noseTip = 49   // contour 17, brows 10, eyes 16, pupils 2, crest 5 (tip last)
    func noseOffset(_ i: Int) -> SIMD2<Float> {
        let pose = result.groundTruthPoses[i].inverse
        let nose = scene.landmarks(cameraFromWorld: pose)[noseTip]
        let center = scene.intrinsics.project(SIMD3((pose * SIMD4(scene.headCenter, 1)).x, (pose * SIMD4(scene.headCenter, 1)).y,
                                                    (pose * SIMD4(scene.headCenter, 1)).z))
        return nose - center
    }
    let frontal = noseOffset(0)
    #expect(noseOffset(1).x > frontal.x + 10)   // left
    #expect(noseOffset(2).x < frontal.x - 10)   // right
    #expect(noseOffset(3).y < frontal.y - 5)    // up
    #expect(noseOffset(4).y > frontal.y + 5)    // down
}

@Test func syntheticKeyframesCarryPlausibleMetadata() throws {
    var options = SyntheticScan.Options()
    options.meshResolution = 48
    let result = SyntheticScan.make(options)
    let capture = result.capture
    #expect(capture.device?.name == "Synthetic D455")
    #expect(capture.device?.depthScale == 0.0001)
    #expect(capture.configuration?.color.width == 640)
    #expect(capture.notes["source"] == "synthetic")
    #expect(capture.keyframes.map(\.scanPose) == [.front, .front, .up, .up, .down, .down, .left, .left, .right, .right])
    for kf in capture.keyframes {
        let frame = kf.frame
        #expect(frame.color.width == 640 && frame.color.height == 400 && frame.color.bytesPerRow == 1920)
        #expect(frame.color.pixels.count == 640 * 400 * 3 && frame.alignedDepth.pixels.count == 640 * 400 * 2)
        #expect(abs(frame.color.intrinsics.fx - 320.2) < 1e-3 && abs(frame.color.intrinsics.ppy - 200.65) < 1e-3)
        #expect(frame.alignedDepth.intrinsics == frame.color.intrinsics)
        #expect(frame.imu?.accel?.count == 3)
        #expect(kf.landmarks.count == 76)
        let inside = kf.landmarks.filter { kf.faceRect.contains(CGPoint(x: CGFloat($0.x), y: CGFloat($0.y))) }.count
        #expect(inside >= 70)
        #expect(kf.faceRect.width > 40 && kf.faceRect.height > 40)
        let mask = try #require(kf.mask)
        #expect(mask.count == 640 * 400)
        let head = mask.reduce(0) { $0 + ($1 == 255 ? 1 : 0) }
        #expect(head > 4_000 && head < 30_000)
        #expect(kf.quality.validDepthRatio > 0.8)
        #expect(kf.quality.validDepthPoints > 4_000)
        #expect(abs(kf.quality.faceDistanceM - options.distanceM) < 0.07)
        #expect(kf.quality.sharpness > 10)
    }
    #expect(capture.keyframes.map(\.frame.sequence) == capture.keyframes.map(\.frame.sequence).sorted())
}

@Test func syntheticScansAreDeterministic() throws {
    var options = SyntheticScan.Options()
    options.meshResolution = 48
    options.width = 320
    options.height = 200
    options.framesPerPose = 1
    let a = SyntheticScan.make(options), b = SyntheticScan.make(options)
    options.seed = 2
    let c = SyntheticScan.make(options)
    for i in a.capture.keyframes.indices {
        #expect(a.capture.keyframes[i].frame.alignedDepth.pixels == b.capture.keyframes[i].frame.alignedDepth.pixels)
        #expect(a.capture.keyframes[i].frame.color.pixels == b.capture.keyframes[i].frame.color.pixels)
        #expect(a.capture.keyframes[i].headPose == b.capture.keyframes[i].headPose)
        #expect(a.groundTruthPoses[i] == b.groundTruthPoses[i])
    }
    #expect(a.capture.keyframes[2].frame.alignedDepth.pixels != c.capture.keyframes[2].frame.alignedDepth.pixels)
    #expect(a.groundTruthPoses[2] != c.groundTruthPoses[2])
    #expect(a.groundTruthMesh.positions == c.groundTruthMesh.positions)
}

@Test func syntheticEvaluationSeparatesRotationFromItsLeverArm() {
    let scene = SyntheticScene(options: noiseFree())
    let truth = [matrix_identity_float4x4, scene.cameraFromWorld(headPose: HeadPose(yaw: 45, pitch: 0, roll: 0)).inverse]
    #expect(MeshEvaluation.relativeHeadMisalignment(estimated: truth, groundTruth: truth, center: scene.headCenter) == [0, 0])

    // Frame 1 off by 1 degree about the head center: the head surface (9 cm sphere) moves by r * angle = 1.57 mm,
    // while the camera center, 0.68 m away, moves by ~12 mm.
    let c = scene.headCenter
    let spin = simd_float4x4(simd_quatf(angle: Float.pi / 180, axis: [0, 1, 0]))
    let toCenter = simd_float4x4(columns: ([1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], SIMD4(c, 1)))
    let estimated = [truth[0], toCenter * spin * toCenter.inverse * truth[1]]
    let misalignment = MeshEvaluation.relativeHeadMisalignment(estimated: estimated, groundTruth: truth, center: c)
    #expect(abs(misalignment[1] - 0.09 * Float.pi / 180) < 0.0001)
    let error = MeshEvaluation.relativePoseErrors(estimated: estimated, groundTruth: truth)[1]
    #expect(abs(error.rotationDeg - 1) < 0.01)
    #expect(error.translationM > 0.01)
}

@Test func syntheticGroundTruthMeshIsAClosedOutwardSurface() throws {
    let options = noiseFree { $0.meshResolution = 64 }
    let mesh = SyntheticScan.make(options).groundTruthMesh
    #expect(mesh.triangleCount == 12 * 64 * 64)
    #expect(mesh.normals.count == mesh.vertexCount && mesh.colors.count == mesh.vertexCount)
    // Every directed edge appears once and its reverse once: closed, consistently oriented, manifold.
    var edges: [UInt64: Int] = [:]
    for t in 0 ..< mesh.triangleCount {
        for k in 0 ..< 3 {
            let a = UInt64(mesh.indices[3 * t + k]), b = UInt64(mesh.indices[3 * t + (k + 1) % 3])
            edges[a << 32 | b, default: 0] += 1
        }
    }
    #expect(edges.values.allSatisfy { $0 == 1 })
    #expect(edges.keys.allSatisfy { edges[($0 & 0xFFFF_FFFF) << 32 | $0 >> 32] == 1 })
    // Positive signed volume = outward (counter-clockwise from outside) winding.
    var volume: Double = 0
    for t in 0 ..< mesh.triangleCount {
        let a = mesh.positions[Int(mesh.indices[3 * t])], b = mesh.positions[Int(mesh.indices[3 * t + 1])]
        let c = mesh.positions[Int(mesh.indices[3 * t + 2])]
        volume += Double(simd_dot(a, simd_cross(b, c))) / 6
    }
    #expect(volume > 0.003 && volume < 0.012)   // head + 45 cm of neck, liters / 1000
    // The head is in front of the camera, face toward it.
    let zs = mesh.positions.map(\.z)
    #expect(zs.min()! > options.distanceM - 0.05 && zs.min()! < options.distanceM)
    let width = mesh.positions.map(\.x).max()! - mesh.positions.map(\.x).min()!
    #expect(width > 0.15 && width < 0.21)   // with the ears
}
