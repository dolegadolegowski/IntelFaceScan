// License: Apache 2.0. See LICENSE file in root directory.
//
// fr_eval.h on analytic cases: parallel planes, concentric spheres, the Voronoi regions of a triangle, plane fits
// with noise and outliers, pose errors.

import FaceReconCore
import Foundation
import ReconKit
import simd
import Testing

// MARK: - Helpers

/// Deterministic generator for test data (splitmix64 + Box-Muller).
private struct TestRandom {
    var state: UInt64
    init(_ seed: UInt64) { state = seed }
    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
    mutating func uniform() -> Double { Double(next() >> 11) / 9_007_199_254_740_992.0 }
    mutating func gaussian() -> Double {
        let u1 = max(uniform(), 1e-300), u2 = uniform()
        return (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
    }
}

/// Square grid in the plane z = height, n x n cells, counter-clockwise seen from +Z.
private func gridMesh(size: Float, cells n: Int, height: Float, center: SIMD2<Float> = .zero) -> ReconMesh {
    var positions: [SIMD3<Float>] = []
    for j in 0 ... n {
        for i in 0 ... n {
            let x = center.x - size / 2 + size * Float(i) / Float(n)
            let y = center.y - size / 2 + size * Float(j) / Float(n)
            positions.append(SIMD3(x, y, height))
        }
    }
    var indices: [UInt32] = []
    for j in 0 ..< n {
        for i in 0 ..< n {
            let a = UInt32(j * (n + 1) + i), b = a + 1, c = a + UInt32(n + 1), d = c + 1
            indices += [a, b, d, a, d, c]
        }
    }
    return ReconMesh(positions: positions, indices: indices)
}

/// Icosphere (subdivided icosahedron), counter-clockwise seen from outside.
private func icosphere(radius: Float, center: SIMD3<Float>, subdivisions: Int) -> ReconMesh {
    let t = Float((1 + 5.0.squareRoot()) / 2)
    var vertices: [SIMD3<Float>] = [
        [-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t],
        [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1],
    ].map { simd_normalize($0) }
    var faces: [(UInt32, UInt32, UInt32)] = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
        (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
        (8, 6, 7), (9, 8, 1),
    ]
    for _ in 0 ..< subdivisions {
        var cache: [UInt64: UInt32] = [:]
        func midpoint(_ a: UInt32, _ b: UInt32) -> UInt32 {
            let key = UInt64(min(a, b)) << 32 | UInt64(max(a, b))
            if let index = cache[key] { return index }
            vertices.append(simd_normalize(vertices[Int(a)] + vertices[Int(b)]))
            let index = UInt32(vertices.count - 1)
            cache[key] = index
            return index
        }
        var next: [(UInt32, UInt32, UInt32)] = []
        next.reserveCapacity(faces.count * 4)
        for (a, b, c) in faces {
            let ab = midpoint(a, b), bc = midpoint(b, c), ca = midpoint(c, a)
            next += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        }
        faces = next
    }
    return ReconMesh(positions: vertices.map { center + radius * $0 },
                     indices: faces.flatMap { [$0.0, $0.1, $0.2] })
}

private func distance(_ from: ReconMesh, _ to: ReconMesh, samples: UInt32 = 20_000) throws -> (fr_status, fr_distance_stats) {
    let a = try from.makeCoreMesh(), b = try to.makeCoreMesh()
    defer { fr_mesh_free(a); fr_mesh_free(b) }
    var stats = fr_distance_stats()
    let status = fr_mesh_distance(a, b, samples, &stats)
    return (status, stats)
}

private func rotation(axis: SIMD3<Float>, degrees: Float) -> simd_float4x4 {
    simd_float4x4(simd_quatf(angle: degrees * .pi / 180, axis: simd_normalize(axis)))
}

private func poseError(_ a: simd_float4x4, _ b: simd_float4x4) -> (Float, Float) {
    var ma = a, mb = b
    var rotation: Float = 0, translation: Float = 0
    withUnsafePointer(to: &ma) { pa in
        withUnsafePointer(to: &mb) { pb in
            pa.withMemoryRebound(to: Float.self, capacity: 16) { fa in
                pb.withMemoryRebound(to: Float.self, capacity: 16) { fb in
                    fr_pose_error(fa, fb, &rotation, &translation)
                }
            }
        }
    }
    return (rotation, translation)
}

// MARK: - fr_mesh_distance

@Test func evalParallelPlanesGiveTheExactOffset() throws {
    let offset: Float = 0.003
    let lower = gridMesh(size: 0.2, cells: 20, height: 0.6)
    let upper = gridMesh(size: 0.2, cells: 7, height: 0.6 + offset)
    let (status, stats) = try distance(upper, lower)
    #expect(status == FR_OK)
    #expect(stats.samples == 20_000)
    for value in [stats.mean_m, stats.rms_m, stats.median_m, stats.p95_m, stats.max_m] {
        #expect(abs(value - Double(offset)) < 1e-7)
    }
}

@Test func evalConcentricSpheresGiveTheRadiusDifference() throws {
    let center = SIMD3<Float>(0.01, -0.02, 0.6)
    let inner = icosphere(radius: 0.100, center: center, subdivisions: 6)
    let outer = icosphere(radius: 0.103, center: center, subdivisions: 5)
    // Flat facets lie inside the true spheres by at most r * (1 - cos(circumradius angle)) ~ 0.01-0.03 mm here.
    for (from, to) in [(outer, inner), (inner, outer)] {
        let (status, stats) = try distance(from, to, samples: 50_000)
        #expect(status == FR_OK)
        #expect(abs(stats.mean_m - 0.003) < 4e-5)
        #expect(abs(stats.median_m - 0.003) < 4e-5)
        #expect(stats.max_m < 0.003 + 6e-5)
        #expect(stats.rms_m >= stats.mean_m)
        #expect(stats.p95_m <= stats.max_m && stats.median_m <= stats.p95_m)
    }
}

@Test func evalMeshAgainstItselfIsZeroAndDeterministic() throws {
    let sphere = icosphere(radius: 0.09, center: [0, 0, 0.5], subdivisions: 4)
    let (status, stats) = try distance(sphere, sphere)
    #expect(status == FR_OK)
    #expect(stats.max_m < 1e-9)
    let (_, again) = try distance(sphere, sphere)
    #expect(again.mean_m == stats.mean_m && again.max_m == stats.max_m)

    let offset = icosphere(radius: 0.091, center: [0, 0, 0.5], subdivisions: 4)
    let (_, first) = try distance(offset, sphere, samples: 5_000)
    let (_, second) = try distance(offset, sphere, samples: 5_000)
    #expect(first.mean_m == second.mean_m && first.p95_m == second.p95_m && first.max_m == second.max_m)
}

@Test(arguments: [
    // (point, exact distance to the triangle (0,0,0) (1,0,0) (0,1,0))
    (SIMD3<Float>(0.2, 0.2, 0.5), 0.5),                     // face region
    (SIMD3<Float>(0.3, 0.3, -0.25), 0.25),                  // face region, below
    (SIMD3<Float>(-0.3, -0.4, 0), 0.5),                     // vertex a
    (SIMD3<Float>(1.3, -0.4, 0), 0.5),                      // vertex b
    (SIMD3<Float>(-0.6, 1.8, 0), 1.0),                      // vertex c
    (SIMD3<Float>(0.5, -0.2, 0.1), (0.05 as Double).squareRoot()),        // edge ab
    (SIMD3<Float>(-0.3, 0.5, 0.4), 0.5),                    // edge ca
    (SIMD3<Float>(1, 1, 0), (0.5 as Double).squareRoot()),  // edge bc
])
func evalPointToTriangleRegionsAreExact(point: SIMD3<Float>, expected: Double) throws {
    let triangle = ReconMesh(positions: [[0, 0, 0], [1, 0, 0], [0, 1, 0]], indices: [0, 1, 2])
    // A tiny triangle around the point: every sample is within 1e-6 m of it.
    let h: Float = 1e-6
    let probe = ReconMesh(positions: [point, point + SIMD3(h, 0, 0), point + SIMD3(0, h, 0)], indices: [0, 1, 2])
    let (status, stats) = try distance(probe, triangle, samples: 64)
    #expect(status == FR_OK)
    #expect(abs(stats.mean_m - expected) < 3e-6)
}

@Test func evalHandlesDegenerateAndUnreferencedGeometry() throws {
    // 'to' has a zero-area triangle (collinear), a collapsed one and an unreferenced vertex.
    let to = ReconMesh(positions: [[0, 0, 0], [1, 0, 0], [0, 1, 0], [2, 0, 0], [5, 5, 5]],
                       indices: [0, 1, 2, 0, 1, 3, 1, 1, 1])
    let probe = ReconMesh(positions: [[1.5, 0, 0.2], [1.5 + 1e-6, 0, 0.2], [1.5, 1e-6, 0.2]], indices: [0, 1, 2])
    let (status, stats) = try distance(probe, to, samples: 16)
    #expect(status == FR_OK)
    #expect(abs(stats.mean_m - 0.2) < 3e-6)   // closest to the degenerate segment (1,0,0)-(2,0,0)
    #expect(stats.max_m.isFinite)

    // 'from' with only zero-area triangles has no surface to sample.
    let flat = ReconMesh(positions: [[0, 0, 0], [1, 0, 0], [2, 0, 0]], indices: [0, 1, 2])
    let (flatStatus, _) = try distance(flat, to)
    #expect(flatStatus == FR_ERR_NOT_ENOUGH_DATA)
}

@Test func evalRejectsInvalidArguments() throws {
    let good = gridMesh(size: 0.1, cells: 2, height: 0)
    let a = try good.makeCoreMesh(), b = try good.makeCoreMesh()
    defer { fr_mesh_free(a); fr_mesh_free(b) }
    var stats = fr_distance_stats()
    #expect(fr_mesh_distance(a, b, 0, &stats) == FR_ERR_INVALID_ARGUMENT)
    #expect(fr_mesh_distance(a, b, 10, nil) == FR_ERR_INVALID_ARGUMENT)
    #expect(fr_mesh_distance(nil, b, 10, &stats) == FR_ERR_INVALID_ARGUMENT)
    #expect(fr_mesh_distance(a, nil, 10, &stats) == FR_ERR_INVALID_ARGUMENT)
    #expect(!String(cString: fr_last_error()).isEmpty)

    b.pointee.indices[4] = 999
    #expect(fr_mesh_distance(a, b, 10, &stats) == FR_ERR_INVALID_ARGUMENT)
    b.pointee.indices[4] = 1
    a.pointee.positions[2] = .nan
    #expect(fr_mesh_distance(a, b, 10, &stats) == FR_ERR_INVALID_ARGUMENT)

    var empty: UnsafeMutablePointer<fr_mesh>?
    #expect(fr_mesh_create(0, 0, 0, &empty) == FR_OK)
    #expect(fr_mesh_distance(b, empty, 10, &stats) == FR_ERR_NOT_ENOUGH_DATA)
    fr_mesh_free(empty)
}

@Test func evalLargeMeshesAreFast() throws {
    let inner = icosphere(radius: 0.1, center: [0, 0, 0.6], subdivisions: 6)   // 82k triangles
    let outer = icosphere(radius: 0.1005, center: [0, 0, 0.6], subdivisions: 6)
    let start = Date()
    let (status, stats) = try distance(outer, inner, samples: 500_000)
    #expect(status == FR_OK)
    #expect(abs(stats.mean_m - 0.0005) < 3e-5)
    #expect(Date().timeIntervalSince(start) < 10)
}

// MARK: - fr_fit_plane

private func planePoints(normal n: SIMD3<Double>, d: Double, count: Int, noise: Double, seed: UInt64) -> [Float] {
    var random = TestRandom(seed)
    let normal = simd_normalize(n)
    let u = simd_normalize(simd_cross(normal, abs(normal.x) < 0.9 ? SIMD3(1, 0, 0) : SIMD3(0, 1, 0)))
    let v = simd_cross(normal, u)
    var points: [Float] = []
    for _ in 0 ..< count {
        let p = -d * normal + (random.uniform() - 0.5) * 0.3 * u + (random.uniform() - 0.5) * 0.3 * v
            + noise * random.gaussian() * normal
        points += [Float(p.x), Float(p.y), Float(p.z)]
    }
    return points
}

private func fit(_ points: [Float], threshold: Float) -> (fr_status, fr_plane_fit) {
    var result = fr_plane_fit()
    let status = points.withUnsafeBufferPointer { fr_fit_plane($0.baseAddress, UInt32(points.count / 3), threshold, &result) }
    return (status, result)
}

@Test func evalPlaneFitRecoversAnExactPlane() {
    // A wall seen by the camera: normal toward the camera (-Z), 0.8 m away.
    let normal = simd_normalize(SIMD3<Double>(0.1, -0.2, -1))
    let points = planePoints(normal: normal, d: 0.8, count: 2_000, noise: 0, seed: 3)
    let (status, plane) = fit(points, threshold: 0)
    #expect(status == FR_OK)
    let n = SIMD3<Double>(Double(plane.normal.0), Double(plane.normal.1), Double(plane.normal.2))
    #expect(simd_dot(n, normal) > 1 - 1e-6)     // oriented toward the origin
    #expect(abs(Double(plane.d) - 0.8) < 1e-5)
    #expect(plane.rms_m < 1e-6)
    #expect(plane.inliers == 2_000)
}

@Test func evalPlaneFitMeasuresNoise() {
    let normal = SIMD3<Double>(0, 0, -1)
    let sigma = 0.0005
    let points = planePoints(normal: normal, d: 0.6, count: 20_000, noise: sigma, seed: 11)
    let (status, plane) = fit(points, threshold: 0)
    #expect(status == FR_OK)
    #expect(abs(plane.rms_m - sigma) < 0.03 * sigma)
    #expect(abs(Double(plane.d) - 0.6) < 2e-5)
}

@Test func evalRansacPlaneFitIgnoresOutliers() {
    let normal = simd_normalize(SIMD3<Double>(0.3, 0.1, -1))
    let sigma = 0.0005
    var points = planePoints(normal: normal, d: 0.7, count: 7_000, noise: sigma, seed: 5)
    var random = TestRandom(99)
    for _ in 0 ..< 3_000 {   // 30% clutter in front of the wall
        points += [Float(random.uniform() - 0.5) * 0.3, Float(random.uniform() - 0.5) * 0.3,
                   Float(0.3 + random.uniform() * 0.3)]
    }
    let (status, robust) = fit(points, threshold: Float(3 * sigma))
    #expect(status == FR_OK)
    let n = SIMD3<Double>(Double(robust.normal.0), Double(robust.normal.1), Double(robust.normal.2))
    #expect(simd_dot(n, normal) > 0.9999)
    #expect(abs(Double(robust.d) - 0.7) < 1e-4)
    #expect(robust.inliers >= 6_900 && robust.inliers <= 7_100)
    #expect(robust.rms_m < 1.1 * sigma)

    let (_, plain) = fit(points, threshold: 0)
    #expect(plain.rms_m > 10 * sigma)   // least squares alone is ruined by the clutter
}

@Test func evalPlaneFitRejectsDegenerateInput() {
    #expect(fit([0, 0, 0, 1, 1, 1], threshold: 0).0 == FR_ERR_NOT_ENOUGH_DATA)
    let collinear: [Float] = (0 ..< 10).flatMap { [Float($0), 2 * Float($0), 0.5] }
    #expect(fit(collinear, threshold: 0).0 == FR_ERR_NOT_ENOUGH_DATA)
    #expect(fit(collinear, threshold: 0.001).0 == FR_ERR_NOT_ENOUGH_DATA)
    #expect(fit([0, 0, 1, 1, 0, 1, 0, 1, 1], threshold: -1).0 == FR_ERR_INVALID_ARGUMENT)
    var result = fr_plane_fit()
    #expect(fr_fit_plane(nil, 5, 0, &result) == FR_ERR_INVALID_ARGUMENT)
    // Non-finite points (invalid depth) are skipped.
    let withNaN: [Float] = [0, 0, 1, 1, 0, 1, .nan, 0, 1, 0, 1, 1, .infinity, 2, 2]
    let (status, plane) = fit(withNaN, threshold: 0)
    #expect(status == FR_OK && plane.inliers == 3)
}

// MARK: - fr_pose_error

@Test func evalPoseErrorMeasuresRotationAndTranslation() {
    var a = rotation(axis: [0.2, 1, 0.1], degrees: 20)
    a.columns.3 = SIMD4(0.01, -0.02, 0.6, 1)
    var b = a * rotation(axis: [1, -0.5, 0.3], degrees: 30)
    b.columns.3 = SIMD4(0.01 + 0.01, -0.02 + 0.02, 0.6 - 0.02, 1)
    let (r, t) = poseError(a, b)
    #expect(abs(r - 30) < 1e-3)
    #expect(abs(t - 0.03) < 1e-6)
    #expect(poseError(a, a).0 < 1e-3 && poseError(a, a).1 == 0)

    let small = poseError(matrix_identity_float4x4, rotation(axis: [0, 0, 1], degrees: 0.01))
    #expect(abs(small.0 - 0.01) < 1e-4)
    let half = poseError(matrix_identity_float4x4, rotation(axis: [0, 1, 0], degrees: 180))
    #expect(abs(half.0 - 180) < 1e-3)
    let large = poseError(matrix_identity_float4x4, rotation(axis: [1, 1, 0], degrees: 170))
    #expect(abs(large.0 - 170) < 1e-3)

    var rotationDeg: Float = 0, translationM: Float = 0
    fr_pose_error(nil, nil, &rotationDeg, &translationM)
    #expect(rotationDeg == -1 && translationM == -1)
}
