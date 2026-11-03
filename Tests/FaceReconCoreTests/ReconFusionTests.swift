// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Foundation
import Testing

/// Spatial hash for nearest-vertex queries in tests.
struct PointHash {
    let cell: Double
    var buckets: [Int64: [V3]] = [:]

    init(points: [V3], cell: Double) {
        self.cell = cell
        for p in points { buckets[key(p), default: []].append(p) }
    }

    private func coords(_ p: V3) -> (Int64, Int64, Int64) {
        (Int64((p.x / cell).rounded(.down)), Int64((p.y / cell).rounded(.down)), Int64((p.z / cell).rounded(.down)))
    }

    private func key(_ p: V3) -> Int64 { let c = coords(p); return pack(c.0, c.1, c.2) }
    private func pack(_ x: Int64, _ y: Int64, _ z: Int64) -> Int64 { (x & 0x1FFFFF) << 42 | (y & 0x1FFFFF) << 21 | (z & 0x1FFFFF) }

    /// Distance to the nearest stored point within one cell (infinity if none).
    func nearest(_ p: V3) -> Double {
        let c = coords(p)
        var best = Double.infinity
        for dx in -1 ... 1 { for dy in -1 ... 1 { for dz in -1 ... 1 {
            for q in buckets[pack(c.0 + Int64(dx), c.1 + Int64(dy), c.2 + Int64(dz))] ?? [] { best = min(best, (q - p).length) }
        } } }
        return best
    }
}

@Suite("Recon fusion")
struct ReconFusionTests {
    static let center = V3(0, 0, 0.5)
    static let radius = 0.05
    static let camera = SynthCamera(width: 320, height: 240, fx: 400, fy: 400, cx: 159.5, cy: 119.5)

    /// The sphere rendered from views all around it (camera 0.4 m from the surface).
    static let sphereViews: [RenderedFrame] = {
        let poses = lookAtPoses(target: center, distance: 0.45, directions: cubeDirections)
        return poses.enumerated().map { i, pose in
            render(SphereScene(center: center, radius: radius), camera: camera, worldFromCamera: pose, seed: UInt64(i + 1))
        }
    }()

    static func tsdfParams(voxel: Float, gpu: Bool, color: Bool = true) -> fr_tsdf_params {
        var p = fr_tsdf_params()
        fr_tsdf_params_default(&p)
        p.voxel_size_m = voxel
        p.truncation_m = 0.004
        p.use_gpu = gpu ? 1 : 0
        p.integrate_color = color ? 1 : 0
        let m = Float(radius + 0.015)
        p.bounds_min = (Float(center.x) - m, Float(center.y) - m, Float(center.z) - m)
        p.bounds_max = (Float(center.x) + m, Float(center.y) + m, Float(center.z) + m)
        return p
    }

    static func fuse(_ views: [RenderedFrame], params: fr_tsdf_params, minWeight: Float = 0.1) throws -> (CoreMesh, Bool) {
        var p = params
        var tsdf: OpaquePointer?
        try #require(fr_tsdf_create(&p, &tsdf) == FR_OK, "\(String(cString: fr_last_error()))")
        let volume = try #require(tsdf)
        defer { fr_tsdf_free(volume) }
        for view in views {
            var frame = view.coreFrame()
            let pose = view.worldFromCamera.floats
            try #require(fr_tsdf_integrate(volume, &frame, pose) == FR_OK, "\(String(cString: fr_last_error()))")
        }
        var mesh: UnsafeMutablePointer<fr_mesh>?
        try #require(fr_tsdf_extract_mesh(volume, minWeight, &mesh) == FR_OK, "\(String(cString: fr_last_error()))")
        return (CoreMesh(try #require(mesh)), fr_tsdf_uses_gpu(volume) == 1)
    }

    @Test func sphereIsWatertightWithSubVoxelRadiusError() throws {
        let voxel: Float = 0.001
        let (mesh, _) = try Self.fuse(Self.sphereViews, params: Self.tsdfParams(voxel: voxel, gpu: true))
        #expect(mesh.vertexCount > 10000)
        var sum = 0.0, sumSq = 0.0, maxErr = 0.0
        var alignedNormals = 0
        for i in 0 ..< mesh.vertexCount {
            let d = mesh.vertex(i) - Self.center
            let err = d.length - Self.radius
            sum += err; sumSq += err * err; maxErr = max(maxErr, abs(err))
            if mesh.normal(i).dot(d.normalized) > 0.95 { alignedNormals += 1 }
        }
        let n = Double(mesh.vertexCount)
        let bias = sum / n, rms = (sumSq / n).squareRoot()
        #expect(abs(bias) < 0.3 * Double(voxel), "radius bias \(bias * 1000) mm")
        #expect(rms < 0.3 * Double(voxel), "radius RMS error \(rms * 1000) mm")
        #expect(maxErr < Double(voxel), "max radius error \(maxErr * 1000) mm")
        #expect(Double(alignedNormals) > 0.99 * n, "gradient normals point outward")
        // Counter-clockwise seen from outside.
        var outward = 0
        for t in 0 ..< mesh.triangleCount {
            let (a, b, c) = mesh.triangle(t)
            let pa = mesh.vertex(a), pb = mesh.vertex(b), pc = mesh.vertex(c)
            if (pb - pa).cross(pc - pa).dot(pa - Self.center) > 0 { outward += 1 }
        }
        #expect(Double(outward) > 0.995 * Double(mesh.triangleCount))
        #expect(mesh.isWatertight)
        #expect(mesh.eulerCharacteristic == 2)
        // Vertex colors were fused (the sphere is textured, nothing should stay black).
        let colors = mesh.pointer.pointee.colors!
        var dark = 0
        for i in 0 ..< mesh.vertexCount where Int(colors[3 * i]) + Int(colors[3 * i + 1]) + Int(colors[3 * i + 2]) < 30 { dark += 1 }
        #expect(dark < mesh.vertexCount / 100)
    }

    @Test func gpuAndCpuPathsAgree() throws {
        let views = Array(Self.sphereViews.prefix(10))
        let (gpuMesh, usedGPU) = try Self.fuse(views, params: Self.tsdfParams(voxel: 0.001, gpu: true))
        let (cpuMesh, cpuUsedGPU) = try Self.fuse(views, params: Self.tsdfParams(voxel: 0.001, gpu: false))
        #expect(!cpuUsedGPU)
        guard usedGPU else {
            Issue.record("no Metal device: GPU path not exercised")
            return
        }
        let relative = abs(Double(gpuMesh.vertexCount - cpuMesh.vertexCount)) / Double(cpuMesh.vertexCount)
        #expect(relative < 0.001, "vertex counts \(gpuMesh.vertexCount) vs \(cpuMesh.vertexCount)")
        let cpuVertices = (0 ..< cpuMesh.vertexCount).map { cpuMesh.vertex($0) }
        let hash = PointHash(points: cpuVertices, cell: 0.001)
        var maxDistance = 0.0
        for i in 0 ..< gpuMesh.vertexCount { maxDistance = max(maxDistance, min(hash.nearest(gpuMesh.vertex(i)), 0.01)) }
        #expect(maxDistance < 0.00005, "max GPU-CPU vertex distance \(maxDistance * 1000) mm")
    }

    @Test func maskIsHonored() throws {
        // Mask out the left half of every image: only surface seen in right halves may appear.
        let pose = lookAtPoses(target: Self.center, distance: 0.45, directions: [V3(0, 0, -1)])[0]
        let view = render(SphereScene(center: Self.center, radius: Self.radius), camera: Self.camera, worldFromCamera: pose)
        for v in 0 ..< Self.camera.height { for u in 0 ..< Self.camera.width / 2 { view.mask[v * Self.camera.width + u] = 0 } }
        let (mesh, _) = try Self.fuse([view], params: Self.tsdfParams(voxel: 0.001, gpu: true))
        #expect(mesh.vertexCount > 1000)
        var left = 0
        for i in 0 ..< mesh.vertexCount where mesh.vertex(i).x < Self.center.x - 0.002 { left += 1 }
        #expect(left == 0)
    }

    @Test func absurdVolumesAndBadParametersAreRefused() throws {
        var p = Self.tsdfParams(voxel: 0.0005, gpu: true)
        p.bounds_min = (-1, -1, -1)
        p.bounds_max = (1, 1, 1)
        var tsdf: OpaquePointer?
        #expect(fr_tsdf_create(&p, &tsdf) == FR_ERR_INVALID_ARGUMENT)
        #expect(tsdf == nil)
        #expect(String(cString: fr_last_error()).contains("voxel"))
        p = Self.tsdfParams(voxel: 0.001, gpu: false)
        p.truncation_m = 0.0005
        #expect(fr_tsdf_create(&p, &tsdf) == FR_ERR_INVALID_ARGUMENT)
        p = Self.tsdfParams(voxel: 0.001, gpu: false)
        p.bounds_max.0 = p.bounds_min.0
        #expect(fr_tsdf_create(&p, &tsdf) == FR_ERR_INVALID_ARGUMENT)
        #expect(fr_tsdf_create(nil, &tsdf) == FR_ERR_INVALID_ARGUMENT)
    }

    @Test func fuseFramesReportsProgressAndCancels() throws {
        let views = Array(Self.sphereViews.prefix(4))
        var frames = views.map { $0.coreFrame() }
        let poses = views.flatMap { $0.worldFromCamera.floats }
        var params = Self.tsdfParams(voxel: 0.0015, gpu: true)
        final class Box { var calls = 0; var cancelAt = Int.max }
        let box = Box()
        let callback: fr_progress_cb = { user, _, _ in
            let b = Unmanaged<Box>.fromOpaque(user!).takeUnretainedValue()
            b.calls += 1
            return b.calls >= b.cancelAt ? 1 : 0
        }
        var mesh: UnsafeMutablePointer<fr_mesh>?
        let user = Unmanaged.passUnretained(box).toOpaque()
        #expect(fr_fuse_frames(&frames, poses, UInt32(frames.count), &params, 0.1, &mesh, callback, user) == FR_OK)
        #expect(box.calls >= 5)
        #expect(mesh != nil && mesh!.pointee.triangle_count > 0)
        fr_mesh_free(mesh)
        box.calls = 0
        box.cancelAt = 2
        mesh = nil
        #expect(fr_fuse_frames(&frames, poses, UInt32(frames.count), &params, 0.1, &mesh, callback, user) == FR_ERR_CANCELLED)
        #expect(mesh == nil)
    }
}

@Suite("Recon Poisson")
struct ReconPoissonTests {
    @Test func poissonReconstructsSphereFromOrientedPoints() throws {
        let center = V3(0, 0, 0.5), radius = 0.05
        var rng = SplitMix64(seed: 3)
        var points: [V3] = [], normals: [V3] = []
        for _ in 0 ..< 60000 {
            let d = V3(rng.gaussian(), rng.gaussian(), rng.gaussian()).normalized
            points.append(center + d * radius)
            normals.append(d)
        }
        let cloud = try #require(CoreCloud.make(points: points, normals: normals))
        var params = fr_poisson_params()
        fr_poisson_params_default(&params)
        params.depth = 8
        var mesh: UnsafeMutablePointer<fr_mesh>?
        #expect(fr_poisson_reconstruct(cloud.pointer, &params, &mesh, nil, nil) == FR_OK, "\(String(cString: fr_last_error()))")
        let m = CoreMesh(try #require(mesh))
        #expect(m.vertexCount > 1000)
        // Depth 8 over the 1.1x-padded bounding cube: cells of ~0.43 mm.
        var sumSq = 0.0
        for i in 0 ..< m.vertexCount { let e = (m.vertex(i) - center).length - radius; sumSq += e * e }
        let rms = (sumSq / Double(m.vertexCount)).squareRoot()
        #expect(rms < 0.0005, "Poisson radius RMS error \(rms * 1000) mm")
        var outward = 0
        for t in 0 ..< m.triangleCount {
            let (a, b, c) = m.triangle(t)
            let pa = m.vertex(a), pb = m.vertex(b), pc = m.vertex(c)
            if (pb - pa).cross(pc - pa).dot(pa - center) > 0 { outward += 1 }
        }
        #expect(Double(outward) > 0.99 * Double(m.triangleCount), "counter-clockwise seen from outside")
    }

    @Test func poissonNeedsNormals() throws {
        var params = fr_poisson_params()
        fr_poisson_params_default(&params)
        var mesh: UnsafeMutablePointer<fr_mesh>?
        params.depth = 3
        let cloud = try #require(CoreCloud.make(points: bumpySpherePoints(count: 500, radius: 0.05, center: V3(0, 0, 0.5), jitterSeed: 1)))
        #expect(fr_poisson_reconstruct(cloud.pointer, &params, &mesh, nil, nil) == FR_ERR_INVALID_ARGUMENT)
    }
}
