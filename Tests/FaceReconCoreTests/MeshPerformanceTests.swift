// License: Apache 2.0. See LICENSE file in root directory.
//
// Timings on production-sized data. Opt-in (slow to set up in debug builds):
//   IFS_PERF=1 scripts/test.sh -c release -Xswiftc -enable-testing --filter MeshPerformanceTests

import FaceReconCore
import Foundation
import ReconKit
import simd
import Testing

@Suite("Mesh performance", .enabled(if: ProcessInfo.processInfo.environment["IFS_PERF"] != nil))
struct MeshPerformanceTests {
    @Test func millionTriangleCleanupAndExport() throws {
        var mesh = MeshFactory.gridSphere(rings: 710, segments: 710)
        var rng = TestRandom(seed: 5)
        mesh.positions = mesh.positions.map { $0 * (1 + 0.002 * rng.gaussian()) }
        mesh.colors = mesh.positions.map { p in SIMD3(UInt8(128 + 1000 * p.x), UInt8(128 + 1000 * p.y), 90) }
        // A neck cut and a few holes, as a fused head would have.
        mesh.indices = stride(from: 0, to: mesh.indices.count, by: 3).flatMap { t -> [UInt32] in
            let c = (mesh.positions[Int(mesh.indices[t])] + mesh.positions[Int(mesh.indices[t + 1])]) / 2
            return c.y < -0.08 || simd_distance(c, [0.1, 0, 0]) < 0.004 ? [] : Array(mesh.indices[t ..< t + 3])
        }
        print("perf mesh: \(mesh.vertexCount) vertices, \(mesh.triangleCount) triangles")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var params = fr_smooth_params()
        fr_smooth_params_default(&params)
        params.bilateral_sigma_spatial_m = 0
        let dir = try TemporaryDirectory()
        var timings: [(String, Double)] = []
        timings.append(("clean", try measure { try check(fr_mesh_clean(core, 1e-6)) }))
        timings.append(("components", try measure { try check(fr_mesh_remove_small_components(core, 0.05, 0, nil)) }))
        timings.append(("smooth", try measure { try check(fr_mesh_smooth(core, &params)) }))
        timings.append(("fill holes", try measure { try check(fr_mesh_fill_holes(core, 400, nil)) }))
        timings.append(("normals", try measure { try check(fr_mesh_compute_normals(core)) }))
        var options = fr_export_options()
        fr_export_options_default(&options)
        let obj = dir.url.appendingPathComponent("perf.obj"), ply = dir.url.appendingPathComponent("perf.ply")
        timings.append(("export obj", try measure { try check(fr_export_obj(core, nil, obj.path, &options)) }))
        timings.append(("export ply", try measure { try check(fr_export_ply(core, ply.path, &options)) }))
        var imported: UnsafeMutablePointer<fr_mesh>?
        timings.append(("import obj", try measure { try check(fr_import_obj(obj.path, &imported)) }))
        fr_mesh_free(imported)
        let copy = try mesh.makeCoreMesh()
        defer { fr_mesh_free(copy) }
        timings.append(("decimate to 200k", try measure { try check(fr_mesh_decimate(copy, 200_000, 0)) }))
        let size = (try FileManager.default.attributesOfItem(atPath: obj.path)[.size] as? Int) ?? 0
        print("perf 1M: " + timings.map { "\($0.0) \(String(format: "%.3f", $0.1)) s" }.joined(separator: ", ")
            + "; obj \(size / 1_000_000) MB")
        let total = timings.prefix(7).reduce(0) { $0 + $1.1 }
        print("perf 1M: clean + smoothing + holes + normals + OBJ + PLY = \(String(format: "%.2f", total)) s")
        #expect(total < 10)
    }

    @Test func texture4096FromFortyFrames() throws {
        var mesh = MeshFactory.icosphere(level: 7, radius: TexturedSphere.radius)  // 327k triangles
        var uv = fr_uv_params()
        fr_uv_params_default(&uv)
        let unwrap = try measure {
            try mesh.withCoreMesh {
                try check(fr_mesh_compute_normals($0))
                try check(fr_mesh_uv_unwrap($0, &uv, nil, nil))
            }
        }
        // 40 views around the sphere at 0.35 m, 1280 x 800 (D455 color).
        var poses: [simd_float4x4] = []
        for k in 0 ..< 40 {
            let azimuth = Float(k) * 2 * .pi * 0.382, elevation = asin(Float(k) / 20 - 0.975) * 0.8
            poses.append(TexturedSphere.lookAt(eye: 0.35 * SIMD3(cos(elevation) * sin(azimuth), -sin(elevation),
                                                                 -cos(elevation) * cos(azimuth))))
        }
        var views: [SyntheticView] = []
        let render = measure { views = poses.map { SyntheticView.render(pose: $0, width: 1280, height: 800, focal: 900) } }
        let store = FrameStore(views)
        var params = fr_texture_params()
        fr_texture_params_default(&params)
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var texture: UnsafeMutablePointer<fr_texture>?
        let bake = try measure {
            try store.withPointers { frames, poses, count in
                try check(fr_bake_texture(core, frames, poses, nil, count, &params, &texture, nil, nil))
            }
        }
        let t = try #require(texture)
        defer { fr_texture_free(t) }
        let psnr = TextureMeasure.spherePSNR(mesh: mesh, rgba: t.pointee.rgba, width: 4096, height: 4096)
        let dir = try TemporaryDirectory()
        let png = dir.url.appendingPathComponent("atlas.png")
        let write = try measure {
            try check(fr_write_png(png.path, t.pointee.rgba, 4096, 4096, 4))
        }
        print("perf texture: \(mesh.triangleCount) triangles, unwrap \(String(format: "%.2f", unwrap)) s, bake 4096 from 40 x 1280x800 "
            + "\(String(format: "%.2f", bake)) s (psnr \(String(format: "%.1f", psnr)) dB), png \(String(format: "%.2f", write)) s, "
            + "rendering the views \(String(format: "%.1f", render)) s")
        #expect(bake < 20)
        #expect(psnr > 30)
    }
}
