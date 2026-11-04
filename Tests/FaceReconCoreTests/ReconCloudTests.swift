// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Foundation
import Testing

@Suite("Recon clouds")
struct ReconCloudTests {
    static let camera = SynthCamera(width: 160, height: 120, fx: 150, fy: 150, cx: 79.5, cy: 59.5)

    /// A frame with a depth image computed by `depthAt(u, v)` (meters, 0 = none).
    static func frame(depthAt: (Int, Int) -> Double) -> RenderedFrame {
        let f = RenderedFrame(camera: camera, depthScale: 0.0001, worldFromCamera: .identity)
        for v in 0 ..< camera.height {
            for u in 0 ..< camera.width {
                let i = v * camera.width + u
                let d = depthAt(u, v)
                guard d > 0 else { continue }
                f.depth[i] = UInt16((d / 0.0001).rounded())
                f.mask[i] = 255
                f.rgb[3 * i] = UInt8(u % 256); f.rgb[3 * i + 1] = UInt8(v % 256); f.rgb[3 * i + 2] = 100
            }
        }
        return f
    }

    static func cloud(_ f: RenderedFrame, params: fr_cloud_params? = nil, useMask: Bool = true) throws -> CoreCloud {
        var frame = f.coreFrame(useMask: useMask)
        var p = params ?? {
            var d = fr_cloud_params()
            fr_cloud_params_default(&d)
            return d
        }()
        var out: OpaquePointer?
        try #require(fr_cloud_from_rgbd(&frame, &p, &out) == FR_OK, "\(String(cString: fr_last_error()))")
        return CoreCloud(try #require(out))
    }

    @Test func planeFacingTheCameraGivesCameraFacingNormals() throws {
        let f = Self.frame { _, _ in 0.6 }
        let c = try Self.cloud(f)
        // Image border and mask_erode_px (2) pixels around it are removed: (w - 6) x (h - 6) points remain.
        #expect(c.count == (Self.camera.width - 6) * (Self.camera.height - 6))
        for (p, n) in zip(c.points, c.normals) {
            #expect(abs(p.z - 0.6) < 1e-4)
            #expect(n.z < -0.999)
        }
        #expect(fr_cloud_colors(c.pointer) != nil)
    }

    @Test func maskAndDepthRangeAreHonored() throws {
        let f = Self.frame { u, _ in u < 120 ? 0.6 : 2.0 }   // far background on the right
        for v in 0 ..< Self.camera.height { for u in 0 ..< 40 { f.mask[v * Self.camera.width + u] = 0 } }
        let c = try Self.cloud(f)
        let us = c.points.map { $0.x / $0.z * Self.camera.fx + Self.camera.cx }
        #expect(c.count > 0)
        #expect(us.allSatisfy { $0 >= 40 + 2 - 0.01 }, "eroded mask boundary")
        #expect(c.points.allSatisfy { $0.z < 1.5 }, "max depth 1.5 m")
    }

    @Test func flyingPixelsAtSilhouettesAreRemoved() throws {
        // Foreground square at 0.5 m on a 1.0 m background, with a two-pixel ramp of interpolated depths between
        // them - what stereo matching produces along silhouettes.
        let f = Self.frame { u, v in
            let inside = u >= 50 && u < 110 && v >= 30 && v < 90
            if inside { return 0.5 }
            let du = u < 50 ? 50 - u : (u >= 110 ? u - 109 : 0)
            let dv = v < 30 ? 30 - v : (v >= 90 ? v - 89 : 0)
            let ring = max(du, dv)
            if ring <= 2 { return 0.5 + 0.5 * Double(ring) / 3 }
            return 1.0
        }
        let c = try Self.cloud(f)
        let flying = c.points.filter { $0.z > 0.51 && $0.z < 0.99 }
        #expect(flying.isEmpty, "\(flying.count) flying points survived")
        #expect(c.points.contains { $0.z < 0.51 } && c.points.contains { $0.z > 0.99 })
    }

    @Test func isolatedDepthHolesAreNotTreatedAsSilhouettes() throws {
        // A plane at 0.6 m with 8% scattered one-pixel holes (stereo matching failures) and one 12 x 12 hole. Only
        // the large hole is a silhouette: its 2-pixel rim goes, the speckle neighbors stay. Treating every speckle as
        // a silhouette erased (2 * 2 + 1)^2 pixels per hole: 88% of this plane, 98% of a synthetic head cloud.
        var rng = SplitMix64(seed: 5)
        var speckle = Set<Int>()
        for i in 0 ..< Self.camera.width * Self.camera.height where rng.uniform() < 0.08 { speckle.insert(i) }
        let big = (x: 70 ..< 82, y: 50 ..< 62)
        let f = Self.frame { u, v in
            if big.x.contains(u) && big.y.contains(v) { return 0 }
            return speckle.contains(v * Self.camera.width + u) ? 0 : 0.6
        }
        let c = try Self.cloud(f)
        let pixels = c.points.map { (Int(($0.x / $0.z * Self.camera.fx + Self.camera.cx).rounded()),
                                     Int(($0.y / $0.z * Self.camera.fy + Self.camera.cy).rounded())) }
        // Valid pixels away from the image border and the big hole (2-pixel erosion each).
        var eligible = 0
        for v in 3 ..< Self.camera.height - 3 {
            for u in 3 ..< Self.camera.width - 3 where !speckle.contains(v * Self.camera.width + u) {
                if (big.x.lowerBound - 2 ..< big.x.upperBound + 2).contains(u) && (big.y.lowerBound - 2 ..< big.y.upperBound + 2).contains(v) {
                    continue
                }
                eligible += 1
            }
        }
        #expect(c.count > eligible * 95 / 100, "\(c.count) points of \(eligible) valid pixels")
        let nearBig = pixels.filter { (big.x.lowerBound - 2 ..< big.x.upperBound + 2).contains($0.0)
            && (big.y.lowerBound - 2 ..< big.y.upperBound + 2).contains($0.1) }
        #expect(nearBig.isEmpty, "\(nearBig.count) points on the rim of the large hole")
        #expect(c.normals.allSatisfy { $0.z < -0.99 })
    }

    @Test func voxelDownsampleAveragesPerVoxel() throws {
        let points = [V3(0.0011, 0.0011, 0.5011), V3(0.0019, 0.0019, 0.5019), V3(0.0051, 0.0011, 0.5011)]
        let normals = [V3(0, 0, -1), V3(0, 0.6, -0.8), V3(1, 0, 0)]
        let cloud = try #require(CoreCloud.make(points: points, normals: normals, colors: [(0, 0, 0), (100, 200, 50), (7, 7, 7)]))
        var out: OpaquePointer?
        #expect(fr_cloud_voxel_downsample(cloud.pointer, 0.002, &out) == FR_OK)
        let down = CoreCloud(try #require(out))
        #expect(down.count == 2)
        let merged = try #require(down.points.first { $0.x < 0.002 })
        #expect((merged - V3(0.0015, 0.0015, 0.5015)).length < 1e-6)
        let n = try #require(zip(down.points, down.normals).first { $0.0.x < 0.002 }?.1)
        #expect(abs(n.length - 1) < 1e-5)
        #expect(fr_cloud_voxel_downsample(cloud.pointer, 0, &out) == FR_ERR_INVALID_ARGUMENT)
    }

    @Test func transformAppendAndBounds() throws {
        let a = try #require(CoreCloud.make(points: [V3(0, 0, 1), V3(1, 0, 1)], normals: [V3(0, 0, -1), V3(0, 0, -1)]))
        let b = try #require(CoreCloud.make(points: [V3(0, 2, 1), V3(0, 3, 1), V3(0, 4, 1)]))   // normals estimated
        #expect(fr_cloud_normals(b.pointer) != nil)
        let shift = Rigid(axis: V3(0, 0, 1), angleDeg: 90, t: V3(0, 0, 1))
        #expect(fr_cloud_transform(a.pointer, shift.floats) == FR_OK)
        #expect((a.points[1] - V3(0, 1, 2)).length < 1e-6)
        #expect(abs(a.normals[0].z + 1) < 1e-6)
        #expect(fr_cloud_append(a.pointer, b.pointer) == FR_OK)
        #expect(a.count == 5)
        #expect(fr_cloud_colors(a.pointer) == nil)
        var lo: (Float, Float, Float) = (0, 0, 0), hi: (Float, Float, Float) = (0, 0, 0)
        withUnsafeMutablePointer(to: &lo) { lp in
            withUnsafeMutablePointer(to: &hi) { hp in
                lp.withMemoryRebound(to: Float.self, capacity: 3) { l in
                    hp.withMemoryRebound(to: Float.self, capacity: 3) { h in
                        #expect(fr_cloud_bounds(a.pointer, l, h) == FR_OK)
                    }
                }
            }
        }
        #expect(lo.1 == 0 && hi.1 == 4 && lo.2 == 1 && hi.2 == 2)
        let empty = try #require(CoreCloud.make(points: []))
        var dlo = [Float](repeating: 0, count: 3), dhi = [Float](repeating: 0, count: 3)
        #expect(fr_cloud_bounds(empty.pointer, &dlo, &dhi) == FR_ERR_NOT_ENOUGH_DATA)
    }

    @Test func invalidFramesAreRejected() throws {
        let f = Self.frame { _, _ in 0.6 }
        var frame = f.coreFrame()
        var p = fr_cloud_params()
        fr_cloud_params_default(&p)
        var out: OpaquePointer?
        frame.depth_scale = 0
        #expect(fr_cloud_from_rgbd(&frame, &p, &out) == FR_ERR_INVALID_ARGUMENT)
        frame = f.coreFrame()
        frame.intrinsics.fx = .nan
        #expect(fr_cloud_from_rgbd(&frame, &p, &out) == FR_ERR_INVALID_ARGUMENT)
        frame = f.coreFrame()
        p.stride = 0
        #expect(fr_cloud_from_rgbd(&frame, &p, &out) == FR_ERR_INVALID_ARGUMENT)
        #expect(out == nil)
    }
}
