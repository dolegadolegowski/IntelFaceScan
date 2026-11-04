// License: Apache 2.0. See LICENSE file in root directory.
//
// Ray-cast RGB-D views of an analytically textured sphere (with optional occluder, exposure gain, lighting and static
// background), used as ground truth for texture baking and vertex coloring.

import FaceReconCore
import Foundation
import ReconKit
import simd

enum TexturedSphere {
    static let radius: Float = 0.1

    /// Ground-truth albedo (0...255) at a unit direction from the sphere center.
    static func color(_ d: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3(128 + 90 * sin(2.0 * d.x + 0.5),
              128 + 90 * sin(2.5 * d.y + 1.0) * cos(1.5 * d.z),
              128 + 80 * cos(1.8 * d.z - 0.3 + d.x))
    }

    /// world_from_camera of a camera at `eye` looking at `target` (camera +Y down, world +Y down).
    static func lookAt(eye: SIMD3<Float>, target: SIMD3<Float> = .zero) -> simd_float4x4 {
        let z = simd_normalize(target - eye)
        let x = simd_normalize(simd_cross(SIMD3<Float>(0, 1, 0), z))
        let y = simd_cross(z, x)
        return simd_float4x4(columns: (SIMD4(x, 0), SIMD4(y, 0), SIMD4(z, 0), SIMD4(eye, 1)))
    }

    /// Unit direction at an azimuth (deg, from -Z toward +X) and elevation (deg, toward -Y: world +Y is down).
    static func direction(azimuth: Float, elevation: Float) -> SIMD3<Float> {
        let a = azimuth * .pi / 180, e = elevation * .pi / 180
        return SIMD3(cos(e) * sin(a), -sin(e), -cos(e) * cos(a))
    }

    /// Azimuth (deg) of a direction, as in direction(azimuth:elevation:).
    static func azimuth(_ d: SIMD3<Float>) -> Float { atan2(d.x, -d.z) * 180 / .pi }

    /// A camera looking at the center from `distance` in the given direction.
    static func pose(azimuth: Float, elevation: Float, distance: Float = 0.5) -> simd_float4x4 {
        lookAt(eye: distance * direction(azimuth: azimuth, elevation: elevation))
    }

    /// 8 views around the equator and 4 above / 4 below, at `distance` from the center.
    static func surroundingPoses(distance: Float = 0.5) -> [simd_float4x4] {
        var poses: [simd_float4x4] = []
        for k in 0 ..< 8 { poses.append(pose(azimuth: Float(k) * 45, elevation: 0, distance: distance)) }
        for k in 0 ..< 4 { poses.append(pose(azimuth: 22.5 + Float(k) * 90, elevation: 50, distance: distance)) }
        for k in 0 ..< 4 { poses.append(pose(azimuth: 67.5 + Float(k) * 90, elevation: -50, distance: distance)) }
        return poses
    }

    /// Like a guided head scan: three near-duplicate frames each from the front, 50 deg to either side and 35 deg above
    /// and below.
    static func scanPoses() -> [simd_float4x4] {
        let directions: [(Float, Float)] = [(0, 0), (50, 0), (-50, 0), (0, 35), (0, -35)]
        return directions.flatMap { a, e in [-2, 0, 2].map { pose(azimuth: a + Float($0), elevation: e) } }
    }

    /// A cool light from the camera's left and a warm one from its right plus a little ambient light, by the unit
    /// normal in camera coordinates (x right, y down, z forward): the lights of a room, which stay with the camera
    /// while the head turns, so every view sees a surface point lit differently.
    static func roomLight(_ n: SIMD3<Float>) -> SIMD3<Float> {
        let cool = simd_normalize(SIMD3<Float>(-1, -0.3, -1)), warm = simd_normalize(SIMD3<Float>(1, -0.2, -1))
        return SIMD3(0.2, 0.2, 0.2) + max(0, simd_dot(n, cool)) * SIMD3(0.35, 0.45, 0.7)
            + max(0, simd_dot(n, warm)) * SIMD3(0.7, 0.5, 0.3)
    }

    /// Unit vector d in the camera coordinates of world_from_camera `pose`.
    static func inCamera(_ d: SIMD3<Float>, _ pose: simd_float4x4) -> SIMD3<Float> {
        SIMD3(simd_dot(d, SIMD3(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z)),
              simd_dot(d, SIMD3(pose.columns.1.x, pose.columns.1.y, pose.columns.1.z)),
              simd_dot(d, SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)))
    }

    /// Mid-tone pattern of a static scene behind the sphere, by pixel.
    static func backdrop(_ x: Int, _ y: Int) -> SIMD3<Float> {
        let fx = Float(x), fy = Float(y)
        return SIMD3(140 + 50 * sin(fx / 9) * cos(fy / 13), 130 + 45 * sin(fx / 11 + 1), 120 + 40 * cos(fy / 7 + fx / 15))
    }
}

/// One rendered RGB-D frame (pixel centers at integer coordinates, like the library).
struct SyntheticView {
    var width: Int
    var height: Int
    var fx: Float, fy: Float, cx: Float, cy: Float
    var worldFromCamera: simd_float4x4
    var rgb: [UInt8]
    var depth: [UInt16]
    var mask: [UInt8]
    static let depthScale: Float = 0.0001

    struct Occluder {
        var center: SIMD3<Float>
        var radius: Float
        var color: SIMD3<UInt8>
    }

    /// shading: factor of the sphere's albedo by the unit normal in camera coordinates (TexturedSphere.roomLight).
    /// background: color of the pixels that miss the sphere (depth 0 = out of range, mask 0), by pixel. The gain (the
    /// camera's exposure and white balance) applies to both.
    static func render(pose: simd_float4x4, width: Int = 480, height: Int = 360, focal: Float = 420,
                       gain: SIMD3<Float> = [1, 1, 1], occluder: Occluder? = nil,
                       shading: ((SIMD3<Float>) -> SIMD3<Float>)? = nil,
                       background: ((Int, Int) -> SIMD3<Float>)? = nil) -> SyntheticView {
        let cx = Float(width - 1) / 2, cy = Float(height - 1) / 2
        var rgb = [UInt8](repeating: 0, count: width * height * 3)
        var depth = [UInt16](repeating: 0, count: width * height)
        var mask = [UInt8](repeating: 0, count: width * height)
        let r = SIMD3(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z)
        let u = SIMD3(pose.columns.1.x, pose.columns.1.y, pose.columns.1.z)
        let f = SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        let eye = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        /// Ray parameter t (= camera z, since the camera-frame ray has z = 1) of the first hit, or nil.
        func hit(_ d: SIMD3<Float>, _ center: SIMD3<Float>, _ radius: Float) -> Float? {
            let oc = eye - center
            let a = simd_dot(d, d), b = 2 * simd_dot(oc, d), c = simd_dot(oc, oc) - radius * radius
            let disc = b * b - 4 * a * c
            guard disc >= 0 else { return nil }
            let t = (-b - sqrt(disc)) / (2 * a)
            return t > 0 ? t : nil
        }
        for y in 0 ..< height {
            for x in 0 ..< width {
                let dc = SIMD3((Float(x) - cx) / focal, (Float(y) - cy) / focal, 1)
                let d = r * dc.x + u * dc.y + f * dc.z
                var t = hit(d, .zero, TexturedSphere.radius)
                var color = SIMD3<Float>.zero
                if let t0 = t {
                    let n = simd_normalize(eye + t0 * d)
                    color = TexturedSphere.color(n) * gain
                    if let light = shading { color *= light(TexturedSphere.inCamera(n, pose)) }
                }
                if let occ = occluder, let to = hit(d, occ.center, occ.radius), to < (t ?? .infinity) {
                    t = to
                    color = SIMD3<Float>(occ.color)
                }
                let i = y * width + x
                guard let z = t else {
                    if let scene = background {
                        let c = simd_clamp((scene(x, y) * gain).rounded(.toNearestOrEven), .zero, SIMD3(repeating: 255))
                        rgb[3 * i] = UInt8(c.x); rgb[3 * i + 1] = UInt8(c.y); rgb[3 * i + 2] = UInt8(c.z)
                    }
                    continue
                }
                depth[i] = UInt16(min(65535, (z / depthScale).rounded()))
                mask[i] = 255
                rgb[3 * i] = UInt8(max(0, min(255, color.x.rounded())))
                rgb[3 * i + 1] = UInt8(max(0, min(255, color.y.rounded())))
                rgb[3 * i + 2] = UInt8(max(0, min(255, color.z.rounded())))
            }
        }
        return SyntheticView(width: width, height: height, fx: focal, fy: focal, cx: cx, cy: cy, worldFromCamera: pose,
                             rgb: rgb, depth: depth, mask: mask)
    }
}

/// fr_rgbd_frame structs over copies of the views' pixels, valid for the lifetime of this object.
final class FrameStore {
    private(set) var frames: [fr_rgbd_frame] = []
    let poses: [Float]
    private var buffers: [UnsafeMutableRawPointer] = []

    init(_ views: [SyntheticView], withMask: Bool = true, withDepth: Bool = true) {
        poses = views.flatMap { $0.worldFromCamera.columnMajor }
        for v in views {
            var f = fr_rgbd_frame()
            f.rgb = UnsafePointer(copy(v.rgb))
            f.depth = withDepth ? UnsafePointer(copy(v.depth)) : nil
            f.mask = withMask ? UnsafePointer(copy(v.mask)) : nil
            f.depth_scale = SyntheticView.depthScale
            f.intrinsics.width = Int32(v.width); f.intrinsics.height = Int32(v.height)
            f.intrinsics.fx = v.fx; f.intrinsics.fy = v.fy; f.intrinsics.cx = v.cx; f.intrinsics.cy = v.cy
            f.weight = 1
            frames.append(f)
        }
    }

    private func copy<T>(_ array: [T]) -> UnsafeMutablePointer<T> {
        let p = UnsafeMutablePointer<T>.allocate(capacity: array.count)
        p.initialize(from: array, count: array.count)
        buffers.append(UnsafeMutableRawPointer(p))
        return p
    }

    deinit { buffers.forEach { $0.deallocate() } }

    func withPointers<R>(_ body: (UnsafePointer<fr_rgbd_frame>, UnsafePointer<Float>, UInt32) throws -> R) rethrows -> R {
        try frames.withUnsafeBufferPointer { f in
            try poses.withUnsafeBufferPointer { p in try body(f.baseAddress!, p.baseAddress!, UInt32(f.count)) }
        }
    }
}

enum TextureMeasure {
    /// Bilinear lookup (0...255) with OBJ UVs ((0,0) = bottom-left) in an RGBA top-left-origin image.
    static func sample(_ rgba: UnsafePointer<UInt8>, width: Int, height: Int, uv: SIMD2<Float>) -> SIMD3<Float> {
        let x = uv.x * Float(width) - 0.5, y = (1 - uv.y) * Float(height) - 0.5
        let x0 = max(0, min(width - 1, Int(floor(x)))), y0 = max(0, min(height - 1, Int(floor(y))))
        let x1 = min(width - 1, x0 + 1), y1 = min(height - 1, y0 + 1)
        let fx = max(0, min(1, x - Float(x0))), fy = max(0, min(1, y - Float(y0)))
        func px(_ x: Int, _ y: Int) -> SIMD3<Float> {
            let i = (y * width + x) * 4
            return SIMD3(Float(rgba[i]), Float(rgba[i + 1]), Float(rgba[i + 2]))
        }
        return (1 - fy) * ((1 - fx) * px(x0, y0) + fx * px(x1, y0)) + fy * ((1 - fx) * px(x0, y1) + fx * px(x1, y1))
    }

    /// PSNR (dB) of the texture against the sphere's ground truth at points spread over every triangle.
    static func spherePSNR(mesh: ReconMesh, rgba: UnsafePointer<UInt8>, width: Int, height: Int) -> Double {
        let barycentrics: [SIMD3<Float>] = [[1.0 / 3, 1.0 / 3, 1.0 / 3], [0.6, 0.2, 0.2], [0.2, 0.6, 0.2], [0.2, 0.2, 0.6]]
        var sum = 0.0, count = 0
        for t in 0 ..< mesh.triangleCount {
            let i = (0 ..< 3).map { Int(mesh.indices[3 * t + $0]) }
            for b in barycentrics {
                let p = b.x * mesh.positions[i[0]] + b.y * mesh.positions[i[1]] + b.z * mesh.positions[i[2]]
                let uv = b.x * mesh.uvs[i[0]] + b.y * mesh.uvs[i[1]] + b.z * mesh.uvs[i[2]]
                let e = sample(rgba, width: width, height: height, uv: uv) - TexturedSphere.color(simd_normalize(p))
                sum += Double(simd_dot(e, e))
                count += 3
            }
        }
        return psnr(mse: sum / Double(max(count, 1)))
    }

    static func psnr(mse: Double) -> Double { 10 * log10(255.0 * 255.0 / max(mse, 1e-12)) }
}
