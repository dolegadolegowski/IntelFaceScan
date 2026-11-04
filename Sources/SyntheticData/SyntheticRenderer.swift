// License: Apache 2.0. See LICENSE file in root directory.
//
// CPU renderer for the synthetic head: z-buffer visibility on all cores, then per-pixel ray casting against the
// visible triangle (so depth equals the exact ray/mesh intersection at the pixel center), solid texture shading,
// and a D455-like depth model: invalid pixels at grazing angles, at depth discontinuities and as random speckle,
// Gaussian noise with sigma = noise(1 m) * z^2, quantization to the depth unit.

import CameraKit
import Dispatch
import Foundation

struct RenderSettings: Sendable {
    var intrinsics: Intrinsics
    var depthScale: Float = 0.0001
    var depthNoiseAt1m: Float = 0.002
    /// Surfaces seen at a larger angle between normal and viewing ray have no depth (stereo matching fails there).
    var maxIncidenceDeg: Float = 80
    var dropoutRate: Float = 0.01
    /// Sensor noise of the color image (8-bit levels, standard deviation).
    var colorNoise: Float = 1.5
    /// 0 = flat albedo, 1 = full Lambertian shading from a light at the camera.
    var lighting: Float = 0.25
    /// Fronto-parallel wall behind the head (camera-frame z in meters), nil = nothing.
    var backgroundDistance: Float? = 1.1
    var minDepth: Float = 0.1
    var maxDepth: Float = 6.5
    var seed: UInt64 = 1
}

struct RenderOutput: Sendable {
    var color: Data       // RGB8, tightly packed
    var depth: Data       // little-endian UInt16
    var mask: Data        // 255 = head
    /// Noise-free depth of the visible surface (meters, 0 = nothing), before any invalidation.
    var cleanDepth: [Float]
}

enum SyntheticRenderer {
    static func render(model: HeadModel, cameraFromLocal pose: Rigid, settings: RenderSettings,
                       frameKey: UInt64) -> RenderOutput {
        let width = settings.intrinsics.width, height = settings.intrinsics.height
        let pixelCount = width * height
        let fx = settings.intrinsics.fx, fy = settings.intrinsics.fy
        let cx = settings.intrinsics.ppx, cy = settings.intrinsics.ppy
        let vertexCount = model.vertexCount, triangleCount = model.triangleCount
        let near: Float = 0.02

        var camera = [Float](repeating: 0, count: vertexCount * 3)
        var screen = [Float](repeating: 0, count: vertexCount * 2)
        var bounds = [Int32](repeating: -1, count: triangleCount * 4)
        var inverseDepth = [Float](repeating: 0, count: pixelCount)
        var visible = [Int32](repeating: -1, count: pixelCount)
        var clean = [Float](repeating: 0, count: pixelCount)
        var grazing = [UInt8](repeating: 0, count: pixelCount)
        var color = Data(count: pixelCount * 3)
        var mask = Data(count: pixelCount)
        var depth = Data(count: pixelCount * 2)

        model.positions.withUnsafeBufferPointer { positionBuffer in
        model.normals.withUnsafeBufferPointer { normalBuffer in
        model.indices.withUnsafeBufferPointer { indexBuffer in
        camera.withUnsafeMutableBufferPointer { cameraBuffer in
        screen.withUnsafeMutableBufferPointer { screenBuffer in
        bounds.withUnsafeMutableBufferPointer { boundsBuffer in
        inverseDepth.withUnsafeMutableBufferPointer { inverseDepthBuffer in
        visible.withUnsafeMutableBufferPointer { visibleBuffer in
        clean.withUnsafeMutableBufferPointer { cleanBuffer in
        grazing.withUnsafeMutableBufferPointer { grazingBuffer in
        color.withUnsafeMutableBytes { colorBytes in
        mask.withUnsafeMutableBytes { maskBytes in
        depth.withUnsafeMutableBytes { depthBytes in
            let local = positionBuffer.baseAddress!, localNormal = normalBuffer.baseAddress!
            let index = indexBuffer.baseAddress!
            let cam = cameraBuffer.baseAddress!, scr = screenBuffer.baseAddress!, box = boundsBuffer.baseAddress!
            let zinv = inverseDepthBuffer.baseAddress!, tri = visibleBuffer.baseAddress!
            let zClean = cleanBuffer.baseAddress!, graze = grazingBuffer.baseAddress!
            let rgb = colorBytes.bindMemory(to: UInt8.self).baseAddress!
            let head = maskBytes.bindMemory(to: UInt8.self).baseAddress!
            let raw = depthBytes.bindMemory(to: UInt16.self).baseAddress!

            // 1. Vertices to the camera frame and to pixels.
            parallelChunks(vertexCount, minChunk: 2048) { range in
                var i = range.lowerBound
                while i < range.upperBound {
                    let p = pose.apply(Vec3(local[3 * i], local[3 * i + 1], local[3 * i + 2]))
                    cam[3 * i] = p.x; cam[3 * i + 1] = p.y; cam[3 * i + 2] = p.z
                    if p.z > near {
                        scr[2 * i] = fx * p.x / p.z + cx
                        scr[2 * i + 1] = fy * p.y / p.z + cy
                    }
                    i += 1
                }
            }

            // 2. Triangle setup: back-face and near-plane culling, pixel-center bounding boxes.
            parallelChunks(triangleCount, minChunk: 4096) { range in
                var t = range.lowerBound
                while t < range.upperBound {
                    let a = Int(index[3 * t]), b = Int(index[3 * t + 1]), c = Int(index[3 * t + 2])
                    let pa = Vec3(cam[3 * a], cam[3 * a + 1], cam[3 * a + 2])
                    let pb = Vec3(cam[3 * b], cam[3 * b + 1], cam[3 * b + 2])
                    let pc = Vec3(cam[3 * c], cam[3 * c + 1], cam[3 * c + 2])
                    box[4 * t] = -1
                    if pa.z > near && pb.z > near && pc.z > near && (pb - pa).cross(pc - pa).dot(pa) < 0 {
                        let ua = scr[2 * a], ub = scr[2 * b], uc = scr[2 * c]
                        let va = scr[2 * a + 1], vb = scr[2 * b + 1], vc = scr[2 * c + 1]
                        let minX = max(0, Int(ceilf(min(ua, ub, uc)).clamped(-1, Float(width))))
                        let maxX = min(width - 1, Int(floorf(max(ua, ub, uc)).clamped(-1, Float(width))))
                        let minY = max(0, Int(ceilf(min(va, vb, vc)).clamped(-1, Float(height))))
                        let maxY = min(height - 1, Int(floorf(max(va, vb, vc)).clamped(-1, Float(height))))
                        if minX <= maxX && minY <= maxY {
                            box[4 * t] = Int32(minX); box[4 * t + 1] = Int32(maxX)
                            box[4 * t + 2] = Int32(minY); box[4 * t + 3] = Int32(maxY)
                        }
                    }
                    t += 1
                }
            }

            // 3. Visibility: each band of rows is owned by one worker, so the z-buffer needs no locking. Edges are
            // tested with a small tolerance (shared edges are covered twice and the depth test picks one) so that
            // rounding never leaves a pixel between two triangles uncovered.
            let bandHeight = 8
            let bands = (height + bandHeight - 1) / bandHeight
            DispatchQueue.concurrentPerform(iterations: bands) { band in
                let y0 = band * bandHeight, y1 = min(height, y0 + bandHeight) - 1
                var t = 0
                while t < triangleCount {
                    let minX = Int(box[4 * t])
                    if minX < 0 { t += 1; continue }
                    let minY = Int(box[4 * t + 2]), maxY = Int(box[4 * t + 3])
                    if maxY < y0 || minY > y1 { t += 1; continue }
                    let maxX = Int(box[4 * t + 1])
                    let a = Int(index[3 * t]), b = Int(index[3 * t + 1]), c = Int(index[3 * t + 2])
                    let ua = scr[2 * a], va = scr[2 * a + 1], ub = scr[2 * b], vb = scr[2 * b + 1]
                    let uc = scr[2 * c], vc = scr[2 * c + 1]
                    let area = (ub - ua) * (vc - va) - (vb - va) * (uc - ua)
                    if abs(area) < 1e-9 { t += 1; continue }
                    let inv = 1 / area
                    let iza = 1 / cam[3 * a + 2], izb = 1 / cam[3 * b + 2], izc = 1 / cam[3 * c + 2]
                    let eps: Float = -1e-5
                    var y = max(minY, y0)
                    let yEnd = min(maxY, y1)
                    while y <= yEnd {
                        let py = Float(y)
                        var x = minX
                        let row = y * width
                        while x <= maxX {
                            let px = Float(x)
                            let w0 = ((uc - ub) * (py - vb) - (vc - vb) * (px - ub)) * inv
                            let w1 = ((ua - uc) * (py - vc) - (va - vc) * (px - uc)) * inv
                            let w2 = 1 - w0 - w1
                            if w0 >= eps && w1 >= eps && w2 >= eps {
                                let z = w0 * iza + w1 * izb + w2 * izc
                                if z > zinv[row + x] {
                                    zinv[row + x] = z
                                    tri[row + x] = Int32(t)
                                }
                            }
                            x += 1
                        }
                        y += 1
                    }
                    t += 1
                }
            }

            // 4. Shading: exact ray/triangle intersection for depth, interpolated normal and texture coordinates.
            let cosMax = cosf(settings.maxIncidenceDeg * Float.pi / 180)
            let lighting = clamp01(settings.lighting)
            let r = pose.r
            let colorKey = SplitMix64.mix(settings.seed ^ 0xC0_102) ^ frameKey
            let background = settings.backgroundDistance
            parallelChunks(height, minChunk: 4) { rows in
                var y = rows.lowerBound
                while y < rows.upperBound {
                    var x = 0
                    while x < width {
                        let i = y * width + x
                        let dx = (Float(x) - cx) / fx, dy = (Float(y) - cy) / fy
                        let rayLength = (dx * dx + dy * dy + 1).squareRoot()
                        var red: Float = 0.08, green: Float = 0.08, blue: Float = 0.09
                        let t = Int(tri[i])
                        if t >= 0 {
                            let a = Int(index[3 * t]), b = Int(index[3 * t + 1]), c = Int(index[3 * t + 2])
                            let p0 = Vec3(cam[3 * a], cam[3 * a + 1], cam[3 * a + 2])
                            let e1 = Vec3(cam[3 * b], cam[3 * b + 1], cam[3 * b + 2]) - p0
                            let e2 = Vec3(cam[3 * c], cam[3 * c + 1], cam[3 * c + 2]) - p0
                            let dir = Vec3(dx, dy, 1)
                            let pv = dir.cross(e2)
                            let det = e1.dot(pv)
                            var b1: Float = 1 / 3, b2: Float = 1 / 3, z: Float = 1 / zinv[i]
                            var exact = false
                            if abs(det) > 1e-14 {
                                let invDet = 1 / det
                                let tv = p0 * -1
                                let qv = tv.cross(e1)
                                b1 = tv.dot(pv) * invDet
                                b2 = dir.dot(qv) * invDet
                                z = e2.dot(qv) * invDet
                                exact = z > near
                            }
                            // The pixel center may lie a hair outside the triangle (edge tolerance): clamp the
                            // weights for attributes; depth stays the exact intersection with the triangle's plane.
                            var w1 = max(b1, 0), w2 = max(b2, 0), w0 = max(1 - b1 - b2, 0)
                            let sum = w0 + w1 + w2
                            w0 /= sum; w1 /= sum; w2 /= sum
                            let pl = Vec3(local[3 * a], local[3 * a + 1], local[3 * a + 2]) * w0
                                + Vec3(local[3 * b], local[3 * b + 1], local[3 * b + 2]) * w1
                                + Vec3(local[3 * c], local[3 * c + 1], local[3 * c + 2]) * w2
                            let nl = (Vec3(localNormal[3 * a], localNormal[3 * a + 1], localNormal[3 * a + 2]) * w0
                                + Vec3(localNormal[3 * b], localNormal[3 * b + 1], localNormal[3 * b + 2]) * w1
                                + Vec3(localNormal[3 * c], localNormal[3 * c + 1], localNormal[3 * c + 2]) * w2).normalized
                            let n = r.apply(nl)
                            let cosine = -(n.dot(dir)) / rayLength
                            let footprint = z * rayLength / fx / max(cosine, 0.2)
                            let (ar, ag, ab) = HeadShape.albedo(pl, footprint: footprint)
                            let shade = 1 - lighting + lighting * max(cosine, 0)
                            red = ar * shade; green = ag * shade; blue = ab * shade
                            zClean[i] = exact ? z : 0
                            graze[i] = cosine < cosMax || !exact ? 1 : 0
                            head[i] = pl.y > HeadShape.maskCutU ? 255 : 0
                        } else if let wall = background {
                            let wx = dx * wall, wy = dy * wall
                            let v = 0.06 * ValueNoise.sample(wx * 12, wy * 12, 5) + 0.04 * ValueNoise.sample(wx * 90, wy * 90, 9)
                            let shade = 1 - lighting + lighting / rayLength
                            red = (0.72 + v) * shade; green = (0.71 + v) * shade; blue = (0.67 + v) * shade
                            zClean[i] = wall
                        }
                        let noise = settings.colorNoise
                        let n0 = noise * CounterRandom.gaussian(colorKey, UInt64(3 * i))
                        let n1 = noise * CounterRandom.gaussian(colorKey, UInt64(3 * i + 1))
                        let n2 = noise * CounterRandom.gaussian(colorKey, UInt64(3 * i + 2))
                        rgb[3 * i] = UInt8((red * 255 + n0 + 0.5).clamped(0, 255))
                        rgb[3 * i + 1] = UInt8((green * 255 + n1 + 0.5).clamped(0, 255))
                        rgb[3 * i + 2] = UInt8((blue * 255 + n2 + 0.5).clamped(0, 255))
                        x += 1
                    }
                    y += 1
                }
            }

            // 5. Depth: stereo-like invalidation, noise, quantization.
            let noiseKey = SplitMix64.mix(settings.seed ^ 0xDE_9750) ^ frameKey
            let dropKey = SplitMix64.mix(settings.seed ^ 0xD8_0900) ^ frameKey
            let scale = settings.depthScale
            let noise1m = settings.depthNoiseAt1m
            let dropout = settings.dropoutRate
            let minDepth = settings.minDepth, maxDepth = min(settings.maxDepth, Float(UInt16.max) * scale)
            parallelChunks(height, minChunk: 4) { rows in
                var y = rows.lowerBound
                while y < rows.upperBound {
                    var x = 0
                    while x < width {
                        let i = y * width + x
                        var value: UInt16 = 0
                        let z = zClean[i]
                        if z > 0 && graze[i] == 0 {
                            // Matching fails across depth jumps (both sides) and next to empty pixels.
                            let jump = 0.004 + 0.015 * z
                            var edge = false
                            var ny = max(y - 1, 0)
                            while ny <= min(y + 1, height - 1) && !edge {
                                var nx = max(x - 1, 0)
                                while nx <= min(x + 1, width - 1) {
                                    let zn = zClean[ny * width + nx]
                                    if zn == 0 || abs(zn - z) > jump { edge = true; break }
                                    nx += 1
                                }
                                ny += 1
                            }
                            if !edge && CounterRandom.uniform(dropKey, UInt64(i)) >= dropout {
                                let noisy = z + noise1m * z * z * CounterRandom.gaussian(noiseKey, UInt64(i))
                                if noisy >= minDepth && noisy <= maxDepth {
                                    value = UInt16(max(1, min(65535, (noisy / scale).rounded())))
                                }
                            }
                        }
                        raw[i] = value.littleEndian
                        x += 1
                    }
                    y += 1
                }
            }
        }}}}}}}}}}}}}

        return RenderOutput(color: color, depth: depth, mask: mask, cleanDepth: clean)
    }
}

extension Float {
    @inline(__always) func clamped(_ lo: Float, _ hi: Float) -> Float { self < lo ? lo : (self > hi ? hi : self) }
}
