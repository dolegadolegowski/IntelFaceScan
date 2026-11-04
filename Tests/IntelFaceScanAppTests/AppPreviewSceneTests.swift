// License: Apache 2.0. See LICENSE file in root directory.
//
// The SceneKit scene of the 3D preview rendered offscreen (SCNRenderer, Metal): the four display modes on an open
// mesh, seen from the front and from behind through the opening, and the scene's bounds in millimeters.

import AppKit
import Foundation
import Metal
import ReconKit
import SceneKit
import simd
import Testing
@testable import IntelFaceScanApp

@Suite("App 3D preview scene")
struct AppPreviewSceneTests {
    /// The front half of a sphere (radius 0.1 m) bulging toward +Z, open at the back like a head scan without the
    /// back of the head; texture all red, vertex colors all green, outward normals, counter-clockwise from outside.
    static func openShell() -> ReconstructionResult {
        let rings = 24, segments = 48, r: Float = 0.1
        var mesh = ReconMesh()
        for i in 0 ... rings {
            let theta = Float(i) / Float(rings) * .pi / 2          // 0 at the pole (+Z), pi/2 at the open rim
            for j in 0 ... segments {
                let phi = Float(j) / Float(segments) * 2 * .pi
                let n = SIMD3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta))
                mesh.positions.append(n * r)
                mesh.normals.append(n)
                mesh.uvs.append(SIMD2(Float(j) / Float(segments), Float(i) / Float(rings)))
                mesh.colors.append(SIMD3(0, 200, 0))
            }
        }
        let stride = UInt32(segments + 1)
        for i in 0 ..< UInt32(rings) {
            for j in 0 ..< UInt32(segments) {
                let a = i * stride + j, b = a + 1, c = a + stride, d = c + 1
                mesh.indices += [a, c, b, b, c, d]
            }
        }
        var red = Data(count: 8 * 8 * 4)
        for p in 0 ..< 64 { red[4 * p] = 220; red[4 * p + 3] = 255 }
        return ReconstructionResult(mesh: mesh, texture: TextureImage(width: 8, height: 8, rgba: red), cameraPoses: [],
                                    exportTransform: matrix_identity_float4x4, report: ReconstructionReport())
    }

    struct Pixel { var r, g, b: Int }

    /// Renders the scene from `eye` (millimeters) toward the model's center; returns the RGBA8 pixels.
    static func render(_ preview: PreviewScene, eye: SIMD3<Float>, size: Int = 96) throws -> (Int, Int) -> Pixel {
        let device = try #require(MTLCreateSystemDefaultDevice())
        let renderer = SCNRenderer(device: device, options: nil)
        preview.scene.background.contents = NSColor.black
        renderer.scene = preview.scene
        // Built by hand: simdLook(at:) degenerates when the view direction is exactly opposite to localFront
        // (straight from behind) and renders nothing.
        let forward = simd_normalize(preview.center - eye)
        let right = simd_normalize(simd_cross(forward, SIMD3(0, 1, 0)))
        let up = simd_cross(right, forward)
        preview.cameraNode.simdTransform = simd_float4x4(SIMD4(right, 0), SIMD4(up, 0), SIMD4(-forward, 0), SIMD4(eye, 1))
        renderer.pointOfView = preview.cameraNode
        let image = renderer.snapshot(atTime: 0, with: CGSize(width: size, height: size), antialiasingMode: .none)
        let cg = try #require(image.cgImage(forProposedRect: nil, context: nil, hints: nil))
        var pixels = [UInt8](repeating: 0, count: size * size * 4)
        let space = CGColorSpace(name: CGColorSpace.sRGB)!
        let drawn = pixels.withUnsafeMutableBytes { buffer -> Bool in
            guard let context = CGContext(data: buffer.baseAddress, width: size, height: size, bitsPerComponent: 8,
                                          bytesPerRow: size * 4, space: space,
                                          bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else { return false }
            context.draw(cg, in: CGRect(x: 0, y: 0, width: size, height: size))
            return true
        }
        try #require(drawn)
        return { x, y in
            let o = (y * size + x) * 4
            return Pixel(r: Int(pixels[o]), g: Int(pixels[o + 1]), b: Int(pixels[o + 2]))
        }
    }

    @Test("Bounds in millimeters, framed on the face")
    func bounds() {
        let preview = PreviewScene(result: Self.openShell())
        #expect(simd_distance(preview.boundsMin, SIMD3(-100, -100, 0)) < 0.5)
        #expect(simd_distance(preview.boundsMax, SIMD3(100, 100, 100)) < 0.5)
        preview.frameCamera(in: nil)
        #expect(preview.cameraNode.simdPosition.z > preview.boundsMax.z, "the camera looks at the face from +Z")
    }

    /// Every mode draws the surface where it is, in its colors, from the front and from behind through the open
    /// back: the materials are double-sided, so the inside of an open scan is not see-through.
    @Test("Display modes render from the front and from behind",
          arguments: ModelDisplayMode.allCases)
    func modes(_ mode: ModelDisplayMode) throws {
        let preview = PreviewScene(result: Self.openShell())
        #expect(preview.effectiveMode(mode) == mode)
        preview.apply(mode)
        let center = preview.center
        for (side, eye) in [("front", center + SIMD3(0, 0, 600)), ("behind", center - SIMD3(0, 0, 600))] {
            let pixel = try Self.render(preview, eye: eye)
            let mid = pixel(48, 48), corner = pixel(2, 2)
            #expect(corner.r < 10 && corner.g < 10 && corner.b < 10, "\(mode) \(side): background \(corner)")
            #expect(mid.r + mid.g + mid.b > 30, "\(mode) \(side): the surface is see-through (\(mid))")
            switch mode {
            case .texture:
                #expect(mid.r > 2 * mid.g && mid.r > 2 * mid.b, "\(mode) \(side): not the red texture \(mid)")
            case .vertexColors:
                #expect(mid.g > 2 * mid.r && mid.g > 2 * mid.b, "\(mode) \(side): not the green colors \(mid)")
            case .shaded:
                #expect(abs(mid.r - mid.g) < 40 && mid.r >= mid.b, "\(mode) \(side): not the neutral shading \(mid)")
            case .wireframe:
                // The dark surface with light blue edges over it: some pixels of the face are clearly bluish.
                if side == "front" {
                    var bluish = 0
                    for y in 30 ..< 66 { for x in 30 ..< 66 { let p = pixel(x, y); if p.b > p.r + 25 { bluish += 1 } } }
                    #expect(bluish > 20, "no wireframe lines drawn (\(bluish) bluish pixels)")
                }
            }
        }
    }

    /// Without texture and colors the texture and vertex-color modes fall back to shading, as the overlay says.
    @Test func fallbackModes() throws {
        var bare = Self.openShell()
        bare.texture = nil
        bare.mesh.colors = []
        let preview = PreviewScene(result: bare)
        #expect(preview.effectiveMode(.texture) == .shaded)
        #expect(preview.effectiveMode(.vertexColors) == .shaded)
        preview.apply(.texture)
        let mid = try Self.render(preview, eye: preview.center + SIMD3(0, 0, 600))(48, 48)
        #expect(abs(mid.r - mid.g) < 40 && mid.r > 30, "texture without a texture is not the neutral shading: \(mid)")
    }
}
