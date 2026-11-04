// License: Apache 2.0. See LICENSE file in root directory.
//
// Renders the IntelFaceScan app icon (a wireframe head under a scan line) into an .iconset folder for iconutil.
// Every size is drawn from the vector description, so the small sizes stay crisp.
//
//   swiftc -O tools/release/make-app-icon.swift -o build/tools/make-app-icon
//   build/tools/make-app-icon AppIcon.iconset && iconutil -c icns AppIcon.iconset

import CoreGraphics
import Foundation
import ImageIO

struct Vec3 {
    var x, y, z: Double
    static func - (a: Vec3, b: Vec3) -> Vec3 { Vec3(x: a.x - b.x, y: a.y - b.y, z: a.z - b.z) }
    func cross(_ o: Vec3) -> Vec3 { Vec3(x: y * o.z - z * o.y, y: z * o.x - x * o.z, z: x * o.y - y * o.x) }
}

/// A stylized head: an egg (narrower jaw) with a nose and brow ridge, v from the crown (0) to the chin (pi), u around
/// the vertical axis with the face at u = pi/2.
func head(_ u: Double, _ v: Double) -> Vec3 {
    // Narrower side to side than front to back, the jaw tapering toward the chin.
    let jaw = 1 - 0.22 * pow(max(0, -cos(v)), 1.4)
    var x = 0.72 * sin(v) * cos(u) * jaw
    var z = 0.88 * sin(v) * sin(u) * jaw
    let y = -1.05 * cos(v)
    let front = exp(-pow((u - .pi / 2) / 0.20, 2))
    let face = exp(-pow((u - .pi / 2) / 0.55, 2))
    let nose = exp(-pow((v - 1.80) / 0.20, 2)) * front * 0.30
    let brow = exp(-pow((v - 1.40) / 0.09, 2)) * face * 0.07
    let eyes = -exp(-pow((v - 1.55) / 0.10, 2)) * exp(-pow((abs(u - .pi / 2) - 0.42) / 0.14, 2)) * 0.06
    let lips = exp(-pow((v - 2.18) / 0.07, 2)) * front * 0.06
    let chin = exp(-pow((v - 2.45) / 0.12, 2)) * face * 0.10
    let r = 1 + nose + brow + eyes + lips + chin
    x *= r
    z *= r
    return Vec3(x: x, y: y, z: z)
}

/// Turned toward the viewer's left and tipped forward a little; +z toward the viewer after the rotation.
func rotate(_ p: Vec3) -> Vec3 {
    let yaw = -0.95, pitch = 0.12
    let x1 = p.x * cos(yaw) + p.z * sin(yaw)
    let z1 = -p.x * sin(yaw) + p.z * cos(yaw)
    let y2 = p.y * cos(pitch) - z1 * sin(pitch)
    let z2 = p.y * sin(pitch) + z1 * cos(pitch)
    return Vec3(x: x1, y: y2, z: z2)
}

func surface(_ u: Double, _ v: Double) -> (point: Vec3, facing: Double) {
    let e = 1e-3
    let p = rotate(head(u, v))
    let du = rotate(head(u + e, v)) - rotate(head(u - e, v))
    let dv = rotate(head(u, v + e)) - rotate(head(u, v - e))
    var n = dv.cross(du)
    let len = max(1e-9, (n.x * n.x + n.y * n.y + n.z * n.z).squareRoot())
    n = Vec3(x: n.x / len, y: n.y / len, z: n.z / len)
    return (p, n.z)
}

func render(size: Int) -> CGImage? {
    let s = CGFloat(size)
    guard let ctx = CGContext(data: nil, width: size, height: size, bitsPerComponent: 8, bytesPerRow: 0,
                              space: CGColorSpace(name: CGColorSpace.sRGB)!,
                              bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else { return nil }
    ctx.setShouldAntialias(true)
    ctx.interpolationQuality = .high

    // macOS icon grid: the rounded square covers ~80% of the canvas.
    let inset = s * 0.098
    let tile = CGRect(x: inset, y: inset, width: s - 2 * inset, height: s - 2 * inset)
    let radius = tile.width * 0.225
    let tilePath = CGPath(roundedRect: tile, cornerWidth: radius, cornerHeight: radius, transform: nil)

    ctx.saveGState()
    ctx.setShadow(offset: CGSize(width: 0, height: -s * 0.012), blur: s * 0.03,
                  color: CGColor(srgbRed: 0, green: 0, blue: 0, alpha: 0.45))
    ctx.addPath(tilePath)
    ctx.setFillColor(CGColor(srgbRed: 0.04, green: 0.07, blue: 0.13, alpha: 1))
    ctx.fillPath()
    ctx.restoreGState()

    ctx.saveGState()
    ctx.addPath(tilePath)
    ctx.clip()
    let space = CGColorSpace(name: CGColorSpace.sRGB)!
    if let bg = CGGradient(colorsSpace: space, colors: [
        CGColor(srgbRed: 0.10, green: 0.20, blue: 0.34, alpha: 1),
        CGColor(srgbRed: 0.03, green: 0.05, blue: 0.10, alpha: 1)] as CFArray, locations: [0, 1]) {
        ctx.drawRadialGradient(bg, startCenter: CGPoint(x: s * 0.45, y: s * 0.62), startRadius: 0,
                               endCenter: CGPoint(x: s * 0.5, y: s * 0.5), endRadius: s * 0.62, options: [])
    }

    // Head: projected orthographically into the tile.
    let scale = tile.width * 0.32
    let center = CGPoint(x: s * 0.49, y: s * 0.54)
    func project(_ p: Vec3) -> CGPoint { CGPoint(x: center.x + CGFloat(p.x) * scale, y: center.y - CGFloat(p.y) * scale) }

    let lines = size >= 128 ? (lat: 16, lon: 22) : size >= 48 ? (lat: 10, lon: 12) : (lat: 6, lon: 8)
    let width = max(0.6, s * 0.0045)
    func stroke(_ samples: [(Vec3, Double)]) {
        for (a, b) in zip(samples, samples.dropFirst()) {
            let facing = (a.1 + b.1) / 2
            let depth = (a.0.z + b.0.z) / 2  // -1 (back) ... +1 (front)
            let visible = facing > 0
            let alpha = visible ? 0.55 + 0.45 * min(1, facing * 1.5) : 0.10
            let t = CGFloat(max(0, min(1, (depth + 1) / 2)))
            ctx.setStrokeColor(CGColor(srgbRed: 0.20 + 0.25 * t, green: 0.55 + 0.40 * t, blue: 1.0 - 0.15 * t,
                                       alpha: CGFloat(alpha)))
            ctx.setLineWidth(visible ? width : width * 0.7)
            ctx.move(to: project(a.0))
            ctx.addLine(to: project(b.0))
            ctx.strokePath()
        }
    }
    ctx.setLineCap(.round)
    let steps = 90
    for i in 1..<lines.lat {
        let v = Double.pi * Double(i) / Double(lines.lat)
        stroke((0...steps).map { j in
            let r = surface(2 * .pi * Double(j) / Double(steps), v)
            return (r.point, r.facing)
        })
    }
    for i in 0..<lines.lon {
        let u = 2 * Double.pi * Double(i) / Double(lines.lon) + .pi / 2
        stroke((0...steps).map { j in
            let r = surface(u, 0.02 + (Double.pi - 0.04) * Double(j) / Double(steps))
            return (r.point, r.facing)
        })
    }

    // Scan line across the face.
    let scanY = center.y - scale * 0.05
    if let glow = CGGradient(colorsSpace: space, colors: [
        CGColor(srgbRed: 0.35, green: 1.0, blue: 0.75, alpha: 0),
        CGColor(srgbRed: 0.35, green: 1.0, blue: 0.75, alpha: 0.35),
        CGColor(srgbRed: 0.35, green: 1.0, blue: 0.75, alpha: 0)] as CFArray, locations: [0, 0.5, 1]) {
        ctx.drawLinearGradient(glow, start: CGPoint(x: 0, y: scanY - s * 0.06), end: CGPoint(x: 0, y: scanY + s * 0.06),
                               options: [])
    }
    ctx.setStrokeColor(CGColor(srgbRed: 0.55, green: 1.0, blue: 0.85, alpha: 0.95))
    ctx.setLineWidth(max(1, s * 0.008))
    ctx.move(to: CGPoint(x: tile.minX + tile.width * 0.08, y: scanY))
    ctx.addLine(to: CGPoint(x: tile.maxX - tile.width * 0.08, y: scanY))
    ctx.strokePath()

    // Corner brackets of a viewfinder.
    if size >= 32 {
        let b = tile.insetBy(dx: tile.width * 0.09, dy: tile.width * 0.09)
        let arm = b.width * 0.14
        ctx.setStrokeColor(CGColor(srgbRed: 0.85, green: 0.93, blue: 1.0, alpha: 0.75))
        ctx.setLineWidth(max(1, s * 0.012))
        for (x, y, dx, dy) in [(b.minX, b.minY, 1.0, 1.0), (b.maxX, b.minY, -1.0, 1.0),
                               (b.minX, b.maxY, 1.0, -1.0), (b.maxX, b.maxY, -1.0, -1.0)] {
            ctx.move(to: CGPoint(x: x, y: y + CGFloat(dy) * arm))
            ctx.addLine(to: CGPoint(x: x, y: y))
            ctx.addLine(to: CGPoint(x: x + CGFloat(dx) * arm, y: y))
            ctx.strokePath()
        }
    }
    ctx.restoreGState()
    return ctx.makeImage()
}

func writePNG(_ image: CGImage, to url: URL) -> Bool {
    guard let dest = CGImageDestinationCreateWithURL(url as CFURL, "public.png" as CFString, 1, nil) else { return false }
    CGImageDestinationAddImage(dest, image, nil)
    return CGImageDestinationFinalize(dest)
}

let args = CommandLine.arguments
guard args.count == 2 else {
    FileHandle.standardError.write(Data("usage: make-app-icon <output.iconset>\n".utf8))
    exit(64)
}
let out = URL(fileURLWithPath: args[1])
do {
    try FileManager.default.createDirectory(at: out, withIntermediateDirectories: true)
} catch {
    FileHandle.standardError.write(Data("make-app-icon: \(error)\n".utf8))
    exit(1)
}
for base in [16, 32, 128, 256, 512] {
    for scale in [1, 2] {
        let name = scale == 1 ? "icon_\(base)x\(base).png" : "icon_\(base)x\(base)@2x.png"
        guard let image = render(size: base * scale), writePNG(image, to: out.appendingPathComponent(name)) else {
            FileHandle.standardError.write(Data("make-app-icon: cannot write \(name)\n".utf8))
            exit(1)
        }
    }
}
