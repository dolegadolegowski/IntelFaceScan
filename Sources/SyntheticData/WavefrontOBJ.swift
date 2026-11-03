// License: Apache 2.0. See LICENSE file in root directory.
//
// Minimal Wavefront OBJ reading and writing for ground-truth meshes and evaluation (ifscan synth/eval/info). The
// textured app export is MeshExport's job; this only needs geometry (+ vertex colors, normals) and must work on
// any OBJ a user brings, so the reader accepts polygons, v/vt/vn face forms, negative indices and "v x y z r g b".

import Foundation
import ReconKit
import simd

public enum WavefrontOBJError: Error, LocalizedError, Equatable {
    case io(String)
    case parse(file: String, line: Int, message: String)

    public var errorDescription: String? {
        switch self {
        case .io(let m): return m
        case .parse(let file, let line, let message): return "\(file):\(line): \(message)"
        }
    }
}

public enum WavefrontOBJ {
    public struct Statistics: Sendable, Equatable {
        public var vertices = 0
        public var triangles = 0
        public var polygons = 0
        public var normals = 0
        public var textureCoordinates = 0
        public var vertexColors = false
        public var materialLibraries: [String] = []
        public var boundsMin = SIMD3<Float>(repeating: 0)
        public var boundsMax = SIMD3<Float>(repeating: 0)
    }

    /// Reads positions, triangles (polygons are fan-triangulated) and vertex colors (when every vertex has one).
    public static func read(_ url: URL) throws -> ReconMesh {
        try parse(url).mesh
    }

    public static func statistics(_ url: URL) throws -> Statistics {
        try parse(url).statistics
    }

    private static func parse(_ url: URL) throws -> (mesh: ReconMesh, statistics: Statistics) {
        let name = url.lastPathComponent
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw WavefrontOBJError.io("Cannot read \(url.path): \(error.localizedDescription)")
        }
        // NUL-terminate every line so that strtof/strtol can never run into the next line.
        var text = [CChar](repeating: 0, count: data.count + 1)
        data.withUnsafeBytes { raw in
            for i in 0 ..< raw.count {
                let c = raw[i]
                text[i] = c == 0x0A || c == 0x0D ? 0 : CChar(bitPattern: c)
            }
        }
        var positions: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        var colored = 0
        var indices: [UInt32] = []
        var stats = Statistics()
        var polygon: [UInt32] = []

        try text.withUnsafeMutableBufferPointer { buffer in
            let base = buffer.baseAddress!
            let count = data.count
            var start = 0
            var line = 1
            while start < count {
                var end = start
                while end < count && base[end] != 0 { end += 1 }
                defer {
                    // \n and lone \r end a line; the empty "line" between \r and \n is skipped without counting.
                    if end < count {
                        let terminator = data[data.startIndex + end]
                        if terminator == 0x0A || !(end + 1 < count && data[data.startIndex + end + 1] == 0x0A) { line += 1 }
                    }
                    start = end + 1
                }
                var p = base + start
                while p.pointee == 0x20 || p.pointee == 0x09 { p += 1 }
                let c0 = p.pointee, c1 = p[0] == 0 ? 0 : p[1]
                let separator = c1 == 0x20 || c1 == 0x09
                if c0 == 0x76 /* v */ && separator {
                    var values = [Float](repeating: 0, count: 6)
                    var n = 0
                    var cursor = UnsafeMutablePointer<CChar>(p + 1)
                    while n < 6 {
                        var next: UnsafeMutablePointer<CChar>?
                        let value = strtof(cursor, &next)
                        guard let next, next != cursor else { break }
                        values[n] = value
                        n += 1
                        cursor = next
                    }
                    guard n == 3 || n == 4 || n == 6, values[0 ..< 3].allSatisfy({ $0.isFinite }) else {
                        throw WavefrontOBJError.parse(file: name, line: line, message: "vertex needs 3 coordinates (+ optional r g b)")
                    }
                    positions.append(SIMD3(values[0], values[1], values[2]))
                    if n == 6 {
                        // 0...1 floats as written by MeshLab/Blender; some tools write 0...255.
                        let s: Float = values[3] > 1 || values[4] > 1 || values[5] > 1 ? 1 : 255
                        colors.append(SIMD3(UInt8(clamping: Int((values[3] * s).rounded())), UInt8(clamping: Int((values[4] * s).rounded())),
                                            UInt8(clamping: Int((values[5] * s).rounded()))))
                        colored += 1
                    } else {
                        colors.append(SIMD3(repeating: 0))
                    }
                } else if c0 == 0x76 && c1 == 0x6E /* vn */ {
                    stats.normals += 1
                } else if c0 == 0x76 && c1 == 0x74 /* vt */ {
                    stats.textureCoordinates += 1
                } else if c0 == 0x66 /* f */ && separator {
                    polygon.removeAll(keepingCapacity: true)
                    var cursor = UnsafeMutablePointer<CChar>(p + 1)
                    while true {
                        while cursor.pointee == 0x20 || cursor.pointee == 0x09 { cursor += 1 }
                        if cursor.pointee == 0 { break }
                        var next: UnsafeMutablePointer<CChar>?
                        let raw = strtol(cursor, &next, 10)
                        guard let next, next != cursor else {
                            throw WavefrontOBJError.parse(file: name, line: line, message: "invalid face index")
                        }
                        let index = raw > 0 ? raw - 1 : positions.count + raw
                        guard raw != 0, index >= 0, index < positions.count else {
                            throw WavefrontOBJError.parse(file: name, line: line,
                                                          message: "face index \(raw) out of range (\(positions.count) vertices so far)")
                        }
                        polygon.append(UInt32(index))
                        cursor = next
                        while cursor.pointee != 0 && cursor.pointee != 0x20 && cursor.pointee != 0x09 { cursor += 1 }   // /vt/vn
                    }
                    guard polygon.count >= 3 else {
                        throw WavefrontOBJError.parse(file: name, line: line, message: "face with fewer than 3 vertices")
                    }
                    stats.polygons += 1
                    for k in 1 ..< polygon.count - 1 {
                        indices += [polygon[0], polygon[k], polygon[k + 1]]
                    }
                } else if strncmp(p, "mtllib", 6) == 0 {
                    stats.materialLibraries.append(String(cString: p + 6).trimmingCharacters(in: .whitespaces))
                }
            }
        }
        stats.vertices = positions.count
        stats.triangles = indices.count / 3
        stats.vertexColors = colored == positions.count && colored > 0
        if var lo = positions.first, var hi = positions.first {
            for p in positions {
                lo = simd_min(lo, p)
                hi = simd_max(hi, p)
            }
            stats.boundsMin = lo
            stats.boundsMax = hi
        }
        let mesh = ReconMesh(positions: positions, colors: stats.vertexColors ? colors : [], indices: indices)
        return (mesh, stats)
    }

    /// Writes positions (multiplied by `scale`, e.g. 1000 for meters -> millimeters), optional normals and vertex
    /// colors, and triangles.
    public static func write(_ mesh: ReconMesh, to url: URL, scale: Float = 1000, comments: [String] = [],
                             normals: Bool = true, colors: Bool = true) throws {
        let n = mesh.vertexCount
        let writeNormals = normals && mesh.normals.count == n && n > 0
        let writeColors = colors && mesh.colors.count == n && n > 0
        var out = [UInt8]()
        out.reserveCapacity(n * (writeColors ? 70 : 40) + (writeNormals ? n * 30 : 0) + mesh.triangleCount * 30)
        for comment in comments {
            out += Array("# \(comment)\n".utf8)
        }
        for i in 0 ..< n {
            let p = mesh.positions[i] * scale
            out += [0x76, 0x20]   // "v "
            appendFixed(&out, p.x, 4); out.append(0x20)
            appendFixed(&out, p.y, 4); out.append(0x20)
            appendFixed(&out, p.z, 4)
            if writeColors {
                let c = mesh.colors[i]
                out.append(0x20); appendFixed(&out, Float(c.x) / 255, 4)
                out.append(0x20); appendFixed(&out, Float(c.y) / 255, 4)
                out.append(0x20); appendFixed(&out, Float(c.z) / 255, 4)
            }
            out.append(0x0A)
        }
        if writeNormals {
            for v in mesh.normals {
                out += [0x76, 0x6E, 0x20]   // "vn "
                appendFixed(&out, v.x, 5); out.append(0x20)
                appendFixed(&out, v.y, 5); out.append(0x20)
                appendFixed(&out, v.z, 5); out.append(0x0A)
            }
        }
        for t in 0 ..< mesh.triangleCount {
            out += [0x66]   // "f"
            for k in 0 ..< 3 {
                let index = Int(mesh.indices[3 * t + k]) + 1
                out.append(0x20)
                appendInteger(&out, index)
                if writeNormals {
                    out += [0x2F, 0x2F]   // "//"
                    appendInteger(&out, index)
                }
            }
            out.append(0x0A)
        }
        do {
            try Data(out).write(to: url, options: .atomic)
        } catch {
            throw WavefrontOBJError.io("Cannot write \(url.path): \(error.localizedDescription)")
        }
    }

    /// Fixed-point decimal formatting (much faster than String(format:) for millions of numbers).
    private static func appendFixed(_ out: inout [UInt8], _ value: Float, _ decimals: Int) {
        guard value.isFinite else { out += Array("0".utf8); return }
        var factor: Int64 = 1
        for _ in 0 ..< decimals { factor *= 10 }
        let scaled = Int64((Double(value) * Double(factor)).rounded())
        if scaled < 0 { out.append(0x2D) }
        let magnitude = scaled.magnitude
        appendInteger(&out, Int(magnitude / UInt64(factor)))
        out.append(0x2E)
        var fraction = magnitude % UInt64(factor)
        var digits = [UInt8](repeating: 0x30, count: decimals)
        var i = decimals - 1
        while i >= 0 {
            digits[i] = 0x30 + UInt8(fraction % 10)
            fraction /= 10
            i -= 1
        }
        out += digits
    }

    private static func appendInteger(_ out: inout [UInt8], _ value: Int) {
        if value == 0 { out.append(0x30); return }
        var v = value, digits: [UInt8] = []
        while v > 0 {
            digits.append(0x30 + UInt8(v % 10))
            v /= 10
        }
        out += digits.reversed()
    }
}
