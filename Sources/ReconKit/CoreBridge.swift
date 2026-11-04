// License: Apache 2.0. See LICENSE file in root directory.
//
// Conversions between the Swift types (Keyframe, ReconMesh, TextureImage) and the FaceReconCore C API.
// Shared by the registration/fusion and the mesh/texture/export code; keep it free of pipeline logic.

import CameraKit
import FaceReconCore
import Foundation
import ScanKit
import simd

/// Throws ReconstructionError.core when a FaceReconCore call failed.
@inline(__always)
func checkCore(_ status: fr_status, _ stage: String) throws {
    guard status == FR_OK else {
        if status == FR_ERR_CANCELLED { throw ReconstructionError.cancelled }
        let detail = String(cString: fr_last_error())
        let message = detail.isEmpty ? String(cString: fr_status_string(status)) : detail
        throw ReconstructionError.core(stage: stage, status: Int32(bitPattern: status.rawValue), message: message)
    }
}

// MARK: - Matrices

extension simd_float4x4 {
    /// Column-major floats, the layout of the C API's float[16].
    var floats: [Float] {
        [columns.0.x, columns.0.y, columns.0.z, columns.0.w,
         columns.1.x, columns.1.y, columns.1.z, columns.1.w,
         columns.2.x, columns.2.y, columns.2.z, columns.2.w,
         columns.3.x, columns.3.y, columns.3.z, columns.3.w]
    }

    init(floats m: [Float]) {
        precondition(m.count == 16)
        self.init(columns: (SIMD4(m[0], m[1], m[2], m[3]), SIMD4(m[4], m[5], m[6], m[7]),
                            SIMD4(m[8], m[9], m[10], m[11]), SIMD4(m[12], m[13], m[14], m[15])))
    }

    /// Reads a C float[16] (imported as a 16-tuple).
    init<T>(tuple: T) {
        var copy = tuple
        let m = withUnsafeBytes(of: &copy) { Array($0.bindMemory(to: Float.self).prefix(16)) }
        self.init(floats: m)
    }

    /// Writes into a C float[16] (imported as a 16-tuple).
    func store<T>(into tuple: inout T) {
        let m = floats
        withUnsafeMutableBytes(of: &tuple) { raw in
            m.withUnsafeBytes { raw.copyMemory(from: $0) }
        }
    }
}

// MARK: - RGB-D frames

extension fr_intrinsics {
    init(_ i: Intrinsics) {
        self.init()
        width = Int32(i.width); height = Int32(i.height)
        fx = i.fx; fy = i.fy; cx = i.ppx; cy = i.ppy
        let c = i.coeffs + Array(repeating: 0, count: max(0, 5 - i.coeffs.count))
        coeffs = (c[0], c[1], c[2], c[3], c[4])
        switch i.model {
        case "brown_conrady": model = 1
        case "inverse_brown_conrady": model = 2
        default: model = 0
        }
    }
}

/// Keyframes as fr_rgbd_frame structs whose pixel pointers stay valid for the lifetime of this object.
public final class RGBDFrameSet: @unchecked Sendable {
    public private(set) var frames: [fr_rgbd_frame] = []
    private var storage: [NSData] = []

    /// - Parameters:
    ///   - initialPoses: world_from_camera initial guesses (one per keyframe) or nil.
    ///   - weights: per-frame quality weights in (0, 1] or nil.
    ///   - includeColor: false skips the RGB pointer (geometry-only processing).
    public init(keyframes: [Keyframe], initialPoses: [simd_float4x4]? = nil, weights: [Float]? = nil,
                includeColor: Bool = true) {
        frames.reserveCapacity(keyframes.count)
        for (i, kf) in keyframes.enumerated() {
            var f = fr_rgbd_frame()
            let depth = kf.frame.alignedDepth
            let color = kf.frame.color
            precondition(depth.width == color.width && depth.height == color.height,
                         "aligned depth must have the color resolution")
            precondition(color.bytesPerRow == color.width * 3, "color rows must be tightly packed")
            f.depth = pin(depth.pixels).assumingMemoryBound(to: UInt16.self)
            f.depth_scale = depth.depthScale
            f.rgb = includeColor ? pin(color.pixels).assumingMemoryBound(to: UInt8.self) : nil
            if let mask = kf.mask, mask.count == color.width * color.height {
                f.mask = pin(mask).assumingMemoryBound(to: UInt8.self)
            }
            f.intrinsics = fr_intrinsics(color.intrinsics)
            f.timestamp_ms = kf.frame.timestampMs
            if let poses = initialPoses, i < poses.count {
                poses[i].store(into: &f.initial_world_from_camera)
                f.has_initial_pose = 1
            } else {
                matrix_identity_float4x4.store(into: &f.initial_world_from_camera)
            }
            f.weight = weights.map { i < $0.count ? $0[i] : 1 } ?? 1
            frames.append(f)
        }
    }

    private func pin(_ data: Data) -> UnsafeRawPointer {
        let ns = data as NSData
        storage.append(ns)
        return ns.bytes
    }

    public var count: Int { frames.count }

    public func withFrames<R>(_ body: (UnsafePointer<fr_rgbd_frame>, UInt32) throws -> R) rethrows -> R {
        try frames.withUnsafeBufferPointer { try body($0.baseAddress!, UInt32($0.count)) }
    }
}

/// Packs poses as consecutive float[16] for the C API.
func packPoses(_ poses: [simd_float4x4]) -> [Float] {
    poses.flatMap { $0.floats }
}

func unpackPoses(_ floats: [Float]) -> [simd_float4x4] {
    stride(from: 0, to: floats.count - 15, by: 16).map { simd_float4x4(floats: Array(floats[$0 ..< $0 + 16])) }
}

// MARK: - Meshes

extension ReconMesh {
    /// Copies a FaceReconCore mesh.
    public init(core mesh: UnsafePointer<fr_mesh>) {
        let m = mesh.pointee
        let n = Int(m.vertex_count), t = Int(m.triangle_count)
        var positions = [SIMD3<Float>](repeating: .zero, count: n)
        var normals: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        var uvs: [SIMD2<Float>] = []
        if let p = m.positions {
            for i in 0 ..< n { positions[i] = SIMD3(p[3 * i], p[3 * i + 1], p[3 * i + 2]) }
        }
        if let p = m.normals {
            normals = (0 ..< n).map { SIMD3(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2]) }
        }
        if let p = m.colors {
            colors = (0 ..< n).map { SIMD3(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2]) }
        }
        if let p = m.uvs {
            uvs = (0 ..< n).map { SIMD2(p[2 * $0], p[2 * $0 + 1]) }
        }
        let indices = m.indices.map { Array(UnsafeBufferPointer(start: $0, count: 3 * t)) } ?? []
        self.init(positions: positions, normals: normals, colors: colors, uvs: uvs, indices: indices)
    }

    /// Creates a FaceReconCore copy of this mesh; release it with fr_mesh_free.
    public func makeCoreMesh() throws -> UnsafeMutablePointer<fr_mesh> {
        let n = positions.count
        var attributes: UInt32 = 0
        if normals.count == n && n > 0 { attributes |= FR_MESH_NORMALS }
        if colors.count == n && n > 0 { attributes |= FR_MESH_COLORS }
        if uvs.count == n && n > 0 { attributes |= FR_MESH_UVS }
        var out: UnsafeMutablePointer<fr_mesh>?
        try checkCore(fr_mesh_create(UInt32(n), UInt32(indices.count / 3), attributes, &out), "mesh allocation")
        guard let mesh = out else { throw ReconstructionError.core(stage: "mesh allocation", status: -1, message: "NULL") }
        let m = mesh.pointee
        for i in 0 ..< n {
            m.positions[3 * i] = positions[i].x; m.positions[3 * i + 1] = positions[i].y; m.positions[3 * i + 2] = positions[i].z
        }
        if let p = m.normals {
            for i in 0 ..< n { p[3 * i] = normals[i].x; p[3 * i + 1] = normals[i].y; p[3 * i + 2] = normals[i].z }
        }
        if let p = m.colors {
            for i in 0 ..< n { p[3 * i] = colors[i].x; p[3 * i + 1] = colors[i].y; p[3 * i + 2] = colors[i].z }
        }
        if let p = m.uvs {
            for i in 0 ..< n { p[2 * i] = uvs[i].x; p[2 * i + 1] = uvs[i].y }
        }
        if let p = m.indices, !indices.isEmpty {
            indices.withUnsafeBufferPointer { p.update(from: $0.baseAddress!, count: indices.count) }
        }
        return mesh
    }

    /// Runs body on a temporary FaceReconCore copy and replaces self with the (modified) result. The synthesized flags
    /// (which fr_mesh does not carry) stay with their vertices: unchanged when the vertex count is, else by position
    /// (decimation keeps the positions of the vertices it keeps; UV seam copies share their vertex's position).
    public mutating func withCoreMesh<R>(_ body: (UnsafeMutablePointer<fr_mesh>) throws -> R) throws -> R {
        let mesh = try makeCoreMesh()
        defer { fr_mesh_free(mesh) }
        let result = try body(mesh)
        let flags = synthesized.count == vertexCount ? synthesized : []
        let before = flags.isEmpty ? [] : positions
        self = ReconMesh(core: mesh)
        if !flags.isEmpty {
            synthesized = Self.carry(flags, from: before, to: positions)
        }
        return result
    }

    /// Flags of `old` vertices for the `new` ones.
    static func carry(_ flags: [Bool], from old: [SIMD3<Float>], to new: [SIMD3<Float>]) -> [Bool] {
        if old.count == new.count { return flags }
        var marked = Set<SIMD3<Float>>()
        for (p, flag) in zip(old, flags) where flag { marked.insert(p) }
        return new.map { marked.contains($0) }
    }

    /// The synthesized flags as bytes for the C API (empty when there are none).
    var synthesizedBytes: [UInt8] {
        synthesized.count == vertexCount && synthesized.contains(true) ? synthesized.map { $0 ? 1 : 0 } : []
    }
}

extension TextureImage {
    public init(core texture: UnsafePointer<fr_texture>) {
        let t = texture.pointee
        let bytes = Int(t.width) * Int(t.height) * 4
        self.init(width: Int(t.width), height: Int(t.height), rgba: Data(bytes: t.rgba, count: bytes))
    }

    /// Creates a FaceReconCore copy; release it with fr_texture_free.
    public func makeCoreTexture() throws -> UnsafeMutablePointer<fr_texture> {
        var out: UnsafeMutablePointer<fr_texture>?
        try checkCore(fr_texture_create(Int32(width), Int32(height), &out), "texture allocation")
        guard let texture = out else { throw ReconstructionError.core(stage: "texture allocation", status: -1, message: "NULL") }
        rgba.withUnsafeBytes { texture.pointee.rgba.update(from: $0.bindMemory(to: UInt8.self).baseAddress!, count: min(rgba.count, width * height * 4)) }
        return texture
    }
}

// MARK: - Progress

/// Bridges a Swift progress closure to fr_progress_cb (pass Unmanaged.passUnretained(box).toOpaque() as user).
final class CoreProgressBox: @unchecked Sendable {
    let handler: (Float, String) -> Bool   // return false to cancel
    init(_ handler: @escaping (Float, String) -> Bool) { self.handler = handler }

    static let callback: fr_progress_cb = { user, fraction, stage in
        guard let user else { return 0 }
        let box = Unmanaged<CoreProgressBox>.fromOpaque(user).takeUnretainedValue()
        return box.handler(fraction, stage.map { String(cString: $0) } ?? "") ? 0 : 1
    }

    func withUserPointer<R>(_ body: (UnsafeMutableRawPointer) throws -> R) rethrows -> R {
        try withExtendedLifetime(self) { try body(Unmanaged.passUnretained(self).toOpaque()) }
    }
}
