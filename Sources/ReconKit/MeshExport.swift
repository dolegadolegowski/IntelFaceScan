// License: Apache 2.0. See LICENSE file in root directory.
//
// OBJ (+MTL +PNG) and PLY export in millimeters, in the export frame (head upright +Y, face toward +Z, origin at
// the head center).

import FaceReconCore
import Foundation

public enum MeshExport {
    /// Writes <name>.obj, <name>.mtl and <name>.png (when textured); returns the written files.
    ///
    /// A textured OBJ carries its color in the texture only: ModelIO and SceneKit (QuickLook, Preview, Xcode) multiply
    /// the diffuse texture by the vertex colors of "v x y z r g b" lines, which renders the head about twice as dark
    /// and oversaturated. `includeVertexColors` applies to untextured OBJs (and to the PLY).
    @discardableResult
    public static func writeOBJ(_ result: ReconstructionResult, to url: URL,
                                options: ExportOptions = ExportOptions()) throws -> [URL] {
        let mesh = result.mesh
        try checkExportable(mesh, url: url)
        let textured = options.includeTexture && result.texture != nil && !mesh.uvs.isEmpty
        var coreOptions = exportOptions(result, options, writeUVs: textured)
        if textured { coreOptions.write_colors = 0 }
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var texture: UnsafeMutablePointer<fr_texture>?
        if textured, let image = result.texture {
            guard image.width > 0, image.height > 0, image.rgba.count >= image.width * image.height * 4 else {
                throw ReconstructionError.core(stage: "export", status: Int32(bitPattern: FR_ERR_INVALID_ARGUMENT.rawValue),
                                               message: "texture data does not match its size")
            }
            texture = try image.makeCoreTexture()
        }
        defer { fr_texture_free(texture) }
        try "head".withCString { material in
            try comment(for: result).withCString { text in
                coreOptions.material_name = material
                coreOptions.comment = text
                try checkCore(fr_export_obj(core, texture, url.path, &coreOptions), "export")
            }
        }
        guard textured else { return [url] }
        // Same file names as the core writes (fr_export.h): file-name-safe stem next to the OBJ.
        let directory = url.deletingLastPathComponent()
        let stem = safeFileName(url.deletingPathExtension().lastPathComponent)
        return [url, directory.appendingPathComponent(stem + ".mtl"), directory.appendingPathComponent(stem + ".png")]
    }

    @discardableResult
    public static func writePLY(_ result: ReconstructionResult, to url: URL,
                                options: ExportOptions = ExportOptions()) throws -> URL {
        let mesh = result.mesh
        try checkExportable(mesh, url: url)
        var coreOptions = exportOptions(result, options, writeUVs: options.includeTexture && !mesh.uvs.isEmpty)
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        try comment(for: result).withCString { text in
            coreOptions.comment = text
            try checkCore(fr_export_ply(core, url.path, &coreOptions), "export")
        }
        return url
    }

    // MARK: - Helpers

    static func exportOptions(_ result: ReconstructionResult, _ options: ExportOptions, writeUVs: Bool) -> fr_export_options {
        var o = fr_export_options()
        fr_export_options_default(&o)
        o.scale = options.unitsScale
        result.exportTransform.store(into: &o.transform)
        o.has_transform = 1
        o.write_normals = options.includeNormals ? 1 : 0
        o.write_colors = options.includeVertexColors ? 1 : 0
        o.write_uvs = writeUVs ? 1 : 0
        return o
    }

    static func comment(for result: ReconstructionResult) -> String {
        let date = ISO8601DateFormatter().string(from: Date())
        return "IntelFaceScan head scan, \(date)\nframe: +Y up, +Z out of the face, origin at the head center"
    }

    static func checkExportable(_ mesh: ReconMesh, url: URL) throws {
        guard url.isFileURL else {
            throw ReconstructionError.core(stage: "export", status: Int32(bitPattern: FR_ERR_INVALID_ARGUMENT.rawValue),
                                           message: "not a file URL: \(url)")
        }
        guard mesh.triangleCount > 0 else {
            throw ReconstructionError.core(stage: "export", status: Int32(bitPattern: FR_ERR_NOT_ENOUGH_DATA.rawValue),
                                           message: "the mesh is empty")
        }
        try MeshProcessing.validate(mesh, stage: "export")
    }

    /// The core's file-name rule for the MTL/PNG next to an OBJ: characters other than A-Z a-z 0-9 . _ - become "_".
    static func safeFileName(_ name: String) -> String {
        let allowed = Set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-".utf8)
        let bytes = name.utf8.map { allowed.contains($0) ? $0 : UInt8(ascii: "_") }
        return bytes.isEmpty ? "model" : String(decoding: bytes, as: UTF8.self)
    }
}
