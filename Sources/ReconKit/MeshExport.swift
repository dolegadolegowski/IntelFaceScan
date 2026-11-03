// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: mesh/texture/export module) - OBJ (+MTL +PNG) and PLY export in millimeters.

import Foundation

public enum MeshExport {
    /// Writes <name>.obj, <name>.mtl and <name>.png (when textured); returns the written files.
    @discardableResult
    public static func writeOBJ(_ result: ReconstructionResult, to url: URL,
                                options: ExportOptions = ExportOptions()) throws -> [URL] {
        throw ReconstructionError.core(stage: "export", status: 2, message: "not implemented yet")
    }

    @discardableResult
    public static func writePLY(_ result: ReconstructionResult, to url: URL,
                                options: ExportOptions = ExportOptions()) throws -> URL {
        throw ReconstructionError.core(stage: "export", status: 2, message: "not implemented yet")
    }
}
