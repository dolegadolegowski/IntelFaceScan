// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: mesh/texture/export module) - cleanup, smoothing, hole filling, decimation.

import Foundation

public enum MeshProcessing {
    /// Removes small components, smooths (detail preserving), fills small holes, optionally decimates.
    public static func clean(_ mesh: inout ReconMesh, options: ReconstructionOptions) throws {}
}
