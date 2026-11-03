// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: mesh/texture/export module) - interactive 3D preview of the reconstructed head.

import ReconKit
import SwiftUI

enum ModelDisplayMode: String, CaseIterable, Identifiable {
    case texture, vertexColors, shaded, wireframe
    var id: String { rawValue }
}

struct ModelPreviewView: View {
    let result: ReconstructionResult
    @Binding var displayMode: ModelDisplayMode

    var body: some View {
        Text("\(result.mesh.triangleCount) triangles")
    }
}
