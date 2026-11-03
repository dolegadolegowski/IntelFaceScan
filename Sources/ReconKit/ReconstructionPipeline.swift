// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: reconstruction module) - ScanCapture -> registered, fused, cleaned, textured mesh.

import Foundation
import ScanKit
import simd

public final class ReconstructionPipeline: Sendable {
    public let options: ReconstructionOptions

    public init(options: ReconstructionOptions = ReconstructionOptions()) {
        self.options = options
    }

    public func run(_ capture: ScanCapture,
                    progress: @escaping @Sendable (ReconstructionProgress) -> Void = { _ in }) async throws -> ReconstructionResult {
        throw ReconstructionError.notEnoughKeyframes(capture.keyframes.count)
    }
}
