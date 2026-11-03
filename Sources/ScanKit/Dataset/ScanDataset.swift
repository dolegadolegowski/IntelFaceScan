// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests/data module) - saving and loading a ScanCapture as a dataset folder
// (format: docs/ARCHITECTURE.md "ScanDataset").

import CameraKit
import Foundation

public enum ScanDataset {
    public static let fileExtension = "ifsdataset"

    public static func write(_ capture: ScanCapture, to directory: URL) throws {
        throw CameraError.unsupported("ScanDataset.write not implemented yet")
    }

    public static func read(from directory: URL) throws -> ScanCapture {
        throw CameraError.unsupported("ScanDataset.read not implemented yet")
    }
}
