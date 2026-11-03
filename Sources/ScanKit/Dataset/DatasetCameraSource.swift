// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests/data module) - replays the frames of a ScanDataset folder as a CameraSource.

import CameraKit
import CoreGraphics
import Foundation

public final class DatasetCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind
    public let directory: URL

    public init(directory: URL) {
        self.directory = directory
        self.kind = .dataset(directory)
    }

    public func connect() async throws -> DeviceInfo { throw CameraError.unsupported("DatasetCameraSource not implemented yet") }
    public func configure(_ configuration: CaptureConfiguration) async throws {}
    public func setFilters(_ filters: FilterConfiguration) async throws {}
    public func start() async throws {}
    public func stop() async throws {}
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {}
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording a dataset") }
    public func stopRecording() async throws -> Int64 { 0 }
    public func hardwareReset() async throws {}
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> { AsyncStream { $0.finish() } }
    public func events() -> AsyncStream<CameraEvent> { AsyncStream { $0.finish() } }
    public func shutdown() async {}
}
