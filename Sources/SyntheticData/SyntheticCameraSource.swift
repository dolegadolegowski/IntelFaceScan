// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests/data module) - a CameraSource streaming a rendered synthetic head that turns through
// the scan poses at 30 fps (UI development and tests without a camera).

import CameraKit
import CoreGraphics
import Foundation

public final class SyntheticCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind = .synthetic
    public let options: SyntheticScan.Options

    public init(options: SyntheticScan.Options = SyntheticScan.Options()) {
        self.options = options
    }

    public func connect() async throws -> DeviceInfo { throw CameraError.unsupported("SyntheticCameraSource not implemented yet") }
    public func configure(_ configuration: CaptureConfiguration) async throws {}
    public func setFilters(_ filters: FilterConfiguration) async throws {}
    public func start() async throws {}
    public func stop() async throws {}
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {}
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording a synthetic source") }
    public func stopRecording() async throws -> Int64 { 0 }
    public func hardwareReset() async throws {}
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> { AsyncStream { $0.finish() } }
    public func events() -> AsyncStream<CameraEvent> { AsyncStream { $0.finish() } }
    public func shutdown() async {}
}
