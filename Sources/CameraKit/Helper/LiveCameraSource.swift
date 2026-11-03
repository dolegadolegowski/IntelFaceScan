// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: platform module) - the live RealSense camera through the privileged helper.

import CoreGraphics
import Foundation

public final class LiveCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind = .live
    public let helperURL: URL

    public init(helperURL: URL = CameraSources.bundledHelperURL) {
        self.helperURL = helperURL
    }

    public func connect() async throws -> DeviceInfo { throw CameraError.unsupported("LiveCameraSource not implemented yet") }
    public func configure(_ configuration: CaptureConfiguration) async throws { throw CameraError.notConnected }
    public func setFilters(_ filters: FilterConfiguration) async throws { throw CameraError.notConnected }
    public func start() async throws { throw CameraError.notConnected }
    public func stop() async throws {}
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws { throw CameraError.notConnected }
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws { throw CameraError.notConnected }
    public func startRecording(to url: URL) async throws { throw CameraError.notConnected }
    public func stopRecording() async throws -> Int64 { throw CameraError.notConnected }
    public func hardwareReset() async throws { throw CameraError.notConnected }
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> { AsyncStream { $0.finish() } }
    public func events() -> AsyncStream<CameraEvent> { AsyncStream { $0.finish() } }
    public func shutdown() async {}
}
