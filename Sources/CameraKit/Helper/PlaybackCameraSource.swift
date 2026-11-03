// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: platform module) - a .db3/.bag recording played by the helper running unprivileged.

import CoreGraphics
import Foundation

public final class PlaybackCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind
    public let file: URL
    public let helperURL: URL
    /// true: frames at recording speed (UI); false: every frame as fast as the consumer reads (offline processing).
    public let realTime: Bool

    public init(file: URL, helperURL: URL = CameraSources.bundledHelperURL, realTime: Bool = true) {
        self.file = file
        self.helperURL = helperURL
        self.realTime = realTime
        self.kind = .playback(file)
    }

    public func connect() async throws -> DeviceInfo { throw CameraError.unsupported("PlaybackCameraSource not implemented yet") }
    public func configure(_ configuration: CaptureConfiguration) async throws { throw CameraError.notConnected }
    public func setFilters(_ filters: FilterConfiguration) async throws { throw CameraError.notConnected }
    public func start() async throws { throw CameraError.notConnected }
    public func stop() async throws {}
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws { throw CameraError.unsupported("options in playback") }
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording during playback") }
    public func stopRecording() async throws -> Int64 { throw CameraError.unsupported("recording during playback") }
    public func hardwareReset() async throws { throw CameraError.unsupported("hardware reset during playback") }
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> { AsyncStream { $0.finish() } }
    public func events() -> AsyncStream<CameraEvent> { AsyncStream { $0.finish() } }
    public func shutdown() async {}
}
