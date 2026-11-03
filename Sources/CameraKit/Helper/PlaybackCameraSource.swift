// License: Apache 2.0. See LICENSE file in root directory.
//
// A .db3/.bag recording played by the helper running unprivileged (no password, no USB): the same librealsense
// post-processing chain as the live camera.

import CoreGraphics
import Foundation

public final class PlaybackCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind
    public let file: URL
    public let helperURL: URL
    /// true: frames at recording speed (UI); false: every frame as fast as the consumer reads (offline processing).
    public let realTime: Bool
    private let session: HelperCameraSession

    public init(file: URL, helperURL: URL = CameraSources.bundledHelperURL, realTime: Bool = true) {
        self.file = file
        self.helperURL = helperURL
        self.realTime = realTime
        self.kind = .playback(file)
        session = HelperCameraSession(mode: .playback(file: file.standardizedFileURL, realTime: realTime),
                                      helperURL: helperURL)
    }

    public func connect() async throws -> DeviceInfo {
        guard FileManager.default.isReadableFile(atPath: file.path) else {
            throw CameraError.commandFailed("Cannot read the recording \(file.path)")
        }
        return try await session.connect()
    }
    public func configure(_ configuration: CaptureConfiguration) async throws { try await session.configure(configuration) }
    public func setFilters(_ filters: FilterConfiguration) async throws { try await session.setFilters(filters) }
    public func start() async throws { try await session.start() }
    public func stop() async throws { try await session.stop() }
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        throw CameraError.unsupported("options in playback")
    }
    /// A recording has no auto exposure to steer; accepted and ignored so callers need no special case.
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    public func startRecording(to url: URL) async throws { throw CameraError.unsupported("recording during playback") }
    public func stopRecording() async throws -> Int64 { throw CameraError.unsupported("recording during playback") }
    public func hardwareReset() async throws { throw CameraError.unsupported("hardware reset during playback") }
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        session.frames.subscribe(bufferingNewest: bufferingNewest)
    }
    public func events() -> AsyncStream<CameraEvent> { session.events.subscribe(bufferingNewest: 256) }

    public func pause() async throws { try await session.playback(pause: true, realTime: nil) }
    /// `realTime` nil keeps the current speed.
    public func resume(realTime: Bool? = nil) async throws { try await session.playback(pause: false, realTime: realTime) }

    public func shutdown() async { await session.shutdown(timeout: 10) }
    public func shutdownAndWait(timeout: TimeInterval = 10) { session.shutdownBlocking(timeout: timeout) }
    public var isConnected: Bool { session.isConnected }
}
