// License: Apache 2.0. See LICENSE file in root directory.
//
// The helper's fake camera (`IntelFaceScanHelper --fake`, no password, no camera): synthetic framesets (a moving color
// pattern with a shaded sphere and its aligned depth, see helper/src/fake_engine.h) through the real helper process,
// socket and protocol. For UI development and IPC tests; behaves like the live camera (kind .live, options,
// recording, hardware reset), so code written against it works with the camera.

import CoreGraphics
import Foundation

public final class FakeCameraSource: CameraSource, @unchecked Sendable {
    public let kind: CameraSourceKind = .live
    public let helperURL: URL
    /// Extra bytes per color row on the wire (exercises the stride repacking).
    public let colorRowPadding: Int
    private let session: HelperCameraSession

    public init(helperURL: URL = CameraSources.bundledHelperURL, colorRowPadding: Int = 0) {
        self.helperURL = helperURL
        self.colorRowPadding = max(0, min(colorRowPadding, 4096))
        session = HelperCameraSession(mode: .fake(colorRowPadding: self.colorRowPadding), helperURL: helperURL)
    }

    public func connect() async throws -> DeviceInfo { try await session.connect() }
    public func configure(_ configuration: CaptureConfiguration) async throws { try await session.configure(configuration) }
    public func setFilters(_ filters: FilterConfiguration) async throws { try await session.setFilters(filters) }
    public func start() async throws { try await session.start() }
    public func stop() async throws { try await session.stop() }
    public func setOption(sensor: SensorKind, name: String, value: Float) async throws {
        try await session.setOption(sensor: sensor, name: name, value: value)
    }
    public func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {
        try await session.setAutoExposureROI(rect, distanceMeters: distanceMeters)
    }
    public func startRecording(to url: URL) async throws { try await session.startRecording(to: url) }
    public func stopRecording() async throws -> Int64 { try await session.stopRecording() }
    public func hardwareReset() async throws { try await session.hardwareReset() }
    public func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        session.frames.subscribe(bufferingNewest: bufferingNewest)
    }
    public func events() -> AsyncStream<CameraEvent> { session.events.subscribe(bufferingNewest: 256) }

    public func shutdown() async { await session.shutdown(timeout: 10) }
    public func shutdownAndWait(timeout: TimeInterval = 10) { session.shutdownBlocking(timeout: timeout) }
    public var isConnected: Bool { session.isConnected }
    public var helperHello: HelperHello? { session.hello }

    /// For tests: the session behind the source.
    var helperSession: HelperCameraSession { session }
}
