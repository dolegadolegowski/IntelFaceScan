// License: Apache 2.0. See LICENSE file in root directory.
//
// The one interface the rest of the app uses to get frames, whatever produces them.

import CoreGraphics
import Foundation

public enum CameraSourceKind: Sendable, Equatable {
    /// RealSense camera through the camera helper: as the user through the camera service, or as root after the
    /// administrator prompt (LiveCameraSource.Access).
    case live
    /// A .db3/.bag recording played through the helper running unprivileged (no password).
    case playback(URL)
    /// An IntelFaceScan dataset folder (accepted keyframes, see docs/ARCHITECTURE.md "ScanDataset").
    case dataset(URL)
    /// Procedurally rendered head (development and tests without a camera).
    case synthetic
}

/// A source of synchronized RGB-D framesets.
///
/// Lifecycle: `connect()` -> `configure(_:)` -> `start()` -> frames -> `stop()` -> ... -> `shutdown()`.
/// `shutdown()` must be called before the app exits: for the live camera it makes the helper stop the streams and
/// exit, and the camera goes back to macOS (so it works as a webcam again) - handed back by the root helper itself,
/// or by the camera service a few seconds later.
public protocol CameraSource: AnyObject, Sendable {
    var kind: CameraSourceKind { get }

    /// Starts the helper (live with the administrator password: the prompt) and returns the device description.
    func connect() async throws -> DeviceInfo
    /// Applies a configuration; while streaming this may restart the streams.
    func configure(_ configuration: CaptureConfiguration) async throws
    /// Changes only the post-processing filters (no stream restart).
    func setFilters(_ filters: FilterConfiguration) async throws
    func start() async throws
    func stop() async throws
    /// Sets a librealsense option by name on the depth or color sensor.
    func setOption(sensor: SensorKind, name: String, value: Float) async throws
    /// Auto-exposure region of interest in color image pixels (e.g. the detected face), with the approximate
    /// distance of that region so the helper can map it onto the depth sensor.
    func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws
    /// Live only: records the raw sensor streams to a .db3/.bag file (created by the app as the user).
    func startRecording(to url: URL) async throws
    /// Returns the number of bytes written.
    func stopRecording() async throws -> Int64
    func hardwareReset() async throws

    /// A new stream of framesets for one consumer, keeping only the newest `bufferingNewest` undelivered ones.
    func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame>
    /// A new stream of events for one consumer.
    func events() -> AsyncStream<CameraEvent>

    /// Stops streaming, releases the device and waits for the helper to exit. Idempotent.
    func shutdown() async
}

/// Factory for the concrete sources (implemented by their modules).
public enum CameraSources {
    /// The camera helper inside the app bundle: IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper.
    public static var bundledHelperURL: URL {
        Bundle.main.bundleURL.appendingPathComponent("Contents/Helpers/IntelFaceScanHelper")
    }
}
