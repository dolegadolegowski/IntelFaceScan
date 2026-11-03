// License: Apache 2.0. See LICENSE file in root directory.
//
// The application state: the camera source and its lifecycle, live tracking, the guided scan, the reconstruction
// and the result. Everything here runs on the main actor; frame work happens in FramePipeline.

import AppKit
import CameraKit
import Foundation
import Observation
import os
import QuartzCore
import ReconKit
import ScanKit
import SyntheticData

/// The face as last seen by the analysis, in color image pixels.
struct TrackingState: Equatable {
    var imageSize: CGSize = .zero
    var faceRect: CGRect?
    var landmarks: [SIMD2<Float>] = []
    var pose: HeadPose?
    var distance: Float?
    var distanceIsEstimate = false
    var advice: DistanceAdvice = .unknown
    var range: ClosedRange<Float> = 0.55 ... 0.75
}

struct AppAlert: Identifiable {
    let id = UUID()
    let title: String
    let message: String
}

enum SourceRequest {
    case live
    case playback(URL)
    case dataset(URL)
    case synthetic
}

@MainActor
@Observable
final class AppModel {
    enum Screen: Equatable {
        case welcome, preview, scanning, scanComplete, reconstructing, result
    }

    var settings: AppSettings {
        didSet { settingsDidChange(from: oldValue) }
    }
    private(set) var screen: Screen = .welcome
    private(set) var sourceKind: CameraSourceKind?
    private(set) var device: DeviceInfo?
    private(set) var isConnecting = false
    private(set) var tracking = TrackingState()
    private(set) var hasFrames = false
    private(set) var stats = FrameStats()
    private(set) var scanUpdate: ScanUpdate?
    private(set) var scanConfiguration = ScanConfiguration()
    private(set) var recordingStartedAt: Date?
    private(set) var isRecordingBusy = false
    private(set) var statusMessage: String?
    private(set) var lastSavedURL: URL?
    private(set) var playbackEnded = false
    private(set) var capture: ScanCapture?
    /// The keyframes of an opened dataset: they were accepted by a scan already and can be reconstructed as is.
    private(set) var datasetCapture: ScanCapture?
    private(set) var reconstructionProgress: ReconstructionProgress?
    private(set) var result: ReconstructionResult?
    private(set) var isQuitting = false
    var displayMode: ModelDisplayMode = .texture
    var alert: AppAlert?

    let renderer: PreviewRenderer?
    let rendererError: String?

    @ObservationIgnored private var source: CameraSource?
    @ObservationIgnored private var pipeline: FramePipeline?
    @ObservationIgnored private var eventsTask: Task<Void, Never>?
    @ObservationIgnored private var scanController: ScanController?
    @ObservationIgnored private var previewController: ScanController?
    @ObservationIgnored private let tracker = HeadTracker()
    @ObservationIgnored private var roiPolicy = ExposureROIPolicy()
    @ObservationIgnored private var reconfigureTask: Task<Void, Never>?
    @ObservationIgnored private var reconstructionTask: Task<Void, Never>?
    @ObservationIgnored private var reconstructionID = 0
    @ObservationIgnored private var statusTask: Task<Void, Never>?
    @ObservationIgnored private var sourceGeneration = 0
    @ObservationIgnored private var recordingURL: URL?
    @ObservationIgnored private let log = Logger(subsystem: "IntelFaceScan", category: "App")

    init() {
        settings = AppSettings.load()
        do {
            renderer = try PreviewRenderer(device: nil)
            rendererError = nil
        } catch {
            renderer = nil
            rendererError = error.localizedDescription
        }
    }

    // MARK: - Derived state

    var isLive: Bool { sourceKind == .live }
    var hasSource: Bool { sourceKind != nil }
    var isRecording: Bool { recordingStartedAt != nil }
    var canStartScan: Bool { screen == .preview && hasFrames && pipeline != nil }
    var canReturnToPreview: Bool { pipeline != nil }

    var previewDisplay: PreviewDisplay {
        PreviewDisplay(mode: settings.previewMode, mirrored: settings.mirrored,
                       nearM: max(0.1, tracking.range.lowerBound - 0.2), farM: tracking.range.upperBound + 0.3,
                       blend: 0.5)
    }

    // MARK: - Sources

    func open(_ request: SourceRequest) {
        Task { await openSource(request) }
    }

    func openSource(_ request: SourceRequest) async {
        guard !isConnecting, !isQuitting else { return }
        if screen == .scanning { cancelScan() }
        await closeSource()
        sourceGeneration += 1
        let generation = sourceGeneration
        let newSource: CameraSource
        switch request {
        case .live: newSource = LiveCameraSource()
        case .playback(let url): newSource = PlaybackCameraSource(file: url)
        case .dataset(let url): newSource = DatasetCameraSource(directory: url)
        case .synthetic: newSource = SyntheticCameraSource(options: SyntheticScan.Options())
        }
        source = newSource
        sourceKind = newSource.kind
        isConnecting = true
        playbackEnded = false
        roiPolicy.reset()
        startEvents(of: newSource, generation: generation)
        defer { if generation == sourceGeneration { isConnecting = false } }
        do {
            let info = try await newSource.connect()
            guard generation == sourceGeneration else { return }
            device = info
            do {
                try await newSource.configure(settings.captureConfiguration)
            } catch {
                // Recordings, datasets and the synthetic head may not support every option; the live camera
                // keeps running with its previous settings, which the user should know about.
                if newSource.kind == .live { present(S.couldNotConfigure, error) }
                log.notice("configure: \(error.localizedDescription, privacy: .public)")
            }
            guard generation == sourceGeneration else { return }
            try await newSource.start()
            guard generation == sourceGeneration else { return }
            startPipeline(newSource)
            screen = .preview
            if case .dataset(let url) = request { loadDatasetCapture(url, generation: generation) }
        } catch {
            guard generation == sourceGeneration else { return }
            present(S.couldNotOpenSource, error)
            isConnecting = false
            await closeSource()
        }
    }

    /// Stops the frame pipeline, finishes a recording and shuts the source down (live: the camera goes back to
    /// macOS). The result screens stay.
    func closeSource() async {
        sourceGeneration += 1
        reconfigureTask?.cancel()
        reconfigureTask = nil
        if screen == .scanning { cancelScan() }
        let closing = source
        source = nil
        if let closing, recordingStartedAt != nil {
            _ = try? await closing.stopRecording()
        }
        recordingStartedAt = nil
        recordingURL = nil
        if let pipeline {
            self.pipeline = nil
            await pipeline.stop()
        }
        eventsTask?.cancel()
        eventsTask = nil
        if let closing { await closing.shutdown() }
        renderer?.clear()
        previewController = nil
        datasetCapture = nil
        sourceKind = nil
        device = nil
        hasFrames = false
        tracking = TrackingState()
        stats = FrameStats()
        isConnecting = false
        if screen == .preview || screen == .scanning { screen = .welcome }
    }

    func closeSourceAction() {
        Task { await closeSource() }
    }

    private func startEvents(of source: CameraSource, generation: Int) {
        let events = source.events()
        eventsTask = Task { [weak self] in
            for await event in events {
                guard let self, generation == self.sourceGeneration else { break }
                self.handle(event)
            }
        }
    }

    private func handle(_ event: CameraEvent) {
        switch event {
        case .connected(let info):
            device = info
        case .disconnected(let reason):
            present(S.cameraDisconnected, message: reason)
            Task { await closeSource() }
        case .streamingStarted, .streamingStopped, .recordingStarted:
            break
        case .streamRestarted(let message):
            log.info("stream restarted: \(message, privacy: .public)")
        case .recordingFinished(let bytes):
            log.info("recording finished: \(bytes) bytes")
        case .playbackFinished:
            playbackEnded = true
            showStatus(S.playbackFinished)
        case .log(let level, let message):
            switch level {
            case .debug: log.debug("helper: \(message, privacy: .public)")
            case .info: log.info("helper: \(message, privacy: .public)")
            case .warning: log.warning("helper: \(message, privacy: .public)")
            case .error: log.error("helper: \(message, privacy: .public)")
            }
        case .error(let message):
            present(S.cameraError, message: message)
        case .helperExited(let status):
            guard !isQuitting else { return }
            present(S.helperExited, message: "exit status \(status)")
            Task { await closeSource() }
        }
    }

    private func startPipeline(_ source: CameraSource) {
        let controller = makePreviewController()
        previewController = controller
        let pipeline = FramePipeline(source: source, renderer: renderer, controller: controller)
        let generation = sourceGeneration
        pipeline.start(onResult: { [weak self] result in
            guard let self, generation == self.sourceGeneration else { return }
            self.apply(result)
        }, onEnd: { [weak self] in
            guard let self, generation == self.sourceGeneration else { return }
            self.playbackEnded = true
        })
        self.pipeline = pipeline
    }

    private func makePreviewController() -> ScanController {
        let controller = ScanController(configuration: settings.scanConfiguration(realTime: true), tracker: tracker)
        controller.prepare(device: device, captureConfiguration: settings.captureConfiguration)
        return controller
    }

    // MARK: - Analysis results

    private func apply(_ result: AnalysisResult) {
        let update = result.update
        hasFrames = true
        stats = result.stats
        tracking = TrackingState(imageSize: result.imageSize, faceRect: update.observation?.faceRect,
                                 landmarks: update.observation?.landmarks ?? [], pose: update.smoothedPose,
                                 distance: update.distanceM, distanceIsEstimate: update.distanceIsEstimate,
                                 advice: update.distanceAdvice, range: update.distanceRangeM ?? tracking.range)
        updateExposureROI()
        guard screen == .scanning, let controller = scanController,
              result.controllerID == ObjectIdentifier(controller) else { return }
        scanUpdate = update
        switch update.phase {
        case .finished:
            completeScan()
        case .failed(let message):
            present(S.error, message: message)
            cancelScan()
        default:
            break
        }
    }

    private func updateExposureROI() {
        guard settings.autoExposureOnFace, isLive, let source, !isConnecting else { return }
        guard let roi = roiPolicy.roi(for: tracking.faceRect, imageSize: tracking.imageSize,
                                      now: CACurrentMediaTime()) else { return }
        let distance = tracking.distanceIsEstimate ? nil : tracking.distance
        let log = self.log
        Task {
            do { try await source.setAutoExposureROI(roi, distanceMeters: distance) }
            catch { log.notice("auto-exposure ROI: \(error.localizedDescription, privacy: .public)") }
        }
    }

    // MARK: - Guided scan

    func startScan() {
        guard canStartScan, let pipeline else { return }
        let configuration = settings.scanConfiguration(realTime: true)
        let controller = ScanController(configuration: configuration, tracker: tracker)
        controller.start(device: device, captureConfiguration: settings.captureConfiguration)
        scanConfiguration = configuration
        scanController = controller
        scanUpdate = nil
        capture = nil
        result = nil
        pipeline.setController(controller)
        screen = .scanning
    }

    func skipPose() {
        scanController?.skipPose()
    }

    /// Ends the scan with the keyframes accepted so far.
    func finishScan() {
        guard screen == .scanning else { return }
        completeScan()
    }

    func cancelScan() {
        scanController?.cancel()
        scanController = nil
        scanUpdate = nil
        if let previewController { pipeline?.setController(previewController) }
        if screen == .scanning { screen = pipeline != nil ? .preview : .welcome }
    }

    private func completeScan() {
        guard let controller = scanController else { return }
        capture = controller.finish()
        scanController = nil
        if let previewController { pipeline?.setController(previewController) }
        screen = .scanComplete
    }

    /// Reads the dataset's keyframes off the main actor; the preview offers them for reconstruction.
    private func loadDatasetCapture(_ url: URL, generation: Int) {
        let log = self.log
        Task { [weak self] in
            do {
                let loaded = try await Task.detached(priority: .utility) { try ScanDataset.read(from: url) }.value
                guard let self, generation == self.sourceGeneration, !loaded.keyframes.isEmpty else { return }
                self.datasetCapture = loaded
            } catch {
                log.notice("dataset keyframes: \(error.localizedDescription, privacy: .public)")
            }
        }
    }

    /// Uses the opened dataset's keyframes instead of scanning again.
    func useDatasetCapture() {
        guard let datasetCapture, screen == .preview else { return }
        capture = datasetCapture
        result = nil
        screen = .scanComplete
    }

    func backToPreview() {
        reconstructionTask?.cancel()
        screen = pipeline != nil ? .preview : .welcome
    }

    // MARK: - Reconstruction

    func reconstruct() {
        guard let capture, !capture.keyframes.isEmpty else { return }
        reconstructionID += 1
        let id = reconstructionID
        let options = settings.reconstructionOptions
        result = nil
        reconstructionProgress = ReconstructionProgress(stage: .preparing, fraction: 0)
        screen = .reconstructing
        let relay = ProgressRelay { [weak self] progress in
            guard let self, id == self.reconstructionID else { return }
            self.reconstructionProgress = progress
        }
        reconstructionTask = Task { [weak self] in
            do {
                let output = try await Self.runReconstruction(capture, options: options, relay: relay)
                guard let self, id == self.reconstructionID, !Task.isCancelled else { return }
                self.result = output
                self.displayMode = output.texture != nil ? .texture : .vertexColors
                self.screen = .result
            } catch {
                guard let self, id == self.reconstructionID else { return }
                if !Self.isCancellation(error) { self.present(S.reconstructionFailed, error) }
                self.screen = .scanComplete
            }
        }
    }

    func cancelReconstruction() {
        reconstructionID += 1
        reconstructionTask?.cancel()
        reconstructionTask = nil
        screen = .scanComplete
    }

    nonisolated private static func runReconstruction(_ capture: ScanCapture, options: ReconstructionOptions,
                                                      relay: ProgressRelay) async throws -> ReconstructionResult {
        try await ReconstructionPipeline(options: options).run(capture) { relay.post($0) }
    }

    nonisolated private static func isCancellation(_ error: Error) -> Bool {
        if error is CancellationError { return true }
        if case .cancelled = error as? ReconstructionError { return true }
        return false
    }

    // MARK: - Saving

    func saveOBJ() async {
        guard let result else { return }
        guard let url = await FilePanels.save(suggestedName: "head.obj", contentTypes: [FilePanels.obj]) else { return }
        await write(S.saveOBJ) { try MeshExport.writeOBJ(result, to: url) }
    }

    func savePLY() async {
        guard let result else { return }
        guard let url = await FilePanels.save(suggestedName: "head.ply", contentTypes: [FilePanels.ply]) else { return }
        await write(S.savePLY) { [try MeshExport.writePLY(result, to: url)] }
    }

    func saveSession() async {
        guard let capture else { return }
        guard let chosen = await FilePanels.save(suggestedName: "scan.\(ScanDataset.fileExtension)", contentTypes: []) else { return }
        let url = chosen.pathExtension == ScanDataset.fileExtension
            ? chosen : chosen.appendingPathExtension(ScanDataset.fileExtension)
        await write(S.saveSession) {
            try ScanDataset.write(capture, to: url)
            return [url]
        }
    }

    /// Runs a file write off the main actor and reports the outcome.
    private func write(_ action: String, _ body: @escaping @Sendable () throws -> [URL]) async {
        do {
            let files = try await Task.detached(priority: .userInitiated) { try body() }.value
            lastSavedURL = files.first
            showStatus(S.saved(files.map(\.lastPathComponent).joined(separator: ", ")))
        } catch {
            present(S.saveFailed, error)
        }
    }

    func revealLastSaved() {
        guard let url = lastSavedURL else { return }
        NSWorkspace.shared.activateFileViewerSelecting([url])
    }

    // MARK: - Recording (live camera)

    func toggleRecording() async {
        guard let source, isLive, !isRecordingBusy else { return }
        isRecordingBusy = true
        defer { isRecordingBusy = false }
        if recordingStartedAt != nil {
            do {
                let bytes = try await source.stopRecording()
                showStatus(S.recordingSaved(bytes, recordingURL?.lastPathComponent ?? ""))
                lastSavedURL = recordingURL
            } catch {
                present(S.recordingFailed, error)
            }
            recordingStartedAt = nil
            recordingURL = nil
        } else {
            let formatter = DateFormatter()
            formatter.dateFormat = "yyyyMMdd-HHmmss"
            let name = "head-\(formatter.string(from: Date())).db3"
            guard let url = await FilePanels.save(suggestedName: name, contentTypes: [FilePanels.db3]) else { return }
            do {
                try await source.startRecording(to: url)
                recordingStartedAt = Date()
                recordingURL = url
            } catch {
                present(S.recordingFailed, error)
            }
        }
    }

    func hardwareReset() async {
        guard let source, isLive else { return }
        do { try await source.hardwareReset() } catch { present(S.cameraError, error) }
    }

    // MARK: - Settings

    func resetSettings() {
        settings = AppSettings()
    }

    // The settings controls only produce values within the sanitized ranges; loading sanitizes stored data.
    private func settingsDidChange(from old: AppSettings) {
        settings.save()
        guard let source, pipeline != nil, settings != old else { return }
        if settings.needsReconfigure(comparedTo: old) {
            scheduleReconfigure()
        } else if settings.needsFilterUpdate(comparedTo: old) {
            let filters = settings.filterConfiguration
            let live = isLive
            let log = self.log
            Task { [weak self] in
                do { try await source.setFilters(filters) } catch {
                    if live { self?.present(S.couldNotConfigure, error) }
                    log.notice("filters: \(error.localizedDescription, privacy: .public)")
                }
            }
            refreshPreviewController()
        } else if settings.scanConfiguration(realTime: true) != old.scanConfiguration(realTime: true) {
            refreshPreviewController()
        }
    }

    /// Applies stream settings after the user stopped changing them (a configure may restart the streams).
    private func scheduleReconfigure() {
        reconfigureTask?.cancel()
        let generation = sourceGeneration
        reconfigureTask = Task { [weak self] in
            try? await Task.sleep(for: .milliseconds(400))
            guard let self, !Task.isCancelled, generation == self.sourceGeneration, let source = self.source else { return }
            do {
                try await source.configure(self.settings.captureConfiguration)
            } catch {
                if self.isLive { self.present(S.couldNotConfigure, error) }
                self.log.notice("configure: \(error.localizedDescription, privacy: .public)")
            }
            self.refreshPreviewController()
        }
    }

    private func refreshPreviewController() {
        guard pipeline != nil else { return }
        let controller = makePreviewController()
        previewController = controller
        if screen != .scanning { pipeline?.setController(controller) }
    }

    // MARK: - Messages

    func present(_ title: String, _ error: Error) {
        present(title, message: error.localizedDescription)
    }

    func present(_ title: String, message: String) {
        log.error("\(title, privacy: .public): \(message, privacy: .public)")
        guard !isQuitting else { return }
        alert = AppAlert(title: title, message: message)
    }

    func showStatus(_ message: String) {
        statusMessage = message
        statusTask?.cancel()
        statusTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(6))
            guard !Task.isCancelled else { return }
            self?.statusMessage = nil
        }
    }

    // MARK: - Termination

    var needsShutdown: Bool { source != nil || isConnecting }

    /// Releases the camera before the app exits (live: the helper hands the device back to macOS). Gives up
    /// after `timeout` seconds: the helper also releases the camera when it sees the app exit.
    func shutdownForTermination(timeout: Double = 15) async {
        isQuitting = true
        reconstructionTask?.cancel()
        let shutdown = Task { await self.closeSource() }
        let finished = await withCheckedContinuation { (continuation: CheckedContinuation<Bool, Never>) in
            let once = OnceFlag()
            Task {
                await shutdown.value
                if once.claim() { continuation.resume(returning: true) }
            }
            Task {
                try? await Task.sleep(for: .seconds(timeout))
                if once.claim() { continuation.resume(returning: false) }
            }
        }
        if !finished { log.error("camera shutdown timed out") }
    }
}

/// Delivers progress from worker threads to the main actor, coalescing bursts (at most one hop pending).
final class ProgressRelay: @unchecked Sendable {
    private let lock = NSLock()
    private var pending: ReconstructionProgress?
    private var scheduled = false
    private let deliver: @MainActor (ReconstructionProgress) -> Void

    init(_ deliver: @escaping @MainActor (ReconstructionProgress) -> Void) { self.deliver = deliver }

    func post(_ progress: ReconstructionProgress) {
        let schedule = lock.withLock { () -> Bool in
            pending = progress
            if scheduled { return false }
            scheduled = true
            return true
        }
        guard schedule else { return }
        Task { @MainActor in
            let latest = self.lock.withLock { () -> ReconstructionProgress? in
                self.scheduled = false
                let p = self.pending
                self.pending = nil
                return p
            }
            if let latest { self.deliver(latest) }
        }
    }
}

final class OnceFlag: @unchecked Sendable {
    private let lock = NSLock()
    private var claimed = false

    func claim() -> Bool {
        lock.withLock {
            if claimed { return false }
            claimed = true
            return true
        }
    }
}

#if DEBUG
extension AppModel {
    /// Puts the model into a given state without a camera (DeveloperSnapshots).
    func applyFixture(screen: Screen, sourceKind: CameraSourceKind?, device: DeviceInfo?, tracking: TrackingState,
                      scanUpdate: ScanUpdate?, scanConfiguration: ScanConfiguration, capture: ScanCapture?,
                      progress: ReconstructionProgress?, result: ReconstructionResult?, stats: FrameStats) {
        self.screen = screen
        self.sourceKind = sourceKind
        self.device = device
        self.tracking = tracking
        self.hasFrames = true
        self.scanUpdate = scanUpdate
        self.scanConfiguration = scanConfiguration
        self.capture = capture
        self.reconstructionProgress = progress
        self.result = result
        self.stats = stats
    }
}
#endif
