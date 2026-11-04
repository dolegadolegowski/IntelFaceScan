// License: Apache 2.0. See LICENSE file in root directory.
//
// The application state: the camera source and its lifecycle, live tracking, the guided scan, the reconstruction
// and the result. Everything here runs on the main actor; frame work happens in FramePipeline.
//
// Closing a source detaches it at once (the UI is free immediately) and shuts it down in a detached task that never
// needs the main actor: quitting can then wait for the camera to be handed back to macOS even when AppKit asks
// applicationShouldTerminate from inside a main-queue block, where no main-actor job can run until it gets its
// reply. Source switches wait for every pending shutdown, so a new helper never meets a camera that the previous
// one is still releasing.

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

/// What has to finish before the process may exit: camera shutdowns and file writes. Waiting never needs the main
/// actor.
struct TerminationWork: Sendable {
    let camera: [Task<Void, Never>]
    let writes: [Task<Void, Never>]

    /// Waits for the camera (bounded: the helper also releases the camera when it sees the app exit) and for file
    /// writes (bounded more generously: an interrupted write loses the user's data). True when everything ended.
    func wait(cameraTimeout: Double = 15, writeTimeout: Double = 120) async -> Bool {
        async let camera = Self.waitAll(self.camera, timeout: cameraTimeout)
        async let writes = Self.waitAll(self.writes, timeout: writeTimeout)
        let (c, w) = await (camera, writes)
        return c && w
    }

    /// Unstructured on purpose: a task group would wait for the uncancellable `value` awaits past the timeout.
    static func waitAll(_ tasks: [Task<Void, Never>], timeout: Double) async -> Bool {
        guard !tasks.isEmpty else { return true }
        return await withCheckedContinuation { (continuation: CheckedContinuation<Bool, Never>) in
            let once = OnceFlag()
            Task.detached {
                for task in tasks { await task.value }
                if once.claim() { continuation.resume(returning: true) }
            }
            Task.detached {
                try? await Task.sleep(for: .seconds(timeout))
                if once.claim() { continuation.resume(returning: false) }
            }
        }
    }
}

@MainActor
@Observable
final class AppModel {
    enum Screen: Equatable {
        case welcome, preview, scanning, scanComplete, reconstructing, result
    }

    typealias SourceFactory = @MainActor (SourceRequest) -> CameraSource

    var settings: AppSettings {
        didSet { settingsDidChange(from: oldValue) }
    }
    private(set) var screen: Screen = .welcome {
        didSet { pipeline?.setActive(screen.showsPreview) }
    }
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

    @ObservationIgnored private let defaults: UserDefaults?
    @ObservationIgnored private let sourceFactory: SourceFactory
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
    /// Shutdowns of detached sources that are still running, by id.
    @ObservationIgnored private var closingTasks: [Int: Task<Void, Never>] = [:]
    /// File writes that are still running, by id.
    @ObservationIgnored private var writeTasks: [Int: Task<Void, Never>] = [:]
    @ObservationIgnored private var nextTaskID = 0
    @ObservationIgnored private let log = Logger(subsystem: "IntelFaceScan", category: "App")

    /// - Parameters:
    ///   - defaults: where the settings persist; nil keeps them in memory (developer checks).
    ///   - sourceFactory: makes the camera source for a request.
    init(defaults: UserDefaults? = .standard, sourceFactory: @escaping SourceFactory = AppModel.makeSource) {
        self.defaults = defaults
        self.sourceFactory = sourceFactory
        settings = defaults.map { AppSettings.load(from: $0) } ?? AppSettings()
        do {
            renderer = try PreviewRenderer(device: nil)
            rendererError = nil
        } catch {
            renderer = nil
            rendererError = error.localizedDescription
        }
    }

    static func makeSource(_ request: SourceRequest) -> CameraSource {
        switch request {
        case .live: return LiveCameraSource()
        case .playback(let url): return PlaybackCameraSource(file: url)
        case .dataset(let url): return DatasetCameraSource(directory: url)
        // The UI defaults (stronger shading: Vision finds the face in every pose) and a hold long enough for the
        // real-time guided scan to accept its frames per pose, as `ifscan reconstruct synthetic-camera`.
        case .synthetic: return SyntheticCameraSource(holdSeconds: 2.5)
        }
    }

    // MARK: - Derived state

    var isLive: Bool { sourceKind == .live }
    var hasSource: Bool { sourceKind != nil }
    var isRecording: Bool { recordingStartedAt != nil }
    var canStartScan: Bool { screen == .preview && hasFrames && pipeline != nil }
    var canReturnToPreview: Bool { pipeline != nil }
    /// Opening another source would end a scan or abandon a reconstruction, or a source is still connecting.
    var canOpenSource: Bool { !isConnecting && !isQuitting && screen != .scanning && screen != .reconstructing }

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
        // A running reconstruction would otherwise finish into the new source's preview or scan.
        guard !isConnecting, !isQuitting, screen != .reconstructing else { return }
        detachSource()
        // From here until the new source runs (or fails) no other request starts: two overlapping requests would
        // each create a source and orphan one of them, with a helper (and maybe the camera) nobody shuts down.
        isConnecting = true
        // The previous helper hands the camera back to macOS first; a new live helper could not capture it before.
        await waitForClosingSources()
        guard !isQuitting else {
            isConnecting = false
            return
        }
        sourceGeneration += 1
        let generation = sourceGeneration
        let newSource = sourceFactory(request)
        source = newSource
        sourceKind = newSource.kind
        playbackEnded = false
        roiPolicy.reset()
        startEvents(of: newSource, generation: generation)
        defer { if generation == sourceGeneration { isConnecting = false } }
        do {
            let info = try await newSource.connect()
            guard generation == sourceGeneration else { return }
            device = info
            let applied = settings
            do {
                try await newSource.configure(applied.captureConfiguration)
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
            // Settings changed while connecting (the password prompt can take minutes) were not applied yet.
            applyCameraSettings(settings, comparedTo: applied)
        } catch {
            guard generation == sourceGeneration else { return }
            present(S.couldNotOpenSource, error)
            detachSource()
        }
    }

    /// Stops the frame pipeline, finishes a recording and shuts the source down (live: the camera goes back to
    /// macOS); returns when that is done. The result screens stay.
    func closeSource() async {
        if let closing = detachSource() { await closing.value }
    }

    func closeSourceAction() {
        detachSource()
    }

    /// Waits until every detached source has shut down.
    func waitForClosingSources() async {
        // Entries are removed by a cleanup hop after each task; remember what was awaited instead of spinning.
        var awaited = Set<Int>()
        while let (id, task) = closingTasks.first(where: { !awaited.contains($0.key) }) {
            awaited.insert(id)
            await task.value
        }
    }

    /// Detaches the current source from the model at once and shuts it down in a task that does not need the main
    /// actor (it finishes a running recording first). Returns that task, nil when there was nothing to close.
    @discardableResult
    private func detachSource() -> Task<Void, Never>? {
        sourceGeneration += 1
        reconfigureTask?.cancel()
        reconfigureTask = nil
        if screen == .scanning { cancelScan() }
        let closing = source
        let pipeline = self.pipeline
        source = nil
        self.pipeline = nil
        let recording = recordingStartedAt != nil ? recordingURL : nil
        let wasRecording = recordingStartedAt != nil
        recordingStartedAt = nil
        recordingURL = nil
        eventsTask?.cancel()
        eventsTask = nil
        pipeline?.cancel()
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
        guard closing != nil || pipeline != nil else { return nil }

        nextTaskID += 1
        let id = nextTaskID
        let renderer = self.renderer
        let task = Task.detached(priority: .userInitiated) { [weak self] in
            var stopped: Result<Int64, Error>?
            if let closing, wasRecording {
                do { stopped = .success(try await closing.stopRecording()) } catch { stopped = .failure(error) }
            }
            await closing?.shutdown()
            await pipeline?.stop()
            renderer?.clear()
            if let stopped {
                // Not awaited: the main actor may be unable to run until termination gets its reply.
                Task { @MainActor [weak self] in self?.recordingStopped(stopped, url: recording) }
            }
        }
        closingTasks[id] = task
        Task { [weak self] in
            await task.value
            self?.closingTasks[id] = nil
        }
        return task
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
            detachSource()
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
            detachSource()
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

    /// Back to metering on the whole image (auto exposure on the face switched off).
    private func resetExposureROI() {
        guard isLive, let source, tracking.imageSize.width > 0, tracking.imageSize.height > 0 else { return }
        roiPolicy.reset()
        let full = CGRect(origin: .zero, size: tracking.imageSize)
        let log = self.log
        Task {
            do { try await source.setAutoExposureROI(full, distanceMeters: nil) }
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
        reconstructionID += 1
        reconstructionTask?.cancel()
        screen = pipeline != nil ? .preview : .welcome
    }

    // MARK: - Reconstruction

    func reconstruct() {
        guard let capture, !capture.keyframes.isEmpty, !isQuitting, screen != .reconstructing else { return }
        reconstructionID += 1
        let id = reconstructionID
        let options = settings.reconstructionOptions
        // A cancelled run may still be unwinding (it stops at its next check); its volume can take gigabytes, so
        // the new run starts only after it ended.
        let previous = reconstructionTask
        previous?.cancel()
        result = nil
        reconstructionProgress = ReconstructionProgress(stage: .preparing, fraction: 0)
        screen = .reconstructing
        let relay = ProgressRelay { [weak self] progress in
            guard let self, id == self.reconstructionID else { return }
            self.reconstructionProgress = progress
        }
        reconstructionTask = Task { [weak self] in
            await previous?.value
            do {
                try Task.checkCancellation()
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
        guard screen == .reconstructing else { return }
        reconstructionID += 1
        // Kept: the next reconstruct() waits for it to end.
        reconstructionTask?.cancel()
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
        guard result != nil else { return }
        guard let url = await FilePanels.save(suggestedName: "head.obj", contentTypes: [FilePanels.obj]) else { return }
        await exportOBJ(to: url)
    }

    func savePLY() async {
        guard result != nil else { return }
        guard let url = await FilePanels.save(suggestedName: "head.ply", contentTypes: [FilePanels.ply]) else { return }
        await exportPLY(to: url)
    }

    func saveSession() async {
        guard capture != nil else { return }
        guard let chosen = await FilePanels.save(suggestedName: "scan.\(ScanDataset.fileExtension)",
                                                 contentTypes: []) else { return }
        await exportSession(to: chosen)
    }

    func exportOBJ(to url: URL) async {
        guard let result else { return }
        await write(S.saveOBJ) { try MeshExport.writeOBJ(result, to: url) }
    }

    func exportPLY(to url: URL) async {
        guard let result else { return }
        await write(S.savePLY) { [try MeshExport.writePLY(result, to: url)] }
    }

    func exportSession(to chosen: URL) async {
        guard let capture else { return }
        let url = Self.datasetDestination(for: chosen)
        await write(S.saveSession) {
            try ScanDataset.write(capture, to: url)
            return [url]
        }
    }

    /// The dataset folder for a name from the save panel. The panel confirmed replacing only the name it showed: when
    /// the extension has to be added, an existing session of that name is not replaced but a free name is used.
    nonisolated static func datasetDestination(for chosen: URL, fileManager: FileManager = .default) -> URL {
        let ext = ScanDataset.fileExtension
        if chosen.pathExtension == ext { return chosen }
        let base = chosen.appendingPathExtension(ext)
        guard fileManager.fileExists(atPath: base.path) else { return base }
        let folder = chosen.deletingLastPathComponent()
        let name = chosen.lastPathComponent
        for n in 2 ... 9999 {
            let candidate = folder.appendingPathComponent("\(name) \(n)").appendingPathExtension(ext)
            if !fileManager.fileExists(atPath: candidate.path) { return candidate }
        }
        return folder.appendingPathComponent("\(name) \(UUID().uuidString)").appendingPathExtension(ext)
    }

    /// Runs a file write off the main actor and reports the outcome. Quitting waits for running writes.
    private func write(_ action: String, _ body: @escaping @Sendable () throws -> [URL]) async {
        guard !isQuitting else { return }
        let work = Task.detached(priority: .userInitiated) { () -> Result<[URL], Error> in Result { try body() } }
        nextTaskID += 1
        let id = nextTaskID
        writeTasks[id] = Task.detached { _ = await work.value }
        let outcome = await work.value
        writeTasks[id] = nil
        switch outcome {
        case .success(let files):
            lastSavedURL = files.first
            showStatus(S.saved(files.map(\.lastPathComponent).joined(separator: ", ")))
        case .failure(let error):
            present(S.saveFailed, error)
        }
    }

    var isWriting: Bool { !writeTasks.isEmpty }

    func revealLastSaved() {
        guard let url = lastSavedURL else { return }
        NSWorkspace.shared.activateFileViewerSelecting([url])
    }

    // MARK: - Recording (live camera)

    func toggleRecording() async {
        guard isLive, !isRecordingBusy, !isQuitting else { return }
        if recordingStartedAt != nil {
            await stopRecording()
        } else {
            let formatter = DateFormatter()
            formatter.locale = Locale(identifier: "en_US_POSIX")
            formatter.calendar = Calendar(identifier: .gregorian)
            formatter.dateFormat = "yyyyMMdd-HHmmss"
            let name = "head-\(formatter.string(from: Date())).db3"
            guard let url = await FilePanels.save(suggestedName: name, contentTypes: [FilePanels.db3]) else { return }
            await startRecording(to: url)
        }
    }

    func startRecording(to url: URL) async {
        guard let source, isLive, !isRecordingBusy, recordingStartedAt == nil, !isQuitting else { return }
        isRecordingBusy = true
        defer { isRecordingBusy = false }
        let generation = sourceGeneration
        do {
            try await source.startRecording(to: url)
            // Closed meanwhile: the helper delivers the recording when it shuts down.
            guard generation == sourceGeneration else { return }
            recordingStartedAt = Date()
            recordingURL = url
        } catch {
            present(S.recordingFailed, error)
        }
    }

    func stopRecording() async {
        guard let source, recordingStartedAt != nil, !isRecordingBusy else { return }
        isRecordingBusy = true
        defer { isRecordingBusy = false }
        let url = recordingURL
        // Cleared first: closing the source meanwhile must not ask the helper to stop the recording a second time.
        recordingStartedAt = nil
        recordingURL = nil
        do {
            let bytes = try await source.stopRecording()
            recordingStopped(.success(bytes), url: url)
        } catch {
            recordingStopped(.failure(error), url: url)
        }
    }

    private func recordingStopped(_ outcome: Result<Int64, Error>, url: URL?) {
        switch outcome {
        case .success(let bytes):
            lastSavedURL = url
            showStatus(S.recordingSaved(bytes, url?.lastPathComponent ?? ""))
        case .failure(let error):
            present(S.recordingFailed, error)
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
        if let defaults { settings.save(to: defaults) }
        guard settings != old else { return }
        if old.autoExposureOnFace && !settings.autoExposureOnFace { resetExposureROI() }
        applyCameraSettings(settings, comparedTo: old)
    }

    /// Sends what changed between two settings to the running source.
    private func applyCameraSettings(_ new: AppSettings, comparedTo old: AppSettings) {
        guard let source, pipeline != nil, new != old else { return }
        if new.needsReconfigure(comparedTo: old) {
            scheduleReconfigure()
        } else if new.needsFilterUpdate(comparedTo: old) {
            let filters = new.filterConfiguration
            let live = isLive
            let log = self.log
            Task { [weak self] in
                do { try await source.setFilters(filters) } catch {
                    if live { self?.present(S.couldNotConfigure, error) }
                    log.notice("filters: \(error.localizedDescription, privacy: .public)")
                }
            }
            refreshPreviewController()
        } else if new.scanConfiguration(realTime: true) != old.scanConfiguration(realTime: true) {
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
            guard generation == self.sourceGeneration else { return }
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

    /// Shows an alert; a second message while one is shown is added to it instead of replacing it.
    func present(_ title: String, message: String) {
        log.error("\(title, privacy: .public): \(message, privacy: .public)")
        guard !isQuitting else { return }
        if let shown = alert {
            alert = AppAlert(title: shown.title, message: shown.message + "\n\n" + title + ": " + message)
        } else {
            alert = AppAlert(title: title, message: message)
        }
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

    var needsShutdown: Bool { source != nil || isConnecting || !closingTasks.isEmpty || !writeTasks.isEmpty }

    /// Starts everything that must end before the process exits: the source is closed (live: the helper hands the
    /// camera back to macOS), running writes continue. Synchronous, and the returned work never needs the main
    /// actor, so it can be waited for while AppKit holds the main queue (applicationShouldTerminate called from a
    /// main-queue block). nil when nothing has to wait.
    func beginTermination() -> TerminationWork? {
        isQuitting = true
        reconstructionID += 1
        reconstructionTask?.cancel()
        detachSource()
        let work = TerminationWork(camera: Array(closingTasks.values), writes: Array(writeTasks.values))
        return work.camera.isEmpty && work.writes.isEmpty ? nil : work
    }

    /// Releases the camera before the app exits and waits (bounded) for it and for running writes.
    func shutdownForTermination(cameraTimeout: Double = 15, writeTimeout: Double = 120) async {
        guard let work = beginTermination() else { return }
        if !(await work.wait(cameraTimeout: cameraTimeout, writeTimeout: writeTimeout)) {
            log.error("shutdown timed out")
        }
    }
}

extension AppModel.Screen {
    /// Screens that show the live preview (the frame pipeline uploads and analyzes frames only for these).
    var showsPreview: Bool { self == .preview || self == .scanning }
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
                      progress: ReconstructionProgress?, result: ReconstructionResult?, stats: FrameStats,
                      recordingStartedAt: Date? = nil, datasetCapture: ScanCapture? = nil) {
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
        self.recordingStartedAt = recordingStartedAt
        self.datasetCapture = datasetCapture
    }

    /// Developer checks: the number of detached sources whose shutdown is still running.
    var pendingSourceShutdowns: Int { closingTasks.count }
    /// Developer checks: whether the frame pipeline currently uploads and analyzes frames.
    var isPipelineActive: Bool? { pipeline?.isActive }
}
#endif
