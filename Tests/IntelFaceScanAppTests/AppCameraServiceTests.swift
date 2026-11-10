// License: Apache 2.0. See LICENSE file in root directory.
//
// "Live camera" and the shared RealSense camera service (docs/CAMERA-SERVICE.md): what the app model does about
// every result of the state check and of the installation. The service is scripted (no probe, no installer, no
// prompt) and the camera is a stand-in source that records how the model asked for it.

import CameraKit
import Foundation
import Testing
@testable import IntelFaceScanApp

/// A camera service whose state check and installer follow a script.
final class ScriptedCameraService: @unchecked Sendable {
    private let lock = NSLock()
    private var state: CameraService.State
    private var outcome: Result<Void, CameraService.InstallError> = .success(())
    private var stateAfterInstall: CameraService.State?
    private var checkCount = 0
    private var promptTexts: [String] = []
    private var settingsCount = 0
    private var gate: DispatchSemaphore?
    private let hasKit: Bool
    private let manifest: URL

    /// - Parameters:
    ///   - state: what the state check answers.
    ///   - install: how the installer ends.
    ///   - afterInstall: what the state check answers once the installer succeeded (nil: the same as before).
    ///   - hasKit: false plays a build without camera service files.
    ///   - manifest: the kit's service.plist (the version this copy of the app carries is read from it).
    init(_ state: CameraService.State, install: Result<Void, CameraService.InstallError> = .success(()),
         afterInstall: CameraService.State? = nil, hasKit: Bool = true,
         manifest: URL = URL(fileURLWithPath: "/nonexistent/service.plist")) {
        self.state = state
        outcome = install
        stateAfterInstall = afterInstall
        self.hasKit = hasKit
        self.manifest = manifest
    }

    var checks: Int { lock.withLock { checkCount } }
    var installs: Int { lock.withLock { promptTexts.count } }
    var prompts: [String] { lock.withLock { promptTexts } }
    var settingsOpened: Int { lock.withLock { settingsCount } }

    func set(install: Result<Void, CameraService.InstallError>, afterInstall: CameraService.State? = nil) {
        lock.withLock {
            outcome = install
            stateAfterInstall = afterInstall
        }
    }

    /// The installer waits (as while its administrator prompt is showing) until releaseInstaller().
    func holdInstaller() { lock.withLock { gate = DispatchSemaphore(value: 0) } }
    func releaseInstaller() { lock.withLock { gate }?.signal() }

    var service: CameraService {
        let kit = CameraService.Kit(daemon: URL(fileURLWithPath: "/nonexistent/realsense-usbd"), manifest: manifest,
                                    installScript: URL(fileURLWithPath: "/nonexistent/install-camera-access.sh"))
        return CameraService(
            kit: hasKit ? kit : nil,
            state: { [self] _ in
                lock.withLock {
                    checkCount += 1
                    return state
                }
            },
            install: { [self] _, prompt in
                let waiting: DispatchSemaphore? = lock.withLock {
                    promptTexts.append(prompt)
                    return gate
                }
                waiting?.wait()
                try lock.withLock {
                    try outcome.get()
                    if let stateAfterInstall { state = stateAfterInstall }
                }
            },
            installedServiceVersion: { [self] in
                lock.withLock {
                    if case .ready(let version) = state { return version }
                    return nil
                }
            },
            openLoginItemsSettings: { [self] in lock.withLock { settingsCount += 1 } })
    }
}

/// A camera that connects at once and delivers nothing.
final class StubCameraSource: CameraSource, @unchecked Sendable {
    let kind: CameraSourceKind
    private let lock = NSLock()
    private var frameStreams: [AsyncStream<CameraFrame>.Continuation] = []
    private var eventStreams: [AsyncStream<CameraEvent>.Continuation] = []

    init(kind: CameraSourceKind = .live) { self.kind = kind }

    func connect() async throws -> DeviceInfo {
        DeviceInfo(name: "Stand-in D455", serial: "0", firmware: "0", usbType: "3.2", productId: "0B5C",
                   productLine: "D400", advancedMode: true, depthScale: 0.0001, baselineMm: 95, sensors: [],
                   profiles: [], presets: [], imu: false, source: "live")
    }
    func configure(_ configuration: CaptureConfiguration) async throws {}
    func setFilters(_ filters: FilterConfiguration) async throws {}
    func start() async throws {}
    func stop() async throws {}
    func setOption(sensor: SensorKind, name: String, value: Float) async throws {}
    func setAutoExposureROI(_ rect: CGRect, distanceMeters: Float?) async throws {}
    func startRecording(to url: URL) async throws {}
    func stopRecording() async throws -> Int64 { 0 }
    func hardwareReset() async throws {}
    func frames(bufferingNewest: Int) -> AsyncStream<CameraFrame> {
        AsyncStream { continuation in lock.withLock { frameStreams.append(continuation) } }
    }
    func events() -> AsyncStream<CameraEvent> {
        AsyncStream { continuation in lock.withLock { eventStreams.append(continuation) } }
    }
    func shutdown() async {
        let (frames, events) = lock.withLock { () -> ([AsyncStream<CameraFrame>.Continuation],
                                                      [AsyncStream<CameraEvent>.Continuation]) in
            defer { frameStreams = []; eventStreams = [] }
            return (frameStreams, eventStreams)
        }
        frames.forEach { $0.finish() }
        events.forEach { $0.finish() }
    }
}

/// How the model asked its source factory for sources.
@MainActor
final class AccessLog {
    weak var model: AppModel?
    /// The access of every `.live` request that reached the factory, in order.
    var opened: [LiveCameraSource.Access] = []
    /// The progress text's stage at each of those moments.
    var stages: [ConnectionStage] = []
    var sources: [StubCameraSource] = []
}

@MainActor
@Suite("App camera service", .serialized, .timeLimit(.minutes(2)))
struct AppCameraServiceTests {
    private func makeModel(_ scripted: ScriptedCameraService) -> (AppModel, AccessLog) {
        let log = AccessLog()
        let model = AppModel(defaults: nil, cameraService: scripted.service) { request, access in
            var kind = CameraSourceKind.synthetic
            if case .live = request {
                kind = .live
                log.opened.append(access)
                log.stages.append(log.model?.connectionStage ?? .connecting)
            }
            let source = StubCameraSource(kind: kind)
            log.sources.append(source)
            return source
        }
        log.model = model
        return (model, log)
    }

    private func close(_ model: AppModel) async {
        let closing = Task {
            await model.closeSource()
            await model.waitForClosingSources()
        }
        #expect(await AppTest.wait(for: closing, timeout: 20), "the stand-in source did not close")
    }

    /// The strings the model makes are compared in one language, whatever the Mac's is.
    private func inEnglish<T: Sendable>(_ body: @MainActor () async throws -> T) async rethrows -> T {
        try await AppLanguage.$override.withValue(.english, operation: body)
    }

    private let administrator = AppAlert.Action.useAdministratorPassword(skipUSBRelease: false)

    @Test func readyServiceOpensTheCameraWithoutAsking() async {
        let scripted = ScriptedCameraService(.ready(serviceVersion: 2))
        let (model, log) = makeModel(scripted)
        await model.openSource(.live)
        #expect(log.opened == [.service])
        // No "waiting for the administrator password" where no prompt comes.
        #expect(log.stages == [.startingCamera])
        #expect(model.alert == nil && model.screen == .preview && model.isLive && !model.isConnecting)
        #expect(scripted.checks == 1 && scripted.installs == 0 && scripted.settingsOpened == 0)
        await close(model)
    }

    @Test(arguments: [CameraService.State.notInstalled, .needsUpdate, .notAnswering("unreachable: no answer")])
    func missingServiceIsInstalledThenUsed(_ state: CameraService.State) async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(state, afterInstall: .ready(serviceVersion: 2))
            scripted.holdInstaller()
            // Also when a wait below fails: the installer runs on the queue of every administrator prompt in this
            // process, and one left waiting would hold up every later test that installs. A second signal is harmless.
            defer { scripted.releaseInstaller() }
            let (model, log) = makeModel(scripted)
            let opening = Task { await model.openSource(.live) }
            // Straight to the installer's prompt, without a dialog of the app's own first.
            try #require(await AppTest.waitUntil(20) { scripted.installs == 1 }, "the installer did not start")
            #expect(model.connectionStage == .installingCameraService)
            #expect(model.showsConnectionProgress && !model.canOpenSource && !model.canShowCameraServiceStatus)
            #expect(model.alert == nil && log.opened.isEmpty)
            scripted.releaseInstaller()
            try #require(await AppTest.wait(for: opening, timeout: 20), "opening did not end")
            #expect(scripted.prompts == [S.cameraServiceInstallPrompt])
            #expect(scripted.checks == 2, "the state is checked again after the installation")
            #expect(log.opened == [.service] && log.stages == [.startingCamera])
            #expect(model.alert == nil && model.screen == .preview)
            await close(model)
        }
    }

    @Test func cancelledInstallationOffersTheChoices() async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(.notInstalled, install: .failure(.cancelled))
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceNotInstalledTitle && alert.message == S.cameraServiceInstallCancelled)
            #expect(alert.actions == [.retryLiveCamera, administrator] && !alert.closesWithOK)
            #expect(scripted.installs == 1, "a cancelled prompt must not come back by itself")
            #expect(log.opened.isEmpty && !model.isConnecting && !model.hasSource && model.screen == .welcome)
            #expect(model.canOpenSource)

            // "Use administrator password this time": the root helper, without asking the service again.
            model.perform(administrator)
            #expect(model.alert == nil)
            try #require(await AppTest.waitUntil(20) { model.screen == .preview }, "the camera did not open")
            #expect(log.opened == [.administrator(skipUSBRelease: false)])
            #expect(log.stages == [.waitingForPassword])
            #expect(scripted.checks == 1 && scripted.installs == 1)
            await close(model)
        }
    }

    @Test func tryAgainRunsTheInstallerAgain() async throws {
        let scripted = ScriptedCameraService(.notInstalled, install: .failure(.cancelled))
        let (model, log) = makeModel(scripted)
        await model.openSource(.live)
        #expect(model.alert?.actions.first == .retryLiveCamera)
        scripted.set(install: .success(()), afterInstall: .ready(serviceVersion: 2))
        model.perform(.retryLiveCamera)
        try #require(await AppTest.waitUntil(20) { model.screen == .preview }, "the camera did not open")
        #expect(scripted.installs == 2)
        #expect(log.opened == [.service] && model.alert == nil)
        await close(model)
    }

    @Test func failedInstallationShowsTheInstallersMessage() async throws {
        try await inEnglish {
            let message = "camera access: the service did not start (launchctl bootstrap failed)"
            let scripted = ScriptedCameraService(.needsUpdate, install: .failure(.failed(message)))
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceInstallFailedTitle)
            #expect(alert.message == "The installer reported: " + message)
            #expect(alert.actions == [.retryLiveCamera, administrator])
            #expect(log.opened.isEmpty && scripted.installs == 1 && !model.isConnecting)
        }
    }

    @Test func untrustedLocationAsksForApplications() async throws {
        try await inEnglish {
            let why = "/Users/x/Downloads/IntelFaceScan.app belongs to a user who is not an administrator"
            let scripted = ScriptedCameraService(.notInstalled, install: .failure(.untrusted(why)))
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceUntrustedTitle)
            #expect(alert.message.contains("move IntelFaceScan to the Applications folder"))
            #expect(alert.message.contains(why))
            // Installing again from the same place would fail the same way.
            #expect(alert.actions == [administrator])
            #expect(log.opened.isEmpty && scripted.installs == 1)
        }
    }

    @Test func installedButNotUsableDoesNotPromptAgain() async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(.notInstalled, afterInstall: .notAnswering("unreachable: no answer"))
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceNotAnsweringTitle)
            #expect(alert.message.contains("installed, not answering (unreachable: no answer)"))
            #expect(alert.actions == [.retryLiveCamera, administrator])
            #expect(scripted.installs == 1 && scripted.checks == 2 && log.opened.isEmpty)

            // The installer succeeded into a state that installing does not fix: that state's dialog.
            let off = ScriptedCameraService(.notInstalled, afterInstall: .switchedOff)
            let (second, secondLog) = makeModel(off)
            await second.openSource(.live)
            #expect(second.alert?.title == S.cameraServiceSwitchedOffTitle)
            #expect(second.alert?.actions == [.openLoginItemsSettings, administrator])
            #expect(off.installs == 1 && secondLog.opened.isEmpty)
        }
    }

    @Test func switchedOffServiceIsNotReinstalled() async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(.switchedOff)
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceSwitchedOffTitle && alert.message == S.cameraServiceSwitchedOff)
            #expect(alert.actions == [.openLoginItemsSettings, administrator])
            #expect(scripted.installs == 0, "an administrator switched it off: never installed over that")
            #expect(log.opened.isEmpty && !model.isConnecting)
            model.perform(.openLoginItemsSettings)
            #expect(scripted.settingsOpened == 1 && model.alert == nil)
            try? await Task.sleep(for: .milliseconds(100))
            #expect(log.opened.isEmpty, "opening the settings opened the camera")
        }
    }

    @Test func anotherSessionGetsTheAdministratorPathWithoutUSBRelease() async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(.notConsoleUser)
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceNotConsoleUserTitle)
            // The service is alive and serves root: it hands the camera back, the root helper must not.
            #expect(alert.actions == [.useAdministratorPassword(skipUSBRelease: true)])
            #expect(scripted.installs == 0)
            model.perform(.useAdministratorPassword(skipUSBRelease: true))
            try #require(await AppTest.waitUntil(20) { model.screen == .preview }, "the camera did not open")
            #expect(log.opened == [.administrator(skipUSBRelease: true)] && log.stages == [.waitingForPassword])
            await close(model)
        }
    }

    @Test(arguments: [CameraService.State.cameraDisabled, .tooManyClients])
    func noWayAroundABanOrAFullService(_ state: CameraService.State) async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(state)
            let (model, log) = makeModel(scripted)
            await model.openSource(.live)
            let alert = try #require(model.alert)
            #expect(alert.title == (state == .cameraDisabled ? S.cameraDisabledTitle : S.cameraServiceBusyTitle))
            #expect(alert.message == (state == .cameraDisabled ? S.cameraDisabled : S.cameraServiceBusy))
            #expect(alert.actions.isEmpty, "no administrator path, nothing to install")
            #expect(scripted.installs == 0 && log.opened.isEmpty && !model.isConnecting)
        }
    }

    @Test func buildWithoutServiceFilesUsesTheAdministratorPath() async {
        let scripted = ScriptedCameraService(.ready(serviceVersion: 2), hasKit: false)
        let (model, log) = makeModel(scripted)
        await model.openSource(.live)
        #expect(log.opened == [.administrator(skipUSBRelease: false)] && log.stages == [.waitingForPassword])
        #expect(scripted.checks == 0 && scripted.installs == 0 && model.alert == nil)
        await close(model)
    }

    @Test func onlyAnAutomaticLiveRequestAsksTheService() async {
        let scripted = ScriptedCameraService(.notInstalled)
        let (model, log) = makeModel(scripted)
        await model.openSource(.live, liveAccess: .service)
        #expect(log.opened == [.service])
        await model.openSource(.synthetic)
        #expect(model.connectionStage == .connecting && model.screen == .preview)
        await model.openSource(.live, liveAccess: .administrator(skipUSBRelease: true))
        #expect(log.opened == [.service, .administrator(skipUSBRelease: true)])
        #expect(log.stages == [.startingCamera, .waitingForPassword])
        #expect(scripted.checks == 0 && scripted.installs == 0 && model.alert == nil)
        await close(model)
    }

    @Test func dialogWaitsForAnAlertThatIsShowing() async throws {
        try await inEnglish {
            let scripted = ScriptedCameraService(.switchedOff)
            let (model, _) = makeModel(scripted)
            model.present("Earlier", message: "something else")
            await model.openSource(.live)
            // SwiftUI does not update an alert that is on screen, so the one showing stays as it is.
            var alert = try #require(model.alert)
            #expect(alert.title == "Earlier" && alert.message == "something else" && alert.actions.isEmpty)
            // Its OK, as the view closes it: the button's action, then SwiftUI ending the presentation.
            model.alert = nil
            model.alert = nil
            try #require(await AppTest.waitUntil(5) { model.alert != nil }, "the dialog never showed")
            alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceSwitchedOffTitle && alert.message == S.cameraServiceSwitchedOff)
            #expect(alert.actions == [.openLoginItemsSettings, administrator], "the dialog lost its buttons")

            // The status command's dialog waits in the same way. A plain message is still added to what is showing.
            await model.showCameraServiceStatus()
            model.present("Later", message: "a plain message")
            alert = try #require(model.alert)
            #expect(alert.title == S.cameraServiceSwitchedOffTitle)
            #expect(alert.message == S.cameraServiceSwitchedOff + "\n\nLater: a plain message")
            #expect(alert.actions == [.openLoginItemsSettings, administrator])
            model.perform(.openLoginItemsSettings)
            #expect(model.alert == nil && scripted.settingsOpened == 1)
            try #require(await AppTest.waitUntil(5) { model.alert != nil }, "the status dialog never showed")
            #expect(model.alert?.title == S.cameraServiceTitle && model.alert?.closesWithOK == true)
            model.alert = nil
            try? await Task.sleep(for: .milliseconds(100))
            #expect(model.alert == nil, "a dialog showed twice")
        }
    }

    @Test(arguments: [CameraService.State.cameraDisabled, .tooManyClients])
    func dialogWithoutButtonsWaitsForAnAlertThatIsShowing(state: CameraService.State) async throws {
        try await inEnglish {
            // These two dialogs have no buttons; added to the alert on screen their text would never show either.
            let (model, _) = makeModel(ScriptedCameraService(state))
            model.present("Earlier", message: "something else")
            await model.openSource(.live)
            var alert = try #require(model.alert)
            #expect(alert.title == "Earlier" && alert.message == "something else")
            model.alert = nil
            try #require(await AppTest.waitUntil(5) { model.alert != nil }, "the dialog never showed")
            alert = try #require(model.alert)
            #expect(alert.title == (state == .cameraDisabled ? S.cameraDisabledTitle : S.cameraServiceBusyTitle))
            #expect(alert.actions.isEmpty)
        }
    }

    @Test func returnOnlyClosesInformation() {
        // A plain alert and the status dialog: OK is the default button. A question: its first action.
        #expect(AppAlert(title: "t", message: "m").closeIsDefault)
        #expect(AppAlert(title: "t", message: "m", actions: [.installCameraService(again: true)],
                         closesWithOK: true).closeIsDefault)
        #expect(!AppAlert(title: "t", message: "m", actions: [.retryLiveCamera]).closeIsDefault)
    }

    @Test func statusCommandShowsStateAndVersions() async throws {
        let dir = try AppTest.temporaryDirectory()
        defer { try? FileManager.default.removeItem(at: dir) }
        let manifest = dir.appendingPathComponent("service.plist")
        try PropertyListSerialization.data(fromPropertyList: ["Version": 2, "ServiceVersion": 3], format: .xml,
                                           options: 0).write(to: manifest)
        try await inEnglish {
            let ready = ScriptedCameraService(.ready(serviceVersion: 2), manifest: manifest)
            let (model, log) = makeModel(ready)
            await model.showCameraServiceStatus()
            var alert = try #require(model.alert)
            #expect(alert.title == "RealSense camera service" && alert.closesWithOK)
            // Return closes it: "Install again" leads to an administrator prompt and is never the default button.
            #expect(alert.closeIsDefault)
            #expect(alert.message.hasPrefix("State: ready (version 2).\nInstalled version: 2.\n"
                                            + "Version in this copy of IntelFaceScan: 3.\n\n"), "\(alert.message)")
            #expect(alert.actions == [.installCameraService(again: true)])
            #expect(!model.cameraServiceBusy && model.canOpenSource && log.opened.isEmpty)

            // Not installed: the versions it does not know are left out; "Install" installs and shows the new state.
            let missing = ScriptedCameraService(.notInstalled, afterInstall: .ready(serviceVersion: 3))
            let (second, secondLog) = makeModel(missing)
            await second.showCameraServiceStatus()
            alert = try #require(second.alert)
            #expect(alert.message.hasPrefix("State: not installed.\n\n"), "\(alert.message)")
            #expect(alert.actions == [.installCameraService(again: false)])
            second.perform(.installCameraService(again: false))
            try #require(await AppTest.waitUntil(20) { second.alert?.message.hasPrefix("State: ready") == true },
                         "no status after the installation")
            #expect(missing.installs == 1 && missing.prompts == [S.cameraServiceInstallPrompt])
            #expect(second.alert?.actions == [.installCameraService(again: true)])
            #expect(secondLog.opened.isEmpty && !second.cameraServiceBusy, "the status command opened the camera")

            // Switched off: the settings are one click away. No service files: nothing to install from.
            let (third, _) = makeModel(ScriptedCameraService(.switchedOff))
            await third.showCameraServiceStatus()
            #expect(third.alert?.actions == [.installCameraService(again: true), .openLoginItemsSettings])
            let (fourth, _) = makeModel(ScriptedCameraService(.notInstalled, hasKit: false))
            await fourth.showCameraServiceStatus()
            #expect(fourth.alert?.actions == [])
            #expect(fourth.alert?.message.hasPrefix("State: this copy of IntelFaceScan carries no service files.")
                    == true)
        }
    }

    @Test func statusCommandInstallationOutcomes() async throws {
        try await inEnglish {
            // Cancelled: the user's own choice in a command they started, nothing to explain.
            let cancelled = ScriptedCameraService(.notInstalled, install: .failure(.cancelled))
            let (model, _) = makeModel(cancelled)
            model.perform(.installCameraService(again: false))
            try #require(await AppTest.waitUntil(20) { cancelled.installs == 1 && !model.cameraServiceBusy })
            #expect(model.alert == nil)

            // Failed: a plain alert with the installer's message.
            let failed = ScriptedCameraService(.notInstalled, install: .failure(.failed("camera access: no")))
            let (second, _) = makeModel(failed)
            second.perform(.installCameraService(again: false))
            try #require(await AppTest.waitUntil(20) { second.alert != nil })
            #expect(second.alert?.title == S.cameraServiceInstallFailedTitle)
            #expect(second.alert?.message == "The installer reported: camera access: no")
            #expect(second.alert?.actions == [] && !second.cameraServiceBusy)

            // While a source is open the command is off: installing restarts the service under a running session.
            let ready = ScriptedCameraService(.ready(serviceVersion: 2))
            let (third, _) = makeModel(ready)
            await third.openSource(.live)
            #expect(third.hasSource && !third.canShowCameraServiceStatus)
            await third.showCameraServiceStatus()
            third.perform(.installCameraService(again: true))
            try? await Task.sleep(for: .milliseconds(200))
            #expect(third.alert == nil && ready.checks == 1 && ready.installs == 0)
            await close(third)
            #expect(third.canShowCameraServiceStatus)
        }
    }

    @Test func textsInBothLanguages() {
        let stages: [ConnectionStage] = [.connecting, .checkingCameraService, .installingCameraService,
                                         .startingCamera, .waitingForPassword]
        var english = Set<String>(), polish = Set<String>()
        for stage in stages {
            let text = AppTest.inBothLanguages { S.connectionStage(stage) }
            #expect(!text.en.isEmpty && text.en != text.pl, "\(stage)")
            english.insert(text.en)
            polish.insert(text.pl)
        }
        #expect(english.count == stages.count && polish.count == stages.count)
        let waiting = AppTest.inBothLanguages { S.connectionStage(.waitingForPassword) }
        #expect(waiting.en == "Waiting for the administrator password…")
        #expect(waiting.pl == "Oczekiwanie na hasło administratora…")

        let actions: [AppAlert.Action] = [.retryLiveCamera, .useAdministratorPassword(skipUSBRelease: false),
                                          .openLoginItemsSettings, .installCameraService(again: false),
                                          .installCameraService(again: true)]
        let titles = actions.map { action in AppTest.inBothLanguages { S.alertAction(action) } }
        #expect(Set(titles.map(\.en)).count == actions.count && Set(titles.map(\.pl)).count == actions.count)
        #expect(titles[1].en == "Use administrator password this time")
        #expect(titles[1].pl == "Tym razem użyj hasła administratora")
        #expect(titles[2].en == "Open Login Items Settings")
        #expect(titles[2].pl == "Otwórz Ustawienia systemowe")
        #expect(AppTest.inBothLanguages { S.alertAction(.useAdministratorPassword(skipUSBRelease: true)) }.en
                == titles[1].en)

        let availabilities: [CameraService.Availability] = [
            .noKit, .state(.ready(serviceVersion: 2)), .state(.ready(serviceVersion: nil)), .state(.notInstalled),
            .state(.switchedOff), .state(.needsUpdate), .state(.notAnswering("probe line")), .state(.notConsoleUser),
            .state(.cameraDisabled), .state(.tooManyClients),
        ]
        let states = availabilities.map { availability in
            AppTest.inBothLanguages { S.cameraServiceState(availability) }
        }
        #expect(Set(states.map(\.en)).count == availabilities.count)
        #expect(Set(states.map(\.pl)).count == availabilities.count)
        for (state, availability) in zip(states, availabilities) {
            #expect(state.en != state.pl && !state.pl.isEmpty, "\(availability)")
        }
        #expect(states[6].pl == "zainstalowana, nie odpowiada (probe line)")

        // The installer's prompt: what is installed, that it is asked once, and what it is good for.
        let prompt = AppTest.inBothLanguages { S.cameraServiceInstallPrompt }
        for part in ["IntelFaceScan", "RealSense camera service", "once", "without a password", "every application"] {
            #expect(prompt.en.contains(part), "\(part)")
        }
        for part in ["IntelFaceScan", "usługę kamery RealSense", "raz", "bez hasła", "każdej aplikacji"] {
            #expect(prompt.pl.contains(part), "\(part)")
        }
        // The start screen no longer promises a password prompt at every session.
        let hint = AppTest.inBothLanguages { S.liveCameraHint }
        #expect(hint.en.contains("once") && !hint.en.contains("asks for the administrator password"))
        #expect(hint.pl.contains("jeden raz") && !hint.pl.contains("poprosi o hasło"))

        let pairs = AppTest.inBothLanguages {
            [S.cameraServiceNotInstalledTitle, S.cameraServiceInstallCancelled, S.cameraServiceInstallFailedTitle,
             S.cameraServiceInstallFailed("x"), S.cameraServiceUntrustedTitle, S.cameraServiceUntrusted("x"),
             S.cameraServiceSwitchedOffTitle, S.cameraServiceSwitchedOff, S.cameraServiceNotConsoleUserTitle,
             S.cameraServiceNotConsoleUser, S.cameraDisabledTitle, S.cameraDisabled, S.cameraServiceBusyTitle,
             S.cameraServiceBusy, S.cameraServiceNotAnsweringTitle, S.cameraServiceStillNotUsable("x"),
             S.cameraServiceMenu, S.cameraServiceTitle, S.cameraServiceStatus(state: "x", installed: 2, bundled: 3)]
        }
        #expect(pairs.en.count == pairs.pl.count)
        for (en, pl) in zip(pairs.en, pairs.pl) {
            #expect(en != pl && !pl.isEmpty, "\(en)")
        }
        #expect(pairs.pl[1].hasPrefix("Prośba o hasło administratora została anulowana. "))
        #expect(pairs.pl.last?.hasPrefix("Stan: x.\nZainstalowana wersja: 2.\nWersja w tej kopii IntelFaceScan: 3.\n\n")
                == true)
    }
}
