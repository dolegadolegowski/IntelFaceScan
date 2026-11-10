// License: Apache 2.0. See LICENSE file in root directory.
//
// The SwiftUI application: one window, a model owned by the app delegate so that quitting can wait for the
// camera to be handed back to macOS. Started from main.swift.

import AppKit
import os
import SwiftUI

struct IntelFaceScanApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) private var appDelegate

    var body: some Scene {
        Window(S.appName, id: "main") {
            ContentView()
                .environment(appDelegate.model)
        }
        .defaultSize(width: 1280, height: 820)
        .commands { AppCommands(model: appDelegate.model) }
    }
}

@MainActor
final class AppDelegate: NSObject, NSApplicationDelegate {
    let model = AppModel(soundPlayer: SystemScanSoundPlayer())
    private var terminationPending = false
    private let log = Logger(subsystem: "IntelFaceScan", category: "App")

    func applicationDidFinishLaunching(_ notification: Notification) {
        // Started as a bare executable (swift run) the process would otherwise get no Dock icon or menu bar.
        NSApp.setActivationPolicy(.regular)
        NSApp.activate()
        TerminationSignals.install()
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    // Cmd-Q, closing the window, logout, SIGTERM/SIGINT/SIGHUP (TerminationSignals) all end here.
    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        // A reply is on its way. (AppKit itself ends the process at once on a second terminate while it waits.)
        if terminationPending { return .terminateCancel }
        // The live camera must be released (helper stops the streams and re-enumerates the USB device) before the
        // process exits, or macOS cannot use the camera until it is re-plugged; running file writes must finish.
        guard let work = model.beginTermination() else { return .terminateNow }
        terminationPending = true
        let log = self.log
        // While AppKit waits for the reply it runs the run loop in the modal panel mode. When it asked from inside
        // a main-queue block (a Task or DispatchQueue.main callout), no main-actor job runs until the reply: so the
        // wait happens off the main actor and the reply comes through the run loop, never through the main queue.
        Task.detached {
            if !(await work.wait()) { log.error("quitting before the camera shutdown or a file write finished") }
            MainRunLoop.perform { NSApp.reply(toApplicationShouldTerminate: true) }
        }
        return .terminateLater
    }
}

/// Runs a block on the main thread through the run loop (performSelector, common modes, which include the modal
/// panel mode of a pending termination) instead of the main dispatch queue.
final class MainRunLoop: NSObject, @unchecked Sendable {
    private let block: @MainActor () -> Void

    private init(_ block: @escaping @MainActor () -> Void) { self.block = block }

    @objc private func run() { MainActor.assumeIsolated { block() } }

    static func perform(_ block: @escaping @MainActor () -> Void) {
        let target = MainRunLoop(block)
        target.performSelector(onMainThread: #selector(run), with: nil, waitUntilDone: false,
                               modes: [RunLoop.Mode.common.rawValue])
    }
}

/// SIGTERM, SIGINT (Ctrl-C in a terminal) and SIGHUP quit like Cmd-Q, so the camera is handed back to macOS and a
/// recording is finished. A second signal ends the process at once (the helper still releases the camera when it
/// sees the app exit).
enum TerminationSignals {
    private nonisolated(unsafe) static var sources: [DispatchSourceSignal] = []

    @MainActor
    static func install() {
        guard sources.isEmpty else { return }
        let first = OnceFlag()
        for sig in [SIGTERM, SIGINT, SIGHUP] {
            // A no-op handler instead of SIG_IGN: it stops the default action just the same, and unlike an ignored
            // signal it is reset by exec, so the helper processes the app starts get the default dispositions.
            signal(sig) { _ in }
            let source = DispatchSource.makeSignalSource(signal: sig, queue: .global(qos: .userInitiated))
            source.setEventHandler {
                if first.claim() {
                    MainRunLoop.perform { NSApp.terminate(nil) }
                } else {
                    _exit(128 + sig)
                }
            }
            source.resume()
            sources.append(source)
        }
    }
}

struct AppCommands: Commands {
    let model: AppModel

    var body: some Commands {
        CommandGroup(after: .appInfo) {
            // State and versions of the shared RealSense camera service; installs it (again) on request.
            Button(S.cameraServiceMenu) { Task { await model.showCameraServiceStatus() } }
                .disabled(!model.canShowCameraServiceStatus)
        }
        CommandGroup(replacing: .newItem) {
            // The same rules as the sidebar's source menu: no switching during a scan or a reconstruction.
            Group {
                Button(S.liveCamera) { model.open(.live) }
                    .keyboardShortcut("l", modifiers: [.command, .shift])
                Button(S.openRecording) { Task { await SourceActions.openRecording(model) } }
                    .keyboardShortcut("o")
                Button(S.openDataset) { Task { await SourceActions.openDataset(model) } }
                    .keyboardShortcut("o", modifiers: [.command, .shift])
                Button(S.syntheticDemo) { model.open(.synthetic) }
                    .keyboardShortcut("d", modifiers: [.command, .shift])
            }
            .disabled(!model.canOpenSource)
            Divider()
            Button(S.closeSource) { model.closeSourceAction() }
                .keyboardShortcut("w", modifiers: [.command, .shift])
                .disabled(!model.hasSource || model.isQuitting)
        }
    }
}

/// Source actions that need a file panel first.
@MainActor
enum SourceActions {
    static func openRecording(_ model: AppModel) async {
        guard let url = await FilePanels.openFile(contentTypes: [FilePanels.db3, FilePanels.bag]) else { return }
        await model.openSource(.playback(url))
    }

    static func openDataset(_ model: AppModel) async {
        guard let url = await FilePanels.openDirectory() else { return }
        await model.openSource(.dataset(url))
    }
}
