// License: Apache 2.0. See LICENSE file in root directory.
//
// The SwiftUI application: one window, a model owned by the app delegate so that quitting can wait for the
// camera to be handed back to macOS. Started from main.swift.

import AppKit
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
    let model = AppModel()

    func applicationDidFinishLaunching(_ notification: Notification) {
        // Started as a bare executable (swift run) the process would otherwise get no Dock icon or menu bar.
        NSApp.setActivationPolicy(.regular)
        NSApp.activate()
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        if model.isQuitting { return .terminateCancel }   // a shutdown is already running and will reply
        guard model.needsShutdown else { return .terminateNow }
        // The live camera must be released (helper stops the streams and re-enumerates the USB device) before
        // the process exits, or macOS cannot use the camera until it is re-plugged.
        Task { @MainActor in
            await model.shutdownForTermination()
            sender.reply(toApplicationShouldTerminate: true)
        }
        return .terminateLater
    }
}

struct AppCommands: Commands {
    let model: AppModel

    var body: some Commands {
        CommandGroup(replacing: .newItem) {
            Button(S.liveCamera) { model.open(.live) }
                .keyboardShortcut("l", modifiers: [.command, .shift])
            Button(S.openRecording) { Task { await SourceActions.openRecording(model) } }
                .keyboardShortcut("o")
            Button(S.openDataset) { Task { await SourceActions.openDataset(model) } }
                .keyboardShortcut("o", modifiers: [.command, .shift])
            Button(S.syntheticDemo) { model.open(.synthetic) }
                .keyboardShortcut("d", modifiers: [.command, .shift])
            Divider()
            Button(S.closeSource) { model.closeSourceAction() }
                .keyboardShortcut("w", modifiers: [.command, .shift])
                .disabled(!model.hasSource)
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
