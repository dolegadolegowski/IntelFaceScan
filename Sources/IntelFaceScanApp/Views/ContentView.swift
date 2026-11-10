// License: Apache 2.0. See LICENSE file in root directory.
//
// Main window: sidebar (sources, camera, settings) and the current screen.

import CameraKit
import SwiftUI

struct ContentView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        NavigationSplitView {
            SidebarView()
                .navigationSplitViewColumnWidth(min: 270, ideal: 310, max: 400)
        } detail: {
            ZStack(alignment: .bottom) {
                DetailView()
                    .padding(16)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .disabled(model.isQuitting)
                if let message = model.statusMessage {
                    StatusBanner(message: message, canReveal: model.revealableURL != nil) { model.revealSavedFile() }
                        .padding(.bottom, 20)
                        .transition(.move(edge: .bottom).combined(with: .opacity))
                }
                if model.isQuitting {
                    ProgressView(S.quitting)
                        .padding(24)
                        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
                        .frame(maxHeight: .infinity)
                }
            }
            .animation(.easeInOut(duration: 0.25), value: model.statusMessage)
        }
        .frame(minWidth: 1040, minHeight: 680)
        .alert(model.alert?.title ?? "", isPresented: Binding(get: { model.alert != nil },
                                                               set: { if !$0 { model.alert = nil } }),
               presenting: model.alert) { alert in
            // The camera service dialogs offer what makes sense for their state; a plain alert only closes.
            // On macOS the first button is the default one (Return) unless another is marked. Information closes on
            // Return: OK comes first and is marked, so Return never starts "Install again" from the status dialog.
            if alert.closeIsDefault {
                Button(S.ok) { model.alert = nil }
                    .keyboardShortcut(.defaultAction)
            }
            ForEach(alert.actions, id: \.self) { action in
                Button(S.alertAction(action)) { model.perform(action) }
            }
            if !alert.closeIsDefault {
                Button(S.cancel, role: .cancel) { model.alert = nil }
            }
        } message: { alert in
            Text(alert.message)
        }
    }
}

/// SwiftUI's State property wrapper. On the macOS 27 SDK the State attribute resolves to a macro whose plugin
/// ships only with Xcode; the property wrapper itself is unchanged and works through this alias with the
/// Command Line Tools.
typealias ViewState<Value> = SwiftUI.State<Value>

private struct DetailView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        switch model.screen {
        case .welcome: WelcomeView()
        case .preview: PreviewScreen()
        case .scanning: ScanScreen()
        case .scanComplete: ScanSummaryView()
        case .reconstructing: ReconstructionView()
        case .result: ResultView()
        }
    }
}

private struct StatusBanner: View {
    let message: String
    let canReveal: Bool
    let reveal: () -> Void

    var body: some View {
        HStack(spacing: 12) {
            // A saved file gets the check mark; other notices (the recording ended) are information only.
            Image(systemName: canReveal ? "checkmark.circle.fill" : "info.circle.fill")
                .foregroundStyle(canReveal ? Color.green : Color.secondary)
            Text(message).lineLimit(2)
            if canReveal {
                Button(S.showInFinder, action: reveal).buttonStyle(.link)
            }
        }
        .padding(.horizontal, 16).padding(.vertical, 10)
        .background(.regularMaterial, in: Capsule())
        .shadow(radius: 4, y: 2)
    }
}

/// Start screen: the four sources as large buttons.
struct WelcomeView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        VStack(spacing: 28) {
            VStack(spacing: 8) {
                Image(systemName: "person.crop.circle.badge.checkmark")
                    .font(.system(size: 56, weight: .light))
                    .foregroundStyle(.tint)
                Text(S.welcomeTitle).font(.largeTitle.weight(.semibold))
                Text(S.welcomeSubtitle)
                    .font(.title3)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .frame(maxWidth: 560)
            }
            if model.showsConnectionProgress {
                // Says what is really happening: the administrator password only when that prompt comes.
                ProgressView(S.connectionStage(model.connectionStage))
                    .controlSize(.large)
            } else {
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 220, maximum: 280), spacing: 16)], spacing: 16) {
                    SourceCard(title: S.liveCamera, detail: S.liveCameraHint, symbol: "camera.aperture") {
                        model.open(.live)
                    }
                    SourceCard(title: S.openRecording, detail: S.openRecordingHint, symbol: "film.stack") {
                        Task { await SourceActions.openRecording(model) }
                    }
                    SourceCard(title: S.openDataset, detail: S.openDatasetHint, symbol: "folder") {
                        Task { await SourceActions.openDataset(model) }
                    }
                    SourceCard(title: S.syntheticDemo, detail: S.syntheticDemoHint, symbol: "cube.transparent") {
                        model.open(.synthetic)
                    }
                }
                .frame(maxWidth: 1000)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}

private struct SourceCard: View {
    let title: String
    let detail: String
    let symbol: String
    let action: () -> Void
    @ViewState private var hovering = false

    var body: some View {
        Button(action: action) {
            VStack(alignment: .leading, spacing: 10) {
                Image(systemName: symbol).font(.system(size: 30)).foregroundStyle(.tint)
                    .frame(height: 36, alignment: .leading)
                Text(title).font(.headline)
                Text(detail).font(.callout).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
                Spacer(minLength: 0)
            }
            .frame(maxWidth: .infinity, minHeight: 150, alignment: .topLeading)
            .padding(16)
            .background(RoundedRectangle(cornerRadius: 14).fill(Color.secondary.opacity(hovering ? 0.16 : 0.08)))
            .overlay(RoundedRectangle(cornerRadius: 14).stroke(Color.secondary.opacity(0.2)))
            .contentShape(RoundedRectangle(cornerRadius: 14))
        }
        .buttonStyle(.plain)
        .onHover { hovering = $0 }
    }
}
