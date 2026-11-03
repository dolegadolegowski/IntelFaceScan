// License: Apache 2.0. See LICENSE file in root directory.
//
// Live preview: image with face overlay, distance gauge, display controls, recording and scan start.

import CameraKit
import SwiftUI

struct PreviewScreen: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        VStack(spacing: 14) {
            LivePreview()
                .overlay(alignment: .topLeading) { PreviewStatus().padding(10) }
            HStack(alignment: .center, spacing: 20) {
                DistanceGauge(distance: model.tracking.distance, isEstimate: model.tracking.distanceIsEstimate,
                              range: model.tracking.range, advice: model.tracking.advice)
                    .frame(maxWidth: 420)
                Spacer(minLength: 12)
                Picker("", selection: $model.settings.previewMode) {
                    ForEach(AppSettings.PreviewMode.allCases) { Text($0.label).tag($0) }
                }
                .pickerStyle(.segmented)
                .labelsHidden()
                .frame(width: 240)
                Toggle(isOn: $model.settings.mirrored) { Image(systemName: "arrow.left.and.right.righttriangle.left.righttriangle.right") }
                    .toggleStyle(.button)
                    .help(S.mirrored)
                if model.isLive {
                    RecordButton()
                }
                if let dataset = model.datasetCapture {
                    Button {
                        model.useDatasetCapture()
                    } label: {
                        Label(S.useDatasetFrames(dataset.keyframes.count), systemImage: "square.stack.3d.up")
                    }
                    .controlSize(.large)
                }
                Button {
                    model.startScan()
                } label: {
                    Label(S.startScan, systemImage: "viewfinder")
                        .padding(.horizontal, 6)
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
                .disabled(!model.canStartScan)
                .keyboardShortcut(.return, modifiers: [.command])
            }
        }
    }
}

/// Source, frame rates and recording state over the preview.
struct PreviewStatus: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        HStack(spacing: 8) {
            if let kind = model.sourceKind {
                Chip(text: S.sourceName(kind), symbol: "dot.radiowaves.left.and.right")
            }
            if model.hasFrames {
                Chip(text: S.fps(model.stats.displayFPS, model.stats.analysisFPS), symbol: "speedometer")
            }
            if let started = model.recordingStartedAt {
                TimelineView(.periodic(from: started, by: 1)) { context in
                    Chip(text: S.recordingStatus(Int(context.date.timeIntervalSince(started))), symbol: "record.circle",
                         tint: .red)
                }
            }
            if model.playbackEnded {
                Chip(text: S.playbackFinished, symbol: "stop.circle")
            }
        }
    }
}

struct Chip: View {
    let text: String
    let symbol: String
    var tint: Color = .white

    var body: some View {
        Label(text, systemImage: symbol)
            .font(.caption.weight(.medium))
            .foregroundStyle(tint)
            .padding(.horizontal, 8).padding(.vertical, 4)
            .background(.black.opacity(0.55), in: Capsule())
    }
}

private struct RecordButton: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        Button {
            Task { await model.toggleRecording() }
        } label: {
            Label(model.isRecording ? S.stopRecording : S.record,
                  systemImage: model.isRecording ? "stop.circle.fill" : "record.circle")
                .foregroundStyle(model.isRecording ? .red : .primary)
        }
        .controlSize(.large)
        .disabled(model.isRecordingBusy || model.screen == .scanning)
    }
}
