// License: Apache 2.0. See LICENSE file in root directory.
//
// After the scan: summary, reconstruction progress and the result (3D preview, export, report).

import ReconKit
import ScanKit
import SwiftUI

struct ScanSummaryView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let keyframes = model.capture?.keyframes ?? []
        let poses = ScanPose.allCases.filter { pose in keyframes.contains { $0.scanPose == pose } }
        VStack(spacing: 22) {
            Image(systemName: keyframes.isEmpty ? "exclamationmark.triangle" : "checkmark.seal")
                .font(.system(size: 54, weight: .light))
                .foregroundStyle(keyframes.isEmpty ? Color.orange : Color.green)
            Text(S.scanComplete).font(.largeTitle.weight(.semibold))
            Text(keyframes.isEmpty ? S.noKeyframes : S.scanSummary(keyframes.count, poses.count))
                .font(.title3).foregroundStyle(.secondary)
            if !keyframes.isEmpty {
                HStack(spacing: 10) {
                    ForEach(poses, id: \.self) { pose in
                        VStack(spacing: 2) {
                            Text("\(keyframes.filter { $0.scanPose == pose }.count)").font(.title3.monospacedDigit())
                            Text(S.poseName(pose)).font(.caption).foregroundStyle(.secondary)
                        }
                        .frame(minWidth: 64)
                        .padding(8)
                        .background(RoundedRectangle(cornerRadius: 8).fill(Color.secondary.opacity(0.08)))
                    }
                }
            }
            HStack(spacing: 12) {
                if model.canReturnToPreview {
                    Button(S.newScan, systemImage: "arrow.counterclockwise") { model.backToPreview() }
                }
                if !keyframes.isEmpty {
                    Button(S.saveSession, systemImage: "square.and.arrow.down") { Task { await model.saveSession() } }
                    Button {
                        model.reconstruct()
                    } label: {
                        Label(S.reconstruct, systemImage: "cube").padding(.horizontal, 6)
                    }
                    .buttonStyle(.borderedProminent)
                    .keyboardShortcut(.return, modifiers: [.command])
                }
            }
            .controlSize(.large)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}

struct ReconstructionView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let progress = model.reconstructionProgress
        VStack(spacing: 18) {
            Text(S.reconstructing).font(.largeTitle.weight(.semibold))
            ProgressView(value: min(max(progress?.fraction ?? 0, 0), 1)) {
                Text(progress.map { S.stageName($0.stage) } ?? "")
                    .font(.headline)
            } currentValueLabel: {
                Text(progress?.message ?? "").foregroundStyle(.secondary)
            }
            .frame(maxWidth: 520)
            .animation(.easeOut(duration: 0.2), value: progress?.fraction ?? 0)
            Button(S.cancel, role: .cancel) { model.cancelReconstruction() }
                .controlSize(.large)
                .keyboardShortcut(.escape, modifiers: [])
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}

struct ResultView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        if let result = model.result {
            HStack(alignment: .top, spacing: 16) {
                ModelPreviewView(result: result, displayMode: $model.displayMode)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .clipShape(RoundedRectangle(cornerRadius: 10))
                VStack(alignment: .leading, spacing: 14) {
                    Text(S.result).font(.title2.weight(.semibold))
                    Picker(S.displayMode, selection: $model.displayMode) {
                        ForEach(ModelDisplayMode.allCases) { Text(S.displayModeName($0.rawValue)).tag($0) }
                    }
                    VStack(alignment: .leading, spacing: 8) {
                        Button { Task { await model.saveOBJ() } } label: {
                            Label(S.saveOBJ, systemImage: "square.and.arrow.down").frame(maxWidth: .infinity)
                        }
                        .buttonStyle(.borderedProminent)
                        .keyboardShortcut("s")
                        Button { Task { await model.savePLY() } } label: {
                            Label(S.savePLY, systemImage: "square.and.arrow.down").frame(maxWidth: .infinity)
                        }
                        Button { Task { await model.saveSession() } } label: {
                            Label(S.saveSession, systemImage: "folder.badge.plus").frame(maxWidth: .infinity)
                        }
                        .disabled(model.capture == nil)
                    }
                    .controlSize(.large)
                    ReportView(report: result.report)
                    Spacer()
                    if model.canReturnToPreview {
                        Button(S.newScan, systemImage: "arrow.counterclockwise") { model.backToPreview() }
                    }
                }
                .frame(width: 320)
            }
        } else {
            WelcomeView()
        }
    }
}

private struct ReportView: View {
    let report: ReconstructionReport

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(S.report).font(.subheadline.weight(.semibold))
            Grid(alignment: .leading, horizontalSpacing: 10, verticalSpacing: 4) {
                row(S.reportKeyframes, "\(report.usedFrames) / \(report.keyframes)")
                row(S.reportRMSE, String(format: "%.2f / %.2f mm", report.meanRegistrationRMSEmm, report.maxRegistrationRMSEmm))
                row(S.reportFitness, String(format: "%.2f", report.meanFitness))
                row(S.reportEdges, "\(report.odometryEdges) / \(report.loopClosures) / \(report.rejectedEdges)")
                row(S.reportVoxel, String(format: "%.2f mm", report.voxelSizeMM))
                row(S.reportMethod, (report.method == .tsdf ? S.methodTSDF : S.methodPoisson)
                    + " · " + (report.usedGPU ? S.gpu : S.cpu))
                row(S.reportMesh, "\(report.vertexCount.formatted()) / \(report.triangleCount.formatted())")
                if !report.durations.isEmpty {
                    Divider().gridCellColumns(2)
                    Text(S.reportTimings).font(.caption.weight(.semibold)).gridCellColumns(2)
                    ForEach(report.durations.sorted { $0.key < $1.key }, id: \.key) { entry in
                        row(entry.key, String(format: "%.2f s", entry.value))
                    }
                    row("Σ", String(format: "%.2f s", report.durations.values.reduce(0, +)))
                }
            }
            .font(.caption)
            .frame(maxWidth: .infinity, alignment: .leading)
            if !report.notes.isEmpty {
                Divider()
                Text(S.reportNotes).font(.caption.weight(.semibold))
                ForEach(Array(report.notes.enumerated()), id: \.offset) { _, note in
                    Text("• " + note).font(.caption).foregroundStyle(.secondary)
                }
            }
        }
        .padding(10)
        .background(RoundedRectangle(cornerRadius: 10).fill(Color.secondary.opacity(0.08)))
    }

    @ViewBuilder
    private func row(_ label: String, _ value: String) -> some View {
        GridRow {
            Text(label).foregroundStyle(.secondary)
            Text(value).monospacedDigit()
        }
    }
}
