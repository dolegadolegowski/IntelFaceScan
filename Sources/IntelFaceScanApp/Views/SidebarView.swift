// License: Apache 2.0. See LICENSE file in root directory.
//
// Sidebar: source selection, camera information and the persisted settings.

import CameraKit
import ReconKit
import SwiftUI

struct SidebarView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        Form {
            SourceSection()
            if let device = model.device {
                CameraInfoSection(device: device)
            }
            SettingsSections()
        }
        .formStyle(.grouped)
    }
}

private struct SourceSection: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        Section(S.sources) {
            if let kind = model.sourceKind {
                HStack {
                    Image(systemName: symbol(kind)).foregroundStyle(.tint)
                    Text(S.sourceName(kind)).lineLimit(1).truncationMode(.middle)
                    Spacer()
                    if model.isConnecting { ProgressView().controlSize(.small) }
                    Button { model.closeSourceAction() } label: { Image(systemName: "xmark.circle.fill") }
                        .buttonStyle(.borderless)
                        .help(S.closeSource)
                }
            }
            Menu {
                Button(S.liveCamera, systemImage: "camera.aperture") { model.open(.live) }
                Button(S.openRecording, systemImage: "film.stack") { Task { await SourceActions.openRecording(model) } }
                Button(S.openDataset, systemImage: "folder") { Task { await SourceActions.openDataset(model) } }
                Button(S.syntheticDemo, systemImage: "cube.transparent") { model.open(.synthetic) }
            } label: {
                Label(model.hasSource ? S.sources + "…" : S.sources, systemImage: "plus.circle")
            }
            .disabled(model.isConnecting || model.screen == .scanning)
        }
    }

    private func symbol(_ kind: CameraSourceKind) -> String {
        switch kind {
        case .live: return "camera.aperture"
        case .playback: return "film.stack"
        case .dataset: return "folder"
        case .synthetic: return "cube.transparent"
        }
    }
}

private struct CameraInfoSection: View {
    @Environment(AppModel.self) private var model
    let device: DeviceInfo

    var body: some View {
        Section(S.camera) {
            LabeledContent(S.model, value: device.name)
            LabeledContent(S.serial, value: device.serial)
            LabeledContent(S.firmware) {
                if let recommended = device.recommendedFirmware, !recommended.isEmpty, recommended != device.firmware {
                    Text("\(device.firmware) → \(recommended)").foregroundStyle(.orange)
                        .help(S.recommendedFirmware + ": " + recommended)
                } else {
                    Text(device.firmware)
                }
            }
            LabeledContent(S.usb, value: device.usbType)
            if let major = Double(device.usbType), major < 3 {
                Label(S.usb2Warning, systemImage: "exclamationmark.triangle.fill")
                    .foregroundStyle(.orange).font(.caption)
            }
            LabeledContent(S.depthScale, value: String(format: "%.2f mm", device.depthScale * 1000))
            if let baseline = device.baselineMm {
                LabeledContent(S.baseline, value: String(format: "%.1f mm", baseline))
            }
            LabeledContent(S.imu, value: device.imu ? S.yes : S.no)
            if !device.profiles.isEmpty {
                DisclosureGroup(S.profiles) {
                    ForEach(profileLines, id: \.self) { line in
                        Text(line).font(.caption.monospaced()).foregroundStyle(.secondary)
                    }
                }
            }
            if model.isLive {
                Button(S.hardwareReset, systemImage: "arrow.clockwise") { Task { await model.hardwareReset() } }
                    .disabled(model.screen == .scanning || model.isRecording)
            }
        }
    }

    /// One line per stream and format, listing the sizes and frame rates.
    private var profileLines: [String] {
        var grouped: [String: Set<String>] = [:]
        for p in device.profiles {
            grouped["\(p.stream) \(p.format)", default: []].insert(p.width > 0 ? "\(p.width)×\(p.height)@\(p.fps)" : "\(p.fps) Hz")
        }
        return grouped.keys.sorted().map { key in
            key + ": " + (grouped[key] ?? []).sorted { $0.localizedStandardCompare($1) == .orderedDescending }.joined(separator: ", ")
        }
    }
}

private struct SettingsSections: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        let scanning = model.screen == .scanning
        Section(S.depthSection) {
            Picker(S.depthResolution, selection: $model.settings.depthResolution) {
                ForEach(AppSettings.DepthResolution.allCases) { Text($0.label).tag($0) }
            }
            Picker(S.preset, selection: $model.settings.preset) {
                ForEach(AppSettings.DepthPreset.allCases) { Text($0.label).tag($0) }
            }
            Toggle(S.closeRange, isOn: $model.settings.closeRange)
                .help(S.closeRangeHint)
            if model.settings.closeRange {
                Stepper(value: $model.settings.disparityShift, in: 10 ... 200, step: 10) {
                    LabeledContent(S.disparityShift, value: "\(model.settings.disparityShift)")
                }
            }
            Toggle(S.laserPower + " – " + S.laserMax, isOn: $model.settings.laserPowerMax)
            if !model.settings.laserPowerMax {
                LabeledContent(S.laserPower) {
                    HStack {
                        Slider(value: $model.settings.laserPower, in: 0 ... 360, step: 30)
                        Text("\(Int(model.settings.laserPower)) mW").monospacedDigit().frame(width: 56, alignment: .trailing)
                    }
                }
            }
            if scanning {
                Text(S.settingsLockedDuringScan).font(.caption).foregroundStyle(.secondary)
            }
        }
        .disabled(scanning)

        Section(S.filters) {
            Toggle(S.spatialFilter, isOn: $model.settings.spatialFilter)
            Toggle(S.temporalFilter, isOn: $model.settings.temporalFilter)
            Toggle(S.holeFilling, isOn: $model.settings.holeFilling)
            LabeledContent(S.maxDistance) {
                HStack {
                    Slider(value: $model.settings.maxDistanceM, in: 0.6 ... 2.0, step: 0.1)
                    Text(String(format: "%.1f m", model.settings.maxDistanceM)).monospacedDigit()
                        .frame(width: 44, alignment: .trailing)
                }
            }
        }
        .disabled(scanning)

        Section(S.scanSection) {
            Toggle(S.keepShoulders, isOn: $model.settings.keepShoulders)
            Toggle(S.diagonalPoses, isOn: $model.settings.diagonalPoses)
            Stepper(value: $model.settings.framesPerPose, in: 1 ... 6) {
                LabeledContent(S.framesPerPose, value: "\(model.settings.framesPerPose)")
            }
        }
        .disabled(scanning)

        Section(S.reconstructionSection) {
            LabeledContent(S.voxelSize) {
                HStack {
                    Slider(value: $model.settings.voxelSizeMM, in: 0.5 ... 1.5, step: 0.05)
                    Text(String(format: "%.2f mm", model.settings.voxelSizeMM)).monospacedDigit()
                        .frame(width: 64, alignment: .trailing)
                }
            }
            Picker(S.surfaceMethod, selection: $model.settings.method) {
                Text(S.methodTSDF).tag(SurfaceMethod.tsdf)
                Text(S.methodPoisson).tag(SurfaceMethod.poisson)
            }
        }
        .disabled(model.screen == .reconstructing)

        Section(S.displaySection) {
            Toggle(S.mirrored, isOn: $model.settings.mirrored)
            Toggle(S.landmarks, isOn: $model.settings.showLandmarks)
            Toggle(S.autoExposureFace, isOn: $model.settings.autoExposureOnFace)
            Button(S.resetSettings, role: .destructive) { model.resetSettings() }
                .disabled(scanning)
        }
    }
}
