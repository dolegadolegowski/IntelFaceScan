// License: Apache 2.0. See LICENSE file in root directory.
//
// Interactive 3D preview of the reconstructed head (SceneKit): texture, vertex colors, neutral shading or wireframe;
// orbit (drag), zoom (scroll / pinch), pan (two-finger drag or option-drag), framed on the face, reset button.

import AppKit
import ReconKit
import SceneKit
import SwiftUI

enum ModelDisplayMode: String, CaseIterable, Identifiable {
    case texture, vertexColors, shaded, wireframe
    var id: String { rawValue }

    var title: String {
        switch self {
        case .texture: return "Texture"
        case .vertexColors: return "Vertex Colors"
        case .shaded: return "Shaded"
        case .wireframe: return "Wireframe"
        }
    }

    var systemImage: String {
        switch self {
        case .texture: return "photo"
        case .vertexColors: return "paintpalette"
        case .shaded: return "cube"
        case .wireframe: return "squareshape.split.3x3"
        }
    }
}

struct ModelPreviewView: View {
    let result: ReconstructionResult
    @Binding var displayMode: ModelDisplayMode

    init(result: ReconstructionResult, displayMode: Binding<ModelDisplayMode>) {
        self.result = result
        self._displayMode = displayMode
    }

    var body: some View {
        // No SwiftUI state here: with the Command Line Tools the @State macro plugin is unavailable, so the reset
        // button lives in the AppKit view.
        SceneModelView(result: result, displayMode: displayMode)
            .overlay(alignment: .bottomLeading) {
                ModelInfoLabel(result: result, displayMode: displayMode)
                    .padding(10)
            }
    }
}

/// Size in millimeters, triangle count and texture size.
private struct ModelInfoLabel: View {
    let result: ReconstructionResult
    let displayMode: ModelDisplayMode

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(sizeText).monospacedDigit()
            Text(detailText).foregroundStyle(.secondary)
            if let note { Text(note).foregroundStyle(.secondary) }
        }
        .font(.caption)
        .padding(.horizontal, 8)
        .padding(.vertical, 5)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 6))
    }

    private var size: SIMD3<Float> {
        let m = result.exportTransform
        var lo = SIMD3<Float>(repeating: .greatestFiniteMagnitude), hi = -lo
        for p in result.mesh.positions {
            let q = m * SIMD4(p, 1)
            let v = SIMD3(q.x, q.y, q.z)
            lo = simd_min(lo, v); hi = simd_max(hi, v)
        }
        return result.mesh.positions.isEmpty ? .zero : (hi - lo) * 1000
    }

    private var sizeText: String {
        let s = size
        return String(format: "%.0f × %.0f × %.0f mm (W × H × D)", s.x, s.y, s.z)
    }

    private var detailText: String {
        var parts = ["\(result.mesh.triangleCount.formatted()) triangles"]
        if let t = result.texture { parts.append("texture \(t.width)×\(t.height)") }
        return parts.joined(separator: " · ")
    }

    private var note: String? {
        switch displayMode {
        case .texture where result.texture == nil || result.mesh.uvs.isEmpty:
            return "No texture: showing \(result.mesh.colors.isEmpty ? "shading" : "vertex colors")"
        case .vertexColors where result.mesh.colors.isEmpty:
            return "No vertex colors: showing shading"
        default:
            return nil
        }
    }
}

/// The SceneKit view. The scene (geometry sources, texture) is built once per result; mode changes swap materials.
private struct SceneModelView: NSViewRepresentable {
    let result: ReconstructionResult
    let displayMode: ModelDisplayMode

    final class Coordinator: NSObject {
        var fingerprint: PreviewFingerprint?
        var preview: PreviewScene?
        var mode: ModelDisplayMode?
        weak var view: SCNView?

        @objc func resetCamera(_ sender: Any?) {
            guard let preview, let view else { return }
            SCNTransaction.begin()
            SCNTransaction.animationDuration = 0.35
            preview.frameCamera(in: view)
            SCNTransaction.commit()
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView(frame: .zero, options: [SCNView.Option.preferredRenderingAPI.rawValue: SCNRenderingAPI.metal.rawValue])
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = false
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = NSColor(calibratedWhite: 0.13, alpha: 1)
        view.rendersContinuously = false
        view.defaultCameraController.interactionMode = .orbitTurntable
        view.defaultCameraController.inertiaEnabled = true
        view.defaultCameraController.maximumVerticalAngle = 89
        view.defaultCameraController.minimumVerticalAngle = -89
        context.coordinator.view = view
        addResetButton(to: view, coordinator: context.coordinator)
        update(view, context: context)
        return view
    }

    func updateNSView(_ view: SCNView, context: Context) {
        update(view, context: context)
    }

    private func addResetButton(to view: SCNView, coordinator: Coordinator) {
        let image = NSImage(systemSymbolName: "scope", accessibilityDescription: "Reset the view") ?? NSImage()
        let button = NSButton(image: image, target: coordinator, action: #selector(Coordinator.resetCamera(_:)))
        button.bezelStyle = .regularSquare
        button.isBordered = true
        button.toolTip = "Reset the view"
        button.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(button)
        NSLayoutConstraint.activate([
            button.topAnchor.constraint(equalTo: view.topAnchor, constant: 10),
            button.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -10),
            button.widthAnchor.constraint(equalToConstant: 28),
            button.heightAnchor.constraint(equalToConstant: 28),
        ])
    }

    private func update(_ view: SCNView, context: Context) {
        let c = context.coordinator
        let fingerprint = PreviewFingerprint(result)
        if c.fingerprint != fingerprint || c.preview == nil {
            let preview = PreviewScene(result: result)
            view.scene = preview.scene
            preview.frameCamera(in: view)
            c.preview = preview
            c.fingerprint = fingerprint
            c.mode = nil
        }
        if let preview = c.preview, c.mode != displayMode {
            preview.apply(displayMode)
            c.mode = displayMode
        }
    }
}
