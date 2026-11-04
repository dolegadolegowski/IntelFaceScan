// License: Apache 2.0. See LICENSE file in root directory.
//
// The live preview: the Metal view, the face overlay (rectangle and landmarks, mirror-aware) and the distance
// gauge with the recommended band.

import MetalKit
import ScanKit
import SwiftUI

/// MTKView hosting the shared PreviewRenderer.
struct MetalPreviewView: NSViewRepresentable {
    let renderer: PreviewRenderer
    let display: PreviewDisplay

    func makeNSView(context: Context) -> MTKView {
        let view = MTKView(frame: .zero, device: renderer.device)
        renderer.attach(view)
        renderer.setDisplay(display)
        return view
    }

    func updateNSView(_ view: MTKView, context: Context) {
        renderer.setDisplay(display)
    }
}

/// Face rectangle and landmarks in color-image pixels, drawn over a view showing the whole image.
struct FaceOverlay: View {
    let tracking: TrackingState
    let mirrored: Bool
    let showLandmarks: Bool

    var body: some View {
        Canvas { context, size in
            guard let face = tracking.faceRect, tracking.imageSize.width > 0, tracking.imageSize.height > 0 else { return }
            let sx = size.width / tracking.imageSize.width
            let sy = size.height / tracking.imageSize.height
            func map(_ x: CGFloat, _ y: CGFloat) -> CGPoint {
                CGPoint(x: (mirrored ? tracking.imageSize.width - x : x) * sx, y: y * sy)
            }
            let a = map(face.minX, face.minY), b = map(face.maxX, face.maxY)
            let rect = CGRect(x: min(a.x, b.x), y: min(a.y, b.y), width: abs(b.x - a.x), height: abs(b.y - a.y))
            let color: Color = tracking.advice == .ok ? .green : .orange
            context.stroke(Path(roundedRect: rect, cornerRadius: 6), with: .color(color.opacity(0.9)), lineWidth: 2)
            if showLandmarks {
                var dots = Path()
                for p in tracking.landmarks {
                    let q = map(CGFloat(p.x), CGFloat(p.y))
                    dots.addEllipse(in: CGRect(x: q.x - 1.5, y: q.y - 1.5, width: 3, height: 3))
                }
                context.fill(dots, with: .color(.cyan.opacity(0.85)))
            }
        }
        .allowsHitTesting(false)
    }
}

/// Preview image with overlays, letterboxed to the image aspect ratio.
struct LivePreview: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let size = model.tracking.imageSize
        let aspect = size.width > 0 && size.height > 0 ? size.width / size.height : 16.0 / 10.0
        ZStack {
            Color.black
            if let renderer = model.renderer {
                ZStack {
                    MetalPreviewView(renderer: renderer, display: model.previewDisplay)
                    FaceOverlay(tracking: model.tracking, mirrored: model.settings.mirrored,
                                showLandmarks: model.settings.showLandmarks)
                }
                .aspectRatio(aspect, contentMode: .fit)
                if !model.hasFrames {
                    ProgressView(S.waitingForFrames).controlSize(.large).tint(.white).foregroundStyle(.white)
                }
            } else {
                VStack(spacing: 8) {
                    Image(systemName: "exclamationmark.triangle").font(.largeTitle)
                    Text(S.previewUnavailable).font(.headline)
                    if let message = model.rendererError { Text(message).font(.caption) }
                }
                .foregroundStyle(.white)
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: 10))
    }
}

/// Distance bar: recommended band in green, the measured distance as a marker, and advice.
struct DistanceGauge: View {
    let distance: Float?
    let isEstimate: Bool
    let range: ClosedRange<Float>
    let advice: DistanceAdvice
    var scale: ClosedRange<Float> = 0.25 ... 1.2

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Label(S.distance, systemImage: "ruler")
                    .font(.subheadline.weight(.semibold))
                    .lineLimit(1)
                Spacer(minLength: 8)
                Text(adviceText)
                    .font(.subheadline.weight(.semibold))
                    .lineLimit(1)
                    .foregroundStyle(adviceColor)
                    .contentTransition(.opacity)
            }
            GeometryReader { geo in
                let width = geo.size.width
                ZStack(alignment: .leading) {
                    Capsule().fill(Color.secondary.opacity(0.2))
                    Capsule().fill(Color.green.opacity(0.45))
                        .frame(width: max(4, x(range.upperBound, width) - x(range.lowerBound, width)))
                        .offset(x: x(range.lowerBound, width))
                    if let d = distance {
                        Circle()
                            .fill(adviceColor)
                            .overlay(Circle().stroke(Color.white, lineWidth: 2))
                            .frame(width: 16, height: 16)
                            .offset(x: x(d, width) - 8)
                            .animation(.easeOut(duration: 0.15), value: d)
                    }
                }
            }
            .frame(height: 14)
            // The measured distance and the recommended range side by side, or stacked when narrow.
            ViewThatFits(in: .horizontal) {
                HStack {
                    measuredText
                    Spacer()
                    recommendedText
                }
                VStack(alignment: .leading, spacing: 2) {
                    measuredText
                    recommendedText
                }
            }
            .font(.caption)
        }
        .accessibilityElement(children: .combine)
    }

    private var measuredText: some View {
        Text(distance.map { S.meters($0) + (isEstimate ? " (\(S.estimated))" : "") } ?? "–")
            .monospacedDigit()
            .lineLimit(1)
    }

    private var recommendedText: some View {
        Text(S.recommended(range)).foregroundStyle(.secondary).lineLimit(1)
    }

    private func x(_ d: Float, _ width: CGFloat) -> CGFloat {
        let t = (min(max(d, scale.lowerBound), scale.upperBound) - scale.lowerBound) / (scale.upperBound - scale.lowerBound)
        return CGFloat(t) * width
    }

    private var adviceText: String {
        switch advice {
        case .ok: return S.goodDistance
        case .tooClose: return S.moveBack
        case .tooFar: return S.moveCloser
        case .unknown: return distance == nil ? S.noFace : ""
        }
    }

    private var adviceColor: Color {
        switch advice {
        case .ok: return .green
        case .tooClose, .tooFar: return .orange
        case .unknown: return .secondary
        }
    }
}
