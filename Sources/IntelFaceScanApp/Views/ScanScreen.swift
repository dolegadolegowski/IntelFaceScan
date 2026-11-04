// License: Apache 2.0. See LICENSE file in root directory.
//
// Guided scan: preview with the instruction, animated pose guide, per-pose and overall progress, coverage of the
// head, rejection hints, skip / finish / cancel.

import ScanKit
import SwiftUI

struct ScanScreen: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        HStack(alignment: .top, spacing: 16) {
            VStack(spacing: 14) {
                LivePreview()
                    .overlay(alignment: .topLeading) { PreviewStatus().padding(10) }
                    .overlay(alignment: .bottom) { InstructionBanner().padding(14) }
                DistanceGauge(distance: model.tracking.distance, isEstimate: model.tracking.distanceIsEstimate,
                              range: model.tracking.range, advice: model.tracking.advice)
                    .frame(maxWidth: 520)
            }
            ScanSidePanel()
                .frame(width: 330)
        }
    }
}

/// Big instruction text over the preview, with the rejection reason below it.
private struct InstructionBanner: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let update = model.scanUpdate
        let text = update.map { S.instruction($0.instructionKind) } ?? ""
        VStack(spacing: 4) {
            if !text.isEmpty {
                Text(text)
                    .font(.title2.weight(.semibold))
                    .foregroundStyle(.white)
            }
            if let rejection = update?.rejection {
                Label(S.rejection(rejection), systemImage: "exclamationmark.circle")
                    .font(.callout.weight(.medium))
                    .foregroundStyle(.yellow)
            }
        }
        // Centered when the preview is narrow and the instruction wraps (the Polish ones do).
        .multilineTextAlignment(.center)
        .padding(.horizontal, 18).padding(.vertical, 10)
        .background(.black.opacity(text.isEmpty && update?.rejection == nil ? 0 : 0.6), in: RoundedRectangle(cornerRadius: 12))
        .animation(.easeInOut(duration: 0.2), value: text)
    }
}

private struct ScanSidePanel: View {
    var body: some View {
        ScrollView { ScanPanelContent() }
    }
}

struct ScanPanelContent: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let update = model.scanUpdate
        let configuration = model.scanConfiguration
        VStack(alignment: .leading, spacing: 16) {
            Text(S.guidedScan).font(.title2.weight(.semibold))
            PoseGuideView(update: update, mirrored: model.settings.mirrored, tolerance: configuration.angleTolerance)
                .frame(height: 170)
                .frame(maxWidth: .infinity)
                .background(RoundedRectangle(cornerRadius: 12).fill(Color.secondary.opacity(0.08)))

            VStack(alignment: .leading, spacing: 8) {
                ForEach(configuration.poses, id: \.self) { pose in
                    PoseProgressRow(pose: pose, update: update,
                                    optional: configuration.optionalPoses.contains(pose))
                }
                Divider()
                HStack {
                    Text(S.overallProgress).font(.subheadline.weight(.semibold))
                    Spacer()
                    Text(S.keyframes(update?.keyframeCount ?? 0)).font(.caption).foregroundStyle(.secondary)
                }
                ProgressView(value: Double(update?.overallProgress ?? 0))
                    .animation(.easeOut(duration: 0.2), value: update?.overallProgress ?? 0)
            }

            VStack(alignment: .leading, spacing: 6) {
                Text(S.coverage).font(.subheadline.weight(.semibold))
                CoverageView(coverage: update?.coverage ?? CoverageMap(), poses: configuration.poses,
                             progress: update?.poseProgress ?? [:], mirrored: model.settings.mirrored)
                    .frame(height: 190)
            }

            HStack {
                if update?.canSkipPose == true {
                    Button(S.skipPose, systemImage: "forward.end") { model.skipPose() }
                }
                Spacer()
                Button(S.cancelScan, role: .cancel) { model.cancelScan() }
                    .keyboardShortcut(.escape, modifiers: [])
                Button(S.finishScan) { model.finishScan() }
                    .buttonStyle(.borderedProminent)
                    .disabled((update?.keyframeCount ?? 0) == 0)
            }
        }
        .padding(.vertical, 4)
    }
}

private struct PoseProgressRow: View {
    let pose: ScanPose
    let update: ScanUpdate?
    let optional: Bool

    var body: some View {
        let progress = update?.poseProgress[pose] ?? 0
        let current = update?.phase == .capturing(pose)
        let skipped = update?.skippedPoses.contains(pose) ?? false
        HStack(spacing: 8) {
            Image(systemName: progress >= 1 ? "checkmark.circle.fill" : (skipped ? "forward.circle" : (current ? "smallcircle.filled.circle" : "circle")))
                .foregroundStyle(progress >= 1 ? .green : (current ? Color.accentColor : .secondary))
                .frame(width: 18)
            Text(S.poseName(pose) + (optional ? " *" : ""))
                .font(current ? .callout.weight(.semibold) : .callout)
                .frame(width: 92, alignment: .leading)
            ProgressView(value: Double(progress))
                .tint(progress >= 1 ? .green : .accentColor)
                .opacity(skipped ? 0.4 : 1)
        }
        .animation(.easeOut(duration: 0.2), value: progress)
    }
}

/// Animated head turning toward the current pose, an arrow toward the remaining correction and a ring that fills
/// while the pose is held.
struct PoseGuideView: View {
    let update: ScanUpdate?
    let mirrored: Bool
    let tolerance: Float

    var body: some View {
        let pose = currentPose
        let target: (yaw: Float, pitch: Float) = pose?.target ?? (yaw: 0, pitch: 0)
        // Direction of the pose on screen (as the user sees the mirrored preview).
        let poseDirection = ScanGuidance.screenDirection(yawDelta: target.yaw, pitchDelta: target.pitch, mirrored: mirrored)
        let delta = update?.targetDelta ?? SIMD2(0, 0)
        let correction = ScanGuidance.screenDirection(yawDelta: delta.x, pitchDelta: delta.y, mirrored: mirrored)
        let atTarget = (update?.angularErrorDeg ?? .infinity) <= tolerance
        TimelineView(.animation) { context in
            let t = context.date.timeIntervalSinceReferenceDate
            // The head swings from the front toward the target and back, easing at both ends.
            let swing = CGFloat((1 - cos(t * 2 * .pi / 2.2)) / 2)
            let dx = CGFloat(poseDirection.x), dy = CGFloat(poseDirection.y)
            ZStack {
                Circle()
                    .trim(from: 0, to: CGFloat(update?.holdProgress ?? 0))
                    .stroke(Color.green, style: StrokeStyle(lineWidth: 5, lineCap: .round))
                    .rotationEffect(.degrees(-90))
                    .frame(width: 118, height: 118)
                    .opacity(atTarget ? 1 : 0)
                Image(systemName: "face.smiling")
                    .font(.system(size: 76, weight: .regular))
                    .foregroundStyle(atTarget ? Color.green : Color.primary)
                    .rotation3DEffect(.degrees(Double(dx * swing * 40)), axis: (x: 0, y: 1, z: 0), perspective: 0.6)
                    .rotation3DEffect(.degrees(Double(dy * swing * 30)), axis: (x: 1, y: 0, z: 0), perspective: 0.6)
                    .offset(x: dx * swing * 10, y: dy * swing * 8)
                if !atTarget, correction != .zero {
                    Image(systemName: "arrow.right")
                        .font(.system(size: 34, weight: .bold))
                        .foregroundStyle(Color.accentColor)
                        .rotationEffect(.radians(Double(atan2(correction.y, correction.x))))
                        .offset(x: CGFloat(correction.x) * (78 + 6 * swing), y: CGFloat(correction.y) * (78 + 6 * swing))
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .overlay(alignment: .bottom) {
            if let pose {
                Text(S.poseName(pose)).font(.headline).padding(.bottom, 6)
            }
        }
        .accessibilityLabel(currentPose.map { S.poseInstruction($0) } ?? "")
    }

    private var currentPose: ScanPose? {
        if case .capturing(let pose) = update?.phase { return pose }
        return nil
    }
}

/// Coverage map drawn as the head regions seen so far (front at the center; turning one way shows the other
/// side of the head), with the pose targets as rings.
struct CoverageView: View {
    let coverage: CoverageMap
    let poses: [ScanPose]
    let progress: [ScanPose: Float]
    let mirrored: Bool

    var body: some View {
        Canvas { context, size in
            let head = CGRect(x: size.width * 0.12, y: 4, width: size.width * 0.76, height: size.height - 8)
            let oval = Path(ellipseIn: head)
            func point(_ p: SIMD2<Float>) -> CGPoint {
                CGPoint(x: head.midX + CGFloat(p.x) * head.width / 2, y: head.midY + CGFloat(p.y) * head.height / 2)
            }
            context.fill(oval, with: .color(.secondary.opacity(0.12)))
            var cells = context
            cells.clip(to: oval)
            let cellW = head.width / CGFloat(CoverageMap.yawBins) * 1.02
            let cellH = head.height / CGFloat(CoverageMap.pitchBins) * 1.02
            for pb in 0 ..< CoverageMap.pitchBins {
                for yb in 0 ..< CoverageMap.yawBins {
                    let value = coverage.value(yawBin: yb, pitchBin: pb)
                    guard value > 0.02 else { continue }
                    let c = CoverageMap.binCenter(yawBin: yb, pitchBin: pb)
                    let center = point(ScanGuidance.coveredRegionPosition(yaw: c.yaw, pitch: c.pitch, mirrored: mirrored))
                    let rect = CGRect(x: center.x - cellW / 2, y: center.y - cellH / 2, width: cellW, height: cellH)
                    cells.fill(Path(rect), with: .color(.green.opacity(Double(0.15 + 0.7 * value))))
                }
            }
            context.stroke(oval, with: .color(.secondary.opacity(0.6)), lineWidth: 1.5)
            // Face features at the front for orientation.
            let front = point(.zero)
            let eye = head.width * 0.06
            for side: CGFloat in [-1, 1] {
                context.fill(Path(ellipseIn: CGRect(x: front.x + side * head.width * 0.12 - eye / 2, y: front.y - head.height * 0.12,
                                                     width: eye, height: eye * 0.7)),
                             with: .color(.secondary.opacity(0.5)))
            }
            for pose in poses {
                let target = pose.target
                let p = point(ScanGuidance.coveredRegionPosition(yaw: target.yaw, pitch: target.pitch, mirrored: mirrored))
                let done = (progress[pose] ?? 0) >= 1
                let ring = Path(ellipseIn: CGRect(x: p.x - 6, y: p.y - 6, width: 12, height: 12))
                if done { context.fill(ring, with: .color(.green)) }
                context.stroke(ring, with: .color(done ? .white : .secondary), lineWidth: 1.5)
            }
        }
        .accessibilityLabel(String(format: "%.0f %%", coverage.scanAreaCoverage * 100))
    }
}
