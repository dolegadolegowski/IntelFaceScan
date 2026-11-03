// License: Apache 2.0. See LICENSE file in root directory.
//
// SyntheticCameraSource: the CameraSource lifecycle, frame rate, resolution, filters and the pose animation.

import CameraKit
import Foundation
import ScanKit
import simd
import SyntheticData
import Testing

private func smallOptions() -> SyntheticScan.Options {
    var options = SyntheticScan.Options()
    options.meshResolution = 48
    return options
}

@Test func syntheticCameraStreamsAtTheConfiguredRateAndResolution() async throws {
    let source = SyntheticCameraSource(options: smallOptions())
    #expect(source.kind == .synthetic)
    let events = source.events()
    let device = try await source.connect()
    #expect(device.name == "Synthetic D455" && device.source == "synthetic" && device.imu)
    #expect(device.profiles.contains { $0.stream == "color" && $0.width == 640 && $0.height == 400 })

    var configuration = CaptureConfiguration.faceScanD455
    configuration.color = StreamSettings(width: 320, height: 200, fps: 30)
    configuration.filters.threshold = .init(enabled: true, minM: 0.2, maxM: 0.9)   // removes the wall at 1.1 m
    try await source.configure(configuration)
    let frames = source.frames(bufferingNewest: 4)
    try await source.start()
    try await source.start()   // idempotent
    var received: [CameraFrame] = []
    var arrivals: [Double] = []
    for await frame in frames {
        received.append(frame)
        arrivals.append(ProcessInfo.processInfo.systemUptime)
        if received.count == 15 { break }
    }
    try await source.stop()

    #expect(received.count == 15)
    for frame in received {
        #expect(frame.color.width == 320 && frame.color.height == 200 && frame.alignedDepth.width == 320)
        #expect(frame.color.pixels.count == 320 * 200 * 3)
        var valid = 0, beyond = 0
        frame.alignedDepth.pixels.withUnsafeBytes { raw in
            for v in raw.bindMemory(to: UInt16.self) where v != 0 {
                valid += 1
                if Float(UInt16(littleEndian: v)) * frame.alignedDepth.depthScale > 0.9 { beyond += 1 }
            }
        }
        #expect(valid > 1_000 && beyond == 0)
    }
    let sequences = received.map(\.sequence)
    #expect(zip(sequences, sequences.dropFirst()).allSatisfy { $0 < $1 })
    #expect(zip(received, received.dropFirst()).allSatisfy { $0.timestampMs < $1.timestampMs })
    // 30 fps: ~33 ms apart (rendering 320x200 takes a few ms even in debug builds).
    let interval = (arrivals.last! - arrivals[1]) / Double(arrivals.count - 2)
    #expect(interval > 0.025 && interval < 0.07, "mean frame interval \(interval) s")

    await source.shutdown()
    await source.shutdown()   // idempotent
    var names: [String] = []
    for await event in events {
        switch event {
        case .connected: names.append("connected")
        case .streamingStarted: names.append("started")
        case .streamingStopped: names.append("stopped")
        default: break
        }
    }
    #expect(names == ["connected", "started", "stopped"])
    var afterShutdown = 0
    for await _ in source.frames(bufferingNewest: 1) { afterShutdown += 1 }
    #expect(afterShutdown == 0)
    await #expect(throws: CameraError.notConnected) { try await source.start() }
}

@Test func syntheticCameraNeedsAConnectionAndValidSettings() async throws {
    let source = SyntheticCameraSource(options: smallOptions())
    await #expect(throws: CameraError.notConnected) { try await source.start() }
    _ = try await source.connect()
    var configuration = CaptureConfiguration.faceScanD455
    configuration.color = StreamSettings(width: 0, height: 200, fps: 30)
    await #expect(throws: CameraError.self) { try await source.configure(configuration) }
    await #expect(throws: CameraError.self) { try await source.startRecording(to: URL(fileURLWithPath: "/tmp/x.db3")) }
    try await source.setOption(sensor: .depth, name: "Laser Power", value: 150)
    await source.shutdown()
}

@Test func syntheticCameraAnimationVisitsEveryScanPose() {
    var options = smallOptions()
    options.poses = ScanPose.required
    let source = SyntheticCameraSource(options: options, transitionSeconds: 1, holdSeconds: 2)
    for (i, pose) in options.poses.enumerated() {
        // middle of the hold phase of pose i (sway is at most 1.5 degrees)
        let (head, translation) = source.headPose(at: Double(i) * 3 + 2)
        #expect(abs(head.yaw - pose.target.yaw) <= 1.6, "\(pose): yaw \(head.yaw)")
        #expect(abs(head.pitch - pose.target.pitch) <= 1.1, "\(pose): pitch \(head.pitch)")
        #expect(abs(head.roll) <= 0.81)
        #expect(simd_length(translation) < 0.003)
    }
    // halfway through the turn from front to up
    let between = source.headPose(at: 3.5).pose
    #expect(between.pitch > 5 && between.pitch < 20)
    // the sequence loops
    let a = source.headPose(at: 1.7).pose, b = source.headPose(at: 1.7 + 15).pose
    #expect(abs(a.pitch - b.pitch) < 2.6 && abs(a.yaw - b.yaw) < 3.1)
}
