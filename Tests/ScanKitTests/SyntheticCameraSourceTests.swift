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
    // 30 fps: ~33 ms apart. The source must not run faster than 30 fps. The upper bound is loose because the
    // reconstruction suites saturate every core while this test runs: rendering took 0.32 s per frame on a 3-core
    // CI runner.
    let interval = (arrivals.last! - arrivals[1]) / Double(arrivals.count - 2)
    #expect(interval > 0.025 && interval < 1.5, "mean frame interval \(interval) s")

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

@Test func syntheticCameraDefaultsSuitVisionTracking() {
    // Vision needs shading cues to find the face at +-45 degrees (see SyntheticCameraSource.defaultOptions).
    #expect(SyntheticCameraSource().options.lighting == 0.6)
    #expect(SyntheticCameraSource().options.width == SyntheticScan.Options().width)
}

@Test func syntheticCameraAnimationVisitsEveryScanPose() {
    var options = smallOptions()
    options.poses = ScanPose.required
    let source = SyntheticCameraSource(options: options, transitionSeconds: 1, holdSeconds: 2, amplitudes: [1, 1.2],
                                       stepSeconds: 0.5)
    // front: one stop; the other poses: the target, then 20 % further.
    #expect(abs(source.cycleSeconds - (3 + 4 * (3 + 2.5))) < 1e-9)
    var t = 0.0
    for pose in options.poses {
        let factors: [Float] = pose == .front ? [1] : [1, 1.2]
        for (k, factor) in factors.enumerated() {
            t += k == 0 ? 1 : 0.5
            // middle of the hold (sway at most 1.5 degrees in yaw, 1 in pitch)
            let (head, translation) = source.headPose(at: t + 1)
            #expect(abs(head.yaw - pose.target.yaw * factor) <= 1.6, "\(pose) x \(factor): yaw \(head.yaw)")
            #expect(abs(head.pitch - pose.target.pitch * factor) <= 1.1, "\(pose) x \(factor): pitch \(head.pitch)")
            #expect(abs(head.roll) <= 0.81)
            #expect(simd_length(translation) < 0.003)
            t += 2
        }
    }
    // halfway through the turn from front to up
    let between = source.headPose(at: 3.5).pose
    #expect(between.pitch > 5 && between.pitch < 20)
    // the step from up to 1.2 x up (25 -> 30 degrees in 0.5 s) stays below the guided scan's 20 deg/s motion gate
    let step = (source.headPose(at: 6.25).pose.pitch - source.headPose(at: 6.2).pose.pitch) / 0.05
    #expect(step > 5 && step < 16, "\(step) deg/s")
    // the sequence loops
    let a = source.headPose(at: 1.7).pose, b = source.headPose(at: 1.7 + source.cycleSeconds).pose
    #expect(abs(a.pitch - b.pitch) < 2.6 && abs(a.yaw - b.yaw) < 3.1)
    // The defaults turn 10 and 20 % beyond each target: left reaches 54 degrees, down -30.
    let defaults = SyntheticCameraSource(options: options)
    #expect(defaults.amplitudes == [1, 1.1, 1.2])
}
