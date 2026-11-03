// License: Apache 2.0. See LICENSE file in root directory.
//
// The JSON examples of docs/PROTOCOL.md decoded into the shared types, commands encoded the way the helper expects,
// and FRAMESET decoding into CameraFrame.

import Foundation
import Testing
@testable import CameraKit

@Suite("Helper JSON")
struct HelperJSONTests {
    private func json(_ text: String) -> Data { Data(text.utf8) }

    private func object(_ data: Data) throws -> [String: Any] {
        try #require(try JSONSerialization.jsonObject(with: data) as? [String: Any])
    }

    @Test func hello() throws {
        let hello = try HelperJSON.decoder.decode(HelperHello.self, from: json(
            #"{"protocol": 1, "helper_version": "0.1.0", "sdk_version": "2.58.4", "pid": 4242, "euid": 0, "mode": "live"}"#))
        #expect(hello == HelperHello(protocol: 1, helperVersion: "0.1.0", sdkVersion: "2.58.4", pid: 4242, euid: 0,
                                     mode: "live"))
    }

    @Test func replies() throws {
        let ok = try HelperJSON.decoder.decode(WireReply.self, from: json(#"{"request_id": 7, "ok": true, "data": {}}"#))
        #expect(ok.requestId == 7 && ok.ok && ok.error == nil)
        let failed = try HelperJSON.decoder.decode(WireReply.self, from: json(
            #"{"request_id": 8, "ok": false, "error": "Laser Power: value out of range [0, 360]"}"#))
        #expect(failed.requestId == 8 && !failed.ok)
        #expect(failed.error == "Laser Power: value out of range [0, 360]")
        let bytes = try HelperCameraSession.decodeData(WireBytes.self, from: json(
            #"{"request_id": 12, "ok": true, "data": {"bytes": 123456789}}"#))
        #expect(bytes.bytes == 123_456_789)
    }

    @Test func logAndEvent() throws {
        let log = try HelperJSON.decoder.decode(WireLog.self, from: json(#"{"level": "warning", "message": "hot"}"#))
        #expect(log.level == "warning" && log.message == "hot")
        let event = try HelperJSON.decoder.decode(WireEvent.self, from: json(
            #"{"event": "recording_finished", "message": "", "data": {"bytes": 42, "file_name": "x"}}"#))
        #expect(event.event == "recording_finished")
        #expect(event.data?["bytes"]?.number == 42)
        #expect(event.data?["file_name"]?.string == "x")  // free-form keys are not rewritten
    }

    static let deviceInfoJSON = #"""
    {"request_id": 1, "ok": true, "data": {
      "name": "Intel RealSense D455", "serial": "123456789012", "firmware": "5.16.0.1",
      "recommended_firmware": "5.17.0.10", "usb_type": "3.2", "product_id": "0B5C", "product_line": "D400",
      "advanced_mode": true, "depth_scale": 0.0001, "baseline_mm": 95.0, "imu": true, "source": "live",
      "presets": ["Custom", "Default", "Hand", "High Accuracy", "High Density", "Medium Density"],
      "sensors": [
        {"name": "Stereo Module", "kind": "depth", "options": [
          {"name": "Laser Power", "value": 150, "min": 0, "max": 360, "step": 30, "default": 150, "read_only": false,
           "value_descriptions": null},
          {"name": "Visual Preset", "value": 3, "min": 0, "max": 5, "step": 1, "default": 1, "read_only": false,
           "value_descriptions": {"0": "Custom", "3": "High Accuracy"}}]},
        {"name": "RGB Camera", "kind": "color", "options": []},
        {"name": "Motion Module", "kind": "motion", "options": []}
      ],
      "profiles": [{"stream": "depth", "index": 0, "format": "z16", "width": 1280, "height": 720, "fps": 30}]
    }}
    """#

    @Test func deviceInfo() throws {
        let info = try HelperCameraSession.decodeData(DeviceInfo.self, from: json(Self.deviceInfoJSON))
        #expect(info.name == "Intel RealSense D455")
        #expect(info.recommendedFirmware == "5.17.0.10")
        #expect(info.usbType == "3.2" && info.productId == "0B5C" && info.productLine == "D400")
        #expect(info.advancedMode && info.imu && info.source == "live")
        #expect(info.depthScale == 0.0001 && info.baselineMm == 95)
        #expect(info.presets.count == 6)
        #expect(info.sensors.map(\.kind) == [.depth, .color, .motion])
        let laser = try #require(info.sensors.first?.options.first)
        #expect(laser.name == "Laser Power" && laser.max == 360 && laser.step == 30 && laser.default == 150)
        #expect(!laser.readOnly && laser.valueDescriptions == nil)
        #expect(info.sensors[0].options[1].valueDescriptions?["3"] == "High Accuracy")
        #expect(info.profiles == [StreamProfile(stream: "depth", index: 0, format: "z16", width: 1280, height: 720, fps: 30)])
    }

    static let filtersJSON = #"""
    {
      "threshold": {"enabled": true, "min_m": 0.2, "max_m": 1.2},
      "disparity_domain": true,
      "spatial": {"enabled": true, "magnitude": 2, "alpha": 0.5, "delta": 20, "holes_fill": 0},
      "temporal": {"enabled": true, "alpha": 0.4, "delta": 20, "persistence": 3},
      "hole_filling": {"enabled": false, "mode": 1}
    }
    """#

    @Test func configurationExampleDecodes() throws {
        let text = #"""
        {
          "request_id": 3,
          "depth": {"width": 1280, "height": 720, "fps": 30},
          "color": {"width": 1280, "height": 800, "fps": 30},
          "enable_ir": false, "enable_imu": true,
          "preset": "High Accuracy", "laser_power": null, "emitter_enabled": true,
          "depth_units": 0.0001, "disparity_shift": 0,
          "depth_auto_exposure": true, "color_auto_exposure": true,
          "send_depth": false, "send_ir": false, "align_to_color": true,
          "filters": FILTERS
        }
        """#.replacingOccurrences(of: "FILTERS", with: Self.filtersJSON)
        let configuration = try HelperJSON.snakeCaseDecoder.decode(CaptureConfiguration.self, from: json(text))
        #expect(configuration == .faceScanD455)
        let filters = try HelperJSON.snakeCaseDecoder.decode(FilterConfiguration.self, from: json(Self.filtersJSON))
        #expect(filters == .faceScan)
    }

    @Test func configureCommandEncoding() throws {
        let data = try HelperJSON.encodeCommand(requestId: 3, body: CaptureConfiguration.faceScanD455,
                                                explicitNulls: ["laserPower"])
        let o = try object(data)
        #expect((o["request_id"] as? NSNumber)?.intValue == 3)
        #expect(o["laser_power"] is NSNull)  // null = maximum, so it must be on the wire
        #expect(o["enable_imu"] as? Bool == true)
        #expect(o["depth_auto_exposure"] as? Bool == true)
        #expect((o["depth_units"] as? NSNumber)?.doubleValue == 0.0001)
        let threshold = try #require((o["filters"] as? [String: Any])?["threshold"] as? [String: Any])
        #expect((threshold["min_m"] as? NSNumber)?.doubleValue == 0.2)
        let spatial = try #require((o["filters"] as? [String: Any])?["spatial"] as? [String: Any])
        #expect(spatial["holes_fill"] != nil)
        // and the helper's side of it decodes back to the same configuration
        let back = try HelperJSON.snakeCaseDecoder.decode(CaptureConfiguration.self, from: data)
        #expect(back == .faceScanD455)

        var withLaser = CaptureConfiguration.faceScanD455
        withLaser.laserPower = 240
        let o2 = try object(try HelperJSON.encodeCommand(requestId: 4, body: withLaser))
        #expect((o2["laser_power"] as? NSNumber)?.floatValue == 240)
    }

    @Test func simpleCommandEncoding() throws {
        let roi = try object(try HelperJSON.encodeCommand(
            requestId: 9, body: SetROIBody(x: 500, y: 220, width: 300, height: 380, distanceM: 0.6)))
        #expect(Set(roi.keys) == ["request_id", "x", "y", "width", "height", "distance_m"])
        let noDistance = try object(try HelperJSON.encodeCommand(
            requestId: 9, body: SetROIBody(x: 1, y: 2, width: 3, height: 4, distanceM: nil)))
        #expect(noDistance["distance_m"] == nil)
        let option = try object(try HelperJSON.encodeCommand(
            requestId: 6, body: SetOptionBody(sensor: "depth", option: "Laser Power", value: 360)))
        #expect(option["sensor"] as? String == "depth" && option["option"] as? String == "Laser Power")
        let playback = try object(try HelperJSON.encodeCommand(
            requestId: 11, body: PlaybackBody(action: "resume", realTime: false)))
        #expect(playback["action"] as? String == "resume" && playback["real_time"] as? Bool == false)
        let empty = try object(try HelperJSON.encodeCommand(requestId: 5, body: EmptyBody()))
        #expect(Set(empty.keys) == ["request_id"])
    }

    // MARK: FRAMESET

    /// The PROTOCOL.md frameset, sized down, with `colorPadding` extra bytes per color row.
    static func frameset(width: Int = 8, height: Int = 4, colorPadding: Int = 0, alignedWidth: Int? = nil,
                         payloadShortBy: Int = 0) -> (Data, Data) {
        let stride = width * 3 + colorPadding
        var payload = Data()
        for y in 0..<height {
            for x in 0..<width { payload.append(contentsOf: [UInt8(x), UInt8(y), 7]) }
            payload.append(Data(repeating: 0xEE, count: colorPadding))
        }
        let colorBytes = payload.count
        let aw = alignedWidth ?? width
        for i in 0..<(aw * height) {
            let v = UInt16(1000 + i)
            payload.append(contentsOf: [UInt8(v & 0xFF), UInt8(v >> 8)])
        }
        let depthBytes = payload.count - colorBytes
        payload.removeLast(payloadShortBy)
        let intrinsics = #"{"width": \#(width), "height": \#(height), "fx": 640.1, "fy": 639.8, "ppx": 641.2, "ppy": 401.3, "model": "inverse_brown_conrady", "coeffs": [0, 0, 0, 0, 0]}"#
        let text = #"""
        {
          "sequence": 1234,
          "timestamp_ms": 1730000000123.4,
          "timestamp_domain": "global_time",
          "depth_scale": 0.0001,
          "depth_to_color": {"rotation": [1,0,0, 0,1,0, 0,0,1], "translation": [-0.059, 0.0001, 0.0004]},
          "imu": {"accel": [0.1, -9.8, 0.2], "gyro": [0.001, 0.0, -0.002], "timestamp_ms": 1730000000120.0},
          "metadata": {"color_exposure_us": 8000, "depth_exposure_us": 8500, "laser_power": 360, "note": "x"},
          "planes": [
            {"kind": "color", "format": "rgb8", "width": \#(width), "height": \#(height), "stride": \#(stride),
             "offset": 0, "bytes": \#(colorBytes), "frame_number": 456, "timestamp_ms": 1730000000123.4,
             "intrinsics": \#(intrinsics)},
            {"kind": "depth_aligned", "format": "z16", "width": \#(aw), "height": \#(height), "stride": \#(aw * 2),
             "offset": \#(colorBytes), "bytes": \#(depthBytes), "frame_number": 789, "timestamp_ms": 1730000000121.0,
             "intrinsics": {"...": "same as color"}}
          ]
        }
        """#
        return (Data(text.utf8), payload)
    }

    @Test func framesetDecodes() throws {
        let (json, payload) = Self.frameset(colorPadding: 5)
        let frame = try HelperFrameDecoder.decode(json: json, payload: payload)
        #expect(frame.sequence == 1234)
        #expect(frame.timestampMs == 1730000000123.4)
        #expect(frame.color.width == 8 && frame.color.height == 4)
        #expect(frame.color.bytesPerRow == 24)  // repacked without the padding
        #expect(frame.color.pixels.count == 8 * 4 * 3)
        #expect(!frame.color.pixels.contains(0xEE))
        let pixel = Array(frame.color.pixels[(2 * 8 + 5) * 3 ..< (2 * 8 + 5) * 3 + 3])
        #expect(pixel == [5, 2, 7])
        #expect(frame.color.frameNumber == 456)
        #expect(frame.color.intrinsics.fx == 640.1 && frame.color.intrinsics.model == "inverse_brown_conrady")
        #expect(frame.alignedDepth.width == 8 && frame.alignedDepth.pixels.count == 64)
        #expect(frame.alignedDepth.intrinsics == frame.color.intrinsics)  // placeholder intrinsics fall back to color
        #expect(frame.alignedDepth.frameNumber == 789)
        #expect(abs(frame.alignedDepth.meters(x: 1, y: 0) - 0.1001) < 1e-6)
        #expect(frame.depth == nil)
        #expect(frame.depthToColor.translation == [-0.059, 0.0001, 0.0004])
        #expect(frame.imu?.accel == [0.1, -9.8, 0.2])
        #expect(frame.imu?.timestampMs == 1730000000120.0)
        #expect(frame.metadata == ["color_exposure_us": 8000, "depth_exposure_us": 8500, "laser_power": 360])
    }

    @Test func framesetValidation() {
        let (_, payload) = Self.frameset()
        let (mismatch, mismatchPayload) = Self.frameset(alignedWidth: 6)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: mismatch, payload: mismatchPayload) }
        let (short, shortPayload) = Self.frameset(payloadShortBy: 1)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: short, payload: shortPayload) }
        let (good, _) = Self.frameset()
        let wrongFormat = Data(String(decoding: good, as: UTF8.self)
            .replacingOccurrences(of: "\"rgb8\"", with: "\"bgr8\"").utf8)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: wrongFormat, payload: payload) }
        let noColor = Data(String(decoding: good, as: UTF8.self)
            .replacingOccurrences(of: "\"kind\": \"color\"", with: "\"kind\": \"ir_left\"").utf8)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: noColor, payload: payload) }
        let badStride = Data(String(decoding: good, as: UTF8.self)
            .replacingOccurrences(of: "\"stride\": 24", with: "\"stride\": 23").utf8)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: badStride, payload: payload) }
        let hugeOffset = Data(String(decoding: good, as: UTF8.self)
            .replacingOccurrences(of: "\"offset\": 0,", with: "\"offset\": 18446744073709551615,").utf8)
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: hugeOffset, payload: payload) }
        #expect(throws: CameraError.self) { _ = try HelperFrameDecoder.decode(json: Data("[]".utf8), payload: payload) }
    }

    @Test func jsonObjectCheck() throws {
        #expect(throws: CameraError.self) { _ = try HelperJSON.object(Data("[1]".utf8)) }
        #expect(throws: CameraError.self) { _ = try HelperJSON.object(Data("{".utf8)) }
        #expect(try HelperJSON.object(Data(#"{"a": [1, true, null]}"#.utf8))["a"]
            == .array([.number(1), .bool(true), .null]))
    }
}

@Suite("Helper launcher")
struct HelperLauncherTests {
    @Test func shellQuoting() {
        #expect(HelperLauncher.shellQuote("plain") == "'plain'")
        #expect(HelperLauncher.shellQuote("it's") == #"'it'\''s'"#)
        #expect(HelperLauncher.shellQuote("a b;$(rm -rf /)`x`") == "'a b;$(rm -rf /)`x`'")
    }

    @Test func appleScriptQuoting() {
        #expect(HelperLauncher.appleScriptQuote(#"say "hi" \ bye"#) == #""say \"hi\" \\ bye""#)
    }

    @Test func privilegedCommand() {
        let command = HelperLauncher.privilegedShellCommand(
            helper: "/Applications/Intel Face's Scan.app/Contents/Helpers/IntelFaceScanHelper",
            arguments: ["--socket", "/private/var/folders/x/T/IntelFaceScan.abc/camera.sock", "--uid", "501"])
        #expect(command == #"exec /usr/bin/env -i '/Applications/Intel Face'\''s Scan.app/Contents/Helpers/IntelFaceScanHelper' '--socket' '/private/var/folders/x/T/IntelFaceScan.abc/camera.sock' '--uid' '501' </dev/null >/dev/null"#)
        let source = HelperLauncher.appleScriptSource(command: #"echo "x" \"#, prompt: "Why")
        #expect(source == #"do shell script "echo \"x\" \\" with administrator privileges with prompt "Why""#)
    }

    @Test func locatorPrefersTheEnvironmentOutsideABundle() throws {
        // The test runner is no app bundle, so the environment variable decides.
        let helper = try #require(TestHelper.url ?? URL(fileURLWithPath: "/bin/ls") as URL?)
        #expect(CameraSources.locateHelper(environment: ["IFS_HELPER_PATH": helper.path]) == helper)
    }

    @Test func missingHelperIsReported() async {
        // A custom path is never searched for, and nothing is launched (no password prompt in tests).
        let source = LiveCameraSource(helperURL: URL(fileURLWithPath: "/nonexistent/IntelFaceScanHelper"))
        await #expect(throws: CameraError.helperNotFound("/nonexistent/IntelFaceScanHelper")) {
            _ = try await source.connect()
        }
        await source.shutdown()
    }

    @Test func commandsNeedAConnection() async {
        let source = FakeCameraSource(helperURL: URL(fileURLWithPath: "/nonexistent/IntelFaceScanHelper"))
        await #expect(throws: CameraError.notConnected) { try await source.start() }
        await #expect(throws: CameraError.notConnected) { _ = try await source.stopRecording() }
        let playback = PlaybackCameraSource(file: URL(fileURLWithPath: "/nonexistent/x.db3"))
        await #expect(throws: CameraError.unsupported("options in playback")) {
            try await playback.setOption(sensor: .depth, name: "Laser Power", value: 1)
        }
    }
}
