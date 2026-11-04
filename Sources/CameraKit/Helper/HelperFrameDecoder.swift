// License: Apache 2.0. See LICENSE file in root directory.
//
// FRAMESET -> CameraFrame: validates every plane against the payload, copies the pixels out of the receive buffer
// (repacking rows when the stride has padding) and enforces the contract the reconstruction relies on: color and
// aligned depth are present and have the same size.

import Foundation

struct WireFrameset: Decodable {
    var sequence: UInt64
    var timestampMs: Double
    var depthScale: Float
    var depthToColor: Extrinsics?
    var imu: WireIMU?
    var metadata: [String: JSONValue]?
    var planes: [WirePlane]

    enum CodingKeys: String, CodingKey {
        case sequence
        case timestampMs = "timestamp_ms"
        case depthScale = "depth_scale"
        case depthToColor = "depth_to_color"
        case imu, metadata, planes
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        sequence = try c.decode(UInt64.self, forKey: .sequence)
        timestampMs = try c.decode(Double.self, forKey: .timestampMs)
        depthScale = try c.decode(Float.self, forKey: .depthScale)
        // Optional extras must not cost the frame when they are malformed.
        depthToColor = try? c.decodeIfPresent(Extrinsics.self, forKey: .depthToColor)
        imu = try? c.decodeIfPresent(WireIMU.self, forKey: .imu)
        metadata = try? c.decodeIfPresent([String: JSONValue].self, forKey: .metadata)
        planes = try c.decode([WirePlane].self, forKey: .planes)
    }
}

struct WireIMU: Decodable {
    var accel: [Float]?
    var gyro: [Float]?
    var timestampMs: Double?

    enum CodingKeys: String, CodingKey {
        case accel, gyro
        case timestampMs = "timestamp_ms"
    }
}

struct WirePlane: Decodable {
    var kind: String
    var format: String
    var width: Int
    var height: Int
    var stride: Int
    var offset: UInt64
    var bytes: UInt64
    var frameNumber: UInt64?
    var timestampMs: Double?
    var intrinsics: Intrinsics?

    enum CodingKeys: String, CodingKey {
        case kind, format, width, height, stride, offset, bytes
        case frameNumber = "frame_number"
        case timestampMs = "timestamp_ms"
        case intrinsics
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        kind = try c.decode(String.self, forKey: .kind)
        format = try c.decode(String.self, forKey: .format)
        width = try c.decode(Int.self, forKey: .width)
        height = try c.decode(Int.self, forKey: .height)
        stride = try c.decode(Int.self, forKey: .stride)
        offset = try c.decode(UInt64.self, forKey: .offset)
        bytes = try c.decode(UInt64.self, forKey: .bytes)
        frameNumber = try? c.decodeIfPresent(UInt64.self, forKey: .frameNumber)
        timestampMs = try? c.decodeIfPresent(Double.self, forKey: .timestampMs)
        intrinsics = try? c.decodeIfPresent(Intrinsics.self, forKey: .intrinsics)
    }
}

enum HelperFrameDecoder {
    static func decode(json: Data, payload: Data) throws -> CameraFrame {
        let wire: WireFrameset
        do {
            wire = try HelperJSON.decoder.decode(WireFrameset.self, from: json)
        } catch {
            throw CameraError.protocolError("invalid frameset metadata: \(error)")
        }
        guard wire.depthScale.isFinite, wire.depthScale > 0 else {
            throw CameraError.protocolError("frameset without a valid depth_scale")
        }
        guard let colorPlane = wire.planes.first(where: { $0.kind == "color" }) else {
            throw CameraError.protocolError("frameset without a color plane")
        }
        guard let alignedPlane = wire.planes.first(where: { $0.kind == "depth_aligned" }) else {
            throw CameraError.protocolError("frameset without aligned depth")
        }
        guard alignedPlane.width == colorPlane.width, alignedPlane.height == colorPlane.height else {
            throw CameraError.protocolError(
                "aligned depth is \(alignedPlane.width)x\(alignedPlane.height), color is \(colorPlane.width)x\(colorPlane.height)")
        }
        guard let colorIntrinsics = colorPlane.intrinsics else {
            throw CameraError.protocolError("color plane without intrinsics")
        }

        let colorPixels = try pixels(of: colorPlane, format: "rgb8", bytesPerPixel: 3, payload: payload)
        let color = ColorImage(width: colorPlane.width, height: colorPlane.height, bytesPerRow: colorPlane.width * 3,
                               pixels: colorPixels, intrinsics: colorIntrinsics,
                               frameNumber: colorPlane.frameNumber ?? 0,
                               timestampMs: colorPlane.timestampMs ?? wire.timestampMs)

        let alignedPixels = try pixels(of: alignedPlane, format: "z16", bytesPerPixel: 2, payload: payload)
        let aligned = DepthImage(width: alignedPlane.width, height: alignedPlane.height, pixels: alignedPixels,
                                 depthScale: wire.depthScale,
                                 intrinsics: alignedPlane.intrinsics ?? colorIntrinsics,
                                 frameNumber: alignedPlane.frameNumber ?? 0,
                                 timestampMs: alignedPlane.timestampMs ?? wire.timestampMs)

        var depth: DepthImage?
        if let plane = wire.planes.first(where: { $0.kind == "depth" }) {
            let data = try pixels(of: plane, format: "z16", bytesPerPixel: 2, payload: payload)
            let intrinsics = plane.intrinsics ?? Intrinsics(width: plane.width, height: plane.height, fx: 0, fy: 0,
                                                            ppx: 0, ppy: 0)
            depth = DepthImage(width: plane.width, height: plane.height, pixels: data, depthScale: wire.depthScale,
                               intrinsics: intrinsics, frameNumber: plane.frameNumber ?? 0,
                               timestampMs: plane.timestampMs ?? wire.timestampMs)
        }
        // "ir_left" has no place in CameraFrame yet and is skipped.

        var extrinsics = Extrinsics.identity
        if let e = wire.depthToColor, e.rotation.count == 9, e.translation.count == 3,
           (e.rotation + e.translation).allSatisfy({ $0.isFinite }) {
            extrinsics = e
        }
        var imu: IMUSample?
        if let w = wire.imu {
            imu = IMUSample(accel: w.accel?.count == 3 ? w.accel : nil, gyro: w.gyro?.count == 3 ? w.gyro : nil,
                            timestampMs: w.timestampMs ?? wire.timestampMs)
        }
        var metadata: [String: Double] = [:]
        for (key, value) in wire.metadata ?? [:] {
            if let n = value.number { metadata[key] = n }
        }
        return CameraFrame(sequence: wire.sequence, timestampMs: wire.timestampMs, color: color, alignedDepth: aligned,
                           depth: depth, depthToColor: extrinsics, imu: imu, metadata: metadata)
    }

    /// The plane's pixels as tightly packed rows.
    static func pixels(of plane: WirePlane, format: String, bytesPerPixel: Int, payload: Data) throws -> Data {
        guard plane.format == format else {
            throw CameraError.protocolError("\(plane.kind) plane has format \(plane.format), expected \(format)")
        }
        let maxSide = 16384
        guard plane.width > 0, plane.height > 0, plane.width <= maxSide, plane.height <= maxSide else {
            throw CameraError.protocolError("\(plane.kind) plane has an invalid size \(plane.width)x\(plane.height)")
        }
        let rowBytes = plane.width * bytesPerPixel
        guard plane.stride >= rowBytes else {
            throw CameraError.protocolError("\(plane.kind) plane stride \(plane.stride) is shorter than a row")
        }
        // The last row may end right after its pixels.
        let needed = UInt64(plane.stride) * UInt64(plane.height - 1) + UInt64(rowBytes)
        let (end, overflow) = plane.offset.addingReportingOverflow(plane.bytes)
        guard plane.bytes >= needed, !overflow, end <= UInt64(payload.count) else {
            throw CameraError.protocolError("\(plane.kind) plane does not fit the payload")
        }
        let offset = Int(plane.offset)
        var out = Data(count: rowBytes * plane.height)
        out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
            payload.withUnsafeBytes { (src: UnsafeRawBufferPointer) in
                guard let d = dst.baseAddress, let s = src.baseAddress else { return }
                if plane.stride == rowBytes {
                    memcpy(d, s + offset, rowBytes * plane.height)
                } else {
                    for y in 0..<plane.height {
                        memcpy(d + y * rowBytes, s + offset + y * plane.stride, rowBytes)
                    }
                }
            }
        }
        return out
    }
}
