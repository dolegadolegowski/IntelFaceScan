// License: Apache 2.0. See LICENSE file in root directory.
//
// JSON of docs/PROTOCOL.md on the app side. The shared types of CameraTypes.swift are decoded with
// .convertFromSnakeCase; the protocol envelopes below spell their snake_case keys out instead, because the key
// strategy would also rewrite free-form keys (e.g. "color_exposure_us" in frame metadata).

import Foundation

enum HelperJSON {
    /// For the types of CameraTypes.swift (DeviceInfo, ...).
    static let snakeCaseDecoder: JSONDecoder = {
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return decoder
    }()

    /// For the protocol envelopes (explicit CodingKeys).
    static let decoder = JSONDecoder()

    static let snakeCaseEncoder: JSONEncoder = {
        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        return encoder
    }()

    /// Encodes a command body with its request id (`request_id`). `explicitNulls` names properties (camelCase) that
    /// must appear as `null` when they are nil, where the protocol gives null a meaning (CONFIGURE `laser_power`).
    static func encodeCommand<Body: Encodable>(requestId: UInt64, body: Body, explicitNulls: [String] = []) throws -> Data {
        try snakeCaseEncoder.encode(CommandEnvelope(requestId: requestId, body: body, explicitNulls: explicitNulls))
    }

    /// The JSON object of a message; anything else is a protocol error.
    static func object(_ data: Data) throws -> [String: JSONValue] {
        guard case .object(let object)? = try? decoder.decode(JSONValue.self, from: data) else {
            throw CameraError.protocolError("message body is not a JSON object")
        }
        return object
    }
}

/// Any JSON value (event data, frame metadata).
enum JSONValue: Codable, Sendable, Equatable {
    case null
    case bool(Bool)
    case number(Double)
    case string(String)
    case array([JSONValue])
    case object([String: JSONValue])

    init(from decoder: Decoder) throws {
        let c = try decoder.singleValueContainer()
        if c.decodeNil() {
            self = .null
        } else if let b = try? c.decode(Bool.self) {
            self = .bool(b)
        } else if let n = try? c.decode(Double.self) {
            self = .number(n)
        } else if let s = try? c.decode(String.self) {
            self = .string(s)
        } else if let a = try? c.decode([JSONValue].self) {
            self = .array(a)
        } else {
            self = .object(try c.decode([String: JSONValue].self))
        }
    }

    func encode(to encoder: Encoder) throws {
        var c = encoder.singleValueContainer()
        switch self {
        case .null: try c.encodeNil()
        case .bool(let b): try c.encode(b)
        case .number(let n): try c.encode(n)
        case .string(let s): try c.encode(s)
        case .array(let a): try c.encode(a)
        case .object(let o): try c.encode(o)
        }
    }

    var number: Double? {
        switch self {
        case .number(let n): return n
        case .bool(let b): return b ? 1 : 0
        default: return nil
        }
    }

    var string: String? {
        if case .string(let s) = self { return s }
        return nil
    }

    subscript(key: String) -> JSONValue? {
        if case .object(let o) = self { return o[key] }
        return nil
    }
}

struct DynamicCodingKey: CodingKey {
    var stringValue: String
    var intValue: Int? { nil }
    init(_ string: String) { stringValue = string }
    init?(stringValue: String) { self.stringValue = stringValue }
    init?(intValue: Int) { nil }
}

/// The body's own keys plus `request_id` in one JSON object (the encoder merges both keyed containers).
struct CommandEnvelope<Body: Encodable>: Encodable {
    let requestId: UInt64
    let body: Body
    let explicitNulls: [String]

    func encode(to encoder: Encoder) throws {
        try body.encode(to: encoder)
        var container = encoder.container(keyedBy: DynamicCodingKey.self)
        try container.encode(requestId, forKey: DynamicCodingKey("requestId"))
        for key in explicitNulls {
            try container.encodeNil(forKey: DynamicCodingKey(key))
        }
    }
}

// MARK: - Helper -> app

/// HELLO.
public struct HelperHello: Decodable, Sendable, Equatable {
    public var `protocol`: Int
    public var helperVersion: String
    public var sdkVersion: String
    public var pid: Int32
    public var euid: UInt32
    /// "live", "playback" or "fake".
    public var mode: String

    enum CodingKeys: String, CodingKey {
        case `protocol`
        case helperVersion = "helper_version"
        case sdkVersion = "sdk_version"
        case pid, euid, mode
    }
}

/// REPLY without its data (decoded separately into the command's result type).
struct WireReply: Decodable {
    var requestId: UInt64
    var ok: Bool
    var error: String?

    enum CodingKeys: String, CodingKey {
        case requestId = "request_id"
        case ok, error
    }
}

/// `{"data": T}` of a REPLY, decoded with the snake_case decoder.
struct WireReplyData<T: Decodable>: Decodable {
    var data: T
}

struct WireBytes: Decodable {
    var bytes: Int64
}

struct WireLog: Decodable {
    var level: String
    var message: String
}

struct WireEvent: Decodable, Sendable {
    var event: String
    var message: String?
    var data: JSONValue?
}

struct WireBye: Decodable {
    var reason: String?
}

// MARK: - App -> helper

struct EmptyBody: Encodable {}

struct SetOptionBody: Encodable {
    var sensor: String
    var option: String
    var value: Float
}

struct SetROIBody: Encodable {
    var x: Int
    var y: Int
    var width: Int
    var height: Int
    var distanceM: Float?
}

struct RecordStartBody: Encodable {
    var format: String
}

struct PlaybackBody: Encodable {
    var action: String
    var realTime: Bool?
}
