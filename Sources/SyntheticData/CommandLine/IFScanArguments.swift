// License: Apache 2.0. See LICENSE file in root directory.
//
// Argument parsing of the ifscan command line tool (hand-written: Package.swift has no dependencies to add an
// argument parser). It lives in SyntheticData rather than in the ifscan executable so that the test targets, which
// cannot depend on an executable target, can test it.
//
//   ifscan synth -o out.ifsdataset [--width 1280] [--height 800] [--frames-per-pose 2] [--noise 2] [--seed 1]
//                [--distance 0.6] [--poses front,up,down,left,right] [--no-background] [--lighting 0.25]
//                [--mesh-resolution 128] [--gt-mesh gt.obj]
//   ifscan reconstruct <input.ifsdataset | recording.db3 | recording.bag> -o head.obj [--ply head.ply]
//                [--voxel-mm 0.75] [--method tsdf|poisson] [--keep-shoulders] [--icp colored|geometric]
//                [--save-dataset out.ifsdataset]
//                [--triangles N] [--texture-size 4096] [--helper <IntelFaceScanHelper>] [--report report.json] [--quiet]
//   ifscan eval <mesh.obj> --gt <gt.obj> [--samples 200000] [--report report.json] [--symmetric] [--json]
//   ifscan info <file> [--json] [--helper <IntelFaceScanHelper>]

import Foundation
import ReconKit
import ScanKit

public struct ArgumentError: Error, LocalizedError, Equatable {
    public var message: String
    public init(_ message: String) { self.message = message }
    public var errorDescription: String? { message }
}

public enum IFScanCommand: Equatable, Sendable {
    case synth(SynthCommand)
    case reconstruct(ReconstructCommand)
    case eval(EvalCommand)
    case info(InfoCommand)
    /// Usage text: nil = overview, otherwise the command's.
    case help(String?)
    case version
}

public struct SynthCommand: Equatable, Sendable {
    public var output: String
    public var width = 1280
    public var height = 800
    public var framesPerPose = 2
    /// Depth noise sigma at 1 m, millimeters.
    public var noiseMM: Float = 2
    public var seed: UInt64 = 1
    public var distanceM: Float = 0.6
    public var poses: [ScanPose] = ScanPose.required
    public var background = true
    /// Shading by the camera-mounted light, 0 (flat albedo) ... 1.
    public var lighting: Float = 0.25
    public var groundTruthMesh: String?
    public var meshResolution = 128

    public init(output: String) { self.output = output }
}

public struct ReconstructCommand: Equatable, Sendable {
    public var input: String
    public var output: String
    public var ply: String?
    public var voxelMM: Float = 0.75
    public var method: SurfaceMethod = .tsdf
    public var keepShoulders = false
    /// Colored ICP (geometry + photometric term) or point-to-plane only.
    public var coloredICP = false
    /// Decimate to this many triangles before texturing (nil = full resolution).
    public var targetTriangles: Int?
    public var textureSize = 4096
    public var saveDataset: String?
    public var helper: String?
    public var report: String?
    public var quiet = false

    public init(input: String, output: String) { self.input = input; self.output = output }
}

public struct EvalCommand: Equatable, Sendable {
    public var mesh: String
    public var groundTruth: String
    public var samples = 200_000
    /// ifscan reconstruct report: maps the exported mesh back to the world frame of the ground truth.
    public var report: String?
    public var symmetric = false
    public var json = false

    public init(mesh: String, groundTruth: String) { self.mesh = mesh; self.groundTruth = groundTruth }
}

public struct InfoCommand: Equatable, Sendable {
    public var path: String
    public var json = false
    public var helper: String?
    public init(path: String) { self.path = path }
}

public enum IFScanArguments {
    public static let version = "0.1.0"

    public static let overview = """
        ifscan \(version) - IntelFaceScan command line tool

        USAGE: ifscan <command> [options]

        COMMANDS:
          synth         Render a synthetic head scan with ground truth into a dataset folder
          reconstruct   Reconstruct a head mesh from a dataset or a .db3/.bag recording
          eval          Surface distance between a mesh and a ground-truth mesh (millimeters)
          info          Describe a dataset, recording, mesh or image file

        Run 'ifscan help <command>' or 'ifscan <command> --help' for the options of a command.
        """

    public static func usage(_ command: String) -> String? {
        switch command {
        case "synth": return """
            USAGE: ifscan synth -o <out.ifsdataset> [options]

            Renders the procedural head from the guided-scan poses with a D455-like noise model and writes a
            dataset with ground truth (ground_truth.json, ground_truth.obj in millimeters, world frame).

            OPTIONS:
              -o, --output <dir>         Dataset folder to create (replaces an existing dataset)
              --width <px>               Color/depth width (default 1280)
              --height <px>              Color/depth height (default 800)
              --frames-per-pose <n>      Keyframes per scan pose (default 2)
              --noise <mm>               Depth noise sigma at 1 m in millimeters, scaled with z^2 (default 2)
              --seed <n>                 Random seed (default 1)
              --distance <m>             Face distance from the camera (default 0.6)
              --poses <list>             Comma-separated scan poses (default front,up,down,left,right;
                                         also upLeft,upRight,downLeft,downRight, or 'all')
              --no-background            No wall behind the head
              --lighting <0-1>           Shading by a light at the camera (default 0.25; 0 = flat albedo)
              --mesh-resolution <n>      Head tessellation, 12*n^2 triangles (default 128)
              --gt-mesh <file.obj>       Also write the ground-truth mesh here (millimeters, world frame)
            """
        case "reconstruct": return """
            USAGE: ifscan reconstruct <input.ifsdataset | recording.db3 | recording.bag | synthetic-camera> -o <head.obj>
                   [options]

            Datasets go straight to the reconstruction. Recordings are played by the camera helper (as the user,
            no password) through the guided-scan logic, exactly as in the app. The input 'synthetic-camera' scans
            the animated synthetic head in real time (Vision + guided scan, the app's live flow without the UI).

            OPTIONS:
              -o, --output <file.obj>    OBJ to write (+ .mtl and .png when textured), millimeters
              --ply <file.ply>           Also write a binary PLY
              --voxel-mm <mm>            TSDF voxel size (default 0.75)
              --method <tsdf|poisson>    Surface reconstruction (default tsdf)
              --keep-shoulders           Keep neck and shoulders instead of cutting below the chin
              --icp <colored|geometric>  Registration: colored ICP or point-to-plane only (default
                                         geometric)
              --triangles <n>            Decimate to n triangles before texturing (default: full resolution)
              --texture-size <px>        Texture atlas size (default 4096)
              --save-dataset <dir>       Save the accepted keyframes as a dataset
              --helper <path>            IntelFaceScanHelper for recordings (default: $IFS_HELPER, next to
                                         ifscan, build/helper/, the app bundle)
              --report <file.json>       Write a JSON report (stages, timings, export transform, ground-truth
                                         errors for synthetic datasets)
              -q, --quiet                No progress output
            """
        case "eval": return """
            USAGE: ifscan eval <mesh.obj> --gt <gt.obj> [options]

            One-sided distances from points sampled uniformly on the mesh surface to the ground-truth surface.
            Both files are read as millimeters. A mesh exported by 'ifscan reconstruct' is in the export frame;
            pass its --report to map it back to the world frame of a synthetic ground truth.

            OPTIONS:
              --gt <file.obj>            Ground-truth mesh
              --samples <n>              Sample points (default 200000)
              --report <file.json>       Report of 'ifscan reconstruct' for the mesh (export transform)
              --symmetric                Also report ground truth -> mesh distances
              --json                     Print JSON
            """
        case "info": return """
            USAGE: ifscan info <file> [--json]

            Describes an .ifsdataset folder, a .db3/.bag recording (device and streams, needs the helper),
            an .obj/.ply mesh or a .png image.

            OPTIONS:
              --json                     Print JSON (datasets: the manifest without frames)
              --helper <path>            IntelFaceScanHelper for recordings
            """
        default: return nil
        }
    }

    // MARK: Parsing

    private struct Spec {
        var name: String
        var short: Character?
        var takesValue: Bool
    }

    private struct Parsed {
        var positionals: [String] = []
        var values: [String: String] = [:]
        var flags: Set<String> = []
    }

    public static func parse(_ arguments: [String]) throws -> IFScanCommand {
        guard let command = arguments.first else { return .help(nil) }
        let rest = Array(arguments.dropFirst())
        switch command {
        case "-h", "--help", "help":
            if let topic = rest.first {
                guard usage(topic) != nil else { throw ArgumentError("unknown command '\(topic)'") }
                return .help(topic)
            }
            return .help(nil)
        case "--version", "version":
            return .version
        case "synth":
            return try parseSynth(rest)
        case "reconstruct":
            return try parseReconstruct(rest)
        case "eval":
            return try parseEval(rest)
        case "info":
            return try parseInfo(rest)
        default:
            throw ArgumentError("unknown command '\(command)' (expected synth, reconstruct, eval or info)")
        }
    }

    private static func parseSynth(_ arguments: [String]) throws -> IFScanCommand {
        let specs = [Spec(name: "output", short: "o", takesValue: true), Spec(name: "width", short: nil, takesValue: true),
                     Spec(name: "height", short: nil, takesValue: true), Spec(name: "frames-per-pose", short: nil, takesValue: true),
                     Spec(name: "noise", short: nil, takesValue: true), Spec(name: "seed", short: nil, takesValue: true),
                     Spec(name: "distance", short: nil, takesValue: true), Spec(name: "poses", short: nil, takesValue: true),
                     Spec(name: "no-background", short: nil, takesValue: false), Spec(name: "lighting", short: nil, takesValue: true),
                     Spec(name: "mesh-resolution", short: nil, takesValue: true), Spec(name: "gt-mesh", short: nil, takesValue: true)]
        guard let p = try parse(arguments, specs, command: "synth") else { return .help("synth") }
        guard p.positionals.isEmpty else { throw ArgumentError("synth: unexpected argument '\(p.positionals[0])'") }
        guard let output = p.values["output"] else { throw ArgumentError("synth: missing -o <out.ifsdataset>") }
        var c = SynthCommand(output: output)
        c.width = try integer(p, "width", c.width, in: 16 ... 4096)
        c.height = try integer(p, "height", c.height, in: 16 ... 4096)
        c.framesPerPose = try integer(p, "frames-per-pose", c.framesPerPose, in: 1 ... 100)
        c.noiseMM = try number(p, "noise", c.noiseMM, in: 0 ... 100)
        if let seed = p.values["seed"] {
            guard let value = UInt64(seed) else { throw ArgumentError("--seed: '\(seed)' is not a non-negative integer") }
            c.seed = value
        }
        c.distanceM = try number(p, "distance", c.distanceM, in: 0.25 ... 3)
        c.meshResolution = try integer(p, "mesh-resolution", c.meshResolution, in: 16 ... 512)
        if let list = p.values["poses"] { c.poses = try poses(list) }
        c.background = !p.flags.contains("no-background")
        c.lighting = try number(p, "lighting", c.lighting, in: 0 ... 1)
        c.groundTruthMesh = p.values["gt-mesh"]
        return .synth(c)
    }

    private static func parseReconstruct(_ arguments: [String]) throws -> IFScanCommand {
        let specs = [Spec(name: "output", short: "o", takesValue: true), Spec(name: "ply", short: nil, takesValue: true),
                     Spec(name: "voxel-mm", short: nil, takesValue: true), Spec(name: "method", short: nil, takesValue: true),
                     Spec(name: "keep-shoulders", short: nil, takesValue: false), Spec(name: "icp", short: nil, takesValue: true),
                     Spec(name: "triangles", short: nil, takesValue: true), Spec(name: "texture-size", short: nil, takesValue: true),
                     Spec(name: "save-dataset", short: nil, takesValue: true), Spec(name: "helper", short: nil, takesValue: true),
                     Spec(name: "report", short: nil, takesValue: true), Spec(name: "quiet", short: "q", takesValue: false)]
        guard let p = try parse(arguments, specs, command: "reconstruct") else { return .help("reconstruct") }
        guard p.positionals.count == 1 else {
            throw ArgumentError(p.positionals.isEmpty ? "reconstruct: missing input (dataset folder or .db3/.bag recording)"
                                                      : "reconstruct: expected one input, got \(p.positionals.count)")
        }
        guard let output = p.values["output"] else { throw ArgumentError("reconstruct: missing -o <head.obj>") }
        guard output.lowercased().hasSuffix(".obj") else { throw ArgumentError("reconstruct: -o must name an .obj file") }
        var c = ReconstructCommand(input: p.positionals[0], output: output)
        c.ply = p.values["ply"]
        if let ply = c.ply, !ply.lowercased().hasSuffix(".ply") { throw ArgumentError("--ply must name a .ply file") }
        c.voxelMM = try number(p, "voxel-mm", c.voxelMM, in: 0.1 ... 10)
        if let method = p.values["method"] {
            guard let value = SurfaceMethod(rawValue: method.lowercased()) else {
                throw ArgumentError("--method: '\(method)' is not tsdf or poisson")
            }
            c.method = value
        }
        c.keepShoulders = p.flags.contains("keep-shoulders")
        if let icp = p.values["icp"] {
            switch icp.lowercased() {
            case "colored": c.coloredICP = true
            case "geometric": c.coloredICP = false
            default: throw ArgumentError("--icp: '\(icp)' is not colored or geometric")
            }
        }
        if p.values["triangles"] != nil { c.targetTriangles = try integer(p, "triangles", 0, in: 1_000 ... 50_000_000) }
        c.textureSize = try integer(p, "texture-size", c.textureSize, in: 256 ... 16384)
        c.saveDataset = p.values["save-dataset"]
        c.helper = p.values["helper"]
        c.report = p.values["report"]
        c.quiet = p.flags.contains("quiet")
        return .reconstruct(c)
    }

    private static func parseEval(_ arguments: [String]) throws -> IFScanCommand {
        let specs = [Spec(name: "gt", short: nil, takesValue: true), Spec(name: "samples", short: nil, takesValue: true),
                     Spec(name: "report", short: nil, takesValue: true), Spec(name: "symmetric", short: nil, takesValue: false),
                     Spec(name: "json", short: nil, takesValue: false)]
        guard let p = try parse(arguments, specs, command: "eval") else { return .help("eval") }
        guard p.positionals.count == 1 else {
            throw ArgumentError(p.positionals.isEmpty ? "eval: missing <mesh.obj>" : "eval: expected one mesh, got \(p.positionals.count)")
        }
        guard let gt = p.values["gt"] else { throw ArgumentError("eval: missing --gt <gt.obj>") }
        var c = EvalCommand(mesh: p.positionals[0], groundTruth: gt)
        c.samples = try integer(p, "samples", c.samples, in: 1 ... 100_000_000)
        c.report = p.values["report"]
        c.symmetric = p.flags.contains("symmetric")
        c.json = p.flags.contains("json")
        return .eval(c)
    }

    private static func parseInfo(_ arguments: [String]) throws -> IFScanCommand {
        let specs = [Spec(name: "json", short: nil, takesValue: false), Spec(name: "helper", short: nil, takesValue: true)]
        guard let p = try parse(arguments, specs, command: "info") else { return .help("info") }
        guard p.positionals.count == 1 else {
            throw ArgumentError(p.positionals.isEmpty ? "info: missing <file>" : "info: expected one file, got \(p.positionals.count)")
        }
        var c = InfoCommand(path: p.positionals[0])
        c.json = p.flags.contains("json")
        c.helper = p.values["helper"]
        return .info(c)
    }

    /// Returns nil when --help/-h was given.
    private static func parse(_ arguments: [String], _ specs: [Spec], command: String) throws -> Parsed? {
        // Help wins over everything else on the line (before "--"), even over invalid options.
        if arguments.prefix(while: { $0 != "--" }).contains(where: { $0 == "--help" || $0 == "-h" }) { return nil }
        var p = Parsed()
        var i = 0
        var onlyPositionals = false
        while i < arguments.count {
            let arg = arguments[i]
            i += 1
            if onlyPositionals || arg == "-" || !arg.hasPrefix("-") || isNumber(arg) {
                p.positionals.append(arg)
                continue
            }
            if arg == "--" { onlyPositionals = true; continue }
            var name: String
            var inlineValue: String?
            if arg.hasPrefix("--") {
                name = String(arg.dropFirst(2))
                if let eq = name.firstIndex(of: "=") {
                    inlineValue = String(name[name.index(after: eq)...])
                    name = String(name[..<eq])
                }
            } else {
                guard arg.count == 2, let short = arg.last, let spec = specs.first(where: { $0.short == short }) else {
                    throw ArgumentError("\(command): unknown option '\(arg)'")
                }
                name = spec.name
            }
            guard let spec = specs.first(where: { $0.name == name }) else {
                throw ArgumentError("\(command): unknown option '\(arg)'")
            }
            if p.values[name] != nil || p.flags.contains(name) {
                throw ArgumentError("\(command): --\(name) given more than once")
            }
            if spec.takesValue {
                if let inlineValue {
                    p.values[name] = inlineValue
                } else {
                    guard i < arguments.count, !arguments[i].hasPrefix("--") else {
                        throw ArgumentError("\(command): --\(name) needs a value")
                    }
                    p.values[name] = arguments[i]
                    i += 1
                }
                if p.values[name]?.isEmpty == true { throw ArgumentError("\(command): --\(name) needs a value") }
            } else {
                guard inlineValue == nil else { throw ArgumentError("\(command): --\(name) takes no value") }
                p.flags.insert(name)
            }
        }
        return p
    }

    private static func isNumber(_ s: String) -> Bool { Double(s) != nil }

    private static func integer(_ p: Parsed, _ name: String, _ fallback: Int, in range: ClosedRange<Int>) throws -> Int {
        guard let text = p.values[name] else { return fallback }
        guard let value = Int(text), range.contains(value) else {
            throw ArgumentError("--\(name): '\(text)' is not an integer in \(range.lowerBound)...\(range.upperBound)")
        }
        return value
    }

    private static func number(_ p: Parsed, _ name: String, _ fallback: Float, in range: ClosedRange<Float>) throws -> Float {
        guard let text = p.values[name] else { return fallback }
        guard let value = Float(text), value.isFinite, range.contains(value) else {
            throw ArgumentError("--\(name): '\(text)' is not a number in \(range.lowerBound)...\(range.upperBound)")
        }
        return value
    }

    static func poses(_ list: String) throws -> [ScanPose] {
        if list == "all" { return ScanPose.allCases }
        var result: [ScanPose] = []
        for name in list.split(separator: ",").map({ $0.trimmingCharacters(in: .whitespaces) }) where !name.isEmpty {
            guard let pose = ScanPose.allCases.first(where: { $0.rawValue.lowercased() == name.lowercased() }) else {
                throw ArgumentError("--poses: unknown pose '\(name)' (front, up, down, left, right, upLeft, upRight, downLeft, downRight)")
            }
            guard !result.contains(pose) else { throw ArgumentError("--poses: '\(name)' listed twice") }
            result.append(pose)
        }
        guard !result.isEmpty else { throw ArgumentError("--poses: empty list") }
        return result
    }
}
