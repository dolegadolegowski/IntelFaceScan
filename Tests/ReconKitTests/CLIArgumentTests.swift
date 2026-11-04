// License: Apache 2.0. See LICENSE file in root directory.
//
// ifscan argument parsing.

import ReconKit
import ScanKit
import SyntheticData
import Testing

private func parse(_ line: String) throws -> IFScanCommand {
    try IFScanArguments.parse(line.split(separator: " ").map(String.init))
}

private func parseError(_ line: String) -> String? {
    do {
        _ = try parse(line)
        return nil
    } catch let error as ArgumentError {
        return error.message
    } catch {
        return "\(error)"
    }
}

@Test func cliParsesSynthWithDefaultsAndOptions() throws {
    #expect(try parse("synth -o out.ifsdataset") == .synth(SynthCommand(output: "out.ifsdataset")))
    let command = try parse("synth --output=a.ifsdataset --width 640 --height=400 --frames-per-pose 3 --noise 1.5 "
        + "--seed 42 --distance 0.7 --poses front,LEFT,upRight --no-background --lighting 0 --mesh-resolution 64 --gt-mesh gt.obj")
    var expected = SynthCommand(output: "a.ifsdataset")
    expected.width = 640
    expected.height = 400
    expected.framesPerPose = 3
    expected.noiseMM = 1.5
    expected.seed = 42
    expected.distanceM = 0.7
    expected.poses = [.front, .left, .upRight]
    expected.background = false
    expected.lighting = 0
    expected.meshResolution = 64
    expected.groundTruthMesh = "gt.obj"
    #expect(command == .synth(expected))
    if case .synth(let all) = try parse("synth -o x --poses all") { #expect(all.poses == ScanPose.allCases) }
}

@Test func cliParsesReconstruct() throws {
    #expect(try parse("reconstruct scan.ifsdataset -o head.obj") == .reconstruct(ReconstructCommand(input: "scan.ifsdataset", output: "head.obj")))
    var expected = ReconstructCommand(input: "rec.db3", output: "out/head.OBJ")
    expected.ply = "head.ply"
    expected.voxelMM = 0.5
    expected.method = .poisson
    expected.keepShoulders = true
    expected.openBack = true
    expected.targetTriangles = 250_000
    expected.textureSize = 2048
    expected.saveDataset = "keys.ifsdataset"
    expected.helper = "/opt/IntelFaceScanHelper"
    expected.report = "report.json"
    expected.quiet = true
    #expect(try parse("reconstruct -o out/head.OBJ --ply head.ply --voxel-mm 0.5 --method Poisson --keep-shoulders "
        + "--open-back --triangles 250000 --texture-size 2048 --save-dataset keys.ifsdataset --helper /opt/IntelFaceScanHelper --report report.json -q rec.db3") == .reconstruct(expected))
    var limited = ReconstructCommand(input: "synthetic-camera", output: "h.obj")
    limited.timeLimitSeconds = 120
    #expect(try parse("reconstruct synthetic-camera -o h.obj --time-limit 120") == .reconstruct(limited))
    #expect(ReconstructCommand(input: "a", output: "b.obj").timeLimitSeconds == 600)
    // --icp: point-to-plane (geometric) unless colored is asked for
    #expect(ReconstructCommand(input: "a", output: "b.obj").coloredICP == false)
    var colored = ReconstructCommand(input: "a.ifsdataset", output: "h.obj")
    colored.coloredICP = true
    #expect(try parse("reconstruct a.ifsdataset -o h.obj --icp Colored") == .reconstruct(colored))
    #expect(try parse("reconstruct a.ifsdataset -o h.obj --icp=geometric")
        == .reconstruct(ReconstructCommand(input: "a.ifsdataset", output: "h.obj")))
    // "--" ends the options: inputs may start with a dash
    #expect(try parse("reconstruct -o h.obj -- -odd.bag") == .reconstruct(ReconstructCommand(input: "-odd.bag", output: "h.obj")))
}

@Test func cliParsesEvalAndInfo() throws {
    #expect(try parse("eval head.obj --gt gt.obj") == .eval(EvalCommand(mesh: "head.obj", groundTruth: "gt.obj")))
    var expected = EvalCommand(mesh: "head.obj", groundTruth: "gt.obj")
    expected.samples = 1000
    expected.report = "r.json"
    expected.symmetric = true
    expected.json = true
    #expect(try parse("eval --gt gt.obj head.obj --samples 1000 --report r.json --symmetric --json") == .eval(expected))
    #expect(try parse("info scan.ifsdataset") == .info(InfoCommand(path: "scan.ifsdataset")))
    var info = InfoCommand(path: "rec.bag")
    info.json = true
    info.helper = "h"
    #expect(try parse("info --json rec.bag --helper h") == .info(info))
}

@Test func cliHelpAndVersion() throws {
    #expect(try IFScanArguments.parse([]) == .help(nil))
    #expect(try parse("help") == .help(nil))
    #expect(try parse("--help") == .help(nil))
    #expect(try parse("help reconstruct") == .help("reconstruct"))
    #expect(try parse("synth --help") == .help("synth"))
    #expect(try parse("eval -o x -h") == .help("eval"))
    #expect(try parse("--version") == .version)
    for command in ["synth", "reconstruct", "eval", "info"] {
        #expect(IFScanArguments.usage(command)?.contains("USAGE: ifscan \(command)") == true)
        #expect(IFScanArguments.overview.contains(command))
    }
}

@Test(arguments: [
    ("frobnicate", "unknown command"),
    ("help frobnicate", "unknown command"),
    ("synth", "missing -o"),
    ("synth -o a extra", "unexpected argument"),
    ("synth -o a --width", "needs a value"),
    ("synth -o a --width --height 3", "needs a value"),
    ("synth -o a --width abc", "not an integer"),
    ("synth -o a --width 5", "not an integer in 16...4096"),
    ("synth -o a --noise -1", "not a number in 0.0...100.0"),
    ("synth -o a --seed -3", "non-negative integer"),
    ("synth -o a --poses front,sideways", "unknown pose"),
    ("synth -o a --poses front,front", "listed twice"),
    ("synth -o a --colour red", "unknown option '--colour'"),
    ("synth -o a -x", "unknown option '-x'"),
    ("synth -o a -o b", "more than once"),
    ("synth -o a --no-background=yes", "takes no value"),
    ("reconstruct -o head.obj", "missing input"),
    ("reconstruct a.ifsdataset b.ifsdataset -o head.obj", "expected one input"),
    ("reconstruct a.ifsdataset", "missing -o"),
    ("reconstruct a.ifsdataset -o head.stl", ".obj"),
    ("reconstruct a.ifsdataset -o head.obj --ply head.obj", ".ply"),
    ("reconstruct a.ifsdataset -o head.obj --method marching", "tsdf or poisson"),
    ("reconstruct a.ifsdataset -o head.obj --icp photometric", "not colored or geometric"),
    ("reconstruct a.ifsdataset -o head.obj --voxel-mm 0", "not a number"),
    ("reconstruct a.ifsdataset -o head.obj --triangles 10", "not an integer in 1000...50000000"),
    ("reconstruct a.ifsdataset -o head.obj --texture-size 100000", "not an integer in 256...16384"),
    ("reconstruct synthetic-camera -o head.obj --time-limit 1", "not a number in 5.0...86400.0"),
    ("eval head.obj", "missing --gt"),
    ("eval --gt gt.obj", "missing <mesh.obj>"),
    ("eval head.obj --gt gt.obj --samples 0", "not an integer"),
    ("info", "missing <file>"),
    ("info a b", "expected one file"),
])
func cliRejectsInvalidArguments(line: String, message: String) {
    let error = parseError(line)
    #expect(error?.contains(message) == true, "'\(line)' -> \(error ?? "no error")")
}
