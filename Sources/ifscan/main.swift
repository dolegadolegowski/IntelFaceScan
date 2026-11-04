// License: Apache 2.0. See LICENSE file in root directory.
//
// ifscan - IntelFaceScan command line tool: synthetic scans, reconstruction from datasets and recordings,
// evaluation against ground truth, file information. The commands are implemented in SyntheticData
// (IFScanTool) so that the test targets can exercise them in-process.
//
//   ifscan synth -o out.ifsdataset [--width --height --frames-per-pose --noise --seed --distance --poses
//          --no-background --lighting --mesh-resolution --gt-mesh gt.obj]
//   ifscan reconstruct <input.ifsdataset | recording.db3 | recording.bag | synthetic-camera> -o head.obj
//          [--ply head.ply] [--voxel-mm 0.75] [--method tsdf|poisson] [--keep-shoulders] [--icp colored|geometric]
//          [--save-dataset out.ifsdataset] [--triangles N] [--texture-size 4096] [--helper <IntelFaceScanHelper>]
//          [--report report.json] [--time-limit 600] [-q]
//   ifscan eval <mesh.obj> --gt <gt.obj> [--samples N] [--report report.json] [--symmetric] [--json]
//   ifscan info <file> [--json] [--helper <IntelFaceScanHelper>]
// The full list with explanations: IFScanArguments.swift, `ifscan help <command>`, docs/TESTING.md.

import Foundation
import SyntheticData

// Writing to a closed pipe (ifscan info x | head) must end the tool quietly, not with SIGPIPE.
signal(SIGPIPE, SIG_IGN)
let status = await IFScanTool.run(Array(CommandLine.arguments.dropFirst()))
exit(status)
