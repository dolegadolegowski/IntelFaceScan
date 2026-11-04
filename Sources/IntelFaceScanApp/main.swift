// License: Apache 2.0. See LICENSE file in root directory.
//
// Entry point. Debug builds can render the main screens with fixture data to PNG files instead of starting the
// app (`IntelFaceScan --render-snapshots <directory>`): SwiftUI previews need Xcode, this works with the
// Command Line Tools. The app model's lifecycle checks are the IntelFaceScanAppTests target (scripts/test.sh).

import Foundation

#if DEBUG
let snapshotArguments = CommandLine.arguments
if let index = snapshotArguments.firstIndex(of: "--render-snapshots") {
    let directory = index + 1 < snapshotArguments.count ? snapshotArguments[index + 1] : "snapshots"
    let status = MainActor.assumeIsolated { DeveloperSnapshots.render(to: URL(fileURLWithPath: directory)) }
    exit(status)
}
#endif

IntelFaceScanApp.main()
