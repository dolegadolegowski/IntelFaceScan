// License: Apache 2.0. See LICENSE file in root directory.

import Foundation

extension CameraSources {
    /// The camera helper for this process: the app bundle's (bundledHelperURL); otherwise, for development builds
    /// without a bundle, $IFS_HELPER_PATH, a Helpers folder next to the executable's folder (ifscan in
    /// Contents/MacOS), or build/helper/IntelFaceScanHelper in a parent folder of the executable (swift run / swift
    /// test from the repository).
    public static func locateHelper(environment: [String: String] = ProcessInfo.processInfo.environment) -> URL? {
        let fm = FileManager.default
        func usable(_ url: URL) -> Bool { fm.isExecutableFile(atPath: url.path) }

        if usable(bundledHelperURL) { return bundledHelperURL }
        if let path = environment["IFS_HELPER_PATH"], !path.isEmpty {
            let url = URL(fileURLWithPath: path)
            if usable(url) { return url }
        }
        guard let executable = Bundle.main.executableURL?.resolvingSymlinksInPath() else { return nil }
        let folder = executable.deletingLastPathComponent()
        let sibling = folder.deletingLastPathComponent().appendingPathComponent("Helpers/IntelFaceScanHelper")
        if usable(sibling) { return sibling }
        var dir = folder
        for _ in 0..<8 {
            let candidate = dir.appendingPathComponent("build/helper/IntelFaceScanHelper")
            if usable(candidate) { return candidate }
            let parent = dir.deletingLastPathComponent()
            if parent.path == dir.path { break }
            dir = parent
        }
        return nil
    }
}
