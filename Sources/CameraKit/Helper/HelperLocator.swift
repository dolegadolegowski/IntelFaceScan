// License: Apache 2.0. See LICENSE file in root directory.

import Darwin
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
        return HelperLocation.searchBuildFolders(from: folder)
    }
}

/// Where a helper may be taken from when it is not the app bundle's own.
enum HelperLocation {
    static let wheelGroup: gid_t = 0
    static let adminGroup: gid_t = 80

    /// build/helper/IntelFaceScanHelper in `folder` or one of its 7 parents. The search can climb out of the
    /// repository into folders other users can write to (/Users/Shared, /private/tmp, a group-writable checkout), and
    /// the helper runs what it finds with the service's trust in its cdhash: a candidate is only taken when nobody but
    /// root and this user can replace it (see `problem(_:)`).
    static func searchBuildFolders(from folder: URL, uid: uid_t = getuid()) -> URL? {
        var dir = folder
        for _ in 0..<8 {
            let candidate = dir.appendingPathComponent("build/helper/IntelFaceScanHelper")
            if FileManager.default.isExecutableFile(atPath: candidate.path) && problem(candidate.path, uid: uid) == nil {
                return candidate
            }
            let parent = dir.deletingLastPathComponent()
            if parent.path == dir.path { break }
            dir = parent
        }
        return nil
    }

    /// Why `path` is not safe to run: nil when the file and every folder above it (after resolving links) belong to
    /// root or `uid` and nobody else can write to them. Group write is accepted for wheel and admin only (whose
    /// members can administer the Mac anyway, e.g. /Applications); a world-writable folder only with the sticky bit,
    /// where other users cannot rename or delete entries they do not own.
    static func problem(_ path: String, uid: uid_t = getuid()) -> String? {
        guard let resolved = realpath(path, nil) else { return "\(path): \(errnoDescription())" }
        var current = String(cString: resolved)
        free(resolved)
        while true {
            var st = stat()
            guard lstat(current, &st) == 0 else { return "\(current): \(errnoDescription())" }
            if st.st_uid != 0 && st.st_uid != uid {
                return "\(current) belongs to another user (uid \(st.st_uid))"
            }
            let sticky = (st.st_mode & S_IFMT) == S_IFDIR && (st.st_mode & S_ISVTX) != 0
            if !sticky && (st.st_mode & S_IWOTH) != 0 {
                return "\(current) is writable by everyone"
            }
            if !sticky && (st.st_mode & S_IWGRP) != 0 && st.st_gid != wheelGroup && st.st_gid != adminGroup {
                return "\(current) is writable by group \(st.st_gid)"
            }
            if current == "/" { return nil }
            let parent = (current as NSString).deletingLastPathComponent
            current = parent.isEmpty ? "/" : parent
        }
    }
}
