// swift-tools-version: 6.0
// License: Apache 2.0. See LICENSE file in root directory.
//
// The unprivileged part of IntelFaceScan: the SwiftUI app, the reconstruction library and the command line tool.
// The privileged camera helper (C++, links librealsense) is built with CMake from helper/ (scripts/build.sh).

import Foundation
import PackageDescription

let thirdParty = "../../third_party"

// Release binaries must not contain absolute build paths (and with them the builder's user name): __FILE__ in
// third-party assertion and error macros (PoissonRecon, xatlas) would embed them, as scripts/build-sdk.sh and the
// helper's CMakeLists.txt already prevent for the C++ side. SwiftPM compiles with the real path, so map both spellings.
let packagePaths = Set([Context.packageDirectory,
                        URL(fileURLWithPath: Context.packageDirectory).resolvingSymlinksInPath().path])
let filePrefixMap = packagePaths.sorted().map { "-ffile-prefix-map=\($0)=." }
// The linker's debug map (the absolute paths of every object file and source folder) would be in the shipped
// executables too; release builds drop it (-S). Symbols stay, so crash reports still name functions.
let releaseLinkerFlags = LinkerSetting.unsafeFlags(["-Xlinker", "-S"], .when(configuration: .release))

let package = Package(
    name: "IntelFaceScan",
    platforms: [.macOS("27.2")],
    products: [
        .executable(name: "IntelFaceScan", targets: ["IntelFaceScanApp"]),
        .executable(name: "ifscan", targets: ["ifscan"]),
        .library(name: "ReconKit", targets: ["ReconKit"]),
    ],
    targets: [
        // Wire protocol shared with the C++ helper (C header).
        .target(
            name: "IFSProtocol",
            path: "Sources/IFSProtocol"),

        // Reconstruction core: C API over C++17 / Objective-C++ (Metal). Third-party code is header-only or
        // compiled through unity wrappers in src/third_party/.
        .target(
            name: "FaceReconCore",
            path: "Sources/FaceReconCore",
            cSettings: [
                .headerSearchPath("src"),
                .unsafeFlags(filePrefixMap, .when(configuration: .release)),
            ],
            cxxSettings: [
                .headerSearchPath("src"),
                .headerSearchPath("\(thirdParty)/eigen"),
                .headerSearchPath("\(thirdParty)/nanoflann/include"),
                .headerSearchPath("\(thirdParty)/xatlas/source/xatlas"),
                .headerSearchPath("\(thirdParty)/meshoptimizer/src"),
                .headerSearchPath("\(thirdParty)/stb"),
                .headerSearchPath("\(thirdParty)/PoissonRecon/Src"),
                .define("EIGEN_MPL2_ONLY"),
                .define("NDEBUG", .when(configuration: .release)),
                .unsafeFlags(["-O3"] + filePrefixMap, .when(configuration: .release)),
                .unsafeFlags(["-O2"], .when(configuration: .debug)),   // tests run real geometry
            ],
            linkerSettings: [
                .linkedFramework("Metal"),
                .linkedFramework("Foundation"),
                .linkedFramework("Accelerate"),
                .linkedFramework("ImageIO"),
                .linkedFramework("CoreGraphics"),
            ]),

        // Camera access: frame types, the helper connection (live / playback).
        .target(
            name: "CameraKit",
            dependencies: ["IFSProtocol"],
            swiftSettings: [.swiftLanguageMode(.v5)]),

        // Guided scan: Vision head pose, scan state machine, quality, coverage, datasets.
        .target(
            name: "ScanKit",
            dependencies: ["CameraKit"],
            swiftSettings: [.swiftLanguageMode(.v5)]),

        // Swift reconstruction pipeline over FaceReconCore.
        .target(
            name: "ReconKit",
            dependencies: ["FaceReconCore", "CameraKit", "ScanKit"],
            swiftSettings: [.swiftLanguageMode(.v5)]),

        // Procedural head scans with ground truth (tests, development without a camera).
        .target(
            name: "SyntheticData",
            dependencies: ["CameraKit", "ScanKit", "ReconKit"],
            swiftSettings: [.swiftLanguageMode(.v5)]),

        .executableTarget(
            name: "IntelFaceScanApp",
            dependencies: ["CameraKit", "ScanKit", "ReconKit", "SyntheticData"],
            swiftSettings: [.swiftLanguageMode(.v5)],
            linkerSettings: [releaseLinkerFlags]),

        .executableTarget(
            name: "ifscan",
            dependencies: ["CameraKit", "ScanKit", "ReconKit", "SyntheticData"],
            swiftSettings: [.swiftLanguageMode(.v5)],
            linkerSettings: [releaseLinkerFlags]),

        .testTarget(
            name: "FaceReconCoreTests",
            dependencies: ["FaceReconCore", "ReconKit", "SyntheticData"],
            swiftSettings: [.swiftLanguageMode(.v5)]),
        .testTarget(
            name: "CameraKitTests",
            dependencies: ["CameraKit", "IFSProtocol"],
            swiftSettings: [.swiftLanguageMode(.v5)]),
        .testTarget(
            name: "ScanKitTests",
            dependencies: ["ScanKit", "CameraKit", "SyntheticData"],
            swiftSettings: [.swiftLanguageMode(.v5)]),
        .testTarget(
            name: "ReconKitTests",
            dependencies: ["ReconKit", "ScanKit", "CameraKit", "SyntheticData", "FaceReconCore"],
            swiftSettings: [.swiftLanguageMode(.v5)]),
    ],
    cLanguageStandard: .c11,
    cxxLanguageStandard: .cxx17
)
