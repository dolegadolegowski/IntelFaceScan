# IntelFaceScan – architecture

IntelFaceScan does a 3D scan of a human head with an Intel RealSense D455 (USB‑C) on Apple Silicon macOS
27.2+ and saves an OBJ (+MTL +PNG) model in millimeters.

## Processes

```
┌──────────────────────────── IntelFaceScan.app (user) ────────────────────────────┐
│ SwiftUI UI ─ ScanKit (Vision head pose, guided scan) ─ ReconKit ─ FaceReconCore │
│        │                                              (C++/Metal: ICP, pose    │
│   CameraKit ── AF_UNIX socket (docs/PROTOCOL.md) ──┐   graph, TSDF, MC, UV,    │
└────────────────────────────────────────────────────┼───  texture, OBJ/PLY) ────┘
                                                     │
┌────────── Contents/Helpers/IntelFaceScanHelper (root, live camera) ─────────────┐
│ IPC + security (main) ─ CaptureEngine (librealsense 2.58.4 + macOS patches)    │
│ streams, presets, filters, align, IMU, recording/playback, USB capture/release │
└──────────────────────────────────────────────────────────────────────────────────┘
```

* **Only the helper links librealsense.** macOS attaches UVCAssistant/HID drivers to the camera; libusb has to
  capture the USB device, which only root may do. The helper is started with the standard administrator
  prompt (`do shell script … with administrator privileges`, like the realsense-viewer-macos launcher).
* **The app never runs as root.** All UI, Vision, reconstruction and file I/O happen in the app as the user.
* **Playback** of `.db3`/`.bag` recordings uses the same helper binary started directly as the user (no USB,
  no password), so the post-processing chain is identical for live and recorded data.
* The camera is captured once per helper lifetime (patched librealsense) and handed back to macOS (USB
  re-enumeration, as `rs-macos-release`) when the helper exits – on `QUIT`, socket EOF, parent exit, or
  SIGINT/SIGTERM/SIGHUP. Never `kill -9` the helper during a camera command (can hang the firmware until
  re-plugged).

### Security rules for the root helper

1. Accept commands only from the app: socket in a `0700` directory owned by the user, peer uid and pid checked,
   strict message validation (sizes, JSON object, known fields), unknown messages are errors.
2. Never open, create, or delete a path supplied by the app. Recordings go to a root-private `mkdtemp`
   directory and are copied into a descriptor the app opened (`SCM_RIGHTS`). If files ever have to be handed
   back by path, use `rs-macos-handback` semantics: only files root created during the session, opened with
   `O_NOFOLLOW`, single link, `fchown` through the descriptor.
3. No `system()`/shell, no environment-dependent behavior; `--playback` refuses to run as root.
4. Log messages go to the app over the socket (no root-owned log files in the user's home).

## Modules and ownership

| Module | Path | Language | Owner (work stream) |
|---|---|---|---|
| SDK build + patches | `third_party/librealsense`, `patches/`, `scripts/build-sdk.sh`, `scripts/apply-patches.sh` | CMake | platform |
| Helper main, IPC, security, USB release, fake camera | `helper/CMakeLists.txt`, `helper/src/main.cpp`, `helper/src/ipc_*`, `helper/src/security_*`, `helper/src/usb_release.*`, `helper/src/fake_engine.*`, `docs/HELPER.md` | C++ | platform |
| Capture engine + measurements | `helper/src/capture_*`, `helper/src/probe.*`, `helper/tests/*` (capture tests, `make-test-recording`), `docs/CAMERA.md` | C++ | capture |
| Wire protocol | `Sources/IFSProtocol`, `docs/PROTOCOL.md` | C | shared (frozen) |
| Camera sources (live/playback) | `Sources/CameraKit/Helper/*` | Swift | platform |
| Shared types | `Sources/CameraKit/CameraTypes.swift`, `CameraSource.swift`, `Sources/ScanKit/ScanTypes.swift`, `Sources/ReconKit/ReconTypes.swift`, `Sources/ReconKit/CoreBridge.swift`, `Sources/FaceReconCore/include/*.h`, `Sources/FaceReconCore/src/common/*` | Swift/C | shared (frozen) |
| Guided scan (Vision, state machine, quality, coverage) | `Sources/ScanKit/Guidance/*` | Swift | ui-scan |
| SwiftUI app (except the 3D viewer) | `Sources/IntelFaceScanApp/*` | Swift | ui-scan |
| Registration, pose graph, TSDF (Metal), MC, Poisson | `Sources/FaceReconCore/src/{registration,fusion}/*`, `Sources/ReconKit/ReconstructionPipeline.swift`, `Sources/ReconKit/Registration*.swift`, `Sources/ReconKit/SegmentationHeadMask.swift`, `docs/RECONSTRUCTION.md` | C++/ObjC++/Swift | recon |
| Mesh cleanup, UV, texture, export, 3D viewer | `Sources/FaceReconCore/src/{mesh,texture,export}/*`, `Sources/ReconKit/{MeshProcessing,Texturing,MeshExport}.swift`, `Sources/IntelFaceScanApp/ModelViewer/*` | C++/Swift | mesh |
| Datasets, synthetic data, eval, CLI, CI | `Sources/ScanKit/Dataset/*`, `Sources/SyntheticData/*`, `Sources/FaceReconCore/src/eval/*`, `Sources/ifscan/*`, `.github/workflows/*`, `scripts/test.sh`, `docs/TESTING.md` | Swift/C++ | tests |
| Build, app bundle, DMG | `scripts/build.sh`, `scripts/bundle-app.sh`, `scripts/package.sh`, `scripts/dev-worktree-setup.sh`, `tools/release/*` | bash/C/Swift | platform |

"Frozen" files are the contract between work streams: change them only together with every user of the change.
Each work stream owns its tests under `Tests/<Module>Tests/` (file names prefixed with the stream's area).

## Data flow

1. **Preview** – `CameraSource.frames()` delivers `CameraFrame`s (RGB8 1280×800 + depth aligned to color, Z16 with
   0.1 mm units, intrinsics, depth→color extrinsics, IMU). The UI renders color and a depth colormap with Metal,
   and the face distance with the recommended range for the current configuration.
2. **Guided scan** – `HeadTracker` (Vision: face rectangle + yaw/pitch/roll, 76 landmarks, person segmentation)
   feeds `ScanController`: poses front → up → down → left → right (+ optional diagonals); a frame is accepted only
   when the pose is within tolerance and held still, sharp (variance of Laplacian), with enough valid depth
   points and a static camera (IMU). Accepted frames become `Keyframe`s (frame + head pose + landmarks + head mask);
   the coverage map shows which head areas are covered.
3. **Reconstruction** (`ReconstructionPipeline`):
   1. segmentation: person mask ∩ depth band around the face ∩ above the chin/neck cut (option: keep shoulders),
   2. initial poses from the Vision head pose (rotation) and landmark/mask centroids (translation),
   3. multi-scale point-to-plane ICP between neighboring keyframes (colored ICP optional; measured comparison in
      docs/RECONSTRUCTION.md),
   4. loop closures between overlapping poses (left↔front↔right, up/down↔front), pose graph optimization,
   5. TSDF fusion on the GPU (Metal), 0.75 mm voxels by default (chosen from the measured accuracy of 0.5, 0.75
      and 1 mm; coarser only when the volume would be too large), marching cubes; screened Poisson as the
      alternative (compared in docs/RECONSTRUCTION.md),
   6. cleanup (small components, detail-preserving smoothing, hole filling, optional decimation),
   7. UV atlas (xatlas) and texture from the most frontal sharp keyframes with blending + vertex colors.
4. **Result** – 3D preview (SceneKit: texture / vertex colors / shaded / wireframe), export OBJ+MTL+PNG
   in millimeters (head upright +Y, face toward +Z, origin at the head center), optional PLY and the raw
   session (ScanDataset).

## Conventions

See `Sources/FaceReconCore/include/fr_types.h`: meters internally, column-major 4×4 matrices, camera frame
+X right / +Y down / +Z forward, poses are `world_from_camera` with the world = camera frame of the first
frontal keyframe; depth aligned to color; triangles counter-clockwise from outside; UV origin bottom-left.
Head pose angles: `ScanKit/ScanTypes.swift` (`HeadPose`).

## ScanDataset (raw session format)

A folder `<name>.ifsdataset/`:

```
manifest.json
color/000000.png   RGB8, color resolution
depth/000000.png   16-bit grayscale, depth aligned to color, units = manifest.frames[i].depth_scale meters
mask/000000.png    8-bit head mask (optional)
```

`manifest.json` (snake_case):
```json
{
  "format": "ifsdataset", "version": 1, "created_at": "2026-11-03T21:00:00Z",
  "device": { "...": "DeviceInfo" }, "configuration": { "...": "CaptureConfiguration" },
  "notes": {"source": "head-1280x720.db3"},
  "frames": [{
    "id": 0, "sequence": 120, "timestamp_ms": 1730000000123.4, "scan_pose": "front",
    "color": "color/000000.png", "depth": "depth/000000.png", "mask": "mask/000000.png",
    "depth_scale": 0.0001, "intrinsics": { "...": "color intrinsics" },
    "depth_to_color": {"rotation": [], "translation": []},
    "head_pose": {"yaw": 0.4, "pitch": -1.2, "roll": 0.3, "confidence": 0.98},
    "face_rect": [x, y, width, height], "landmarks": [[x, y], ...],
    "quality": { "...": "FrameQuality" }, "metadata": {}
  }]
}
```

## Build

`scripts/build.sh`: tool checks → submodules → patched librealsense (`scripts/build-sdk.sh`) → helper (CMake) →
Swift package (`swift build -c release`) → `build/IntelFaceScan.app` (`scripts/bundle-app.sh`: helper in
`Contents/Helpers`, `librealsense2.X.Y.dylib` in `Contents/Frameworks`, `ifscan` in `Contents/MacOS`, ad-hoc
signature). It prints the time of each step: from a fresh clone on an M3 Ultra (28 cores, Command Line Tools only)
2¼–2½ min in three runs – submodules 19 s, librealsense 45–63 s, helper 3–5 s, Swift release build 62–63 s,
bundle 2–3 s; a second run without changes takes 3 s.

Requirements: an Apple Silicon Mac, the Command Line Tools (`xcode-select --install`) or Xcode 27 with the macOS
27.2 SDK, CMake 3.20+ (`brew install cmake`; no other Homebrew package: libusb is built from source and linked
statically) and network access for the first build (submodules; librealsense downloads libusb, nlohmann/json,
fastcdr, yaml-cpp and sqlite while configuring). The script checks these up front and names the fix (missing
tools, an Xcode whose license is not accepted, an SDK older than 27.2). Building works on older macOS 27 releases
(warning); the app needs 27.2 to run. Metal shaders are compiled at runtime from source (the CLT have no `metal`
compiler). Release binaries contain no absolute build paths (`-ffile-prefix-map`, no linker debug map).
`scripts/test.sh` runs the Swift Testing suites (adds the Testing macro plugin path when only the Command Line
Tools are installed).

## Testing without a camera

* `SyntheticData` renders a textured synthetic head from known poses with a D455-like noise model; tests
  check registration (pose error), fusion (surface distance to ground truth) and export.
* `ifscan reconstruct <recording.db3|dataset.ifsdataset> -o head.obj` runs the guided-scan logic offline over a
  recording (helper in playback mode) and the full reconstruction – the same code path as the app;
  `ifscan reconstruct synthetic-camera` does the same live with `SyntheticCameraSource` (the animated synthetic head
  at 30 fps through Vision), which the app also offers as a camera for UI work. See docs/TESTING.md.
