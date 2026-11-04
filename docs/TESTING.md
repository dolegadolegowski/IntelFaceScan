# Testing IntelFaceScan

Everything except the live camera is tested without hardware: unit tests per module, procedurally rendered head
scans with exact ground truth, the raw-session dataset format, accuracy metrics, the `ifscan` command line tool,
and end-to-end reconstructions. Real recordings are used when you point the tests at them; they never live in git.

## Running the tests

```bash
scripts/test.sh                          # all Swift Testing suites (debug build; C++ is built with -O2)
scripts/test.sh --filter ScanKitTests    # one target
scripts/test.sh --filter datasetRoundTrip
IFS_TEST_RECORDINGS=~/IntelFaceScanData/recordings scripts/test.sh --filter endToEndRecordings
```

* `scripts/test.sh` wraps `swift test`. With only the Command Line Tools installed it adds the Swift Testing macro
  plugin path that SwiftPM does not find on its own; with Xcode it is plain `swift test`. XCTest is not used.
* A full run takes about 30 s to build and 5 s to test on an M4 Pro. Tests that need parts still being written are
  skipped with a reason (`➜ Test ... skipped: "..."`), never failed.
* Do not run anything with `sudo` for tests; only the live camera needs administrator rights.

| Target / file | What it checks |
|---|---|
| `FaceReconCoreTests/EvalMetricsTests` | `fr_eval.h`: parallel planes (exact offset), concentric spheres, every Voronoi region of a triangle, degenerate triangles, argument validation, plane fit with noise and 30% outliers (RANSAC), pose errors near 0° and 180° |
| `ScanKitTests/DatasetRoundTripTests` | `ScanDataset`: bit-exact pixels (every 16-bit depth value), all metadata, documented snake_case layout, 16-bit grayscale depth PNGs, broken manifests, path traversal, refusing to replace foreign folders, `DatasetCameraSource` offline/real-time/loop replay |
| `ScanKitTests/SyntheticCameraSourceTests` | `SyntheticCameraSource`: lifecycle and events, 30 fps pacing, configured resolution, threshold filter, the pose animation |
| `ReconKitTests/SyntheticScanTests` | synthetic ground truth: depth equals an independent brute-force ray cast of the ground-truth mesh, back-projected depth lies on the surface, noise statistics, invalid pixels at silhouettes, `HeadPose` sign conventions, determinism, closed outward mesh |
| `ReconKitTests/CLIArgumentTests`, `CLIToolTests` | `ifscan` parsing (28 invalid command lines), commands run in-process, exit codes, OBJ dialects |
| `ReconKitTests/EndToEndSyntheticTests` | synthetic 1280×800 scan → `ReconstructionPipeline` (decimated to 60k triangles before texturing) → rotation error mean < 1°, worst frame < 1.5°, head misalignment < 5 mm, surface distance mean < 1 mm and p95 < 2.5 mm, export frame upright; the same through `ifscan reconstruct` and the distance command. Enabled automatically once `fr_register_frames` and `fr_tsdf_create` are implemented (`CoreAvailability`); recordings from `IFS_TEST_RECORDINGS` |

`CoreAvailability` (SyntheticData) probes the core with arguments a real implementation rejects without side
effects and reports which stages still return `FR_ERR_NOT_IMPLEMENTED`; use it for any test that needs a stage
of another work stream: `@Test(.enabled(if: CoreAvailability.reconstruction, "..."))`.

## Synthetic scans

`SyntheticScan.make(options)` returns the keyframes a guided scan would accept plus the ground truth:

* **Head**: a star-shaped surface around the head center: smooth union of a cranium ellipsoid (15.5 × 19 × 19.5 cm),
  a face/jaw ellipsoid and a neck cylinder, with displaced nose, eye sockets and eyeballs, brow ridge, cheekbones,
  lips, chin and ears. Cube-sphere tessellation, `12 · meshResolution²` triangles (default 128 → 196 608),
  closed and counter-clockwise from outside. The shape does not depend on the seed.
* **Texture**: a solid (3D) texture evaluated per pixel: skin with blotches, mottling, pores and freckles down to
  ~1.2 mm, red lips, eyebrows, eyes (sclera, iris, pupil), hair above a hairline. Octaves finer than ~2 pixels are
  faded out (no aliasing), so colored ICP sees consistent detail across views. A camera-mounted light adds mild
  shading (`lighting`), plus color sensor noise.
* **Camera**: D455 color intrinsics at 1280×800 (fx 640.4, fy 639.9, cx 641.2, cy 401.3) scaled to
  `width × height`; depth aligned to color, Z16 in 0.1 mm units. Depth is the exact ray/mesh intersection at each
  pixel center, plus Gaussian noise with σ = `depthNoiseAt1m · z²` (default 2 mm at 1 m → 0.72 mm at 0.6 m),
  invalid at viewing angles above 80°, at depth jumps (both sides, like stereo matching), and as 1% speckle. An
  optional wall stands behind the head (`backgroundDistanceM`).
* **Poses**: world = camera frame of keyframe 0 (exactly frontal), meters, +X right, +Y down, +Z forward. The head
  turns about a pivot near the neck to each `ScanPose.target` ± `poseVariationDeg` with ±2 mm translation jitter;
  `HeadPose.cameraFrameRotation` gives the rotation, so `world_from_camera` has rotation Rᵀ.
* **Per keyframe**: `headPose` = truth + N(0, 2°) per angle (like Vision), 76 projected landmarks (Vision's groups:
  contour, brows, eyes, pupils, nose crest and base, lips) with 1 px noise, a Vision-like face rectangle, the head
  mask (head above 1.5 cm below the chin), and `FrameQuality` measured on the images (variance of Laplacian,
  valid depth ratio and count, median face depth).
* **Determinism**: the same options give identical bytes (fixed-sequence RNG, counter-based per-pixel noise).
* Measured: noise-free depth back-projected with the true poses lies 0.02 mm (mean) from the true surface; a
  640×400 render takes ~11 ms and a 1280×800 render ~33 ms in a debug build.
* `lighting` (`ifscan synth --lighting`) controls the view-dependent shading, and it pulls two ways. Colored ICP
  prefers little shading: with the first integrated pipeline the worst-frame rotation error was 1.40° at the
  default 0.25 and 0.98° at 0. Apple Vision needs shading cues: at 0.25 `HeadTracker` misses the face in the down
  and ±45° poses and person segmentation fails; at 0.6 every scan pose is detected with the right yaw sign (yaw
  estimates 33° and −50° for ±45°) and segmentation works. So reconstruction datasets default to 0.25,
  `SyntheticCameraSource` (UI, guided scan) to 0.6; use `--lighting 0.6` for datasets that exercise `HeadTracker`.

`SyntheticScene` renders arbitrary poses (`render(cameraFromWorld:frameIndex:)` also returns the noise-free depth);
`SyntheticCameraSource` is a `CameraSource` ("Synthetic D455") that turns the head through the scan poses at the
configured frame rate and resolution, for UI work without a camera.

## Datasets (`.ifsdataset`)

The raw-session format of docs/ARCHITECTURE.md: `manifest.json` (snake_case), `color/*.png` 8-bit RGB,
`depth/*.png` 16-bit grayscale (readable by OpenCV with `IMREAD_UNCHANGED`, MeshLab, ImageMagick), `mask/*.png`.
`ScanDataset.write` assembles the folder next to the destination and swaps it in, and only replaces an existing
dataset or an empty folder. Additional optional frame keys: `imu`, `depth_intrinsics` (only if the aligned depth
reports other intrinsics than color), `color_frame_number`, `color_timestamp_ms`, `depth_frame_number`,
`depth_timestamp_ms` (only when they differ from the frameset's). `created_at` keeps milliseconds.

Synthetic datasets written by `ifscan synth` also contain `ground_truth.json` (true `world_from_camera` and head
pose per frame, scene parameters) and `ground_truth.obj` (true surface, millimeters, world frame); readers ignore
both, `ifscan reconstruct` uses them to report errors.

## ifscan

```bash
swift build && IFSCAN=$(swift build --show-bin-path)/ifscan

$IFSCAN synth -o /tmp/head.ifsdataset                   # 1280x800, 5 poses x 2, 2 mm noise at 1 m, seed 1
$IFSCAN synth -o /tmp/hard.ifsdataset --poses all --noise 3 --seed 7 --gt-mesh /tmp/gt.obj
$IFSCAN info /tmp/head.ifsdataset                       # device, configuration, per-frame pose and quality
$IFSCAN reconstruct /tmp/head.ifsdataset -o /tmp/head.obj --ply /tmp/head.ply --report /tmp/report.json
$IFSCAN reconstruct ~/IntelFaceScanData/recordings/anna-0001.db3 -o /tmp/anna.obj --save-dataset /tmp/anna.ifsdataset
$IFSCAN eval /tmp/head.obj --gt /tmp/head.ifsdataset/ground_truth.obj --report /tmp/report.json --symmetric
```

* `reconstruct` reads a dataset directly, or plays a `.db3`/`.bag` recording through the camera helper (as the
  user, no password, every frame) and the guided-scan logic (`ScanController`, `ScanConfiguration.offline`), then
  runs `ReconstructionPipeline` and `MeshExport` – the app's code path. Options: `--voxel-mm`, `--method
  tsdf|poisson`, `--keep-shoulders`, `--triangles` (decimate before texturing; UV unwrapping a full-resolution
  0.75 mm head mesh takes minutes), `--texture-size`, `--save-dataset`, `--helper`, `--report`, `-q`.
* The helper is looked up as `--helper`, `$IFS_HELPER`, next to `ifscan`, `../Helpers/` (app bundle),
  `build/helper/IntelFaceScanHelper`, `build/IntelFaceScan.app`, `/Applications/IntelFaceScan.app`.
* The report (`--report`) holds the reconstruction report, timings, `export_transform`/`export_scale`
  (world meters → exported millimeters), camera poses, and for synthetic datasets pose errors relative to
  keyframe 0 and the surface distance to the ground truth.
* The distance command reads both OBJs as millimeters and samples the first mesh area-uniformly (fixed seed,
  `--samples`, default 200 000): mean, RMS, median, 95th percentile and maximum (one-sided Hausdorff) distance.
  An exported mesh is in the export frame (head upright, face +Z, origin at the head center); `--report` maps it
  back to the world frame of the ground truth.
* `info` describes datasets, recordings (device and streams, through the helper), OBJ, PLY and PNG files.
* Exit codes (sysexits): 0 ok, 64 usage, 65 invalid input, 66 missing input, 69 a dependency is missing or still a
  placeholder (the message names it), 70 internal error, 73 cannot write the output.

## Recording test sessions

Recordings contain faces: they are personal data. Record only people who agreed to it, keep them outside the
repository (`.gitignore` excludes `*.db3`, `*.bag`, `*.ifsdataset`) and share them only through private storage.
Suggested layout:

```
~/IntelFaceScanData/recordings/<subject>-<nnnn>.db3        # IFS_TEST_RECORDINGS points here
~/IntelFaceScanData/recordings/<subject>-<nnnn>.txt        # consent, date, settings, lighting, notes
~/IntelFaceScanData/datasets/<subject>-<nnnn>.ifsdataset   # ifscan reconstruct --save-dataset
```

How to record a session that the offline scan can use:

1. Settings (the app's defaults, `CaptureConfiguration.faceScanD455`): depth 1280×720 @ 30 fps Z16, color 1280×800
   @ 30 fps RGB8, IMU on, preset *High Accuracy*, laser power maximum, emitter on. Record raw streams; the helper
   applies the same filters and alignment on playback as live.
2. Scene: the subject sits 0.55–0.75 m from the camera on a tripod (the camera must not move), even frontal light,
   hair away from the ears and forehead if possible, plain background ≥ 30 cm behind the head.
3. Motion: hold front for 2 s, then turn slowly to up, down, left (the subject's left), right, holding each pose
   ~1.5 s (about 25° up/down, 45° left/right), back to front; optionally the diagonals. 30–45 s in total.
4. Tools:
   * **The app**: start a live session and use its record function; the helper writes the file into a descriptor
     the app opened, so the recording belongs to you.
   * **RealSense Viewer** from [realsense-viewer-macos](https://github.com/dolegadolegowski/realsense-viewer-macos)
     (same patched librealsense): enable the streams above, press *Record* (`.db3` by default in 2.58, or `.bag`).
     It asks for the administrator password to capture the camera, and hands it back to macOS when it quits.
   * **rs-record** (built by `scripts/build-sdk.sh` into `build/sdk/install/bin`): needs root on macOS for USB
     capture; the default stream profiles differ from the app's, so prefer the app or the viewer.
5. Check it: `ifscan info rec.db3` (streams), then `ifscan reconstruct rec.db3 -o /tmp/rec.obj --save-dataset
   /tmp/rec.ifsdataset --report /tmp/rec.json` and look at the accepted keyframes with `ifscan info`.

`endToEndRecordingsReconstruct` reconstructs every recording in `IFS_TEST_RECORDINGS` and expects at least 5
accepted keyframes each; it needs the helper (`build/helper/IntelFaceScanHelper` or `IFS_HELPER`).

## Continuous integration

`.github/workflows/ci.yml` runs on pushes to `main`, pull requests and manual dispatch, with a read-only token and
no secrets. It uses GitHub's `xcode-27` image (Apple Silicon, macOS 27, Xcode 27.x; GitHub names macOS images after
Xcode since Xcode 27, so there is no `macos-27` label) and selects the newest Xcode 27. Steps: checkout with
submodules → restore `build/sdk/install` from the cache (key: librealsense submodule commit + hash of `patches/`,
`build-sdk.sh`, `apply-patches.sh`) → `scripts/build-sdk.sh` → save the cache → helper (CMake) → `swift build` →
`scripts/test.sh` → `ifscan synth`/`info` smoke test → release build and `scripts/bundle-app.sh` → upload the
zipped app. In October 2026 the image still reports macOS 27.0.1; the job warns that the app targets 27.2.

## Troubleshooting

* Errors about the `Testing` macros (`@Test`, `#expect`) not being found: run the tests through `scripts/test.sh`,
  which adds the macro plugin path when only the Command Line Tools are installed.
* `rs-convert` (also in `build/sdk/install/bin`) converts a `.bag` to PNG/PLY frames for inspection.
* `xcodebuild` / Xcode license prompts: not needed; everything builds with `swift build` and CMake, and Metal
  shaders are compiled at runtime from source.
* `ifscan reconstruct ... exit 69`: a stage is still a placeholder in this build, or the helper is missing; the
  notes below the error name it.
