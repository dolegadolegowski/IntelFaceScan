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
* The five test targets run one after another, as separate test runs (the tests of a target in parallel). Timing
  on an M3 Ultra (28 cores, Command Line Tools), November 2026: ScanKitTests 127 tests ~27 s (Vision),
  ReconKitTests 88 tests ~75–100 s (the longest single test sets it: the combined-imperfections reconstruction of
  `Pipeline robustness`; the two 1280×800 reconstructions of `Pipeline wide side views` run beside it),
  IntelFaceScanAppTests 40 tests ~8 s, FaceReconCoreTests 93 tests ~9 s, CameraKitTests 77 tests ~27 s (most of it
  `Helper playback backpressure`); about 2.5–3 min in all when nothing changed, about 3.5 min from a clean debug
  build (about 40 s of it building).
* The reconstruction tests of ReconKitTests (the suites and tests marked `.heavyReconstruction`,
  `Tests/ReconKitTests/HeavyTestSlots.swift`) share a few slots. Each reconstruction peaks at about 0.5 GB and uses
  every core it gets; with about twenty of them at once, no test finished for 35 minutes on the 3-core, 7 GB CI
  runner (memory pressure or CPU oversubscription: neither was measured there). The number of slots is
  min(cores, 2 × (memory in GB − 3)): 28 on the M3 Ultra (no limit in practice) and 3 on the runner, where
  ReconKitTests then passed in about 400 s (measured before `Pipeline wide side views` was added; in CI run
  41815653340 the runner kept about 70 % of its memory free and used no swap with 3 slots, and ReconKitTests took
  377 s); `IFS_TEST_HEAVY_SLOTS=n` overrides it (with 3: ReconKitTests ~90 s and 1.5 GB at the peak on the M3 Ultra,
  instead of ~65 s and 2.6 GB, also measured before `Pipeline wide side views`).
  `cliSyntheticCameraScanStopsAtTheTimeLimit` runs `ifscan reconstruct synthetic-camera` in-process and in real
  time, and takes every slot (exclusive): next to the other reconstructions it took 105–227 s on the runner, and
  twice it had not ended when the step timed out. A `.timeLimit` counts the wait for a slot, so the marked suites
  that have one allow 30 minutes.
* Skipped, with a reason (`➜ Test ... skipped: "..."`), never failed: benchmarks and measurement sweeps that are
  run by hand (`PipelineBenchmark`, `Mesh performance`, `ReconCaptureRange`, the `IFS_RECON_STUDY=1` tables of
  docs/RECONSTRUCTION.md, `GuidanceSyntheticVisionStudy` with `IFS_VISION_STUDY=1` – `IFS_VISION_STUDY_GRID=1` adds
  a pitch × yaw grid, ...), the app's full guided synthetic scan without `IFS_APP_FULL_SCAN=1` (~4.5 min),
  `endToEndRecordingsReconstruct` without `IFS_TEST_RECORDINGS`, the `CoreAvailability` placeholder tests (the core is integrated), and the
  CameraKit helper integration tests when `build/helper/IntelFaceScanHelper` is not built (or `IFS_HELPER_PATH` is
  not set); the playback ones also need `build/helper/tests/make-test-recording` (`-DIFS_HELPER_TESTS=ON`).
  `IFS_HELPER` is the variable `ifscan` and the recording tests use.
* C++ capture engine tests of the helper (synthetic recordings, no camera): build the helper with
  `-DIFS_HELPER_TESTS=ON`, then `ctest --test-dir build/helper --output-on-failure` (one ctest entry running the 28
  `capture-tests`, ~25 s; 27 run by default, because the benchmark is skipped unless run as
  `build/helper/tests/capture-tests bench`).
* The SDK bookkeeping of the build scripts (moving between versions of `patches/`, the SDK's input stamp):
  `scripts/test-build-scripts.sh`, against a stand-in librealsense in a temporary folder (no network, ~10 s).
* Do not run anything with `sudo` for tests; the live camera needs the camera access service (docs/HELPER.md), not root.

| Target / file | What it checks |
|---|---|
| `FaceReconCoreTests/EvalMetricsTests` | `fr_eval.h`: parallel planes (exact offset), concentric spheres, every Voronoi region of a triangle, degenerate triangles, argument validation, plane fit with noise and 30% outliers (RANSAC), pose errors near 0° and 180° |
| `ScanKitTests/DatasetRoundTripTests` | `ScanDataset`: bit-exact pixels (every 16-bit depth value), all metadata, documented snake_case layout, 16-bit grayscale depth PNGs, broken manifests, path traversal, refusing to replace foreign folders, `DatasetCameraSource` offline/real-time/loop replay |
| `ScanKitTests/SyntheticCameraSourceTests` | `SyntheticCameraSource`: lifecycle and events, 30 fps pacing, configured resolution, threshold filter, the pose animation |
| `ReconKitTests/SyntheticScanTests` | synthetic ground truth: depth equals an independent brute-force ray cast of the ground-truth mesh, back-projected depth lies on the surface, noise statistics, invalid pixels at silhouettes, `HeadPose` sign conventions, determinism, closed outward mesh |
| `ReconKitTests/CLIArgumentTests`, `CLIToolTests` | `ifscan` parsing (32 invalid command lines), commands run in-process, exit codes, outputs checked before any work (unwritable paths, `--save-dataset` onto the input), ground-truth evaluation of a scan that does not start with the frontal pose (world frames aligned), `synthetic-camera` stopping at `--time-limit`, OBJ dialects |
| `ReconKitTests/EndToEndSyntheticTests` | synthetic 1280×800 scan → `ReconstructionPipeline` (decimated to 60k triangles before texturing) → rotation error mean < 1°, worst frame < 1.5°, head misalignment < 5 mm, surface distance of the measured part (`ReconMesh.measuredSurface`: without the synthesized closing of the back of the head) mean < 1 mm and p95 < 2.5 mm, export frame upright; the same through `ifscan reconstruct`, and the exported OBJ measured through `--report` matches the report's whole-mesh distance. Enabled by `CoreAvailability` (the core is integrated); recordings from `IFS_TEST_RECORDINGS` |
| `IntelFaceScanAppTests` | the app executable (`@testable import`): `AppModel` lifecycle with the fake-camera helper (skipped without `build/helper/IntelFaceScanHelper`), settings mapping and loading older stored settings, the scan sounds (which cue when, without playing; one tick per reached pose in any order), the banner listing the open directions with no active pose (English and Polish, the last after "or", the sides person-relative, no list beyond four), the pose guide's arrows only for that instruction and the skip button naming its pose, and the save panel's doubled extension, English/Polish strings, report notes and Polish plurals, result export success and failure paths, the SceneKit preview rendered offscreen; `IFS_APP_FULL_SCAN=1` runs the full guided synthetic scan, reconstruction and OBJ export through `AppModel`. A debug build renders the UI's screens to PNG: `$(swift build --show-bin-path)/IntelFaceScan --render-snapshots <dir>` (add `-AppleLanguages '(pl)'` for Polish); release builds, including the bundled app, ignore the option |

The other suites belong to the modules' work streams: `CameraKitTests` (wire format, JSON, the helper in fake-camera
mode end to end, session lifecycle, frame broadcasting, playback backpressure, the helper's security checks),
`ScanKitTests/Guidance*` (head tracker, masks, quality, scan controller; `Guidance pose recognition`: turns far
beyond the target, no order after the front pose (a turn toward any unfinished pose makes it the active pose, near
the front the instruction is to turn in any direction, turned toward a pose beyond its sideways tolerance the hint
corrects it), short turns held toward whichever pose the head is heading to, their time kept per pose through
jitter between neighbouring diagonals, the front pose within 12°, diagnostics per active pose, skipping exactly the
pose the update names and the remaining optional ones, the former fixed suggestion with `posesInAnyOrder = false`,
turn hints, with the head poses of the first live D455 session), `FaceReconCoreTests/Recon*`, `Mesh*`,
`Texture*`, `ExportFile*` and `ReconKitTests/Pipeline*` (with the registration check, robustness to real-data
imperfections and side views beyond 60°: `Pipeline wide side views`, 65° side views without depth beyond 60°
incidence, all 12 keyframes used, side frames < 0.6° / 2.5 mm), `ReconLensDistortion*` (undistorting keyframes once
before reconstruction), `ReconInitialPose*` (initial rotations with Vision's line of sight taken out),
`MeshProcessing*`, `MeshClosing*` (closing the head into a watertight bust that keeps the measured surface),
`MeshSynthesizedTexture*` (interpolated colors on synthesized surface), `Export*`, `TextureBake*` (registration,
fusion, mesh processing, texturing, export).

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
  faded out (no aliasing), so the optional colored ICP and texturing see consistent detail across views. A
  camera-mounted light adds mild shading (`lighting`), plus color sensor noise.
* **Camera**: D455 color intrinsics at 1280×800 (fx 640.4, fy 639.9, cx 641.2, cy 401.3) scaled to
  `width × height`, an ideal pinhole unless `lensDistortion` is set (e.g. `LensDistortion.d455Color`: color, mask
  and landmarks go through the lens; the aligned depth follows it only for modified Brown-Conrady, as librealsense's
  NEON align does); `headOffsetM` moves the head off the optical axis; depth aligned to color, Z16 in 0.1 mm units. Depth is the exact ray/mesh intersection at each
  pixel center, plus Gaussian noise with σ = `depthNoiseAt1m · z²` (default 2 mm at 1 m → 0.72 mm at 0.6 m),
  invalid at viewing angles above 80°, at depth jumps (both sides, like stereo matching), and as 1% speckle. An
  optional wall stands behind the head (`backgroundDistanceM`). `frontalHalfWidthM` keeps depth and mask of the
  frontal keyframes only within that distance of the head center across the image (a real D455 head mask was
  10.7 cm wide).
* **Poses**: world = camera frame of keyframe 0 (exactly at the first pose's target, frontal by default), meters,
  +X right, +Y down, +Z forward. The head turns about a pivot near the neck to each `ScanPose.target` (left and right
  at ±`sideYawDeg` when set: real side views reach 60–62°) ± `poseVariationDeg` with ±2 mm translation jitter;
  `HeadPose.cameraFrameRotation` gives the rotation, so `world_from_camera` has rotation Rᵀ. The reconstruction's
  world is the camera frame of its first *frontal* keyframe; for a pose list that does not start with `front` the
  evaluation maps it to the ground truth's world through keyframe 0's estimated pose (`ground_truth_from_world` in the
  report).
* **Per keyframe**: `headPose` = truth + Vision's line-of-sight effect (yaw + 0.75 a, pitch − 0.67 b for a face
  seen a / b degrees right of / below the optical axis, `visionLineOfSightGain`) + N(0, 2°) per angle (+ the
  systematic `sidePoseErrorDeg` on left and right: turn overstated, pitch, roll), 76 projected landmarks (Vision's groups:
  contour, brows, eyes, pupils, nose crest and base, lips) with 1 px noise, a Vision-like face rectangle, the head
  mask (head above 1.5 cm below the chin), and `FrameQuality` measured on the images (variance of Laplacian,
  valid depth ratio and count, median face depth).
* **Determinism**: the same options give identical bytes (fixed-sequence RNG, counter-based per-pixel noise).
* Measured: noise-free depth back-projected with the true poses lies 0.02 mm (mean) from the true surface; a
  640×400 render takes ~11 ms and a 1280×800 render ~33 ms in a debug build.
* `lighting` (`ifscan synth --lighting`) controls the view-dependent shading. Apple Vision needs shading cues: at
  0.25 `HeadTracker` misses the face in the down and ±45° poses and person segmentation fails; at 0.6 every scan
  pose is detected with the right yaw sign. What Vision reads at 0.6 depends on the resolution (measured with
  `IFS_VISION_STUDY=1 scripts/test.sh --filter GuidanceSyntheticVisionStudy`): at 640×400 yaw +45° reads ~46° and
  −45° ~−52°, and person segmentation covers the face in nearly every frame; at 1280×800 +45° reads ~30° (+50° ~39°,
  +55° ~53°) and −45° ~−50°, pitch −25° reads ~−23° but −30° only ~−12°, and segmentation covers the frontal face in
  only about half of the frames (the head mask then falls back to the face-rectangle ellipse). Vision also reads
  large yaw partly as pitch (+44° to +52° of yaw reads +4° to +10° of pitch at 640×400), and its angles jump and
  drift by several degrees on a held head. Colored ICP degrades with shading (worst frame 1.29° at 0.25, 8.4° at
  0.6), which is one reason why point-to-plane ICP is the default; it is unaffected (0.12° at both,
  docs/RECONSTRUCTION.md, "Colored versus point-to-plane ICP"). Reconstruction datasets default to 0.25,
  `SyntheticCameraSource` (UI, guided scan) to 0.6; use `--lighting 0.6` for datasets that exercise `HeadTracker`.

`SyntheticScene` renders arbitrary poses (`render(cameraFromWorld:frameIndex:)` also returns the noise-free depth);
`SyntheticCameraSource` is a `CameraSource` ("Synthetic D455") that turns the head through the scan poses at the
configured frame rate and resolution, for UI work without a camera. Like a user who keeps turning while the guidance
asks for more, it holds every pose but the front at its target and 10 % and 20 % beyond it (left up to 54°, down
to −30°). `ifscan reconstruct synthetic-camera` (1280×800) took 3838 frames when the head stopped at the targets
("left" waited 2237 frames, "down" 1090); with these stops and ScanKit's 12° tolerance across the pose direction
(`perpendicularToleranceDeg`: pitch on the side poses) it took 823 and 1021 frames in two runs (each change alone
924 and 947), and the guided scan now mostly waits for the animation to reach the next pose.

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
$IFSCAN reconstruct synthetic-camera -o /tmp/live.obj    # the app's live flow without the UI (~4.5 min; release ~2.5)
$IFSCAN eval /tmp/head.obj --gt /tmp/head.ifsdataset/ground_truth.obj --report /tmp/report.json --symmetric
```

* `reconstruct` reads a dataset directly, or plays a `.db3`/`.bag` recording through the camera helper (as the
  user, no password, every frame) and the guided-scan logic (`ScanController`, `ScanConfiguration.offline`), then
  runs `ReconstructionPipeline` and `MeshExport` – the app's code path. The input `synthetic-camera` scans the
  animated `SyntheticCameraSource` head in real time through Vision and the guided scan instead. Options: `--ply`,
  `--voxel-mm`, `--method tsdf|poisson`, `--keep-shoulders`, `--open-back` (leave the unscanned back of the head
  and the neck open instead of closing them with a synthesized surface), `--icp colored|geometric` (default geometric =
  point-to-plane), `--triangles` (decimate before texturing; UV unwrapping and baking the full-resolution
  0.75 mm head mesh of `ifscan synth`, ~550k triangles with the closed back of the head (~500k with
  `--open-back`), take ~16 s of its ~21 s reconstruction in a release build; closing the head adds ~5 s of cleanup), `--texture-size`, `--save-dataset` (not onto the input), `--helper`, `--report`, `-q`,
  `--time-limit` (synthetic-camera: stop the guided scan after this many seconds, default 600, and reconstruct
  what it accepted; it would otherwise wait forever for a pose Vision never detects). Every
  output path is checked (folders created, writable) before the input is read, so a long scan is not lost at the
  export.
* The helper is looked up as `--helper`, `$IFS_HELPER`, next to `ifscan`, `../Helpers/` (app bundle), then
  relative to the current directory `build/helper/IntelFaceScanHelper` and
  `build/IntelFaceScan.app/Contents/Helpers/`, and `/Applications/IntelFaceScan.app`.
* The report (`--report`) holds the reconstruction report, timings, `export_transform`/`export_scale`
  (world meters → exported millimeters), camera poses, and for synthetic datasets pose errors relative to
  keyframe 0, `ground_truth_from_world` and the surface distance to the ground truth (in the ground truth's frame):
  `surface_to_ground_truth` for the measured part, and `whole_mesh_to_ground_truth` with the synthesized surface
  (filled holes, the closed back of the head) when there is some – what `ifscan eval` measures on the exported OBJ.
* `eval` reads both OBJs as millimeters and samples the first mesh area-uniformly (fixed seed,
  `--samples`, default 200 000): mean, RMS, median, 95th percentile and maximum (one-sided Hausdorff) distance.
  An exported mesh is in the export frame (head upright, face +Z, origin at the head center); `--report` maps it
  back to the world frame of the ground truth.
* `info` describes datasets, recordings (device and streams, through the helper), OBJ, PLY and PNG files.
* Exit codes (sysexits): 0 ok, 64 usage, 65 invalid input, 66 missing input, 69 a dependency is missing or still a
  placeholder (the message names it), 70 internal error, 73 cannot write an output.

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
     It uses the camera access service (no password per session) and hands the camera back to macOS when it quits.
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
submodules → `scripts/test-build-scripts.sh` → restore `build/sdk/install` from the cache (key: librealsense
submodule commit + deployment target + hash of `patches/`, `build-sdk.sh`, `apply-patches.sh`) →
`scripts/build-sdk.sh` (a restored SDK carries its input stamp `.ifs-sdk-inputs` and must count as current: a rebuild
after a cache hit fails the job) → save the cache → helper with its tests (CMake, `-DIFS_HELPER_TESTS=ON`) → `ctest`
→ `swift build` → `scripts/test.sh` → `ifscan synth`/`info` smoke test → `scripts/build.sh` (the documented
one-command build; the SDK comes from the cache) → bundle smoke test (signature, the helper's `@rpath` librealsense,
no absolute build paths under `$HOME` in any shipped file, no linker debug map in the bundled Swift executables and
their dSYMs in `build/dSYMs`, the bundled `ifscan` running `synth` and reading a generated `.db3` through the
bundled helper) → upload the zipped app. The smoke tests and `scripts/build.sh` also run when a test failed (the job
still fails). A run took about 15 minutes with the SDK cache in November 2026 (CI run 41797620726), about 4 more
without it (the SDK build); `scripts/test.sh` took about 10 of them: ReconKitTests ~400 s with its 3 slots (~90 s on
the M3 Ultra with the same 3 slots), ScanKitTests ~60 s (54 s of it the first Vision requests, which hold every
thread), CameraKitTests ~60 s, FaceReconCoreTests ~25 s. The test step logs swap, memory pressure and the busiest
processes every 2 minutes in a folded group, and the next step stops test processes a timed-out step left behind
(they hold the `.build` lock, so every later `swift build` would wait for them). Timing-sensitive tests are less
reliable there: the tests of a target share Swift's cooperative thread pool, which the synchronous tests can fill
(measured there: the synthetic camera rendered 0.32 s per frame, `ifscan reconstruct synthetic-camera --time-limit
5` ran in-process for 4 min); their upper bounds are loose for that reason. The runner has 3 cores and 7 GB of
memory (the job logs both). In November 2026 the image still reports macOS 27.0.1; the job and `scripts/build.sh`
warn that the app needs 27.2 to run (building only needs the 27.2 SDK). While it does, no Swift code in the package
may compare dates with `<=` or `>=` (compare `timeIntervalSince(_:)`): those `Date` operators are back-deployed only
before 27.2, so a build for 27.2 imports the system's, which 27.0.1 lacks. The library targets and the app module
are linked into the test bundles, so a bundle that imports them does not load there (`--filter` and `--skip` load
it too), and `ifscan` would not start; `nm -u <binary> | grep "4DateV2[gl]eoiy"` shows such imports.

## Troubleshooting

* Errors about the `Testing` macros (`@Test`, `#expect`) not being found: run the tests through `scripts/test.sh`,
  which adds the macro plugin path when only the Command Line Tools are installed.
* `rs-convert` (also in `build/sdk/install/bin`) converts a `.bag` to PNG/PLY frames for inspection.
* `xcodebuild` / Xcode license prompts: not needed; everything builds with `swift build` and CMake, and Metal
  shaders are compiled at runtime from source. When Xcode is selected but its license is not accepted, every tool
  (git, clang, swift) fails; `scripts/build.sh` says so up front. Accept it (`sudo xcodebuild -license accept`) or
  use the Command Line Tools: `DEVELOPER_DIR=/Library/Developer/CommandLineTools scripts/build.sh` (same for
  `scripts/test.sh` and `swift build`).
* The first `scripts/build.sh` needs the network: the submodules, and librealsense's CMake downloads libusb,
  nlohmann/json, fastcdr, yaml-cpp and sqlite into `build/sdk/build`.
* A worktree made with `git worktree add` (or by Claude Code): `scripts/dev-worktree-setup.sh` fills the submodules
  from the main checkout and prints the helper's CMake command against the main checkout's SDK. Reconfiguring
  `build/helper` with another `-DIFS_SDK_DIR` links that SDK (the helper's CMake searches it on every configure).
* `scripts/build-sdk.sh` rebuilds librealsense when what it was built from changes – the librealsense commit, a
  patch, `build-sdk.sh` itself, `BUILD_TYPE` or `MACOSX_DEPLOYMENT_TARGET`, recorded in
  `build/sdk/install/.ifs-sdk-inputs` – and prints the new values; file times do not matter, `IFS_SDK_REBUILD=1`
  forces a rebuild. It refuses an `IFS_SDK_DIR` whose build folder was configured from another checkout.
* `apply-patches.sh: ... local changes that conflict with patches/`: something edited `third_party/librealsense` by
  hand. `scripts/apply-patches.sh --reset` restores it and applies every patch again (changes to librealsense belong
  in `patches/`). A pull that changes, adds or removes a patch, or moves librealsense to another commit, needs
  nothing: `scripts/build.sh` restores the submodule and applies the new patches (the applied ones are recorded in
  `third_party/librealsense/.ifs-patches`).
* `rs-record`, `rs-convert` and the other tools in `build/sdk/install/bin` find the library through
  `@loader_path/../lib` (older SDK builds lack that rpath; the next `scripts/build-sdk.sh` rebuilds them).
* `ifscan reconstruct ... exit 69`: a stage is still a placeholder in this build, or the helper is missing; the
  notes below the error name it.
