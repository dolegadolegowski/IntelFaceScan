# Camera: Intel RealSense D455 for face scanning

How IntelFaceScan uses the D455 through librealsense 2.58.4 (`helper/src/capture_*`), why the face-scan
configuration is what it is, and how to measure the camera (`IntelFaceScanHelper --probe`). Numbers marked
*theory* follow from the formulas below. The measured ones still have to be filled in from a probe run with the
camera (see "Measurement procedure").

## D455 facts

| Property | Value |
|---|---|
| Stereo baseline | 95 mm (left to right imager; the helper reads it from the factory extrinsics, `baseline_mm`) |
| Depth imagers | 2 × OV9782, global shutter, 1280×800 native, IR projector (laser 0–360 mW, step 30) |
| Depth FOV | ~87° × 58° (nominal; calibrated units are typically ~89–91° × 58–60° at 1280×720) |
| Depth output | Z16 up to 1280×720 @ 30 fps (848×480 up to 90 fps), USB 3 required for 1280×720 + RGB |
| RGB | OV9782 global shutter, 1280×800 @ 30 fps, ~90° × 65°, fixed focus, inverse Brown–Conrady model |
| RGB position | ~59 mm to the right of the left imager (`depth_to_color.translation ≈ [-0.059, 0, 0]`) |
| IMU | BMI055: accel 63 or 100 Hz, gyro 200 or 400 Hz (the helper uses the lowest rates: only the newest sample is used) |
| Min-Z (datasheet) | ~52 cm at 1280×720 without disparity shift |
| Max disparity search | 126 px (+ disparity shift), at the output resolution |
| Default depth units | 1 mm; with advanced mode down to 1 µm (we use 0.1 mm) |

The depth sensor's frame is the left imager's frame; depth intrinsics equal the left infrared intrinsics.

## Stereo geometry

With focal length *f* (pixels, at the depth resolution), baseline *B* and disparity shift *s*:

* **Min-Z** = f·B / (126 + s): nearer points need more disparity than the search range has: no depth (holes).
* **Max-Z** = f·B / s: farther points have less disparity than the shift: no depth (or rejected false
  matches). Unbounded for s = 0.
* **Depth RMS error** ≈ z² · subpixel / (f·B), with a subpixel RMS of ~0.08 px for a well-textured target
  (Intel's figure for D4xx). The error grows with the square of the distance, so being close matters more than
  anything else.

*f* from the nominal 87° HFOV: **674.4 px at 1280×720** (f·B = 64.07 px·m), **446.8 px at 848×480**
(f·B = 42.45 px·m). Calibrated D455s report ~630–645 px at 1280×720, which lowers Min-Z by ~5% and raises the RMS
by ~5%; the probe prints the real *f* of the connected camera.

### Min-Z / Max-Z (theory)

| Depth resolution | Disparity shift | Min-Z | Max-Z |
|---|---|---|---|
| 1280×720 | 0 | 0.508 m | ∞ |
| 1280×720 | 50 | 0.364 m | 1.281 m |
| 1280×720 | 100 | 0.283 m | 0.641 m |
| 848×480 | 0 | 0.337 m | ∞ |
| 848×480 | 50 | 0.241 m | 0.849 m |
| 848×480 | 100 | 0.188 m | 0.425 m |

### Depth RMS error (theory, subpixel 0.08 px)

| Distance | 0.35 m | 0.40 m | 0.45 m | 0.50 m | 0.55 m | 0.60 m | 0.70 m | 0.80 m | 1.00 m |
|---|---|---|---|---|---|---|---|---|---|
| 1280×720 | 0.15 mm | 0.20 mm | 0.25 mm | 0.31 mm | 0.38 mm | 0.45 mm | 0.61 mm | 0.80 mm | 1.25 mm |
| 848×480 | 0.23 mm | 0.30 mm | 0.38 mm | 0.47 mm | 0.57 mm | 0.68 mm | 0.92 mm | 1.21 mm | 1.88 mm |

The disparity shift does not change the error at a given distance; it moves the usable range closer, where the
error is smaller. At 0.45 m a 16 cm wide head covers ~240 depth pixels at 1280×720 (1.5 px/mm) and ~230 RGB pixels.

### Working distance per configuration

"Face distance" = median depth of the face region (eyes/cheeks plane). The nose tip is ~3–4 cm closer, the ears
~8–10 cm farther. A configuration is usable when the nose stays beyond Min-Z and the back of the head within Max-Z:

* near limit = Min-Z + 0.06 m, far limit = min(Min-Z + 0.20 m, Max-Z − 0.25 m) (beyond Min-Z + 0.20 m the
  noise grows without benefit; the 0.25 m keeps the back of the head inside Max-Z).

| Configuration | Face distance | Depth RMS at the near limit (theory) | Use |
|---|---|---|---|
| 1280×720, shift 0 | 0.57–0.71 m | 0.40 mm | works out of the box, noisier |
| **1280×720, shift 50** | **0.42–0.56 m (aim 0.47 m)** | **0.22 mm** | **recommended** |
| 1280×720, shift 100 | 0.34–0.39 m | 0.15 mm | too narrow: ears leave the range, RGB parallax and focus suffer |
| 848×480, shift 0 | 0.40–0.54 m | 0.30 mm | fallback when processing time is short; 1.5× the noise of 1280×720 at the same distance |
| 848×480, shift 50 | 0.30–0.44 m | 0.17 mm | very close; ears near Max-Z, few pixels per mm |
| 848×480, shift 100 | – | – | not usable for a whole head (Max-Z 0.42 m) |

The UI can take the range from this table, or compute it: Min-Z = fx_depth · `baseline_mm` / (126 + s) with
fx_depth from the `depth` plane's intrinsics (`send_depth`), or ≈ 0.99 × the color fx scaled to the depth width (the
D455's depth and RGB cameras have nearly the same horizontal FOV), then the limits above.

## Recommended face-scan configuration

| Setting | Value | Why |
|---|---|---|
| Depth | 1280×720 @ 30 Z16 | the native imager resolution: f is 1.5× that of 848×480, so the error at a given distance is 1/1.5 and a head has 2.3× the points |
| Color | 1280×800 @ 30 RGB8 | full global-shutter RGB (texture, Vision landmarks), the only 30 fps mode with the full FOV |
| Visual preset | High Accuracy | strict confidence thresholds: fewer outliers. TSDF fusion of many views fills holes, but outliers leave lumps; High Density remains the alternative if the measured fill rate on skin is poor |
| Disparity shift | 50 | Min-Z 0.36 m / Max-Z 1.28 m: the face can be at ~0.47 m, where the error is ~0.27 mm instead of ~0.45 mm at 0.6 m (theory); the background beyond 1.28 m drops out, which segmentation welcomes |
| Laser power | maximum (360 mW, `laser_power: null`) | skin has little natural texture; the projector pattern is what the stereo matcher sees. No saturation at ≥ 0.4 m with auto exposure |
| Emitter | on | same reason |
| Depth units | 0.0001 m | 0.1 mm quantization; with 1 mm units the rounding alone adds 0.29 mm RMS, as much as the sensor noise at 0.45 m. Range stays 6.5 m |
| Auto exposure | on for both sensors, ROI on the face | the stereo exposure follows the skin instead of the background; the RGB exposure gives a well-exposed face texture. The helper maps the face rectangle onto the depth image (distance from the app, 0.5 m when unknown) |
| Filters | threshold 0.2–1.2 m; disparity domain; spatial magnitude 2, α 0.5, δ 20, no hole filling; temporal α 0.4, δ 20, persistence 3; hole filling off (`FilterConfiguration.faceScan`) | edge-preserving smoothing in the disparity domain, where stereo noise is uniform. At 0.47 m, δ 20 (1/32 px units) is a ~2 mm step: real edges and a moving head are not smoothed. No spatial hole filling or hole-filling filter: invented surface is worse than a hole for fusion |
| Align | depth to color | the reconstruction input (`depth_aligned`, color intrinsics) |
| Decimation | never in framesets | preview downsampling happens in the app |
| IMU | on | the latest accel/gyro sample rides with each frameset (static camera check) |
| Power line frequency (RGB) | the local mains (50 Hz in Europe) | avoids exposure beating under artificial light (`set_option color "Power Line Frequency"`) |

Changing `disparity_shift` from 0 to 50 in `CaptureConfiguration.faceScanD455` is the one change to the shared
defaults this analysis asks for (reported to the integrator); confirm it with the probe (below) before shipping.

The configuration needs the camera's **advanced mode** (presets, depth units and disparity shift live there). The
helper enables it on the first `configure`: the camera restarts (3–8 s) and keeps the mode in flash from then on.

## How the helper drives the camera

`helper/src/capture_engine.cpp` (`ifs::CaptureEngine`), with `capture_config` (JSON validation),
`capture_processing` (filters, align, framesets), `capture_util` (names, calibration JSON, ROI mapping) and
`capture_measure` (probe statistics).

* **One device instance.** The engine opens the depth, RGB and motion sensors directly, not through
  `rs2::pipeline`, so the process holds exactly one device object (the patched SDK captures the camera from macOS
  once; a pipeline would create a second instance with its own USB handles). The context and device live until
  `shutdown()`, which releases every librealsense object so all handles close before the USB hand-back.
* **Threads.** Depth (+ left IR) and color feed an `rs2::syncer` (queue of 2: the oldest frameset is dropped when
  processing falls behind); a processing thread drains it and runs threshold → depth→disparity → spatial →
  temporal → disparity→depth → hole filling → align to color, then builds the FRAMESET. The motion sensor's callback
  only stores the newest accel/gyro sample. SDK threads never wait for processing or IPC (except in non-real-time
  playback, where that wait is the intended backpressure).
* **First frames.** The syncer learns the stream set from the frames, so the first color, depth and IR frames
  arrive alone; the engine pairs them again by timestamp, so a recording yields every frameset.
* **Order of `configure`** (docs/PROTOCOL.md): validate the JSON (unknown keys, types, ranges) → check the profiles
  against the device → advanced mode if needed (bounded 20 s wait for the re-enumeration through the
  devices-changed callback, query fallback) → check preset name, depth-units and laser ranges → stop the streams
  only if the profiles changed → preset → depth units (through the option, so the depth scale stays in sync) →
  disparity shift (depth table) → emitter/laser (`null` = max, snapped to the option step) → auto exposure →
  filters → restart.
* **Stream order.** RGB starts before depth and depth stops before RGB: the D4xx firmware restarts a running depth
  stream when RGB is toggled. A depth (or color) frame counter that goes backwards anyway is reported as
  `stream_restarted` (`{"stream", "previous_frame", "frame"}`) and resets the temporal filter.
* **Disconnects.** The devices-changed callback reports `device_disconnected` / `device_connected` (not for the
  intentional re-enumerations of advanced mode and hardware reset). The next command after a reconnect adopts the
  new device instance and re-applies the configuration (advanced mode persists, options do not). Streaming does not
  resume by itself.
* **Auto-exposure ROI.** RGB: the rectangle clamped to the image. Depth: the rectangle's outline deprojected at
  `distance_m` with the color intrinsics, moved by the color→depth extrinsics, projected with the depth
  intrinsics, bounding box clamped. Applied when streaming starts and on every `SET_ROI` while streaming.
* **FRAMESET.** `sequence` (helper-wide counter), `timestamp_ms` / `timestamp_domain` of the color frame,
  `depth_scale` from the depth frame, `depth_to_color` (column-major), `imu` (`null` before the first sample or
  without a motion module), `metadata` when the firmware reports it: `depth_exposure_us`, `depth_gain`,
  `laser_power`, `emitter_mode`, `depth_auto_exposure`, `depth_frame_counter`, `color_exposure_us` (the RGB
  metadata is in UVC 100 µs units and is converted), `color_gain`, `color_auto_exposure`, `color_white_balance`,
  `color_frame_counter`. Planes: `color`, `depth_aligned` (color intrinsics), optional `depth` (filtered, depth
  intrinsics), optional `ir_left` (raw Y8). Each plane references the librealsense frame's memory and keeps the
  frame alive in `Plane::keepalive`.

### Recording and playback

* librealsense 2.58 picks the format by **extension**: the reader opens `.db3` as ROS2 (sqlite) and everything else
  as ROS1 `.bag`. Built with `BUILD_ROSBAG2` (our build), the writer was always the ROS2 one, which refuses any
  path not ending in `.db3`: `.bag` recording failed. **`patches/0002-record-bag-files-with-rosbag2-enabled.patch`**
  picks the writer by extension too, so both formats record and play. Without the patch the engine reports
  "`.bag` recording needs patches/0002". `.db3` is the default.
* `start_recording(path, format)` requires the matching extension, restarts the streams with an `rs2::recorder`
  around the device (it records the raw sensor frames: depth, IR, RGB, IMU, metadata, options);
  `stop_recording()` restarts without it after the recorder's destruction has flushed and closed the file (one
  file, no sqlite side files), emits `recording_finished` with the file size, and never throws once the file is
  complete. The streams cannot change while recording (a file whose profiles change does not play back).
* Size: 1280×720 Z16 + 1280×800 RGB8 is 4.9 MB per frame, **147 MB/s (8.8 GB per minute)** uncompressed (D455
  default: librealsense does not compress D455 recordings). Measured with synthetic frames on an M3 Ultra:
  uncompressed `.db3` writes at 133 fps (4× real time); zstd compression writes at 156 fps, but its ratio on real
  images is far below the synthetic 17× and costs a core.
* Playback (`create_playback`) loads the file into the context (`load_device`, no repeat), opens the recorded
  depth/IR/color/IMU streams with the same code, plays in real time or every frame (`set_real_time(false)`:
  blocking frames, nothing is dropped), pauses/resumes, and reports `playback_finished` after the last frameset
  was delivered (then `streaming_stopped`). `start()` after the end plays the file again.

### Known limitations

* Toggling RGB while depth streams restarts depth (handled by the stream order; reported as `stream_restarted`).
* Never `kill -9` the helper during a camera command: the firmware can hang until the camera is re-plugged. The
  probe handles SIGINT/SIGTERM/SIGHUP: it stops at the next frame, shuts the engine down and prints what it has.
* Visual presets on D4xx exist only in advanced mode; enabling it restarts the camera once.
* `align_to_color = false` is rejected (framesets always carry aligned depth).

### Processing time

Per frameset (1280×720 depth + 1280×800 color, playback reader included, M3 Ultra, synthetic head; from
`capture-tests bench`):

| Filters | ms per frameset |
|---|---|
| none (align only) | 7.3 |
| + threshold | +0.4 |
| + spatial (disparity domain, magnitude 2) | +13.6 |
| + temporal (disparity domain) | +1.5 |
| + hole filling | +0.9 |
| face-scan filters (all of `faceScan`) | 22.3 (45 fps) |

The spatial filter dominates; at 30 fps there is ~1.5× headroom on one core. If a slower Mac drops framesets, use
spatial magnitude 1 before giving up resolution.

## Measurement procedure

The probe measures through the same engine and filters as the app. Run it with the camera on a USB 3 port:

```
sudo build/helper/IntelFaceScanHelper --probe --enumerate                 # device, firmware, options, profiles
sudo build/helper/IntelFaceScanHelper --probe > probe-0.50m.md            # full matrix, markdown + JSON
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:50 --minz 30   # Min-Z approach test
```

Default matrix: 1280×720 and 848×480 × High Accuracy and High Density × disparity shift 0, 50, 100, each raw
(no filters) and filtered (face-scan filters without the distance threshold), 30 warm-up frames after each change,
30 measured frames, central 30% ROI. Ctrl-C stops cleanly and prints what was measured.

1. **Target.** A flat, matte, textured wall (or a large board) — the projector pattern provides texture, but a
   bare glossy wall is not representative. Camera on a tripod, optical axis perpendicular to the wall, nothing
   moving, no sunlight on the wall. Let the camera warm up 10 minutes (depth drifts while the projector warms).
2. **Distances.** Run the full matrix at 0.40, 0.50 and 0.60 m (lens front to wall; the probe reports the median
   it measures). Configurations whose Min-Z/Max-Z exclude the distance show a low fill rate — expected, and the
   table prints the theoretical range next to the result.
3. **What the columns mean.** *Plane RMS*: RMS distance of the ROI points to their best-fit plane (spatial noise +
   wall flatness). *Subpixel RMS*: the same as disparity error (px), comparable to the 0.08 px of the theory.
   *Temporal noise*: median over ROI pixels of the per-pixel standard deviation across the frames. *Fill*: valid
   pixels in the ROI / image. *Nearest valid*: 0.5th percentile of valid depths in the image.
4. **Min-Z.** `--minz 30` with the configuration under test: start ~0.7 m from the wall, move the camera slowly
   toward it; the probe reports the nearest distance at which ≥ 90% of the ROI still has depth.
5. **Skin.** Repeat 1280×720 High Accuracy vs High Density at 0.47 m with a person's cheek filling the ROI
   (`--roi 0.1`, person still) to check the fill rate on skin.
6. Paste the markdown tables below and keep the JSON next to them; decide preset and shift from the results.

### Results (to be measured)

Camera: serial `____`, firmware `____`, calibrated fx at 1280×720 `____` px, baseline `____` mm.

| Configuration | Filters | Distance | Fill ROI | Plane RMS | Subpixel RMS | Temporal noise | RMS theory |
|---|---|---|---|---|---|---|---|
| 1280×720 HA shift 0 | raw | 0.60 m | __ % | __ mm | __ px | __ mm | 0.45 mm |
| 1280×720 HA shift 0 | filtered | 0.60 m | __ % | __ mm | __ px | __ mm | 0.45 mm |
| 1280×720 HA shift 50 | raw | 0.50 m | __ % | __ mm | __ px | __ mm | 0.31 mm |
| 1280×720 HA shift 50 | filtered | 0.50 m | __ % | __ mm | __ px | __ mm | 0.31 mm |
| 1280×720 HD shift 50 | raw | 0.50 m | __ % | __ mm | __ px | __ mm | 0.31 mm |
| 1280×720 HA shift 100 | raw | 0.40 m | __ % | __ mm | __ px | __ mm | 0.20 mm |
| 848×480 HA shift 0 | raw | 0.50 m | __ % | __ mm | __ px | __ mm | 0.47 mm |
| 848×480 HA shift 50 | raw | 0.40 m | __ % | __ mm | __ px | __ mm | 0.30 mm |

| Configuration | Min-Z theory | Min-Z measured (≥ 90% ROI fill) |
|---|---|---|
| 1280×720 shift 0 | 0.508 m (calibrated f: ~0.48 m) | __ m |
| 1280×720 shift 50 | 0.364 m | __ m |
| 848×480 shift 0 | 0.337 m | __ m |

| Skin (cheek, 0.47 m, 1280×720 shift 50) | Fill ROI | Plane RMS |
|---|---|---|
| High Accuracy | __ % | __ mm |
| High Density | __ % | __ mm |

## Testing without a camera

`helper/tests` is a standalone CMake project (the helper build can include it behind an option, see its
`CMakeLists.txt`):

```
cmake -S helper/tests -B build/helper-tests -DIFS_SDK_DIR=$PWD/build/sdk/install
cmake --build build/helper-tests -j
build/helper-tests/capture-tests              # all tests; an argument filters by name, e.g. "playback"
ctest --test-dir build/helper-tests --output-on-failure
```

* `make-test-recording` writes synthetic D455 recordings for end-to-end tests of the other modules:
  ```
  build/helper-tests/make-test-recording -o /tmp/head.db3 --frames 90 --yaw 0.5 --noise 0.0008
  build/helper-tests/make-test-recording -o /tmp/wall.db3 --scene plane --distance 0.6 --noise 0.002 --no-imu
  build/helper-tests/make-test-recording --help
  ```
  An `rs2::software_device` with the D455 sensor layout ("Stereo Module": Z16 depth + Y8 left IR, depth units and
  95 mm baseline; "RGB Camera": RGB8; "Motion Module": accel 100 Hz / gyro 200 Hz), D455-like intrinsics (depth fx
  632 at 1280×720, RGB 640.1/639.8 with inverse Brown–Conrady distortion) and extrinsics (RGB 59 mm right, 0.2°),
  metadata (depth exposure 8500 µs, laser 360, RGB exposure 8000 µs), global-time timestamps from
  1730000000000 ms, frame counters from 100 (depth) / 200 (color). The scene is ray-cast from each camera's own
  viewpoint: a textured 9 cm sphere with a nose at `--distance` in front of a wall (`--wall`), optionally turning
  (`--yaw` degrees per frame), or a tilted plane. `--noise` adds stereo-like noise (σ at 1 m, scaled by z²),
  `--restart-at N` restarts the depth counter (firmware restart), `--fast` skips the real-time pacing. `.bag` output
  needs patch 0002.
* `capture-tests` covers: configuration parsing and every rejection path, protocol names, extrinsics, ROI mapping,
  restart detection, stereo-range formulas, the probe statistics, playback end to end on `.db3` and `.bag`
  (plane order and sizes, intrinsics, extrinsics, IMU, metadata, timestamps, every frame delivered, aligned depth
  against the ray-cast scene: > 97% within 3 mm), filters reducing noise, threshold, stream-restart events,
  pause/resume/stop/replay, device info, the live code path on a software device (configure validation, options,
  streaming, `.db3`/`.bag` recording while streaming and their playback), the probe on a recording, and the
  throughput benchmark.
* The probe runs on recordings too: `IntelFaceScanHelper --probe --playback /tmp/wall.db3 --frames 10`.
