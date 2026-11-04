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
| IMU | accel 63 or 250 Hz (BMI055) / 100 or 200 Hz (BMI085, newer units), gyro 200 or 400 Hz (the helper uses the lowest rates: only the newest sample is used) |
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
  noise grows without benefit; the 0.25 m keeps the back of the head inside Max-Z). The app's distance indicator
  (`WorkingDistance`, from the camera's baseline) uses a 0.05 m near margin, so it shows 0.56–0.71 m for shift 0 and
  0.41–0.56 m for shift 50 at 1280×720.

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
  disparity shift (depth table) → emitter/laser (`null` = max, snapped to the option step; the emitter is switched
  off only after the laser power, because setting the power switches the D4xx emitter on) → auto exposure →
  filters → restart.
* **Presets re-apply themselves.** librealsense re-applies the active visual preset every time the depth sensor is
  opened (stream start), until the preset is set to Custom (which writing the depth table for a disparity shift
  does). High Accuracy and High Density keep the depth table, laser and exposure, but "Default" resets them (1 mm
  depth units, shift 0, laser 150, emitter on, auto exposure on) after the sensor read its depth units, so the
  frames would carry 0.1 mm while the camera sends 1 mm. While a preset is active the engine reads depth units,
  disparity shift, emitter, laser power and both auto exposures before opening the sensors and sets back whatever
  changed before starting them (in the configure order), so values set later with `SET_OPTION` survive as well.
* **Depth units while streaming.** A `Depth Units` change while streaming (`SET_OPTION`, or `CONFIGURE` with the same
  streams) keeps the stream profile; the processing thread then rebuilds the disparity transforms and the align
  block (both cache the depth scale per profile) and restarts the temporal filter.
* **Stream order.** RGB starts before depth and depth stops before RGB: the D4xx firmware restarts a running depth
  stream when RGB is toggled. A depth (or color) frame counter that goes backwards anyway is reported as
  `stream_restarted` (`{"stream", "previous_frame", "frame"}`) and resets the temporal filter.
* **Disconnects.** The devices-changed callback reports `device_disconnected` / `device_connected` (not for the
  intentional re-enumerations of advanced mode and hardware reset; when the camera does not come back from one of
  these within 20 s it counts as disconnected, so its later return is reported as `device_connected`). The next command after a reconnect adopts the
  new device instance and re-applies the configuration (advanced mode persists, options do not). Streaming does not
  resume by itself.
* **Auto-exposure ROI.** RGB: the rectangle clamped to the image. Depth: the rectangle's outline deprojected at
  `distance_m` with the color intrinsics, moved by the color→depth extrinsics, projected with the depth
  intrinsics, bounding box clamped. Both are then grown to at least 32×32 px about their center and kept inside the
  image (a face rectangle that mostly left the image would otherwise leave a sliver of a few pixels to meter).
  Applied when streaming starts and on every `SET_ROI` while streaming.
* **Callbacks end with `shutdown()`.** The processing thread is joined, and a devices-changed event that is being
  emitted when `shutdown()` runs is waited for; after `shutdown()` returns no callback runs.
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
  complete. A recording that the camera ended (the first command after an unplug finishes it) makes the next
  `stop_recording()` succeed once without doing anything more, so the helper still delivers the file. The streams
  cannot change while recording (a file whose profiles change does not play back).
* Size: 1280×720 Z16 + 1280×800 RGB8 is 4.9 MB per frame, **147 MB/s (8.8 GB per minute)** uncompressed (D455
  default: librealsense does not compress D455 recordings). Measured with synthetic frames on an M3 Ultra:
  uncompressed `.db3` writes at 133 fps (4× real time); zstd compression writes at 156 fps, but its ratio on real
  images is far below the synthetic 17× and costs a core.
* Playback (`create_playback`) loads the file into the context (`load_device`, no repeat), opens the recorded
  depth/IR/color/IMU streams with the same code, plays in real time or every frame (`set_real_time(false)`:
  blocking frames, nothing is dropped), pauses/resumes, and reports `playback_finished` after the last frameset
  was delivered (then `streaming_stopped`). `start()` after the end plays the file again from the beginning;
  `start()` after a `stop()` in the middle continues after the last delivered frame (librealsense's resume).
  A paused playback stays paused through `stop()`/`start()` and stream restarts until it is resumed
  (librealsense forgets its pause when the sensors stop, so the engine pauses it again at each start).

### Known limitations

* Toggling RGB while depth streams restarts depth (handled by the stream order; reported as `stream_restarted`).
* Never `kill -9` the helper during a camera command: the firmware can hang until the camera is re-plugged. The
  probe handles SIGINT/SIGTERM/SIGHUP: it stops at the next frame, shuts the engine down, prints what it has and
  hands the camera back to macOS (USB re-enumeration, waiting up to 5 s for the system drivers).
* Visual presets on D4xx exist only in advanced mode; enabling it restarts the camera once.
* `align_to_color = false` is rejected (framesets always carry aligned depth).
* Live: while the syncer learns the stream set after a start, the first frame's color, depth and IR come out as
  separate sets; when the processing thread is busy the queue of 2 can drop one of them, i.e. the first frameset
  after a start may be missing. Recordings are not affected (the recorder sits before the syncer).
* Live, "Default" preset only: the other options it rewrites at every stream start (the RGB image controls:
  brightness, contrast, gamma, hue, saturation, sharpness, white balance) return to the preset's values (see
  "Presets re-apply themselves").

### Processing time

Per frameset (1280×720 depth + 1280×800 color, playback reader included, M3 Ultra, synthetic head; from
`build/helper/tests/capture-tests bench`, which runs only when asked for by name):

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

The probe measures through the same engine and filters as the app (`helper/src/probe.cpp`, statistics in
`capture_measure.cpp`). It needs the live camera, so it runs as root; it only prints (markdown tables, then the
same as JSON) and never writes files. No other program may use the camera meanwhile (quit IntelFaceScan and the
RealSense Viewer).

**Setup.** Build the helper (`scripts/build-sdk.sh`, then `cmake -S helper -B build/helper
-DIFS_SDK_DIR=$PWD/build/sdk/install && cmake --build build/helper -j`), or use the bundled one
(`/Applications/IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper` instead of `build/helper/IntelFaceScanHelper`
below). Camera directly on a USB 3 port (the enumeration shows `usb_type` 3.x), on a tripod, optical axis
perpendicular to a flat, matte, textured wall (or a large board; a bare glossy wall is not representative), nothing
moving, no sunlight on the wall. Let the camera stream for ~10 minutes first (depth drifts while the projector warms
up), e.g. `sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:0 --raw-only --warmup 10000 >
/dev/null` twice (10000 frames = 5.5 min at 30 fps). Measure the distance from the front glass to the wall with a
tape; the probe reports what it measures as *Median distance*. The first run enables the camera's advanced mode
(the camera restarts once, 3–8 s; the probe waits).

**Commands** (from the repository root; each configuration is `WIDTHxHEIGHT:preset:shift`, preset `ha` = High
Accuracy, `hd` = High Density; each takes ~4 s: 30 warm-up frames, 30 raw frames, 10 settling frames, 30 filtered
frames):

```
# 0. The camera: serial, firmware, USB type, baseline, options, stream modes (the calibrated depth fx of each
#    resolution is in the measurement results: "fx" / "theory.focal_px")
sudo build/helper/IntelFaceScanHelper --probe --enumerate > probe-device.md

# 1. Resolution: 1280x720 vs 848x480 (High Accuracy, shift 0), wall at 0.60 m
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:0,848x480:ha:0 > probe-resolution-0.60m.md

# 2. Preset: High Accuracy vs High Density (1280x720, shift 50), wall at 0.47 m
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:50,1280x720:hd:50 > probe-preset-0.47m.md

# 3. Disparity shift 0 vs 50 (1280x720 High Accuracy): the same distance for both (0.60 m: inside both ranges,
#    the noise must be equal), then each at its own working distance (shift 50 at 0.47 m, below shift 0's Min-Z)
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:0,1280x720:ha:50 > probe-shift-0.60m.md
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:0,1280x720:ha:50 > probe-shift-0.47m.md

# 4. Everything at once (8 configurations, ~40 s), once per distance: 0.40, 0.47, 0.60 m
sudo build/helper/IntelFaceScanHelper --probe --configs \
  1280x720:ha:0,1280x720:ha:50,1280x720:hd:0,1280x720:hd:50,848x480:ha:0,848x480:ha:50,848x480:hd:0,848x480:hd:50 \
  > probe-all-0.47m.md

# 5. Min-Z of one configuration: start ~0.7 m from the wall; when stderr says "Min-Z test: move ...", move the
#    camera slowly toward the wall for 30 s (it prints distance and ROI fill once per second)
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:50 --minz 30 --raw-only > probe-minz-shift50.md

# 6. Skin: a person's cheek filling the central 10% at 0.47 m, person still
sudo build/helper/IntelFaceScanHelper --probe --configs 1280x720:ha:50,1280x720:hd:50 --roi 0.1 > probe-skin.md

# The probe hands the camera back to macOS when it ends (USB re-enumeration, as the app's helper does; Ctrl-C
# included). Only if it says the hand-back failed, or the probe was killed: release it, or unplug and replug it
sudo build/helper/IntelFaceScanHelper --release -w
```

Without `--configs` the probe runs the default matrix (1280×720 and 848×480 × High Accuracy and High Density ×
shift 0, 50, 100: 12 configurations, ~1 min). Other options: `--frames N` and `--warmup N` (default 30),
`--roi F` (central fraction of width and height, default 0.3), `--raw-only` / `--filtered-only`, `--json` (JSON
only). Progress goes to stderr, the report to stdout. Ctrl-C stops cleanly and prints what was measured so far.
`--playback FILE` measures a recording instead (development; no sudo).

**Reading the output.** For each configuration the table has a *raw* row (no filters) and a *filtered* row (the
face-scan filters without the distance threshold). Columns (JSON keys of `results[].raw` / `.filtered` /
`.theory` in brackets):

| Column | Meaning | What to look for |
|---|---|---|
| Min-Z / Max-Z (theory) | f·B / (126 + shift) and f·B / shift with the camera's calibrated fx and baseline (`theory.min_z_m`, `max_z_m`; null = ∞) | the wall distance must lie inside, otherwise fill collapses: expected, not a defect |
| Median distance | median depth in the ROI (`median_m`) | within ~2% of the tape measure (the depth origin sits a few mm behind the front glass); larger offsets are a calibration problem (on-chip calibration in the RealSense Viewer) |
| Fill ROI | valid pixels in the ROI (`fill_rate`) | ≥ 99% on a textured wall inside the range; < 95%: out of range, too little texture, or over/under-exposure |
| Fill image | valid pixels in the whole image (`full_fill_rate`) | context only (includes everything around the wall) |
| Plane RMS | RMS distance of the ROI points to their least-squares plane, mm (`plane_rms_mm`) | the spatial noise: compare with *RMS theory*; up to ~1.5× is normal, > 2× means a problem (wall not flat, glossy, tilted, laser too weak) |
| Subpixel RMS | the same error as disparity, px (`subpixel_rms`) | independent of distance and resolution, so it compares configurations directly: ~0.05–0.1 px is good (Intel's figure 0.08), > 0.15 is poor |
| Temporal noise | median over ROI pixels of each pixel's standard deviation across the frames, mm (`temporal_noise_mm`) | the frame-to-frame jitter that the temporal filter and TSDF fusion average out; usually below the plane RMS |
| Nearest valid | 0.5th percentile of all valid depths in the image (`nearest_m`) | the near end of what this configuration sees |
| RMS theory | z² · 0.08 px / (f·B) at the measured distance (`theory.rms_mm_at_median`) | the reference for *Plane RMS* |

How to decide:

1. **Resolution** (command 1): at the same distance 1280×720 should show ~1/1.5 of the 848×480 plane RMS with a
   similar subpixel RMS and fill. Keep 1280×720 unless its fill is clearly lower or its subpixel RMS clearly higher.
2. **Preset** (commands 2 and 6): High Density fills more but lets more outliers through. Keep High Accuracy when
   its fill is ≥ 99% on the wall and ≥ 95% on skin; otherwise compare the plane RMS of both on skin and take High
   Density if it is not worse by more than ~20%.
3. **Disparity shift** (command 3): at 0.60 m shift 0 and 50 must give the same plane and subpixel RMS (the shift
   moves the range, not the noise). At 0.47 m shift 0 is below its Min-Z (low fill) and shift 50 measures; its
   plane RMS there against shift 0's at 0.60 m is the gain of scanning closer (theory: 0.27 vs 0.45 mm). Command 5
   gives the real Min-Z: the face distance should stay ≥ Min-Z + 6 cm (nose), see "Working distance".
4. Fill in the tables below (from the JSON next to the markdown) and keep the probe output files with them.

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

`helper/tests` builds with the helper (`-DIFS_HELPER_TESTS=ON`, as CI does) or as a standalone CMake project:

```
cmake -S helper -B build/helper -DIFS_SDK_DIR=$PWD/build/sdk/install -DIFS_HELPER_TESTS=ON
cmake --build build/helper -j
ctest --test-dir build/helper --output-on-failure
build/helper/tests/capture-tests              # all tests; an argument filters by name, e.g. "playback"
build/helper/tests/capture-tests bench        # the throughput benchmark (only when asked for by name)

cmake -S helper/tests -B build/helper-tests -DIFS_SDK_DIR=$PWD/build/sdk/install   # standalone
cmake --build build/helper-tests -j && build/helper-tests/capture-tests
```

The tests do not depend on the machine's speed (a 3-core CI runner is several times slower than an M-series
desktop, measured by running them at background QoS with competing load): non-real-time playback delivers every
frame; real-time playback tests check only that framesets flow, stop when paused (after the queue drained) and
resume; the live tests feed the software camera one frame at a time and wait for each frameset (bounded), after a
warm-up frame per stream start.

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
* `capture-tests` covers: configuration parsing and every rejection path, protocol names, extrinsics, ROI mapping
  (direction, scale, clamping, minimum size), restart detection, stereo-range formulas, the probe statistics,
  playback end to end on `.db3` and `.bag` (plane order and sizes, intrinsics, extrinsics, IMU, metadata,
  timestamps, every frame delivered, aligned depth against the ray-cast scene: > 97% within 3 mm), filters reducing
  noise, threshold, stream-restart events, pause/resume/stop/replay (a pause survives stop/start), device info, the
  live code path on a software device (configure validation, options, streaming, `.db3`/`.bag` recording while
  streaming and their playback with every recorded frame, depth units changed while streaming), disconnect and
  reconnect events with adoption of the new device instance and no callback after `shutdown()` (a recording loaded
  into the watched context stands in for the camera), a recording ended by an unplug that RECORD_STOP still
  completes, the probe on a recording (and that it reports trying the live camera, which `main.cpp` then hands back
  to macOS), and the throughput benchmark (`bench`, on request).
* The probe runs on recordings too: `IntelFaceScanHelper --probe --playback /tmp/wall.db3 --frames 10`.
