# Helper ⇄ app protocol

The camera helper (`IntelFaceScanHelper`, C++, root for the live camera) and the app (`CameraKit`, Swift) talk over
a local `AF_UNIX`/`SOCK_STREAM` socket. Binary framing is defined in
[`Sources/IFSProtocol/include/ifs_protocol.h`](../Sources/IFSProtocol/include/ifs_protocol.h):

```
ifs_wire_header (24 bytes, little endian) | JSON (json_bytes, UTF-8) | payload (payload_bytes)
```

* `magic` = `IFS1`, `version` = 1. A receiver closes the connection on an invalid header
  (`ifs_wire_header_valid`), a JSON body that does not parse or is not an object, sizes above the limits
  (`IFS_MAX_JSON_BYTES` = 256 KiB, `IFS_MAX_PAYLOAD_BYTES` = 64 MiB), or (the helper) a body nested deeper than 32
  arrays/objects (`ifs::kMaxJsonDepth`; checked on the raw text before parsing, brackets inside strings do not
  count; the deepest command, CONFIGURE, needs 3).
* Only `IFS_CMD_RECORD_START` carries a file descriptor (`SCM_RIGHTS` on the first byte of the header,
  `IFS_FLAG_HAS_FD`). Any other message with a descriptor is a protocol error (the descriptor is closed).
* JSON keys are `snake_case`. Swift decodes them with `.convertFromSnakeCase` into the types of
  `Sources/CameraKit/CameraTypes.swift` (which must stay in sync with this document).
* Numbers: lengths in pixels, distances in meters unless the key says otherwise (`_mm`, `_us`, `_ms`).

## Connection

1. The app creates a private directory (`0700`) in its per-user temporary directory, binds `camera.sock` in it
   and listens (backlog 1).
2. The app starts the helper:
   * live: `do shell script "'<helper>' --socket '<path>' --uid <uid> --gid <gid> --parent-pid <pid>" with
     administrator privileges` (standard macOS password prompt),
   * playback: directly as the user: `<helper> --socket <path> --uid <uid> --parent-pid <pid> --playback <file>
     [--no-real-time]`,
   * fake (development and tests, no camera, no librealsense calls): directly as the user, `--fake` instead of
     `--playback <file>`; synthetic frames go through the same IPC code.
3. The helper validates the socket path (absolute; parent directory owned by `uid`, mode `0700`, not a symlink),
   connects, and checks the peer with `LOCAL_PEERCRED`/`LOCAL_PEERPID`: the peer uid must be `uid` and the peer
   pid `parent-pid`. The app accepts exactly one connection and checks the peer: euid 0 (live) or its own uid
   (playback, fake), that the peer executable (`proc_pidpath`) is the bundled helper, and for playback/fake that the
   peer pid is the process the app started.
4. The helper sends `HELLO`. Then the app sends commands; every command has a unique `request_id` and gets exactly
   one `REPLY`. `FRAMESET`, `EVENT` and `LOG` messages arrive at any time.
   * Validation: a command without an unsigned integer `request_id`, a command with a payload, a body that is not a
     JSON object, or a body nested deeper than 32 levels closes the connection. A field not listed for a command, or
     a field of the wrong type, is an error `REPLY` (`unknown field 'x'`); so is an unknown command type. At most
     1024 commands (16 MiB) may wait in the helper's queue.
   * The app must keep reading: when more than 4096 replies, events and logs (or 16 MiB of them) wait to be sent
     behind the socket buffer, the helper ends the session as if the connection had been closed (no `BYE`).
   * `CONFIGURE` and `SET_FILTERS`: every key is required except `laser_power` (`null` or absent = maximum);
     `align_to_color` must be `true`; `send_ir` requires `enable_ir`.
5. End: the app sends `QUIT` (or closes the socket, or exits: the helper watches the socket and `parent-pid`).
   The helper stops the streams, destroys the librealsense context, re-enumerates the USB device so macOS
   re-attaches its drivers (same as `rs-macos-release`), sends `BYE` if the socket is still open, and exits.
   SIGINT/SIGTERM/SIGHUP do the same. The app never needs to (and as a user cannot) kill the root helper.

## Messages

### HELLO (helper → app)
```json
{"protocol": 1, "helper_version": "0.1.0", "sdk_version": "2.58.4", "pid": 4242, "euid": 0, "mode": "live"}
```
`mode`: `"live"`, `"playback"` or `"fake"` (`DeviceInfo.source` is then `"fake"`).

### BYE (helper → app)
`{"reason": "the app asked the helper to quit"}` – last message before the helper exits; `reason` is human-readable
text (QUIT, the app exited, a received signal, or why the connection broke). Sent only while the connection is
intact.

### REPLY (helper → app)
```json
{"request_id": 7, "ok": true, "data": {}}
{"request_id": 8, "ok": false, "error": "Laser Power: value out of range [0, 360]"}
```

### LOG (helper → app)
`{"level": "info", "message": "..."}` – level `debug|info|warning|error`. The app writes these to its log.

### EVENT (helper → app)
```json
{"event": "device_disconnected", "message": "...", "data": {}}
```
Events: `device_connected`, `device_disconnected`, `streaming_started`, `streaming_stopped`,
`stream_restarted` (the firmware restarts depth when RGB is toggled during a depth stream), `recording_started`,
`recording_finished` (`data.bytes`), `playback_finished`, `error`.

`data` by event: `stream_restarted` `{stream: "depth"|"color", previous_frame, frame}`; `device_disconnected`
`{serial, message}`; `device_connected` `{serial}`; `recording_started` `{format}`; `recording_finished` `{bytes}`;
`error` `{message}`. `recording_started` and `recording_finished` are sent by the helper's IPC layer (only it knows
how many bytes reached the app's file); the capture engine's events with these names are not forwarded.

### QUERY_DEVICE → `data`: DeviceInfo
```json
{
  "name": "Intel RealSense D455", "serial": "123456789012", "firmware": "5.16.0.1",
  "recommended_firmware": "5.17.0.10", "usb_type": "3.2", "product_id": "0B5C", "product_line": "D400",
  "advanced_mode": true, "depth_scale": 0.0001, "baseline_mm": 95.0, "imu": true, "source": "live",
  "presets": ["Custom", "Default", "Hand", "High Accuracy", "High Density", "Medium Density"],
  "sensors": [
    {"name": "Stereo Module", "kind": "depth", "options": [
      {"name": "Laser Power", "value": 150, "min": 0, "max": 360, "step": 30, "default": 150, "read_only": false,
       "value_descriptions": null}]},
    {"name": "RGB Camera", "kind": "color", "options": []},
    {"name": "Motion Module", "kind": "motion", "options": []}
  ],
  "profiles": [{"stream": "depth", "index": 0, "format": "z16", "width": 1280, "height": 720, "fps": 30}]
}
```
Option names are librealsense's (`rs2_option_to_string`). `value_descriptions` (enumerated options such as
"Visual Preset") maps the integer value as a string to its description.

### CONFIGURE (app → helper): CaptureConfiguration
```json
{
  "request_id": 3,
  "depth": {"width": 1280, "height": 720, "fps": 30},
  "color": {"width": 1280, "height": 800, "fps": 30},
  "enable_ir": false, "enable_imu": true,
  "preset": "High Accuracy", "laser_power": null, "emitter_enabled": true,
  "depth_units": 0.0001, "disparity_shift": 0,
  "depth_auto_exposure": true, "color_auto_exposure": true,
  "send_depth": false, "send_ir": false, "align_to_color": true,
  "filters": { "...": "FilterConfiguration" }
}
```
Order applied by the helper: profiles → visual preset → depth table (depth units, disparity shift; advanced
mode) → laser power (`null` = max) / emitter → auto exposure → filters. Applying a preset resets the depth table,
so it always comes first. Enabling advanced mode makes the camera re-enumerate; the helper waits for it.

### SET_FILTERS (app → helper): FilterConfiguration
```json
{
  "request_id": 4,
  "threshold": {"enabled": true, "min_m": 0.2, "max_m": 1.2},
  "disparity_domain": true,
  "spatial": {"enabled": true, "magnitude": 2, "alpha": 0.5, "delta": 20, "holes_fill": 0},
  "temporal": {"enabled": true, "alpha": 0.4, "delta": 20, "persistence": 3},
  "hole_filling": {"enabled": false, "mode": 1}
}
```
Pipeline: threshold → depth→disparity → spatial → temporal → disparity→depth → hole filling → align to color.
Decimation is never applied to framesets (preview downsampling happens in the app).

### START / STOP / HARDWARE_RESET / QUIT
`{"request_id": 5}`.

### SET_OPTION
`{"request_id": 6, "sensor": "depth", "option": "Laser Power", "value": 360}` – `sensor`: `depth|color`.

### SET_ROI
`{"request_id": 9, "x": 500, "y": 220, "width": 300, "height": 380, "distance_m": 0.6}` – color pixels. The helper
sets the RGB auto-exposure ROI directly and maps the rectangle onto the depth image (deprojection at
`distance_m`, extrinsics color→depth, projection) for the stereo auto-exposure ROI.

### RECORD_START / RECORD_STOP (live only)
`RECORD_START` carries a descriptor of a file the app created (as the user, `O_WRONLY|O_CREAT|O_TRUNC`) and
`{"request_id": 10, "format": "db3"}`. The helper records into its own root-private temporary directory, and on
`RECORD_STOP` copies the finished file into the descriptor, closes it, deletes the temporary file and replies
`{"bytes": 123456789}`. The helper never opens a path received from the app. Formats: `db3` (default of
librealsense 2.58) and `bag`. A recording that ended with the camera (unplugged mid-recording: the next command
finishes it) is still delivered by the `RECORD_STOP` that follows, with its reply and `recording_finished`.

### PLAYBACK (playback only)
`{"request_id": 11, "action": "pause"}` / `{"action": "resume", "real_time": false}`.

### FRAMESET (helper → app)
JSON metadata, then the planes back to back in the payload:
```json
{
  "sequence": 1234,
  "timestamp_ms": 1730000000123.4,
  "timestamp_domain": "global_time",
  "depth_scale": 0.0001,
  "depth_to_color": {"rotation": [1,0,0, 0,1,0, 0,0,1], "translation": [-0.059, 0.0001, 0.0004]},
  "imu": {"accel": [0.1, -9.8, 0.2], "gyro": [0.001, 0.0, -0.002], "timestamp_ms": 1730000000120.0},
  "metadata": {"color_exposure_us": 8000, "depth_exposure_us": 8500, "laser_power": 360},
  "planes": [
    {"kind": "color", "format": "rgb8", "width": 1280, "height": 800, "stride": 3840,
     "offset": 0, "bytes": 3072000, "frame_number": 456, "timestamp_ms": 1730000000123.4,
     "intrinsics": {"width": 1280, "height": 800, "fx": 640.1, "fy": 639.8, "ppx": 641.2, "ppy": 401.3,
                    "model": "inverse_brown_conrady", "coeffs": [0, 0, 0, 0, 0]}},
    {"kind": "depth_aligned", "format": "z16", "width": 1280, "height": 800, "stride": 2560,
     "offset": 3072000, "bytes": 2048000, "frame_number": 789, "timestamp_ms": 1730000000121.0,
     "intrinsics": {"...": "same as color"}}
  ]
}
```
* `color` and `depth_aligned` are always present while streaming; `depth` (`send_depth`) and `ir_left`
  (`send_ir`) are optional. `imu` is `null` without a motion module.
* `metadata` may also contain `depth_gain`, `emitter_mode`, `depth_auto_exposure`, `depth_frame_counter`,
  `color_gain`, `color_auto_exposure`, `color_white_balance`, `color_frame_counter` (when the firmware reports
  them).
* `rotation` is column-major (librealsense `rs2_extrinsics`), `translation` in meters.
* Rows are `stride` bytes; Z16 is little-endian `uint16`, multiplied by `depth_scale` gives meters, 0 = no data.
* Backpressure: the helper keeps at most 2 unsent framesets. Replies, events, logs and BYE are never dropped.
  * Live camera and real-time playback: a newer frameset replaces the oldest unsent one, so an app slower than the
    camera gets the newest frames.
  * Non-real-time playback (`--no-real-time`, or `PLAYBACK` with `"real_time": false`; offline processing): nothing
    is dropped. The helper's processing thread waits until one of the 2 slots is free, so the recording plays at the
    speed the app reads it. Commands are never held up by this: while the helper runs one (STOP, QUIT, CONFIGURE,
    PLAYBACK, ... – the engine may wait for that very thread), framesets are queued without waiting (at most 8, then
    the oldest is dropped), and QUIT, socket EOF, a protocol error or a signal end the wait.
  * App side: a `PlaybackCameraSource(realTime: false)` frame stream is lossless as well – once `bufferingNewest`
    frames wait for a consumer, the reader stops reading (and so the helper waits) until that consumer takes one.
    While a command waits for its reply, which arrives behind those frames, frames are queued beyond that limit
    instead; at shutdown or when the helper exits the reader stops waiting for good. Other sources keep only the
    newest `bufferingNewest` frames per consumer.
