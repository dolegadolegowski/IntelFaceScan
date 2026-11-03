# Camera helper

`IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper` is the only process that talks to the RealSense camera.
macOS attaches its own UVC/HID drivers to the D455, and librealsense (RSUSB backend) can only take the USB device over
from them as root. So the helper runs as root for the live camera, started with the standard administrator prompt,
and the app never does. The wire protocol is in [PROTOCOL.md](PROTOCOL.md); this document covers the helper process,
the app side in `Sources/CameraKit/Helper`, and the threat model.

> 🇵🇱 **Po polsku (skrót):** helper to mały proces C++, jedyny, który używa librealsense. Do kamery na żywo działa jako
> root (macOS pozwala tylko rootowi przejąć urządzenie USB od systemowego sterownika), uruchamiany standardowym oknem
> hasła administratora; aplikacja zawsze działa jako użytkownik. Helper łączy się z gniazdem aplikacji tylko po
> sprawdzeniu ścieżki (bezwzględna, bez dowiązań symbolicznych, katalog użytkownika z prawami 0700) i procesu po drugiej
> stronie (uid i pid aplikacji), ściśle waliduje każdą wiadomość, nigdy nie otwiera ścieżek podanych przez aplikację
> (nagrania trafiają do prywatnego katalogu roota i są kopiowane do deskryptora przekazanego przez `SCM_RIGHTS`), ma
> minimalne środowisko i nie uruchamia powłoki. Po zakończeniu (QUIT, zamknięcie gniazda, wyjście aplikacji, SIGINT/
> SIGTERM/SIGHUP) zatrzymuje strumienie, oddaje kamerę systemowi (re-enumeracja USB) i kończy pracę. Tryb `--fake`
> generuje syntetyczne klatki bez kamery i bez uprawnień – do testów IPC i rozwoju interfejsu.

## Command line

```
IntelFaceScanHelper --socket <path> --uid <uid> --gid <gid> --parent-pid <pid>              live camera (root)
IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> [--no-real-time]
IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake [--fake-row-padding <bytes>]
IntelFaceScanHelper --probe ...        camera measurements (capture module, run with sudo)
IntelFaceScanHelper --release [-w]     hand RealSense cameras back to macOS (USB re-enumeration; -w waits 5 s for
                                       the system drivers), same as tools/release/rs-macos-release
IntelFaceScanHelper --help | --version
```

| Mode | Privileges | Engine |
|---|---|---|
| live | requires euid 0, refuses otherwise | `CaptureEngine::create_live` (librealsense) |
| playback | refuses root; `--uid` must be the caller | `CaptureEngine::create_playback` |
| fake | refuses root; `--uid` must be the caller | `create_fake_engine` (`helper/src/fake_engine.h`, no librealsense calls, no USB) |

Exit status: `0` session ended normally (whatever ended it); `64` bad command line; `69` the socket path failed
validation or the connection failed, or the app was already gone; `70` internal error; `77` wrong privileges for the
mode, `--uid` not the caller, or the peer is not the app. Before the connection exists the reason goes to stderr (the
app shows it: `do shell script` reports it as the error message); afterwards everything goes over the socket.

## Session

1. The app (`HelperSocketServer`) creates `<realpath of the user's temporary directory>/IntelFaceScan.XXXXXX/` (mode
   0700), binds `camera.sock` in it (0600) and listens with backlog 1.
2. The app starts the helper: live through `do shell script "exec /usr/bin/env -i '<helper>' --socket ... </dev/null
   >/dev/null" with administrator privileges with prompt "..."` on its own queue (it blocks for the helper's whole
   life and reports its exit status; a cancelled prompt is error -128 → `CameraError.authorizationCancelled`);
   playback/fake through `Process` with an empty environment.
3. The helper hardens itself (below), starts watching `--parent-pid` and its termination signals, validates the socket
   path and connects, then checks the peer: `getpeereid` uid = `--uid`, `LOCAL_PEERPID` = `--parent-pid`.
4. The app accepts one connection, stops listening (closes and unlinks the socket) and checks the peer:
   `LOCAL_PEERCRED` effective uid 0 (live) or its own uid, and `LOCAL_PEERPID` → `proc_pidpath` is the same file
   (device + inode) as the helper it launched.
5. `HELLO` (mode `live`, `playback` or `fake`), then commands. The helper creates the engine right away; if that fails
   (no camera plugged in) it reports an `error` event and retries on each command, so the app can ask again after the
   user plugged the camera in without another password prompt.
6. End: on `QUIT` (answered first), socket EOF, a protocol violation, the parent exiting, or SIGINT/SIGTERM/SIGHUP the
   helper ends a running recording (still delivered), calls `engine->shutdown()`, in live mode re-enumerates the
   RealSense devices and waits up to 5 s for the macOS drivers, sends `BYE {"reason"}` if the connection is intact,
   flushes for at most 2 s, and exits 0.

Waits are bounded except camera commands themselves: a command in progress (advanced mode re-enumeration, a recording
copy) is never interrupted, because killing the process in the middle of a camera command can hang the firmware until
the camera is re-plugged. The termination check runs between commands.

### Helper internals (`helper/src/`)

| File | Role |
|---|---|
| `main.cpp` | argument parsing, privilege checks per mode, the session loop and the shutdown sequence |
| `ipc_socket.*` | socket path validation and connect, peer check |
| `ipc_wire.*` | framing: one validated message per read, gathered writes, `UniqueFd` |
| `ipc_connection.*` | reader and writer threads, the outgoing queue, the incoming command queue |
| `ipc_lifecycle.*` | one kqueue for the parent (`EVFILT_PROC NOTE_EXIT`), signals (`EVFILT_SIGNAL`) and wake-ups (`EVFILT_USER`) |
| `ipc_service.*` | command validation and dispatch to the `CaptureEngine`, engine callbacks → `FRAMESET`/`EVENT`/`LOG`, recording hand-over |
| `security_process.*` | umask, descriptors, environment |
| `security_recording.*` | root-private spool directory, destination checks, copy into the app's descriptor |
| `usb_release.*` | IOKit re-enumeration (port of `tools/release/rs-macos-release.c`, vendors 0x8086 and 0x38E5) |
| `fake_engine.*` | the synthetic camera |

Threads: the main thread waits on the kqueue and runs every command, one at a time; the reader thread receives and
validates messages (at most 64 unanswered commands); the writer thread sends one ordered queue. Engine callbacks
(librealsense threads) only build messages and queue them. A frameset becomes header + JSON + one `iovec` per plane
pointing into the engine's frame memory (kept alive until written; no copy); `offset`/`bytes` of each plane are filled
in here. At most **2 framesets** wait to be sent: a third replaces the oldest queued one. Replies, events, logs and
BYE are never dropped. Socket buffers are 4 MiB so a 5 MB frameset is not moved in 8 KiB steps.

Command validation is strict: every command needs an unsigned `request_id` (otherwise the connection is closed),
fields other than the documented ones are an error reply (`unknown field 'x'`), types are checked (`x` of SET_ROI must
be an integer), unknown command types get an error reply, and each command gets exactly one REPLY. `CONFIGURE` and
`SET_FILTERS` pass the object without `request_id` to the engine, which validates the content.

`recording_started` / `recording_finished` events are emitted by the IPC layer (only it knows how many bytes reached
the app's file); engine events with these names are suppressed.

### Fake camera

`--fake` runs the real IPC code with `fake_engine.cpp`: 1280×800 RGB8 at 30 fps (or what `CONFIGURE` asks for, up to
2048×2048), red = x + 3n, green = y + 2n, blue = (x xor y) + n (mod 256, n = color frame number), with a Lambert-shaded
sphere (r = 0.1 m) drifting around (0, 0, 0.45 m) whose depth is the aligned Z16 plane (0 elsewhere; the threshold
filter applies). Intrinsics (90° HFOV color, 87° depth), extrinsics (−59 mm), IMU and metadata are D455-like; options
and presets behave like the device (range errors included); `send_depth`/`send_ir` add the optional planes; recording
writes an `IFSFAKE1` file; hardware reset emits `device_disconnected` + `device_connected`. `--fake-row-padding`
adds bytes to every color row to exercise stride handling. `FakeCameraSource` uses it (kind `.live`, so UI code
written against it works with the camera unchanged).

## App side (`Sources/CameraKit/Helper`)

* `HelperWire.swift` – header via the shared C functions, blocking message reads (descriptors rejected), writes with
  `SCM_RIGHTS`.
* `HelperJSON.swift` / `HelperFrameDecoder.swift` – envelopes with explicit snake_case keys; the shared types with
  `.convertFromSnakeCase` (which would otherwise rewrite free-form keys such as `color_exposure_us`). FRAMESET planes
  are bounds-checked against the payload, copied out and repacked to tight rows; color and aligned depth must be
  present and of equal size. `CONFIGURE` always sends `laser_power` (`null` = maximum).
* `HelperConnection.swift` – reader thread; requests complete exactly once (reply, timeout, task cancellation,
  connection loss); a malformed message closes the connection, a malformed frameset is dropped with a log event.
* `HelperServer.swift`, `HelperLauncher.swift` – see the session above.
* `HelperSession.swift` – shared by `LiveCameraSource`, `PlaybackCameraSource` and `FakeCameraSource`: connect
  (single flight; a failed device query keeps the helper for the next attempt), commands with timeouts (CONFIGURE 60 s,
  RECORD_STOP 600 s, others 10–30 s), fan-out of frames (`AsyncStream`, `.bufferingNewest(n)` per consumer) and events
  (256 per consumer), and shutdown: QUIT → BYE/EOF → exit, bounded by a timeout (20 s live). A user helper that does
  not exit gets SIGTERM, then SIGKILL; the root helper cannot be signalled by the app and exits on its own once the
  socket is closed.
* `HelperLocator.swift` – `CameraSources.locateHelper()`: the bundle, then (development) `$IFS_HELPER_PATH`,
  `../Helpers` next to the executable, or `build/helper/IntelFaceScanHelper` above it. Sources created with the
  default `bundledHelperURL` fall back to it when there is no bundle (`swift run`).

Frame and event streams end at `shutdown()`. When the helper ends unexpectedly the streams stay open, the events
`.error(...)` and `.helperExited(status:)` are sent, and `connect()` can start a new helper.

Quitting the app: nothing in the session needs the main thread, so

```swift
func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
    Task {
        await camera.shutdown()   // the camera is handed back to macOS before the app exits
        NSApp.reply(toApplicationShouldTerminate: true)
    }
    return .terminateLater
}
```

or, where only synchronous code is possible, `camera.shutdownAndWait(timeout:)`.

`LiveCameraSource.releaseCameraToSystem()` runs `IntelFaceScanHelper --release -w` with the administrator prompt –
a recovery action for a camera left captured by a helper that was killed.

## Threat model

**Asset:** root on the user's Mac. The helper runs as root for as long as the live camera is used and accepts input
from an unprivileged process.

**Adversaries considered:** other local users and their processes; processes of the same user trying to reach root
through the helper without the password; a compromised or confused app sending malformed or hostile messages;
symlink and race tricks in shared directories.

**Out of scope:** a process of the same user that can modify the app bundle (it is owned by the user who installed it):
it could replace the helper before the next prompt, as with any app that asks for the administrator password, and the
same user could equally show a fake password dialog. Installing to `/Applications` from an admin account does not
change ownership of the copied files. Hardening this needs a Developer ID signature plus a privileged helper installed
with `SMAppService` and code-signing requirements on both sides – planned for a signed release.

**Mitigations:**

| Threat | Mitigation |
|---|---|
| The root helper connects to an attacker's socket | path must be absolute and canonical; every component is opened with `O_NOFOLLOW` from `/`; the final directory must be owned by `--uid` with mode exactly 0700; the socket itself must be a socket owned by `--uid`; the connection is made relative to the verified directory descriptor (`fchdir` + relative `sun_path`, no TOCTOU on the path); then peer uid = `--uid` and peer pid = `--parent-pid` |
| Something else connects to the app | the socket directory is private to the user; one connection is accepted and listening stops; peer uid (0 for live) and executable identity (`proc_pidpath`, device + inode) are checked |
| Hostile messages | header validated by the shared C code (magic, version, flags, 256 KiB JSON, 64 MiB payload); commands may not carry payloads; JSON must be an object; unknown fields and wrong types are errors; at most 64 queued commands; no exceptions escape (C++ `noexcept` paths, error replies) |
| File system access as root | the helper never opens, creates or deletes a path from the app. A recording goes to `mkdtemp` inside root's own Darwin temporary directory (`/var/folders/zz/…/T`, verified root-owned and not accessible to others; fallback `/private/var/run/IntelFaceScanHelper`), and is copied into the descriptor the app opened as itself (`SCM_RIGHTS`, only with `RECORD_START`, only on its first byte, must be a regular file open for writing; any other descriptor is closed and the connection dropped), then the spool is removed without following links |
| Environment and inherited state | the app starts the helper with `env -i` (no `DYLD_*` or other variables reach root); the helper still clears its environment to a fixed `PATH`/`HOME`, sets `umask 077`, closes every inherited descriptor above 2 and points stdin at `/dev/null`; no `system()`, no shell, no `exec` |
| Unprivileged modes used to gain or misuse privileges | playback and fake refuse to run as root and require `--uid` to be the caller |
| Helper left running as root | it exits when the app exits (kqueue on the parent pid), when the socket closes, on QUIT and on SIGINT/SIGTERM/SIGHUP (the signals are ignored and read from the kqueue, so no work happens in a signal handler and the default action cannot skip the USB release) |
| Camera left unusable | USB re-enumeration on every exit path except SIGKILL; recovery with `--release` / `rs-macos-release`; patched librealsense bounds USB waits |
| Logs leaking into the user's home as root | logs only go over the socket (`LOG`), stderr only before the connection exists |

## Testing

`Tests/CameraKitTests` runs the real helper binary in `--fake` mode (`build/helper/IntelFaceScanHelper`, or
`$IFS_HELPER_PATH`, e.g. the bundled one; skipped with a message when absent): full sessions through
`FakeCameraSource`, padded rows, the recording hand-over, lost connections, hardware reset, the socket path and peer
checks on both sides, strict command validation, protocol violations, backpressure (a stalled reader loses old
framesets but no reply) and throughput. The codec and the PROTOCOL.md JSON have unit tests of their own. The live mode
needs a camera and the password; it shares everything except `create_live` and the USB release with the fake mode.
