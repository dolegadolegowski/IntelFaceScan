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
IntelFaceScanHelper --probe ...        camera measurements (capture module, run with sudo); a live probe hands
                                       the camera back to macOS when it ends, like a session
IntelFaceScanHelper --release [-w]     hand RealSense cameras back to macOS (USB re-enumeration; -w waits 5 s for
                                       the system drivers), same as tools/release/rs-macos-release; its lines go
                                       to stderr (the app's launch of it sends them to stdout, see below)
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
   playback/fake through `Process` with an empty environment. The script runs in-process (`NSAppleScript`) in an
   AppKit application – the app always is one, and the prompt is then the app's – and through `/usr/bin/osascript`
   in any other process (a command line tool, the test runner), where an in-process `do shell script` can wait
   forever for an Apple event nobody dispatches; both report the same results.
3. The helper hardens itself (below), starts watching `--parent-pid` and its termination signals, validates the socket
   path and connects, then checks the peer: `getpeereid` uid = `--uid`, `LOCAL_PEERPID` = `--parent-pid`.
4. The app accepts one connection, stops listening (closes and unlinks the socket) and checks the peer:
   `LOCAL_PEERCRED` effective uid 0 (live) or its own uid, `LOCAL_PEERPID` → `proc_pidpath` is the same file
   (device + inode) as the helper it launched, and for playback/fake (started with `Process`, so the app knows the
   pid) that `LOCAL_PEERPID` is the process it launched. The privileged helper's pid is not known to the app (it runs
   under a root shell), so the live check stops at the executable.
5. `HELLO` (mode `live`, `playback` or `fake`), then commands. The helper creates the engine right away; if that fails
   (no camera plugged in) it reports an `error` event and retries on each command, so the app can ask again after the
   user plugged the camera in without another password prompt.
6. End: on `QUIT` (answered first), socket EOF, a protocol violation, the app no longer reading (below), the parent
   exiting, or SIGINT/SIGTERM/SIGHUP the helper ends a running recording (still delivered), calls
   `engine->shutdown()`, in live mode re-enumerates the RealSense devices and waits up to 5 s for the macOS drivers,
   sends `BYE {"reason"}` if the connection is intact, flushes for at most 2 s, and exits 0. Commands still queued
   when the connection broke are not run (their replies could not be sent, and each could take a while on the
   camera). An internal error (an exception escaping the session loop) takes the same way out and exits 70.

The USB release re-enumerates every RealSense device that has no macOS driver attached, also one that another root
process has captured (e.g. `sudo realsense-viewer` running at the same time), not only the one this helper captured.
That is deliberate: the patched librealsense (patches/0001) captures the device inside its private USB handle
(`libusb_detach_kernel_driver` in `handle_libusb`) and reports it nowhere, and nothing the helper can see stands in
for it reliably – opening the camera can fail after part of it was captured (so "the engine never opened" does not
mean "nothing captured"), the camera re-enumerates and is captured again during a session (advanced mode, hardware
reset, reconnects), and after `engine->shutdown()` a device this helper captured looks exactly like one somebody
else holds. Releasing too little leaves the camera unusable until it is re-plugged; releasing too much only disturbs
another camera program running as root at the same time, which needs the administrator password and is not a
supported setup.

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
validates messages (at most 1024 unanswered commands or 16 MiB of them); the writer thread sends one ordered queue.
Both connection threads catch every exception (e.g. `bad_alloc`) and mark the connection broken, so the session ends
through the shutdown above instead of `std::terminate` skipping the USB release. Engine callbacks (librealsense
threads) only build messages and queue them. A frameset becomes header + JSON + one `iovec` per plane pointing into
the engine's frame memory (kept alive until written; no copy); `offset`/`bytes` of each plane are filled in here. At
most **2 framesets** wait to be sent. Live camera and real-time playback: a third replaces the oldest queued one, so a
slow app gets the newest frames. Non-real-time playback (`--no-real-time`, or `PLAYBACK` with `"real_time": false`)
is lossless: the engine's processing thread waits in the frameset callback until one of the 2 slots is free, except
while the command thread is inside an engine call (`Connection::EngineCall`: the engine may be waiting for that very
thread), when up to 8 framesets are queued without waiting (docs/PROTOCOL.md, backpressure;
`HelperPlaybackBackpressureTests`). Replies, events, logs and BYE are never dropped; since the app reads all the time,
more than 4096 of them or 16 MiB waiting behind the socket buffer means it stopped reading (a peer sending commands
and never reading the replies grew the root helper by every reply), and the connection is marked broken. Socket
buffers are 4 MiB so a 5 MB frameset is not moved in 8 KiB steps.

Measured with the fake camera (M-series Mac, 1280×800 RGB8 + aligned Z16, 5.1 MB per frameset): 30 fps sustained,
median 2.5–4 ms from the helper creating a frameset to the decoded `CameraFrame` in the app, about 1 ms of app CPU
per frameset (the reader reuses one receive buffer and copies the planes out once).

Command validation is strict: every command needs an unsigned `request_id` (otherwise the connection is closed), a
body nested deeper than 32 arrays/objects closes the connection (checked on the raw text before parsing: copying a
deeply nested value recurses once per level and would overflow the stack),
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
* `HelperServer.swift`, `HelperLauncher.swift` – see the session above. `HelperProcessHandle` reports the exit of a
  launched helper (status, launch error, stderr) and, for playback/fake, its pid for the peer check.
* `HelperSession.swift` – shared by `LiveCameraSource`, `PlaybackCameraSource` and `FakeCameraSource`: connect
  (single flight; a failed device query keeps the helper for the next attempt), commands with timeouts (CONFIGURE 60 s,
  RECORD_STOP 600 s, others 10–30 s), fan-out of frames (`AsyncStream`, `.bufferingNewest(n)` per consumer; lossless
  for a non-real-time playback: the reader waits while a consumer has `n` frames, except while a reply is due) and
  events (256 per consumer), and shutdown: QUIT → BYE/EOF → exit, bounded by a timeout (20 s live). A user helper that
  does not exit gets SIGTERM, then SIGKILL; the root helper cannot be signalled by the app and exits on its own once
  the socket is closed. A helper that never connected is not waited for: closing the source while the administrator
  prompt is still showing returns at once (the live handle only ends when the prompt is answered), the socket is
  removed so a helper started after all cannot connect and exits, and a user helper is terminated right away.
* `HelperLocator.swift` – `CameraSources.locateHelper()`: the bundle, then (development) `$IFS_HELPER_PATH`,
  `../Helpers` next to the executable, or `build/helper/IntelFaceScanHelper` above it. The search above the
  executable can leave the repository, so a candidate there is only taken when it and every folder above it belong to
  root or the user and nobody else can write to them (sticky folders such as `/Users/Shared`, and the wheel/admin
  groups, excepted): otherwise any local user could plant `/Users/Shared/build/helper/IntelFaceScanHelper` for a
  checkout below `/Users/Shared` and have it run as root after the password prompt. Sources created with the
  default `bundledHelperURL` fall back to it when there is no bundle (`swift run`).

Frame and event streams end at `shutdown()`. When the helper ends unexpectedly the streams stay open, the events
`.error(...)` and `.helperExited(status:)` are sent, and `connect()` can start a new helper.

Quitting the app: nothing in the session needs the main thread, but the reply to AppKit must not depend on the main
actor either. `Task { await camera.shutdown(); NSApp.reply(toApplicationShouldTerminate: true) }` in a `@MainActor`
delegate hangs: the task inherits the main actor, and when the terminate request itself came from a main-queue block
or a main-actor task (a menu action wrapped in a `Task`, a signal handler that dispatches to the main queue), AppKit
waits for the reply in a modal run loop mode while that block still holds the main queue, so the task never runs.
The app (`Sources/IntelFaceScanApp/IntelFaceScanApp.swift`) does this instead:

```swift
func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
    if terminationPending { return .terminateCancel }       // a reply is already on its way
    guard let work = model.beginTermination() else { return .terminateNow }   // starts closing the source, returns at once
    terminationPending = true
    Task.detached {                                         // waits off the main actor (TerminationWork.wait)
        _ = await work.wait()                               // camera shutdown and file writes, bounded
        // performSelector(onMainThread:with:waitUntilDone:modes: [.common]): the common modes include the modal
        // mode AppKit waits in, and the main dispatch queue is not involved.
        MainRunLoop.perform { NSApp.reply(toApplicationShouldTerminate: true) }
    }
    return .terminateLater
}
```

A second terminate while the reply is pending (another Cmd-Q) makes AppKit end the process at once; a second
SIGTERM/SIGINT/SIGHUP ends it with `_exit` (`TerminationSignals`). Either way the root helper still releases the
camera when it sees the app exit. Where only synchronous code is possible, `camera.shutdownAndWait(timeout:)` blocks
on the session's own threads.

`LiveCameraSource.releaseCameraToSystem()` runs `IntelFaceScanHelper --release -w` with the administrator prompt –
a recovery action for a camera left captured by a helper that was killed – and returns the tool's lines (its stderr
is sent to stdout for this launch: `do shell script` returns stdout, and reports it as the error message when stderr
is empty).

Live mode needs no particular thread, but the in-process administrator prompt needs an AppKit application whose main
run loop runs (as in the app); other processes go through `/usr/bin/osascript` (session step 2). An `NSApplication`
that was created but never run is not supported.

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
| The root helper connects to an attacker's socket | path must be absolute and canonical; every component is opened with `O_NOFOLLOW` from `/`; the final directory must be owned by `--uid` with mode exactly 0700; the socket itself must be a socket owned by `--uid`; the connection is made relative to the verified directory descriptor (`fchdir` + relative `sun_path`, no TOCTOU on the directory); `connect()` resolves the name once more and would follow a link swapped in by the directory's owner, so the name is checked again afterwards (same device and inode, or already removed by the app) – that only narrows the window; the real defense is the peer check: peer uid = `--uid` and peer pid = `--parent-pid` |
| Something else connects to the app | the socket directory is private to the user; one connection is accepted and listening stops; peer uid (0 for live) and executable identity (`proc_pidpath`, device + inode) are checked, and for playback/fake the pid of the launched process (a same-user process that connected first and then executed the real helper would pass the other checks) |
| Hostile messages | header validated by the shared C code (magic, version, flags, 256 KiB JSON, 64 MiB payload); commands may not carry payloads; JSON must be an object nested at most 32 levels deep; unknown fields and wrong types are errors; at most 1024 queued commands (16 MiB); a peer that does not read its replies is disconnected once 4096 messages or 16 MiB wait to be sent; every exception of a command becomes its error reply, nothing thrown in the IPC layer reaches a librealsense thread, and an exception in a connection thread ends the session through the normal shutdown |
| File system access as root | the helper never opens, creates or deletes a path from the app. A recording goes to `mkdtemp` inside root's own Darwin temporary directory (`/var/folders/zz/…/T`, verified root-owned and not accessible to others; fallback `/private/var/run/IntelFaceScanHelper`), and is copied into the descriptor the app opened as itself (`SCM_RIGHTS`, only with `RECORD_START`, only on its first byte, must be a regular file without set-user-ID/set-group-ID bits (root writing does not clear them, unlike any other writer) open for writing; any other descriptor is closed and the connection dropped), then the spool is removed without following links; spools left by a helper that died mid-recording (untouched for a day) are removed when the next recording starts |
| Environment and inherited state | the app starts the helper with `env -i` (no `DYLD_*` or other variables reach root); the helper still clears its environment to a fixed `PATH`/`HOME`, sets `umask 077`, closes every inherited descriptor above 2 (from the kernel's list of its descriptors, so also those above a lowered `RLIMIT_NOFILE`) and points stdin at `/dev/null`; no `system()`, no shell, no `exec` |
| Unprivileged modes used to gain or misuse privileges | playback and fake refuse to run as root and require `--uid` to be the caller |
| Helper left running as root | it exits when the app exits (kqueue on the parent pid), when the socket closes, on QUIT and on SIGINT/SIGTERM/SIGHUP (the signals are ignored and read from the kqueue, so no work happens in a signal handler and the default action cannot skip the USB release) |
| Camera left unusable | USB re-enumeration on every exit path except SIGKILL, and after a live `--probe`; recovery with `--release` / `rs-macos-release`; patched librealsense bounds USB waits |
| Logs leaking into the user's home as root | logs only go over the socket (`LOG`), stderr only before the connection exists |

## Testing

`Tests/CameraKitTests` runs the real helper binary in `--fake` mode (`build/helper/IntelFaceScanHelper`, or
`$IFS_HELPER_PATH`, e.g. the bundled one; skipped with a message when absent): full sessions through
`FakeCameraSource`, padded rows, the recording hand-over, lost connections, hardware reset, the socket path and peer
checks on both sides, strict command validation, protocol violations, backpressure (live/fake: a stalled reader
loses old framesets but no reply; `HelperPlaybackBackpressureTests`: a non-real-time playback loses nothing, and
stop, QUIT, pause, EOF and signals stay prompt while the helper waits for room), a recording ended by the camera that
RECORD_STOP still delivers, and throughput. `HelperSessionLifecycleTests` cover helpers that end right after HELLO,
one that never connects (closing does not wait for it) and one the app did not start (rejected by pid).
`HelperSecurityTests` attack the boundary: deeply nested bodies, a peer flooding commands without reading the
replies, set-id recording destinations, integer overflow in commands, a descriptor inherited above the soft limit,
the administrator command's quoting run through `osascript` and `sh` (quotes, backslashes, newlines, shell syntax,
emoji), the release tool's output and the osascript launch path (results, a cancelled prompt), and where a helper
may be taken from. The codec and the PROTOCOL.md JSON have unit tests of their own. The live mode
needs a camera and the password; it shares everything except `create_live` and the USB release with the fake mode.
