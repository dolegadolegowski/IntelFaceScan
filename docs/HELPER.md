# Camera helper

`IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper` is the only process of IntelFaceScan that uses librealsense
and talks to the RealSense camera. macOS attaches its own UVC/HID drivers to the D455, and only root can take the USB
device over from them. The wire protocol is in [PROTOCOL.md](PROTOCOL.md); this document covers the helper process,
the app side in `Sources/CameraKit/Helper`, and the threat model.

## Two ways to run the live camera

| | Through the camera service (the normal way) | With the administrator password (one session) |
|---|---|---|
| When | the RealSense camera service is installed and admits the user ([CAMERA-SERVICE.md](CAMERA-SERVICE.md)) | the service is not usable and the user chose *Use administrator password this time*, or the build carries no service files |
| The helper runs as | the user, started directly by the app (`Process`) | root, started through the macOS administrator prompt |
| Who takes the USB device over | `realsense-usbd`, the shared root LaunchDaemon; the helper's librealsense asks it over XPC | the helper's librealsense, in the helper process |
| Password | none (an administrator's name and password once, when the service is installed) | at every camera session |
| Camera back to macOS | the service, a few seconds after its last client let go; the helper never re-enumerates USB | the helper re-enumerates the USB device when the session ends, unless started with `--no-usb-release` |
| The app checks the peer | its own uid, and the pid of the process it started | uid 0; the pid is not known to the app |
| `LiveCameraSource.Access` | `.service` (or `.automatic` when the service is ready) | `.administrator(skipUSBRelease:)` |

The helper binary, the socket, the protocol and everything from `HELLO` on are the same; `HELLO` reports `mode`
`"live"` in both and the helper's `euid`. The app (`AppModel.openSource`) decides which way to go from the state of
the service; `--no-usb-release` is passed when the service is alive but does not admit this user (not the user at the
Mac's screen): root's librealsense then goes through the service as well, the service hands the camera back, and a
re-enumeration by the helper would pull the camera away from it.

> 🇵🇱 **Po polsku (skrót):** helper to mały proces C++, jedyny w IntelFaceScan, który używa librealsense. Kamera na
> żywo działa na dwa sposoby. Zwykle helper działa jako użytkownik, uruchamiany bezpośrednio przez aplikację,
> a jego librealsense sięga do kamery przez wspólną usługę kamery RealSense (`realsense-usbd`, demon roota
> instalowany raz; [CAMERA-SERVICE.md](CAMERA-SERVICE.md)): bez hasła, a kamerę oddaje systemowi usługa, kilka
> sekund po tym, jak ostatni program przestał jej używać. Gdy z usługi nie da się skorzystać, helper działa przez
> jedną sesję jako root (macOS pozwala tylko rootowi przejąć urządzenie USB od systemowego sterownika),
> uruchamiany standardowym oknem hasła administratora, i po zakończeniu sam oddaje kamerę systemowi (re-enumeracja
> USB; pomijana z `--no-usb-release`, gdy kamerę oddaje usługa). Aplikacja zawsze działa jako użytkownik. Helper
> łączy się z gniazdem aplikacji tylko po sprawdzeniu ścieżki (bezwzględna, bez dowiązań symbolicznych, katalog
> użytkownika z prawami 0700) i procesu po drugiej stronie (uid i pid aplikacji), ściśle waliduje każdą wiadomość,
> nigdy nie otwiera ścieżek podanych przez aplikację (nagrania trafiają do prywatnego katalogu i są kopiowane do
> deskryptora przekazanego przez `SCM_RIGHTS`), ma minimalne środowisko i nie uruchamia powłoki. Kończy pracę po
> QUIT, zamknięciu gniazda, wyjściu aplikacji albo SIGINT/SIGTERM/SIGHUP, zatrzymując wcześniej strumienie. Tryb
> `--fake` generuje syntetyczne klatki bez kamery i bez uprawnień – do testów IPC i rozwoju interfejsu.

## Command line

```
IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid>                          live camera, as the user
                                                                                           (camera service)
IntelFaceScanHelper --socket <path> --uid <uid> --gid <gid> --parent-pid <pid> [--no-usb-release]
                                                                                           live camera, as root
IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --playback <file> [--no-real-time]
IntelFaceScanHelper --socket <path> --uid <uid> --parent-pid <pid> --fake [--fake-row-padding <bytes>]
IntelFaceScanHelper --probe ...        camera measurements (capture module): as the user when the camera service
                                       is ready, otherwise with sudo; a live probe run as root hands the camera
                                       back to macOS when it ends, like a session
IntelFaceScanHelper --release [-w]     hand RealSense cameras back to macOS (root; USB re-enumeration; -w waits 5 s
                                       for the system drivers), same as tools/release/rs-macos-release; its lines
                                       go to stderr (the app's launch of it sends them to stdout, see below)
IntelFaceScanHelper --help | --version
```

| Mode | Privileges | Engine |
|---|---|---|
| live | the user (`--uid` must be the caller; `--gid` is accepted and ignored) or root | `CaptureEngine::create_live` (librealsense; through the camera service, or the USB device itself as root) |
| playback | refuses root; `--uid` must be the caller | `CaptureEngine::create_playback` |
| fake | refuses root; `--uid` must be the caller | `create_fake_engine` (`helper/src/fake_engine.h`, no librealsense calls, no USB) |

`--no-usb-release` is for the live camera only (a usage error in the other modes): the root helper skips the USB
re-enumeration at the end of the session.

Exit status: `0` session ended normally (whatever ended it); `64` bad command line; `69` the socket path failed
validation or the connection failed, or the app was already gone; `70` internal error; `77` wrong privileges for the
mode, `--uid` not the caller, or the peer is not the app. Before the connection exists the reason goes to stderr (the
app shows it: `do shell script` reports it as the error message); afterwards everything goes over the socket.

## Session

1. The app (`HelperSocketServer`) creates `<realpath of the user's temporary directory>/IntelFaceScan.XXXXXX/` (mode
   0700), binds `camera.sock` in it (0600) and listens with backlog 1.
2. The app starts the helper (`HelperLaunchPlan` holds the arguments and the checks of each way): playback, fake and
   live through the camera service with `Process` and an empty environment; live on the administrator path through
   `do shell script "exec /usr/bin/env -i '<helper>' --socket ... </dev/null >/dev/null" with administrator
   privileges with prompt "..."` on its own queue (it blocks for the helper's whole life and reports its exit
   status; a cancelled prompt is error -128 → `CameraError.authorizationCancelled`). The script runs in-process
   (`NSAppleScript`) in an AppKit application – the app always is one, and the prompt is then the app's – and
   through `/usr/bin/osascript` in any other process (a command line tool, the test runner), where an in-process
   `do shell script` can wait forever for an Apple event nobody dispatches; both report the same results. The app
   waits 20 s for a helper it started directly to connect, and 10 minutes on the administrator path (the prompt
   waits for the user).
3. The helper hardens itself (below), starts watching `--parent-pid` and its termination signals, validates the socket
   path and connects, then checks the peer: `getpeereid` uid = `--uid`, `LOCAL_PEERPID` = `--parent-pid`.
4. The app accepts one connection, stops listening (closes and unlinks the socket) and checks the peer:
   `LOCAL_PEERCRED` effective uid 0 (the administrator path) or its own uid, `LOCAL_PEERPID` → `proc_pidpath` is the
   same file (device + inode) as the helper it launched, and for every helper it started with `Process` (playback,
   fake, live through the camera service), whose pid it therefore knows, that `LOCAL_PEERPID` is that process. The
   privileged helper's pid is not known to the app (it runs under a root shell), so that check stops at the
   executable.
5. `HELLO` (mode `live`, `playback` or `fake`), then commands. The helper creates the engine right away; if that fails
   (no camera plugged in) it reports an `error` event and retries on each command, so the app can ask again after the
   user plugged the camera in, without a new helper (and so without another password prompt on the administrator
   path).
6. End: on `QUIT` (answered first), socket EOF, a protocol violation, the app no longer reading (below), the parent
   exiting, or SIGINT/SIGTERM/SIGHUP the helper ends a running recording (still delivered), calls
   `engine->shutdown()`, as root in live mode (and without `--no-usb-release`) re-enumerates the RealSense devices
   and waits up to 5 s for the macOS drivers, sends `BYE {"reason"}` if the connection is intact, flushes for at most
   2 s, and exits 0. Commands still queued when the connection broke are not run (their replies could not be sent,
   and each could take a while on the camera). An internal error (an exception escaping the session loop) takes the
   same way out and exits 70. A live helper of the user has nothing to hand back: the camera service notices that its
   client is gone and gives the camera back to macOS itself a few seconds later.

**When the USB release runs** (`helper/src/main.cpp`): at the end of a live session of a helper that runs as root
and was started without `--no-usb-release`, on every way out except SIGKILL; and after a live `--probe` run as
root, which has no such flag. It is skipped by every helper of the user (playback, fake, and live through the camera
service: the service hands the camera back, and only root can re-enumerate a device) and by a root helper started
with `--no-usb-release`, which logs that the camera service hands the camera back. The app passes the flag in one
case, when the service is alive and only refuses this user (*not the user at the screen*). Every other
administrator session releases: service not installed, too old, not answering or switched off, or a build without
service files.

The release re-enumerates every RealSense device that has no macOS driver attached, also one that another root
process has captured (e.g. `sudo realsense-viewer` running at the same time) and one that the camera service holds
for another program, not only the one this helper captured.
That is deliberate: the patched librealsense (patches/0001) captures the device inside its private USB handle
(`libusb_detach_kernel_driver` in `handle_libusb`) and reports it nowhere, and nothing the helper can see stands in
for it reliably – opening the camera can fail after part of it was captured (so "the engine never opened" does not
mean "nothing captured"), the camera re-enumerates and is captured again during a session (advanced mode, hardware
reset, reconnects), and after `engine->shutdown()` a device this helper captured looks exactly like one somebody
else holds. Releasing too little leaves the camera unusable until it is re-plugged; releasing too much ends the
session of another program that has a RealSense camera open at that moment, as root or through the camera service:
its camera re-enumerates under it. With the service installed that is no longer only an unsupported `sudo` setup.
It takes a root helper without the flag that ends while the service holds a camera for another program: for
example an administrator's `AllowedCDHashes` list admits RealSense Viewer but not IntelFaceScan, the Viewer is
streaming, and the user chose *Use administrator password this time*.

**Limitation.** The root helper decides from its effective uid and the app's flag, not from whether its
librealsense went through the service (librealsense makes that choice once per process, at its first device query,
and the helper does not look at it). The flag comes from the app's state check, made before the administrator
prompt, which can wait for minutes. In the rare case that the service starts answering between that check and the
start of a root helper without the flag (another application installed it, an administrator switched it on),
root's librealsense uses the service, and the release at the end re-enumerates what the service holds: it can take
the camera away from another program that uses the service. A live `--probe` run with `sudo` on a Mac where the
service answers does the same; run it without `sudo` there. The other way round, a helper started with the flag
whose librealsense did not get the service (the service refuses root too while camera use is disabled or too many
programs are connected, or it went away in the meantime) opens the USB device itself and skips the release: a
camera it captured stays without macOS drivers until it is re-plugged or `sudo IntelFaceScanHelper --release -w` is
run.

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
| `security_recording.*` | the spool directory private to the helper's user (root on the administrator path), destination checks, copy into the app's descriptor |
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
  launched helper (status, launch error, stderr) and, for a helper started directly, its pid for the peer check.
  Everything that shows an administrator prompt (the root helper, the release tool, the camera service's installer)
  runs on one serial queue, so two prompts never show at once.
* `HelperSession.swift` – shared by `LiveCameraSource`, `PlaybackCameraSource` and `FakeCameraSource`: the launch
  (`LiveLaunch`: `.asUser` or `.administrator(skipUSBRelease:)` for the live camera; `HelperLaunchPlan`), connect
  (single flight; a failed device query keeps the helper for the next attempt), commands with timeouts (CONFIGURE 60 s,
  RECORD_STOP 600 s, others 10–30 s), fan-out of frames (`AsyncStream`, `.bufferingNewest(n)` per consumer; lossless
  for a non-real-time playback: the reader waits while a consumer has `n` frames, except while a reply is due) and
  events (256 per consumer), and shutdown: QUIT → BYE/EOF → exit, bounded by a timeout (20 s live). A user helper that
  does not exit gets SIGTERM, then SIGKILL (also the live one behind the camera service: the service owns the USB
  device and survives its client); the root helper cannot be signalled by the app and exits on its own once
  the socket is closed. A helper that never connected is not waited for: closing the source while the administrator
  prompt is still showing returns at once (the root helper's handle only ends when the prompt is answered), the
  socket is removed so a helper started after all cannot connect and exits, and a user helper is terminated right
  away.
* `LiveCameraSource.swift` – `Access`: `.automatic` (the default: asks the camera service and throws
  `CameraError.cameraServiceNotReady` when it cannot be used, so it never prompts by itself), `.service`,
  `.administrator(skipUSBRelease:)`; `commandLineAccess` is the policy for tools without a user interface.
* `../CameraService.swift` – the camera service as this process sees it: its files (`CameraServiceKit.locate()`),
  the state check and the installation (the vendored `RealSenseCameraAccess`, behind closures that tests replace),
  and `CameraService.advice(for:)`, what to do about each state ([CAMERA-SERVICE.md](CAMERA-SERVICE.md)).
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
SIGTERM/SIGINT/SIGHUP ends it with `_exit` (`TerminationSignals`). Either way the camera goes back to macOS: the
helper ends when it sees the app exit, and then the camera service hands the camera back, or the root helper does
it itself. Where only synchronous code is possible, `camera.shutdownAndWait(timeout:)` blocks
on the session's own threads.

`LiveCameraSource.releaseCameraToSystem()` runs `IntelFaceScanHelper --release -w` with the administrator prompt –
a recovery action for a camera left captured by a root helper that was killed – and returns the tool's lines (its
stderr is sent to stdout for this launch: `do shell script` returns stdout, and reports it as the error message when
stderr is empty). After a session through the camera service it is not needed: the service hands the camera back
also when the helper was killed.

Live mode needs no particular thread, but the in-process administrator prompt (the root helper's and the camera
service installer's) needs an AppKit application whose main run loop runs (as in the app); other processes go
through `/usr/bin/osascript` (session step 2). An `NSApplication` that was created but never run is not supported.

## Threat model

**What changed with the camera service.** On the normal path nothing of IntelFaceScan runs as root except the
service's installer, once: the helper is a process of the user that gains nothing over the app that started it, and
the rest of this section does not apply to it (its checks stay in place; they keep a confused or hostile process of
the same user from driving the helper, not from reaching root). Root is then held by `realsense-usbd`, which is not
IntelFaceScan's code: its threat model, its installer's trust rule and what it does not protect against are in
`third_party/realsense-usbd/DESIGN.md` and summarised in [CAMERA-SERVICE.md](CAMERA-SERVICE.md) (in short: any
program of the user logged in at the Mac's screen can use RealSense cameras through it, without a prompt and without
the macOS camera indicator). The root helper remains for the administrator path, and so does everything below.

**Asset:** root on the user's Mac. On the administrator path the helper runs as root for as long as the live camera
is used and accepts input from an unprivileged process.

**Adversaries considered:** other local users and their processes; processes of the same user trying to reach root
through the helper without the password; a compromised or confused app sending malformed or hostile messages;
symlink and race tricks in shared directories.

**Out of scope:** a process of the same user that can modify the app bundle (it is owned by the user who installed it):
it could replace the helper before the next prompt, as with any app that asks for the administrator password, and the
same user could equally show a fake password dialog. Installing to `/Applications` from an admin account does not
change ownership of the copied files. Hardening this needs a Developer ID signature plus a privileged helper installed
with `SMAppService` and code-signing requirements on both sides – planned for a signed release. The camera service
narrows the exposure in the meantime: there is one prompt, at the installation, instead of one per session, and the
installer only installs from files that only root or an administrator can change.

**Mitigations:**

| Threat | Mitigation |
|---|---|
| The root helper connects to an attacker's socket | path must be absolute and canonical; every component is opened with `O_NOFOLLOW` from `/`; the final directory must be owned by `--uid` with mode exactly 0700; the socket itself must be a socket owned by `--uid`; the connection is made relative to the verified directory descriptor (`fchdir` + relative `sun_path`, no TOCTOU on the directory); `connect()` resolves the name once more and would follow a link swapped in by the directory's owner, so the name is checked again afterwards (same device and inode, or already removed by the app) – that only narrows the window; the real defense is the peer check: peer uid = `--uid` and peer pid = `--parent-pid` |
| Something else connects to the app | the socket directory is private to the user; one connection is accepted and listening stops; peer uid (0 on the administrator path, otherwise the app's own) and executable identity (`proc_pidpath`, device + inode) are checked, and for every helper the app started directly (playback, fake, live through the camera service) the pid of the launched process (a same-user process that connected first and then executed the real helper would pass the other checks) |
| Hostile messages | header validated by the shared C code (magic, version, flags, 256 KiB JSON, 64 MiB payload); commands may not carry payloads; JSON must be an object nested at most 32 levels deep; unknown fields and wrong types are errors; at most 1024 queued commands (16 MiB); a peer that does not read its replies is disconnected once 4096 messages or 16 MiB wait to be sent; every exception of a command becomes its error reply, nothing thrown in the IPC layer reaches a librealsense thread, and an exception in a connection thread ends the session through the normal shutdown |
| File system access as root | the helper never opens, creates or deletes a path from the app. A recording goes to `mkdtemp` inside the Darwin temporary directory of the user the helper runs as (root's `/var/folders/zz/…/T` on the administrator path; verified owned by that user and not accessible to others; fallback `/private/var/run/IntelFaceScanHelper`), and is copied into the descriptor the app opened as itself (`SCM_RIGHTS`, only with `RECORD_START`, only on its first byte, must be a regular file without set-user-ID/set-group-ID bits (root writing does not clear them, unlike any other writer) open for writing; any other descriptor is closed and the connection dropped), then the spool is removed without following links; spools left by a helper that died mid-recording (untouched for a day, and not locked by a helper that is still running) are removed when the next recording starts |
| Environment and inherited state | the app starts the helper with `env -i` (no `DYLD_*` or other variables reach root); the helper still clears its environment to a fixed `PATH`/`HOME`, sets `umask 077`, closes every inherited descriptor above 2 (from the kernel's list of its descriptors, so also those above a lowered `RLIMIT_NOFILE`) and points stdin at `/dev/null`; no `system()`, no shell, no `exec` |
| Unprivileged modes used to gain or misuse privileges | playback and fake refuse to run as root; every helper that is not root, the live one included, requires `--uid` to be the caller |
| Helper left running as root | it exits when the app exits (kqueue on the parent pid), when the socket closes, on QUIT and on SIGINT/SIGTERM/SIGHUP (the signals are ignored and read from the kqueue, so no work happens in a signal handler and the default action cannot skip the USB release) |
| Camera left unusable | root helper: USB re-enumeration on every exit path except SIGKILL (skipped with `--no-usb-release`: *When the USB release runs*, above), and after a live `--probe` run as root; recovery with `--release` / `rs-macos-release`; patched librealsense bounds USB waits. Through the camera service the daemon hands the camera back whatever happens to the helper |
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
may be taken from. The codec and the PROTOCOL.md JSON have unit tests of their own. The two ways to start the live
helper are tested without a camera, the service or a prompt (`Helper live launch paths`): the arguments and peer
checks of each (`HelperLaunchPlan`), a stand-in helper started as the user on the service path (accepted; one the
app did not start is rejected by its pid), the administrator launch replaced by a stand-in whose peer is refused
because it is not root, `--no-usb-release`, and the real helper running live as the user up to "no camera" (skipped
where the camera service is installed). The helper's C++ tests (`ctest --test-dir build/helper`,
[TESTING.md](TESTING.md)) have `helper-tests` next to the capture engine's `capture-tests`: the recording spool
that helpers of one user share, the command line and exit status (live mode accepted as the user,
`--no-usb-release` for the live camera only), and a live session as the user that ends without a USB release.
The camera service decisions have tests of their own ([CAMERA-SERVICE.md](CAMERA-SERVICE.md)). Streaming from a
camera needs a camera, and the service or the password; the live mode shares everything except `create_live` and
the USB release with the fake mode.
