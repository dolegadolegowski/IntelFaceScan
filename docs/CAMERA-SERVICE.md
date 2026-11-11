# The RealSense camera service in IntelFaceScan

macOS attaches its own UVC/HID drivers to a RealSense camera, and only root may take the USB device over from them.
IntelFaceScan used to solve that by running its camera helper as root, with the administrator password at every
camera session. Now it uses the **RealSense camera service**: `realsense-usbd`, a small root LaunchDaemon that owns
the camera over libusb and serves librealsense clients over XPC. It is installed once (an administrator's name and
password, that one time); afterwards the camera works without a password, in IntelFaceScan and in every other
application that uses the service (RealSense Viewer for macOS installs and uses the same one).

The service is not part of IntelFaceScan. Its home is the folder `macos/broker` of
[realsense-viewer-macos](https://github.com/dolegadolegowski/realsense-viewer-macos); this repository carries an
identical copy in `third_party/realsense-usbd` (see [Where the kit comes from](#where-the-kit-comes-from)).
`third_party/realsense-usbd/README.md` is the guide for application authors, `DESIGN.md` and `CONTRACT.md` next to
it are the specification of the daemon. This document only says how IntelFaceScan uses it.

## The chain from the app to the camera

```
IntelFaceScan.app, as the user
│  "Live camera"                                   AppModel.openSource(.live)
│   1. state check                                 CameraService.check()
│        Contents/Helpers/realsense-usbd --probe     (run as the user; asks the installed service, never touches USB)
│   2. only when the service is missing, too old or not answering:
│        install once                              CameraService.install(prompt:)
│        macOS administrator prompt, then            Contents/Resources/realsense-usbd/install-camera-access.sh as root
│   3. camera                                      LiveCameraSource(access: .service)
▼
Contents/Helpers/IntelFaceScanHelper, as the user  started with Process; AF_UNIX socket (PROTOCOL.md)
│  librealsense 2.58.4 + patches/ + the kit's macos-usb-broker.patch (the service's client inside librealsense)
▼  XPC, Mach service io.github.dolegadolegowski.realsense-usbd
/Library/PrivilegedHelperTools/io.github.dolegadolegowski.realsense-usbd, root LaunchDaemon, started on demand
│  libusb
▼
the RealSense camera (USB)
```

With the service, nothing of IntelFaceScan runs as root except the installer, once. The helper is the same binary as
before; started as the user, its librealsense asks the service for the camera instead of opening the USB device
itself (it says hello to the service once per process and uses it when the service admits it). Frames travel from
the service to the helper through shared memory, and from the helper to the app over the socket as before.

## At the start of a live session

IntelFaceScan checks the service, and installs it when that is needed, each time the user chooses *Live camera*,
and at no other time by itself. Starting the app does neither: recordings, datasets and the synthetic demo need no
camera, and an administrator prompt at launch would come before the user asked for the camera. (The kit's guide
puts the check at an application's start; in IntelFaceScan that moment is the start of the live camera.) The menu
command described below shows the state on request.

`AppModel.openSource(.live)` first checks the state of the service, off the main thread
(`RealSenseCameraAccess.state(kit:)`: the app's own `realsense-usbd --probe`, plus what is installed on disk and
whether the service is switched off). Normally that takes well under a second. Then:

| State | What the app does |
|---|---|
| ready | Starts the helper as the user. No dialog, no password. |
| not installed; too old (another protocol version, or the first generation that only admitted the programs of one RealSense Viewer build); installed but not answering | Installs: straight to the macOS administrator prompt, whose text says that IntelFaceScan installs the RealSense camera service, once, and that the camera then works without a password in every application that uses it. After a successful installation the state is checked again and the camera starts as in *ready*. |
| the prompt was cancelled, or the installer failed | A dialog with the reason: **Try again**, **Use administrator password this time**, **Cancel**. Nothing is asked a second time by itself. |
| the installer refuses the app's location | The dialog says that an administrator has to move IntelFaceScan to the Applications folder (the installer only installs from files that no other user can change; a copy in a standard user's folder or one run from a download does not qualify): **Use administrator password this time**, **Cancel**. |
| the installation finished and the service still cannot be used | A dialog with the state: **Try again**, **Use administrator password this time**, **Cancel**. |
| switched off (an administrator turned it off in System Settings > General > Login Items & Extensions) | Never reinstalled over that choice. Dialog: **Open Login Items Settings**, **Use administrator password this time**, **Cancel**. |
| not the user at the Mac's screen (a remote login, a session switched to the background) | The service only serves the user logged in at the screen, and root. Dialog: **Use administrator password this time**, **Cancel**. |
| camera use disabled on this Mac (device management or Screen Time) | Dialog, **OK** only: the administrator path would get around the ban. |
| too many programs connected to the service | Dialog, **OK** only: a root helper would fight with the service over the camera. |
| this build carries no service files (`swift run` without the bundle) | The administrator path, as before the service existed. |

The progress text on the start screen follows these steps (*Checking the RealSense camera service…*, *Installing
the RealSense camera service: macOS asks for an administrator's name and password…*, *Starting the camera…*);
*Waiting for the administrator password…* only shows when the root helper's prompt really comes.

The decision table is `CameraService.advice(for:)` in `Sources/CameraKit/CameraService.swift`; the flow is
`AppModel.resolveLiveAccess`. The menu command **IntelFaceScan > RealSense Camera Service…** runs the same state
check when it is chosen and shows the state, the installed version and the version this copy of the app carries.
It installs nothing by itself: its button **Install** (**Install again** when a service is installed) runs the
installer, which then follows the rules under [Versions and compatibility](#versions-and-compatibility). The
command is disabled while a source is open: an installation that replaces the daemon restarts the service under a
running camera session.

**Use administrator password this time** is the old way, for one session: the helper runs as root after the macOS
administrator prompt and opens the USB device itself ([HELPER.md](HELPER.md)). In the *not the user at the screen*
case the service is alive and serves root, so the root helper's librealsense goes through the service as well; the
app then starts the helper with `--no-usb-release`, because the service hands the camera back and a USB
re-enumeration by the helper would pull the camera away from it. In every other case the root helper re-enumerates
the camera when its session ends; what that does to another program that uses a camera through the service at that
moment, and where the flag can be wrong, is in [HELPER.md](HELPER.md) (*When the USB release runs*).

## At the end of a session

Closing the source or quitting sends `QUIT`; the helper stops the streams, closes librealsense and exits. It does
not re-enumerate the USB device (it could not as the user, and need not). The service notices that its last client
let go of the camera and hands it back to macOS a few seconds later (`HandBackGraceSeconds`, 5 by default): the
system drivers attach again and FaceTime and other programs see the camera. A program that opens the camera within
those seconds gets it without a new capture, so switching sources or restarting the app is quick. With no client
left the daemon exits after a minute (`IdleExitSeconds`) and launchd starts it again on demand.

The same happens when the helper or the app crashes or is killed: the service sees its connection end. The recovery
tools of the administrator path (`IntelFaceScanHelper --release -w`, `rs-macos-release`,
`LiveCameraSource.releaseCameraToSystem()`) are not needed after a session through the service; they remain for a
root helper that was killed.

## Errors during a session

* **No camera plugged in.** The helper runs and reports the error; the app shows it and closes the source. Plug the
  camera in and choose *Live camera* again (no password either time).
* **Another program is using the camera** (RealSense Viewer streaming from it, a second copy of IntelFaceScan):
  the service gives each USB interface of a camera to one program at a time and refuses a second program's open
  with an access error; the program that has the camera keeps it (`third_party/realsense-usbd/DESIGN.md`, 2.5.3
  and 3.4). The state check does not see this: the service is *ready*, so no dialog comes first. The helper starts
  as the user and then cannot open the camera, or cannot start its streams. It reports librealsense's error, and
  the app shows it and closes the source. The text does not name the other program, and which text appears has
  not been tried with a camera. Stop the camera in the other program and choose *Live camera* again. This is not
  the *too many programs connected* state above, which is the service's limit on connections.
* **The service goes away** (an administrator switches it off or removes it, another application installs a newer
  version, which restarts it, or the daemon crashes): the helper's librealsense loses the device, the helper reports
  the camera as disconnected, the app says so and closes the source. Choosing *Live camera* again runs the state
  check again.
* **Another user logs in at the screen** (fast user switching): the service ends the connections of the user who is
  no longer at the screen; the app sees the camera disconnect.
* **The camera becomes disabled** by device management or Screen Time: the service refuses and ends running
  sessions.
* The service only knows RealSense cameras; errors of the camera itself (USB 2 cable, firmware) arrive through
  librealsense as before.

## Files in the app bundle

```
IntelFaceScan.app/Contents/Helpers/realsense-usbd                             the daemon; also the probe tool
IntelFaceScan.app/Contents/Resources/realsense-usbd/service.plist             its manifest
IntelFaceScan.app/Contents/Resources/realsense-usbd/install-camera-access.sh
IntelFaceScan.app/Contents/Resources/realsense-usbd/uninstall-camera-access.sh
IntelFaceScan.app/Contents/Resources/realsense-usbd/ensure-camera-access.sh
```

`scripts/build.sh` builds the daemon from the kit (`third_party/realsense-usbd/build-daemon.sh`, into
`build/usbd/realsense-usbd`, linked against the static libusb of the librealsense build and system libraries only).
`scripts/bundle-app.sh` copies the files, signs the daemon ad hoc with the hardened runtime and the service's own
identifier, writes `service.plist` with the kit's `make-service-manifest.sh` (service version, protocol versions and
the cdhash of the daemon as signed), and signs the bundle, whose signature then covers the manifest.
`RealSenseCameraAccess.Kit.inBundle(_:)` finds the files in this layout (viewer applications keep the daemon in
`Contents/MacOS`; it looks in both).

## What is installed, and how to remove it

```
/Library/PrivilegedHelperTools/io.github.dolegadolegowski.realsense-usbd                   the daemon
/Library/LaunchDaemons/io.github.dolegadolegowski.realsense-usbd.plist                     its launchd job
/Library/Application Support/io.github.dolegadolegowski.realsense-usbd/usbd.plist          its configuration
/Library/Application Support/RealSense Viewer/usbd.plist                                   a marker for old RealSense
                                                                                           Viewer builds
```

macOS shows its "Background Items Added" notification, and the service appears in System Settings > General > Login
Items & Extensions, where an administrator can switch it off.

To remove it: `sudo /Applications/IntelFaceScan.app/Contents/Resources/realsense-usbd/uninstall-camera-access.sh`.
That removes the service for every RealSense application on the Mac; IntelFaceScan offers to install it again at the
next *Live camera*. Deleting IntelFaceScan.app does not remove the service.

## Versions and compatibility

* The daemon has a **service version** (an integer that grows with every release; 2 is the first shared one, 1 stands
  for the earlier builds that only admitted one RealSense Viewer build) and speaks a range of **protocol versions**.
  `Contents/Helpers/realsense-usbd --version` prints both; `service.plist` records them for the daemon in the
  bundle, the installed `usbd.plist` for the installed one.
* **What the installer does** when it runs, from IntelFaceScan or from any other application: it compares the
  installed service version with the one it carries (the exact rules: `third_party/realsense-usbd/README.md`,
  *Compatibility rules*).
  * It **never replaces a newer service**. An application that carries an older daemon keeps the installed one and
    only makes sure it is loaded. So whichever application was installed last, the newest service stays.
  * A service of the **same version** that is installed and answering is **left alone**. IntelFaceScan and
    RealSense Viewer carry different builds of the same service version, and neither replaces the other's daemon.
    **Install again** from the menu command therefore changes nothing while such a service answers.
  * It **replaces** an older service and one of the first generation, and **repairs** one that does not answer.
* A **first-generation service**, or one that **does not speak this app's protocol version**, is replaced at the
  next *Live camera*: that is the *too old* row above. Old RealSense Viewer builds that only know the first
  generation then say that the installed camera access is newer than they are, and need a current build.
* An **older service that still speaks the protocol** is used as it is, without a prompt: the app decides by the
  probe and does not compare service versions. It is replaced only when an installer that carries a newer daemon
  runs (**IntelFaceScan > RealSense Camera Service…** and **Install again**, or another application's
  installation).
* Applications **built later** need no update of the service as long as the protocol ranges overlap: the service does
  not check which program connects, only who runs it (below). IntelFaceScan's helper and the daemon it carries are
  built from the same kit, so the app's own probe is a fair test for the helper's librealsense.
* When the service was updated while the helper was running, the running session ends (see *Errors*); the next one
  uses the new service.

## Command line

* `Contents/Resources/realsense-usbd/ensure-camera-access.sh` (as yourself, not with sudo) checks the service and
  installs it when that is the fix, with the same administrator prompt; `--check` only reports (exit 0 ready, 10 not
  installed, otherwise the probe's code: 2 not reachable, 3 not the user at the screen, 4 camera disabled, 5 version,
  6 too many programs).
* `Contents/Helpers/realsense-usbd --probe` is the bare check.
* `ifscan` has no live camera input (it reads recordings and datasets), so it never needs the service. For a tool
  that does open the camera, `LiveCameraSource.commandLineAccess(cameraService:note:)` is the policy: the service
  when it is ready; otherwise the administrator path as before, with one line for stderr that says why and how to
  install the service (choose *Live camera* in the app once, or run `ensure-camera-access.sh`); no access where the
  administrator path is not allowed either. A command line tool never installs the service by itself.
* `IntelFaceScanHelper --probe` (the camera measurements of [CAMERA.md](CAMERA.md)) runs as the user when the
  service is ready, and needs `sudo` without it.

## Checked on a D455

On 2026-11-10, on an M3 Ultra Mac with macOS 27.2 and a D455 (firmware 5.16.0.1) on USB 3.2, as an administrator
account logged in at the screen, with the app run from `build/IntelFaceScan.app`:

* **First use.** *Live camera* showed the administrator prompt; after the password the installer wrote the four
  files, launchd started `realsense-usbd` (service version 2, protocol 1) and the installed daemon's probe answered.
  The app went on to the camera by itself.
* **Live session.** The service's log shows the helper admitted as the user (uid 501), the device leased and
  captured; the helper ran as the user and the daemon as root; the app showed the device (serial, firmware, USB 3.2,
  0.1 mm depth units, IMU) and the color and depth previews at 30 fps.
* **End of a session.** Five seconds after the source was closed the service re-enumerated the camera, and macOS
  listed its RGB camera again.
* **Later sessions.** *Live camera* started without any prompt.
* **Another application.** RealSense Viewer for macOS, which carries a different build of the same service version,
  started without a prompt and opened the camera through the service this app had installed; its
  `rs-macos-selftest` passed all 24 checks without `sudo` (depth, infrared, color, IMU at 400 Hz, presets, recording
  and playback). `ensure-camera-access.sh` printed `ok 2` from both applications.

On 2026-11-11, on the same Mac and camera:

* **Longer streaming.** `IntelFaceScanHelper --probe` (1280×720 High Accuracy, disparity shift 50, depth and color)
  ran as the user through the service five times back to back, about half an hour in all, without an error. The
  service stayed at 8.5–9.3 MB and 1–3 % of a core, the helper at 122–124 MB.
* **Another application's installer over the installed service.** RealSense Viewer's installer, run as root
  (it carries another build of the same service version), reported that the service was already installed and
  answering and replaced nothing: the daemon's code hash and the dates of the daemon and of its configuration were
  unchanged.
* **The stress self-test of RealSense Viewer's tools** (`rs-macos-selftest --stress`) passed without `sudo` through
  the service this app had installed: 36 checks, among them every sensor for 60 s at 30.0 fps and a hardware reset.

Not checked yet on hardware: a complete guided scan and a recording through the service, a standard (not
administrator) account, a second user session, the service switched off in Login Items, an update from an older
service, a restart of the Mac, the administrator path next to an installed service, two cameras, and what the app
shows when another program holds the camera.

## Development and testing without a camera

* **Where a development build finds the service files** (`CameraServiceKit.locate()`): the bundle of the running
  process; else `$IFS_CAMERA_SERVICE_KIT` (a folder with `realsense-usbd`, `service.plist` and
  `install-camera-access.sh` side by side, or an application bundle that carries them); else the bundle the
  executable sits in; else `build/IntelFaceScan.app` in a parent folder of the executable (`swift run` after
  `scripts/build.sh`). The last search can climb out of the checkout, so a kit found that way is only used when
  nobody but root and the user can replace its files, the rule of the helper search. With no kit the app goes
  straight to the administrator path.
* **Swift tests** never run the real probe, install anything or prompt. `CameraService` takes closures in place of
  the probe, the installer and System Settings; `HelperCameraSession.privilegedLauncher` replaces the administrator
  launch. `Tests/CameraKitTests/CameraServiceTests.swift` covers the decision for every state, the command line
  policy, the kit locations and their safety rule, and the installer's command line run through `osascript` and the
  shell without the administrator clause; `HelperSessionLifecycleTests.swift` the two launch paths (arguments, uid
  and pid checks of the peer, `--no-usb-release`, a stand-in helper started as the user);
  `Tests/IntelFaceScanAppTests/AppCameraServiceTests.swift` the app model's reaction to every outcome.
* **The helper's C++ tests** (`helper-tests`, one of the two entries of `ctest --test-dir build/helper`;
  [TESTING.md](TESTING.md)) run the helper binary itself: live mode accepted as the user, `--no-usb-release` for
  the live camera only, and a live session as the user from `HELLO` to `BYE` that never releases USB.
* **The kit's own tests** for the installer scripts are dry runs into a temporary folder:
  `third_party/realsense-usbd/tests/test-camera-access.sh build/usbd/realsense-usbd`.
* **Looking at a Mac by hand**, without changing it: `build/IntelFaceScan.app/Contents/Helpers/realsense-usbd
  --probe; echo $?`, `... --version`, `ensure-camera-access.sh --check`, and
  `install-camera-access.sh --dry-run --root <empty folder> --source <dir> ...`.
* The live path through the service needs a camera and an installed service; everything up to `HELLO` is shared with
  the fake camera and playback modes, which the integration tests run.

## Security and privacy

* **Who can use the camera.** Once the service is installed, any program run by the user logged in at the Mac's
  screen (and root) can use RealSense cameras through it, without a password and without asking. The service does
  not check which program connects; that is what lets applications built later use it. An administrator can limit it
  to named programs (`AllowedCDHashes` in the installed `usbd.plist`; the kit's README). Other users of the Mac,
  background and remote sessions are not served.
* **No macOS camera prompt, no camera indicator.** The service reads the camera over USB, not through the macOS
  camera frameworks: macOS shows neither its camera permission prompt nor the green camera indicator while a program
  uses a RealSense camera this way, and privacy rules of device management cannot target single programs. A general
  camera ban (`allowCamera` false from device management or Screen Time) is honoured.
* **What runs as root.** With the service: the daemon (a C++ program without librealsense, validating every request,
  limited to RealSense devices and filtering firmware commands; its threat model is in `DESIGN.md`), and the
  installer once. Nothing of IntelFaceScan. On the administrator path: IntelFaceScan's helper, as described in
  [HELPER.md](HELPER.md).
* **The installer** runs as root with a clean environment. It only installs from files and folders that only root or
  an administrator can change, checks the daemon against the cdhash in the manifest before installing it, and never
  runs the binary it was given before it is installed. That is why the app has to be in `/Applications` (copied
  there from an administrator's account) for the installation, and why `scripts/bundle-app.sh` builds the bundle
  with no group- or world-writable file.
* The app and the daemon are signed ad hoc, not with a Developer ID.

## Where the kit comes from

`third_party/realsense-usbd` is a copy of `macos/broker` from realsense-viewer-macos, identical file for file, and
is **never edited here**. `scripts/sync-camera-service.sh <realsense-viewer-macos checkout>` updates it;
`scripts/sync-camera-service.sh --check <checkout>` only compares. What IntelFaceScan takes from it:

| Kit file | Used by |
|---|---|
| `librealsense/macos-usb-broker.patch` | `scripts/apply-patches.sh`: the service's client inside librealsense |
| `daemon/`, `build-daemon.sh`, `rs-usb-broker-protocol.h` | `scripts/build.sh`: `build/usbd/realsense-usbd` |
| `install-camera-access.sh`, `uninstall-camera-access.sh`, `ensure-camera-access.sh`, `make-service-manifest.sh` | `scripts/bundle-app.sh` |
| `client/RealSenseCameraAccess.swift` | `Package.swift`, target `RealSenseCameraClient`: state check and installation, compiled unchanged |
| `tests/test-camera-access.sh` | the installer tests (above) |
