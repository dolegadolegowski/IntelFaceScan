# IntelFaceScan

Native macOS app (Apple Silicon, macOS 27.2+) for a full, detailed **3D scan of a human head** with an
**Intel RealSense D455** connected over USB‑C. The app guides you through a short sequence of head poses, checks every
frame with Apple Vision, registers and fuses the views on the GPU and saves a textured **OBJ + MTL + PNG in
millimeters** that opens in Blender and macOS Preview (MeshLab should read it too but has not been tried yet).

**[🇵🇱 Polska wersja poniżej](#po-polsku)**

> **Status:** the whole pipeline is implemented and tested without a camera (synthetic head rendered at 30 fps and
> analyzed by Apple Vision, recordings played back through the camera helper, 400+ automated tests). Exported models were checked
> by importing them in Blender 5.0 and rendering them with macOS Quick Look (what Preview uses); MeshLab has not been
> tried yet. It has **not yet been run against a physical D455**: the camera measurements in
> [docs/CAMERA.md](docs/CAMERA.md) are still theoretical. See [Accuracy](#accuracy) and [Limitations](#limitations).

## Contents

- [Features](#features) · [Requirements](#requirements) · [Build and install](#build-and-install)
- [Permissions and the camera](#permissions-and-the-camera) · [How to scan](#how-to-scan) · [Settings](#settings)
- [Output files](#output-files) · [Recordings and the command line](#recordings-and-the-command-line)
- [Accuracy](#accuracy) · [Limitations](#limitations) · [Libraries and why](#libraries-and-why)
- [Architecture and docs](#architecture-and-docs) · [Development](#development) · [License](#license)

## Features

- **Live preview** of color and depth (Metal), the face distance and the recommended working range for the current
  camera configuration.
- **Guided scan:** front → up → down → left → right (optional diagonals), with an animated head showing where to turn,
  a progress ring and a coverage map. Targets are measured relative to *your* frontal pose. A frame is accepted only
  when the pose is reached and held still, the image is sharp, the depth has enough points and the camera is not
  moving (IMU). Rejections are explained on screen (too far, too close, moving, blurry, no face…).
- **Camera configuration for faces:** depth 1280×720 or 848×480, RGB 1280×800, 30 fps, High Accuracy or High Density,
  maximum laser power, emitter on, auto exposure on the face, 0.1 mm depth units, optional close range (disparity
  shift 50). Post-processing in the camera helper: threshold → disparity → spatial → temporal → hole filling (off by
  default) → align depth to color (no decimation: the app scales the preview itself). Factory calibration (intrinsics, extrinsics,
  baseline) from the camera.
- **Reconstruction:** lens undistortion, head segmentation (person mask ∩ depth band ∩ chin cut, option to keep the
  shoulders), initial poses from Vision, multi-scale point-to-plane ICP (colored ICP optional), pose graph with loop
  closures, TSDF fusion on the GPU (0.75 mm voxels by default) with marching cubes, screened Poisson as an
  alternative, mesh cleanup (small pieces, detail-preserving smoothing, hole filling), closing the unscanned back of
  the head into a **watertight** model (on by default), UV atlas and a texture blended from the best frames, plus
  vertex colors.
- **Result:** interactive 3D preview (texture / vertex colors / shaded / wireframe), export OBJ+MTL+PNG in mm,
  optional binary PLY and the raw session (keyframes with depth, masks and poses).
- **Recording and playback** of `.db3` (ROS 2) and `.bag` files, so a session can be reconstructed again later.
- **Safe camera handling:** the camera is captured once per session by a small privileged helper and handed back to
  macOS when the app quits, closes the camera, or receives SIGINT/SIGTERM/SIGHUP.
- English and Polish user interface (follows the system language).

## Requirements

- A Mac with Apple Silicon and **macOS 27.2** or later.
- **Intel RealSense D455** on a **USB 3** port or cable (1280×720 depth + RGB at 30 fps needs USB 3).
- To build: the Command Line Tools (`xcode-select --install`) or Xcode 27 with the macOS 27.2 SDK, and CMake 3.20+
  (`brew install cmake`). No other Homebrew package is used: libusb is built from source and linked statically.
- Network access for the first build (git submodules; the librealsense build downloads its dependencies).

## Build and install

```bash
git clone --recurse-submodules https://github.com/dolegadolegowski/IntelFaceScan.git
cd IntelFaceScan
scripts/build.sh
```

`scripts/build.sh` checks the tools first and tells you what is missing, then builds the patched librealsense
2.58.4, the camera helper, the Swift app and the bundle `build/IntelFaceScan.app`. From a fresh clone it takes about
2–2.5 minutes on an M3 Ultra; a rebuild without changes takes seconds. `scripts/package.sh` makes a DMG.

Start the app with `open build/IntelFaceScan.app`, or copy it to `/Applications`. The app is signed ad hoc (not
notarized): on another Mac, open it the first time with right-click → **Open**.

## Permissions and the camera

- **Administrator password.** macOS attaches its own drivers to the camera. Taking the USB device over requires root,
  so when you choose **Live camera** macOS shows the standard administrator prompt. Only the small camera helper
  (`Contents/Helpers/IntelFaceScanHelper`) runs as root; the app itself, Vision, the reconstruction and all file
  writing run as you. The helper accepts commands only from the app over a private socket, never opens paths the app
  sends, and writes recordings into a file the app opened. Details: [docs/HELPER.md](docs/HELPER.md).
- **Recordings, datasets and the synthetic demo** need no password.
- **Handing the camera back.** When the app quits (⌘Q, closing the window, `kill -TERM`, Ctrl‑C) or you close the
  camera, the helper stops the streams and re-enumerates the USB device so macOS takes it back (e.g. FaceTime can use
  it again). If the app ever crashes and the camera stays unusable, run
  `sudo build/IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper --release -w` or unplug and replug it.
- **Never `kill -9` the helper** while it talks to the camera: the D455 firmware can hang until it is replugged.
- No other permission is needed: the app does not use the macOS camera framework, so there is no camera privacy
  prompt.

## How to scan

1. Mount the camera on a tripod or a monitor at **eye level**, about an arm's length away. Sit on a chair that does not
   rotate; **turn only your head**, keep your shoulders still.
2. Use **even, diffuse light** (no strong light from one side, no direct sun). Take off glasses; tie back long hair if
   you can (stereo depth sees little of fine hair). Avoid shiny makeup.
3. Choose **Live camera**, enter the administrator password and wait for the preview. Move until the distance
   indicator is green: about **0.56–0.71 m** with the default configuration, **0.41–0.56 m** with *Close-range mode* on
   (the exact range for your camera is shown under the preview).
4. Press **Start scan** and follow the animated head: front, up, down, left, right (and the diagonals if enabled).
   Each pose captures 3 sharp, still frames (configurable). The whole scan takes about a minute.
5. Check the summary, then press **Reconstruct 3D model**. On an M3 Ultra, reconstructing a 15-keyframe scan takes
   about 20–40 s.
6. Look at the model in the 3D preview and export **OBJ** (and optionally PLY and the raw session).

Tips: if a pose does not complete, read the instruction at the bottom of the preview and the reason below it (most
often *Move closer* / *Move back*, *Hold still…* or the turn instruction, such as *Turn your head to your left*).
*Skip pose* moves on. *Finish* ends the scan with the frames captured so far; then press *Reconstruct 3D model* on the
summary to build the model from them.

## Settings

| Setting | Default | Notes |
|---|---|---|
| Depth resolution | 1280×720 | 848×480 is 1.5× noisier at the same distance; use it only on a slow Mac |
| Preset | High Accuracy | fewer outliers; High Density fills more of the skin |
| Close-range mode | off | disparity shift 50: the face can be at ~0.47 m, where depth noise is ~40% lower (theory); turn it on after measuring your camera ([docs/CAMERA.md](docs/CAMERA.md)) |
| Laser power | maximum | skin has little texture: the projector pattern is what stereo matching sees |
| Spatial / temporal filter | on | edge-preserving smoothing in the disparity domain |
| Hole filling | off | invented surface is worse than a hole for fusion |
| Maximum distance | 1.2 m | depth farther than this is dropped |
| Auto exposure on the face | on | stereo and RGB exposure follow your face, not the background |
| Keep neck and shoulders | off | keeps neck and shoulders instead of cutting below the chin |
| Diagonal poses (optional) | off | adds up-left, up-right, down-left, down-right for better coverage |
| Frames per pose | 3 | more frames average more noise, the scan takes longer |
| Voxel size | 0.75 mm | best (or within 1 % of the best) of the measured 0.5, 0.75 and 1 mm voxels at 0.5–1.5 mm depth noise ([docs/RECONSTRUCTION.md](docs/RECONSTRUCTION.md)); the slider goes from 0.5 to 1.5 mm |
| Surface | TSDF (GPU) | Screened Poisson is the alternative (smoother, less accurate on synthetic tests) |
| Close the back of the head | on | closes the unscanned back of the head and the neck with a smooth synthesized surface colored from the neighboring hair and skin, so the model is watertight (Preview does not draw the inside of an open model); the closed part is a guess, not a measurement |
| Mirror the preview | on | the preview behaves like a mirror |
| Show face landmarks | on | draws the face landmarks found by Apple Vision on the live preview |

## Output files

- `head.obj` – vertices in **millimeters**, origin at the head center, **+Y up**, face toward **+Z**, normals,
  counter-clockwise faces, texture coordinates with the bottom-left origin; watertight unless *Close the back of the
  head* is off.
- `head.mtl` + `head.png` – material and texture (RGB, up to 4096×4096).
- `head.ply` (optional) – binary PLY with normals and vertex colors.
- `scan.ifsdataset/` (optional) – the raw session: color, aligned depth, masks, poses and camera calibration of every
  keyframe; reopen it in the app (**Open dataset…**, then **Use the saved frames**) or reconstruct it with
  `ifscan reconstruct`.

Opening the OBJ: **macOS Preview / Quick Look** – double-click. **MeshLab** – File → Import Mesh. **Blender** – File →
Import → Wavefront (.obj), keep *Forward −Z, Up Y*, and set the import *Scale* to **0.001** (Blender treats units as
meters, the file is in millimeters); the face looks at you in the front view (numpad 1).

## Recordings and the command line

- In the app: **Record** saves the camera stream as a `.db3` file (ROS 2 format); **Open recording…** plays `.db3` or
  `.bag` (e.g. from realsense-viewer) back through the same post-processing and the same guided scan, without the
  camera or a password.
- `ifscan` (inside the bundle: `build/IntelFaceScan.app/Contents/MacOS/ifscan`) runs the same pipeline from the
  terminal:

```bash
ifscan reconstruct recording.db3 -o head.obj --ply head.ply --report report.json
ifscan reconstruct session.ifsdataset -o head.obj --voxel-mm 0.5
ifscan reconstruct session.ifsdataset -o head.obj --open-back   # leave the back of the head open
ifscan reconstruct synthetic-camera -o demo.obj      # the animated synthetic head, no camera
ifscan synth -o test.ifsdataset                      # synthetic dataset with ground truth
ifscan reconstruct test.ifsdataset -o test.obj --report report.json
ifscan eval test.obj --gt test.ifsdataset/ground_truth.obj --report report.json   # surface distance in mm
ifscan info recording.db3
```

## Accuracy

Measured without a camera, on the synthetic head rendered with a D455-like noise model (5 poses × 2 frames,
0.75 mm voxels, point-to-plane ICP; details in [docs/RECONSTRUCTION.md](docs/RECONSTRUCTION.md)). The surface
distance covers the measured surface; the closed back of the head (about 40 % of the closed model's surface on the
synthetic head) is a guess: with it included, the mean distance of the whole model grows to about 3.6 mm.

| Case | Rotation error mean / max | Surface distance to ground truth mean / p95 |
|---|---|---|
| clean synthetic scan, 1280×800 (`ifscan reconstruct`) | 0.036° / 0.067° | 0.13 mm / 0.39 mm |
| D455 lens distortion, head 15 cm right and 6 cm down from the optical axis | 0.04° / 0.08° | 0.13 mm / 0.37 mm |
| correlated depth noise + waviness | 0.29° / 0.51° | 0.34 mm / 0.92 mm |
| all imperfections combined (correlated depth noise + waviness, no depth on hair, silhouettes eroded by 6 px, 5 % depth holes, Vision error σ 3° per angle, exposure ±20 %, frontal frames last) | 0.41° / 0.87° | 0.36 mm / 0.96 mm |

Expected D455 depth noise (theory, subpixel 0.08 px): 0.45 mm RMS at 0.60 m with the default configuration, 0.25 mm
at 0.45 m with close range. **Real-camera accuracy and noise have not been measured yet**;
[docs/CAMERA.md](docs/CAMERA.md) has the exact `--probe` commands to measure 1280×720 vs 848×480, High Accuracy vs
High Density and disparity shift 0 vs 50 on a flat wall and on skin.

## Limitations

- Not yet validated with a physical D455 (see the status note at the top).
- The back of the head is never seen (poses reach ±45° yaw). By default the model is closed there with a smooth
  synthesized surface: plausible, not measured. Turn off *Close the back of the head* (or use `--open-back`) to leave
  it open; small holes are filled either way.
- Some camera-side assumptions were measured on synthetic data and need a real D455 to confirm: that librealsense's
  aligned depth is free of lens distortion while color is not, and how much Apple Vision's head angles lean toward the
  line of sight for a face off the image center.
- Hair, glasses, very dark or shiny surfaces give little or no stereo depth.
- Apple Vision's head-pose angles can be off by several degrees for some faces; the scan compensates by measuring
  targets relative to your frontal pose, and registration refines the poses.
- The app is signed ad hoc and not notarized.

## Libraries and why

| Library | Used for | Why this one |
|---|---|---|
| [librealsense](https://github.com/realsenseai/librealsense) 2.58.4 + macOS patches | camera control, filters, alignment, recording | the official SDK; the patches from [realsense-viewer-macos](https://github.com/dolegadolegowski/realsense-viewer-macos) make it work on Apple Silicon macOS (USB capture once per process, no infinite USB waits, …) |
| libusb 1.0.30 (static) | USB access for librealsense | what librealsense uses on macOS; linked statically with the Security framework so the bundle has no Homebrew dependency |
| Apple Vision | face rectangle, landmarks, head pose, person segmentation | on-device, fast, part of macOS |
| Metal | TSDF fusion, preview rendering | the native GPU API (no OpenGL); shaders compile at runtime |
| SceneKit | 3D result preview | native, reads the same mesh data as the export |
| [Eigen](https://eigen.tuxfamily.org) 5.0.1 (MPL2 parts only) | linear algebra (ICP, pose graph) | standard, header-only |
| [nanoflann](https://github.com/jlblancoc/nanoflann) 1.10 | k-d trees for correspondences | small, header-only, fast |
| [PoissonRecon](https://github.com/mkazhdan/PoissonRecon) | screened Poisson surface (alternative method) | the reference implementation |
| [xatlas](https://github.com/jpcy/xatlas) | UV atlas | robust chart generation and packing |
| [meshoptimizer](https://github.com/zeux/meshoptimizer) 1.3 | optional decimation | fast, quality-preserving simplification |
| Apple ImageIO | PNG writing (texture atlas, dataset images) | part of macOS; about 2× faster than stb_image_write on a 4096² atlas |

Marching cubes, ICP, the pose graph, TSDF fusion, mesh cleanup and texture blending are our own code in
`Sources/FaceReconCore` (C++17 / Objective‑C++ / Metal). Licenses: [NOTICE](NOTICE) and `third_party/*/`; the app
bundle carries them in `Contents/Resources`.

## Architecture and docs

```
IntelFaceScan.app (you)                                   IntelFaceScanHelper (root, live camera only)
SwiftUI ─ ScanKit (Vision, guided scan) ─ ReconKit        librealsense 2.58.4 + patches, libusb
     │                                    └ FaceReconCore     streams, presets, filters, align, IMU,
CameraKit ─────────── AF_UNIX socket (docs/PROTOCOL.md) ──── recording / playback, USB capture + release
```

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) – processes, security rules, data flow, conventions
- [docs/HELPER.md](docs/HELPER.md) – the privileged helper, socket, signals, USB release
- [docs/PROTOCOL.md](docs/PROTOCOL.md) – the app ↔ helper wire protocol
- [docs/CAMERA.md](docs/CAMERA.md) – D455 configuration, theory, measurement procedure
- [docs/RECONSTRUCTION.md](docs/RECONSTRUCTION.md) – registration, fusion, meshing, measured comparisons
- [docs/TESTING.md](docs/TESTING.md) – tests, synthetic data, `ifscan`, recording test sessions, CI

## Development

```bash
scripts/test.sh                 # all Swift Testing suites (works with the Command Line Tools only)
scripts/test.sh --filter ICP    # a subset
cmake -S helper -B build/helper -DIFS_HELPER_TESTS=ON && cmake --build build/helper -j && ctest --test-dir build/helper
```

The **Synthetic demo** source and `ifscan reconstruct synthetic-camera` exercise the whole live flow without a camera.
CI (GitHub Actions, macOS 27 runner) builds everything with `scripts/build.sh` and runs all tests.

## License

Apache License 2.0 (see [LICENSE](LICENSE) and [NOTICE](NOTICE)), like librealsense. RealSense is a trademark of
RealSense, Inc.; Intel is a trademark of Intel Corporation. This is an unofficial community project.

---

<a id="po-polsku"></a>

# IntelFaceScan – po polsku

Natywna aplikacja na macOS (Apple Silicon, macOS 27.2+) do pełnego, dokładnego **skanu 3D głowy** kamerą
**Intel RealSense D455** podłączoną przez USB‑C. Aplikacja prowadzi przez krótką sekwencję póz głowy, sprawdza każdą
klatkę przez Apple Vision, rejestruje i łączy widoki na GPU i zapisuje teksturowany **OBJ + MTL + PNG w
milimetrach**, który otwiera się w Blenderze i Podglądzie macOS (MeshLab powinien go wczytać, ale nie był jeszcze sprawdzany).

> **Stan:** cały potok jest zaimplementowany i przetestowany bez kamery (syntetyczna głowa renderowana w 30 fps
> i analizowana przez Apple Vision, nagrania odtwarzane przez helper kamery, ponad 400 testów automatycznych). Wyeksportowane modele
> sprawdzono, importując je w Blenderze 5.0 i renderując przez Quick Look macOS (z niego korzysta Podgląd); MeshLaba
> jeszcze nie próbowano. **Nie był jeszcze uruchomiony z fizyczną kamerą D455**: pomiary kamery w
> [docs/CAMERA.md](docs/CAMERA.md) są na razie teoretyczne. Zobacz [Dokładność](#dokładność) i
> [Ograniczenia](#ograniczenia).

## Spis treści

- [Funkcje](#funkcje) · [Wymagania](#wymagania) · [Budowanie i instalacja](#budowanie-i-instalacja)
- [Uprawnienia i kamera](#uprawnienia-i-kamera) · [Jak skanować](#jak-skanować) · [Ustawienia](#ustawienia)
- [Pliki wynikowe](#pliki-wynikowe) · [Nagrania i wiersz poleceń](#nagrania-i-wiersz-poleceń)
- [Dokładność](#dokładność) · [Ograniczenia](#ograniczenia) · [Biblioteki i uzasadnienie](#biblioteki-i-uzasadnienie)
- [Architektura i dokumentacja](#architektura-i-dokumentacja) · [Rozwój](#rozwój) · [Licencja](#licencja)

## Funkcje

- **Podgląd na żywo** koloru i głębi (Metal), odległość twarzy i zalecany zakres pracy dla bieżącej konfiguracji.
- **Prowadzony skan:** prosto → góra → dół → lewo → prawo (opcjonalnie skosy), z animowaną głową pokazującą, gdzie się
  obrócić, pierścieniem postępu i mapą pokrycia. Cele są mierzone względem *Twojej* pozycji na wprost. Klatka jest
  przyjmowana tylko wtedy, gdy poza jest osiągnięta i utrzymana, obraz jest ostry, głębia ma dość punktów, a kamera
  się nie rusza (IMU). Odrzucenia są wyjaśniane na ekranie (za daleko, za blisko, ruch, nieostro, brak twarzy…).
- **Konfiguracja kamery pod twarz:** głębia 1280×720 lub 848×480, RGB 1280×800, 30 fps, High Accuracy lub High Density,
  maksymalna moc lasera, emiter włączony, automatyczna ekspozycja na twarz, jednostki głębi 0,1 mm, opcjonalny tryb
  bliskiego zasięgu (przesunięcie dysparycji 50). Przetwarzanie w helperze: threshold → disparity → spatial →
  temporal → hole filling (domyślnie wyłączony) → wyrównanie głębi do koloru (bez decymacji: podgląd skaluje sama
  aplikacja). Kalibracja fabryczna odczytana z kamery (parametry wewnętrzne, zewnętrzne, baza stereo).
- **Rekonstrukcja:** usuwanie dystorsji obiektywu, segmentacja głowy (maska osoby ∩ pasmo głębi ∩ cięcie pod brodą,
  opcja zachowania ramion), początkowe pozy z Vision, wieloskalowe ICP point-to-plane (colored ICP opcjonalnie), graf
  póz z domknięciami pętli, fuzja TSDF na GPU (domyślnie woksele 0,75 mm) i marching cubes, screened Poisson jako
  alternatywa, czyszczenie siatki (małe fragmenty, wygładzanie z zachowaniem detali, łatanie dziur), domknięcie
  nieskanowanego tyłu głowy w **szczelny** model (domyślnie włączone), atlas UV i tekstura mieszana z najlepszych
  klatek oraz kolory wierzchołków.
- **Wynik:** interaktywny podgląd 3D (tekstura / kolory wierzchołków / cieniowanie / siatka), eksport OBJ+MTL+PNG
  w mm, opcjonalnie binarny PLY i surowa sesja (klatki kluczowe z głębią, maskami i pozami).
- **Nagrywanie i odtwarzanie** plików `.db3` (ROS 2) i `.bag`, aby sesję można było zrekonstruować ponownie.
- **Bezpieczna obsługa kamery:** kamerę przejmuje raz na sesję mały uprzywilejowany helper i oddaje ją macOS, gdy
  aplikacja się zamyka, zamyka kamerę albo dostaje SIGINT/SIGTERM/SIGHUP.
- Interfejs po polsku i po angielsku (według języka systemu).

## Wymagania

- Mac z Apple Silicon i **macOS 27.2** lub nowszym.
- **Intel RealSense D455** na porcie lub kablu **USB 3** (głębia 1280×720 + RGB w 30 fps wymaga USB 3).
- Do budowania: Command Line Tools (`xcode-select --install`) albo Xcode 27 z SDK macOS 27.2 oraz CMake 3.20+
  (`brew install cmake`). Żaden inny pakiet Homebrew nie jest używany: libusb jest budowany ze źródeł i linkowany
  statycznie.
- Dostęp do sieci przy pierwszym budowaniu (submoduły git; build librealsense pobiera swoje zależności).

## Budowanie i instalacja

```bash
git clone --recurse-submodules https://github.com/dolegadolegowski/IntelFaceScan.git
cd IntelFaceScan
scripts/build.sh
```

`scripts/build.sh` najpierw sprawdza narzędzia i mówi, czego brakuje, potem buduje spatchowany librealsense 2.58.4,
helper kamery, aplikację Swift i pakiet `build/IntelFaceScan.app`. Ze świeżego klonu trwa to ok. 2–2,5 minuty na
M3 Ultra; ponowne budowanie bez zmian – kilka sekund. `scripts/package.sh` tworzy DMG.

Uruchom aplikację przez `open build/IntelFaceScan.app` albo skopiuj ją do `/Applications`. Aplikacja ma podpis ad hoc
(bez notaryzacji): na innym Macu otwórz ją za pierwszym razem przez prawy przycisk → **Otwórz**.

## Uprawnienia i kamera

- **Hasło administratora.** macOS podłącza do kamery własne sterowniki. Przejęcie urządzenia USB wymaga roota, więc po
  wybraniu **Kamera na żywo** macOS pokazuje standardowe okno z prośbą o hasło administratora. Jako root działa tylko
  mały helper kamery (`Contents/Helpers/IntelFaceScanHelper`); sama aplikacja, Vision, rekonstrukcja i zapis plików
  działają jako Ty. Helper przyjmuje polecenia wyłącznie od aplikacji przez prywatne gniazdo, nigdy nie otwiera
  ścieżek podanych przez aplikację, a nagrania zapisuje do pliku otwartego przez aplikację. Szczegóły:
  [docs/HELPER.md](docs/HELPER.md).
- **Nagrania, zestawy danych i demo syntetyczne** nie wymagają hasła.
- **Oddawanie kamery.** Gdy aplikacja się zamyka (⌘Q, zamknięcie okna, `kill -TERM`, Ctrl‑C) albo zamykasz kamerę,
  helper zatrzymuje strumienie i ponownie enumeruje urządzenie USB, więc macOS je odzyskuje (np. FaceTime może znowu
  z niej korzystać). Jeśli aplikacja kiedyś się wysypie, a kamera pozostanie nieużywalna, uruchom
  `sudo build/IntelFaceScan.app/Contents/Helpers/IntelFaceScanHelper --release -w` albo odłącz i podłącz ją ponownie.
- **Nigdy nie zabijaj helpera przez `kill -9`**, gdy rozmawia z kamerą: firmware D455 może się zawiesić do czasu
  ponownego podłączenia.
- Inne uprawnienia nie są potrzebne: aplikacja nie używa systemowego frameworka kamery, więc nie ma pytania o dostęp do
  kamery.

## Jak skanować

1. Zamocuj kamerę na statywie lub monitorze **na wysokości oczu**, mniej więcej na długość ręki. Usiądź na krześle,
   które się nie obraca; **obracaj tylko głową**, ramiona trzymaj nieruchomo.
2. Zadbaj o **równe, rozproszone światło** (bez mocnego światła z boku, bez bezpośredniego słońca). Zdejmij okulary;
   jeśli możesz, zwiąż długie włosy (głębia stereo słabo widzi cienkie włosy). Unikaj błyszczącego makijażu.
3. Wybierz **Kamera na żywo**, wpisz hasło administratora i poczekaj na podgląd. Ustaw się tak, aby wskaźnik
   odległości był zielony: ok. **0,56–0,71 m** w konfiguracji domyślnej, **0,41–0,56 m** z włączonym *Trybem bliskiego
   zasięgu* (dokładny zakres dla Twojej kamery jest widoczny pod podglądem).
4. Naciśnij **Rozpocznij skan** i podążaj za animowaną głową: prosto, góra, dół, lewo, prawo (i pozy ukośne, jeśli są
   włączone). Każda poza zbiera 3 ostre, nieruchome klatki (do ustawienia). Cały skan trwa około minuty.
5. Sprawdź podsumowanie i naciśnij **Rekonstruuj model 3D**. Na M3 Ultra rekonstrukcja skanu z 15 klatek kluczowych
   trwa ok. 20–40 s.
6. Obejrzyj model w podglądzie 3D i wyeksportuj **OBJ** (opcjonalnie PLY i surową sesję).

Wskazówki: jeśli poza nie zostaje zaliczona, przeczytaj polecenie u dołu podglądu i powód podany pod nim (najczęściej
*Podejdź bliżej* / *Odsuń się*, *Nie ruszaj się…* albo polecenie obrotu, np. *Obróć głowę w swoją lewą stronę*).
*Pomiń pozę* przechodzi dalej. *Zakończ* kończy skan z dotychczasowymi klatkami; potem naciśnij *Rekonstruuj model 3D*
w podsumowaniu, aby zbudować z nich model.

## Ustawienia

| Ustawienie | Domyślnie | Uwagi |
|---|---|---|
| Rozdzielczość głębi | 1280×720 | 848×480 ma 1,5× większy szum przy tej samej odległości; tylko na wolnym Macu |
| Preset | Wysoka dokładność | mniej błędnych punktów; Wysoka gęstość wypełnia więcej skóry |
| Tryb bliskiego zasięgu | wyłączony | przesunięcie dysparycji 50: twarz może być ok. 0,47 m od kamery, gdzie szum głębi jest ok. 40% mniejszy (teoria); włącz po zmierzeniu swojej kamery ([docs/CAMERA.md](docs/CAMERA.md)) |
| Moc lasera | maksymalna | skóra ma mało tekstury: dopasowanie stereo widzi wzór projektora |
| Filtr przestrzenny / czasowy | włączone | wygładzanie z zachowaniem krawędzi w dziedzinie dysparycji |
| Wypełnianie dziur | wyłączone | wymyślona powierzchnia jest dla fuzji gorsza niż dziura |
| Maksymalna odległość | 1,2 m | głębia dalej niż ta odległość jest odrzucana |
| Automatyczna ekspozycja na twarz | włączona | ekspozycja stereo i RGB podąża za twarzą, nie za tłem |
| Zachowaj szyję i ramiona | wyłączone | zostawia szyję i ramiona zamiast cięcia pod brodą |
| Pozy ukośne (opcjonalne) | wyłączone | dodaje górę-lewo, górę-prawo, dół-lewo, dół-prawo dla lepszego pokrycia |
| Klatki na pozę | 3 | więcej klatek uśrednia więcej szumu, skan trwa dłużej |
| Rozmiar woksela | 0,75 mm | najlepszy (lub w granicach 1% najlepszego) spośród zmierzonych wokseli 0,5, 0,75 i 1 mm przy szumie głębi 0,5–1,5 mm ([docs/RECONSTRUCTION.md](docs/RECONSTRUCTION.md)); suwak 0,5–1,5 mm |
| Powierzchnia | TSDF (GPU) | Screened Poisson jako alternatywa (gładszy, mniej dokładny w testach syntetycznych) |
| Domknij tył głowy | włączone | domyka nieskanowany tył głowy i szyję gładką, wygenerowaną powierzchnią w kolorach sąsiednich włosów i skóry, więc model jest szczelny (Podgląd nie rysuje wnętrza otwartego modelu); domknięta część jest przybliżeniem, nie pomiarem |
| Lustrzany podgląd | włączony | podgląd działa jak lustro |
| Pokaż punkty twarzy | włączone | rysuje na podglądzie na żywo punkty twarzy wykryte przez Apple Vision |

## Pliki wynikowe

- `head.obj` – wierzchołki w **milimetrach**, początek układu w środku głowy, **+Y w górę**, twarz w stronę **+Z**,
  normalne, ściany przeciwnie do ruchu wskazówek zegara, współrzędne tekstury z początkiem w lewym dolnym rogu;
  szczelny, chyba że *Domknij tył głowy* jest wyłączone.
- `head.mtl` + `head.png` – materiał i tekstura (RGB, do 4096×4096).
- `head.ply` (opcjonalnie) – binarny PLY z normalnymi i kolorami wierzchołków.
- `scan.ifsdataset/` (opcjonalnie) – surowa sesja: kolor, wyrównana głębia, maski, pozy i kalibracja kamery każdej
  klatki kluczowej; można ją otworzyć w aplikacji (**Otwórz zestaw danych…**, potem **Użyj zapisanych klatek**) albo
  zrekonstruować przez `ifscan reconstruct`.

Otwieranie OBJ: **Podgląd / Quick Look** – podwójne kliknięcie. **MeshLab** – File → Import Mesh. **Blender** – File →
Import → Wavefront (.obj), zostaw *Forward −Z, Up Y* i ustaw *Scale* importu na **0.001** (Blender traktuje jednostki
jako metry, plik jest w milimetrach); w widoku z przodu (numpad 1) twarz patrzy na Ciebie.

## Nagrania i wiersz poleceń

- W aplikacji: **Nagrywaj** zapisuje strumień kamery do pliku `.db3` (format ROS 2); **Otwórz nagranie…** odtwarza
  `.db3` lub `.bag` (np. z realsense-viewer) przez to samo przetwarzanie i ten sam prowadzony skan, bez kamery i bez
  hasła.
- `ifscan` (w pakiecie: `build/IntelFaceScan.app/Contents/MacOS/ifscan`) uruchamia ten sam potok z terminala:

```bash
ifscan reconstruct nagranie.db3 -o glowa.obj --ply glowa.ply --report raport.json
ifscan reconstruct sesja.ifsdataset -o glowa.obj --voxel-mm 0.5
ifscan reconstruct sesja.ifsdataset -o glowa.obj --open-back   # tył głowy zostaje otwarty
ifscan reconstruct synthetic-camera -o demo.obj      # animowana syntetyczna głowa, bez kamery
ifscan synth -o test.ifsdataset                      # syntetyczny zestaw danych ze wzorcem
ifscan reconstruct test.ifsdataset -o test.obj --report raport.json
ifscan eval test.obj --gt test.ifsdataset/ground_truth.obj --report raport.json   # odległość powierzchni w mm
ifscan info nagranie.db3
```

## Dokładność

Zmierzona bez kamery, na syntetycznej głowie renderowanej z modelem szumu podobnym do D455 (5 póz × 2 klatki,
woksele 0,75 mm, ICP point-to-plane; szczegóły w [docs/RECONSTRUCTION.md](docs/RECONSTRUCTION.md)). Odległość dotyczy
zmierzonej powierzchni; domknięty tył głowy (ok. 40% powierzchni domkniętego modelu syntetycznej głowy) jest
przybliżeniem: razem z nim średnia odległość całego modelu rośnie do ok. 3,6 mm.

| Przypadek | Błąd obrotu średni / maks. | Odległość od wzorca średnia / p95 |
|---|---|---|
| czysty skan syntetyczny, 1280×800 (`ifscan reconstruct`) | 0,036° / 0,067° | 0,13 mm / 0,39 mm |
| dystorsja obiektywu D455, głowa 15 cm w prawo i 6 cm w dół od osi optycznej | 0,04° / 0,08° | 0,13 mm / 0,37 mm |
| skorelowany szum głębi + falowanie | 0,29° / 0,51° | 0,34 mm / 0,92 mm |
| wszystkie zakłócenia naraz (skorelowany szum głębi + falowanie, brak głębi na włosach, sylwetki zerodowane o 6 px, 5% dziur w głębi, błąd Vision σ 3° na kąt, ekspozycja ±20%, klatki frontalne na końcu) | 0,41° / 0,87° | 0,36 mm / 0,96 mm |

Oczekiwany szum głębi D455 (teoria, subpiksel 0,08 px): 0,45 mm RMS przy 0,60 m w konfiguracji domyślnej, 0,25 mm
przy 0,45 m w trybie bliskiego zasięgu. **Dokładność i szum z prawdziwą kamerą nie zostały jeszcze zmierzone**;
[docs/CAMERA.md](docs/CAMERA.md) zawiera dokładne polecenia `--probe` do porównania 1280×720 i 848×480, High Accuracy
i High Density oraz disparity shift 0 i 50 na płaskiej ścianie i na skórze.

## Ograniczenia

- Jeszcze niesprawdzona z fizyczną kamerą D455 (zob. informację o stanie na górze).
- Tył głowy nigdy nie jest widoczny (pozy sięgają ±45° w poziomie). Domyślnie model jest tam domknięty gładką,
  wygenerowaną powierzchnią: wiarygodną, ale niezmierzoną. Wyłącz *Domknij tył głowy* (albo użyj `--open-back`), aby
  zostawić go otwartym; małe dziury są łatane w obu przypadkach.
- Niektóre założenia po stronie kamery zmierzono na danych syntetycznych i trzeba je potwierdzić z prawdziwą D455:
  że wyrównana głębia z librealsense nie ma dystorsji obiektywu, a kolor ma, oraz jak bardzo kąty głowy z Apple
  Vision odchylają się w stronę linii widzenia, gdy twarz jest poza środkiem obrazu.
- Włosy, okulary, bardzo ciemne lub błyszczące powierzchnie dają mało głębi stereo albo nie dają jej wcale.
- Kąty pozy głowy z Apple Vision mogą się mylić o kilka stopni dla niektórych twarzy; skan to kompensuje, mierząc cele
  względem Twojej pozycji na wprost, a rejestracja poprawia pozy.
- Aplikacja ma podpis ad hoc i nie jest notaryzowana.

## Biblioteki i uzasadnienie

| Biblioteka | Do czego | Dlaczego ta |
|---|---|---|
| [librealsense](https://github.com/realsenseai/librealsense) 2.58.4 + patche macOS | sterowanie kamerą, filtry, wyrównanie, nagrywanie | oficjalne SDK; patche z [realsense-viewer-macos](https://github.com/dolegadolegowski/realsense-viewer-macos) pozwalają mu działać na macOS z Apple Silicon (przejęcie USB raz na proces, brak nieskończonych oczekiwań USB, …) |
| libusb 1.0.30 (statycznie) | dostęp USB dla librealsense | tego używa librealsense na macOS; linkowany statycznie z frameworkiem Security, więc pakiet nie zależy od Homebrew |
| Apple Vision | prostokąt twarzy, punkty charakterystyczne, poza głowy, segmentacja osoby | na urządzeniu, szybki, część macOS |
| Metal | fuzja TSDF, renderowanie podglądu | natywne API GPU (bez OpenGL); shadery kompilowane w czasie działania |
| SceneKit | podgląd 3D wyniku | natywny, czyta te same dane siatki co eksport |
| [Eigen](https://eigen.tuxfamily.org) 5.0.1 (tylko części MPL2) | algebra liniowa (ICP, graf póz) | standard, same nagłówki |
| [nanoflann](https://github.com/jlblancoc/nanoflann) 1.10 | drzewa k-d do korespondencji | mały, same nagłówki, szybki |
| [PoissonRecon](https://github.com/mkazhdan/PoissonRecon) | powierzchnia screened Poisson (metoda alternatywna) | implementacja referencyjna |
| [xatlas](https://github.com/jpcy/xatlas) | atlas UV | solidne tworzenie i pakowanie wysp UV |
| [meshoptimizer](https://github.com/zeux/meshoptimizer) 1.3 | opcjonalna decymacja | szybkie upraszczanie z zachowaniem jakości |
| Apple ImageIO | zapis PNG (atlas tekstury, obrazy zestawu danych) | część macOS; ok. 2× szybszy niż stb_image_write na atlasie 4096² |

Marching cubes, ICP, graf póz, fuzja TSDF, czyszczenie siatki i mieszanie tekstur to nasz własny kod w
`Sources/FaceReconCore` (C++17 / Objective‑C++ / Metal). Licencje: [NOTICE](NOTICE) i `third_party/*/`; pakiet
aplikacji zawiera je w `Contents/Resources`.

## Architektura i dokumentacja

Aplikacja (Twoje uprawnienia: SwiftUI, ScanKit z Vision, ReconKit z FaceReconCore, CameraKit) rozmawia przez gniazdo
AF_UNIX ([docs/PROTOCOL.md](docs/PROTOCOL.md)) z helperem `IntelFaceScanHelper`, który działa jako root tylko dla
kamery na żywo (librealsense 2.58.4 z patchami, libusb: strumienie, presety, filtry, wyrównanie, IMU, nagrywanie,
odtwarzanie, przejęcie i oddanie USB). Dokumentacja (po angielsku): [ARCHITECTURE](docs/ARCHITECTURE.md),
[HELPER](docs/HELPER.md), [PROTOCOL](docs/PROTOCOL.md), [CAMERA](docs/CAMERA.md),
[RECONSTRUCTION](docs/RECONSTRUCTION.md), [TESTING](docs/TESTING.md).

## Rozwój

```bash
scripts/test.sh                 # wszystkie zestawy Swift Testing (działa z samymi Command Line Tools)
scripts/test.sh --filter ICP    # wybrane testy
cmake -S helper -B build/helper -DIFS_HELPER_TESTS=ON && cmake --build build/helper -j && ctest --test-dir build/helper
```

Źródło **Demo syntetyczne** i `ifscan reconstruct synthetic-camera` sprawdzają cały przepływ na żywo bez kamery.
CI (GitHub Actions, runner macOS 27) buduje wszystko przez `scripts/build.sh` i uruchamia wszystkie testy.

## Licencja

Apache License 2.0 (zob. [LICENSE](LICENSE) i [NOTICE](NOTICE)), tak jak librealsense. RealSense jest znakiem
towarowym RealSense, Inc.; Intel jest znakiem towarowym Intel Corporation. To nieoficjalny projekt społeczności.
