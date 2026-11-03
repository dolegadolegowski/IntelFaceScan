# IntelFaceScan

Native macOS app (Apple Silicon, macOS 27.2+) for a full, detailed **3D scan of a human head** with an
**Intel RealSense D455** connected over USB‑C. Guided scan (front → up → down → left → right) with Apple Vision
head-pose tracking, rigid registration (colored ICP + pose graph), TSDF fusion on the GPU (Metal), texturing,
and export to **OBJ + MTL + PNG in millimeters**.

> 🇵🇱 **Po polsku:** natywna aplikacja na macOS (Apple Silicon, macOS 27.2+) do pełnego, dokładnego skanu 3D
> głowy kamerą Intel RealSense D455 podłączoną przez USB‑C. Prowadzony skan (prosto → góra → dół → lewo → prawo)
> ze śledzeniem pozy głowy przez Apple Vision, rejestracja (colored ICP + pose graph), fuzja TSDF na GPU (Metal),
> teksturowanie i zapis do OBJ + MTL + PNG w milimetrach.

**Status: under construction.** See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Build

```bash
git clone --recurse-submodules https://github.com/dolegadolegowski/IntelFaceScan.git
cd IntelFaceScan
scripts/build.sh        # -> build/IntelFaceScan.app
scripts/test.sh         # unit tests
```

## Credits

Built on [librealsense](https://github.com/realsenseai/librealsense) v2.58.4 with the macOS patches and tools of
[realsense-viewer-macos](https://github.com/dolegadolegowski/realsense-viewer-macos).

## License

Apache License 2.0, like librealsense. RealSense is a trademark of RealSense, Inc.; Intel is a trademark of
Intel Corporation. This is an unofficial community project.
