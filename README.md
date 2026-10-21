# 5G8atv-rf-hackrf-decoder (fpvdec)

[日本語 README はこちら](README.ja.md)

A software receiver for **5.8 GHz analog FPV video (FM-ATV, NTSC)** using a
HackRF One — real-time color decoding of drone VTX signals on your PC.
C++20 + libhackrf + SDL2, no GNU Radio required.

Forked from
[GOROman/famicom-rf-hackrf-decoder](https://github.com/GOROman/famicom-rf-hackrf-decoder)
(a Famicom VHF RF decoder); the NTSC decoder core is inherited, the RF
front-end was rebuilt for FM-ATV.

Demo video (click to watch on YouTube):

[![Demo video](https://img.youtube.com/vi/dDNk-uRtcGw/maxresdefault.jpg)](https://www.youtube.com/watch?v=dDNk-uRtcGw)

Live decode of a real 25 mW whoop VTX — HackRF One on the left, fpvdec
running on the laptop (Betaflight OSD visible):

![Setup: HackRF One + whoop + fpvdec live decode](docs/IMG_9715.jpeg)

## Supported channels

All 40 standard 5.8 GHz analog FPV channels, selected as band letter +
channel number (e.g. `--channel F4`):

| Band | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| **A** (Boscam) | 5865 | 5845 | 5825 | 5805 | 5785 | 5765 | 5745 | 5725 |
| **B** (Boscam) | 5733 | 5752 | 5771 | 5790 | 5809 | 5828 | 5847 | 5866 |
| **E** | 5705 | 5685 | 5665 | 5645 | 5885 | 5905 | 5925 | 5945 |
| **F** (Airwave) | 5740 | 5760 | 5780 | 5800 | 5820 | 5840 | 5860 | 5880 |
| **R** (Raceband) | 5658 | 5695 | 5732 | 5769 | 5806 | 5843 | 5880 | 5917 |

A two-stage **AFC** tracks the VTX automatically: a coarse acquisition
stage pulls in transmitters that start 2–3 MHz off after a cold power-on,
and a fine stage keeps the receiver centered as the VTX drifts while
warming up (real units drift ~1 MHz in the first minutes).

## Hardware

[HackRF One](https://greatscottgadgets.com/hackrf/one/) receive-only at
10 MSPS. The stock whip antenna works at desk range; for real distance use
a **5.8 GHz circular-polarized patch/helical antenna** — FPV VTX are
circular-polarized, so a linear whip loses 3 dB plus deep multipath fades.
The +14 dB RF amp is enabled by default (`--no-amp` to disable).

## Build

### Windows (MSVC)

Requires Visual Studio 2022, CMake, [PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/)
(for libhackrf) and the SDL2 VC development package extracted to
`third_party/SDL2` (with the headers copied into `include/SDL2/`).

```sh
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

`SDL2.dll` / `hackrf.dll` are copied next to the exe automatically.

### macOS / Linux

```sh
brew install hackrf sdl2 cmake pkg-config   # or apt equivalents
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Usage

```sh
# Live: tune a channel preset (AFC + auto gain take it from there)
./build/Release/fpvdec --channel F4

# Explicit frequency / inverted-polarity VTX
./build/Release/fpvdec --freq 5806e6 --invert

# Record raw IQ while decoding, replay later
./build/Release/fpvdec --channel R1 --record cap.cs8
./build/Release/fpvdec --input file --file cap.cs8 --loop

# Headless: dump decoded frames as PPM (debug / verification)
./build/Release/fpvdec --input file --file cap.cs8 --dump-frames out_ --frames 30
```

### Options

| Option | Description |
|---|---|
| `--channel NAME` | FPV channel preset, band A/B/E/F/R + 1-8 (default F4) |
| `--freq HZ` | explicit carrier frequency |
| `--dev HZ` | FM peak deviation (default 5e6) |
| `--invert` | flip discriminator polarity (non-standard VTX) |
| `--no-afc` | disable automatic frequency centering |
| `--gain auto\|fast\|manual` | RF gain control (default auto). `fast` is the experimental C5VRX-style controller, see below |
| `--lna N` / `--vga N` | fixed gains, imply `--gain manual` |
| `--no-amp` | disable the +14 dB RF preamp (default on) |
| `--rate HZ` | sample rate (default 10e6; 8e6 keeps coarse color, ≤7e6 grayscale, ~6e6 minimum) |
| `--lpf HZ` | optional post-detector video LPF, e.g. 4.2e6 |
| `--mode color\|gray` | decode mode (default color) |
| `--sat F` / `--hue DEG` | color trims |
| `--overscan F` | horizontal crop per side (default 0 — FPV OSD sits at the edges) |
| `--record PATH` | tee raw IQ to .cs8 while decoding |
| `--dump-frames PREFIX` / `--frames N` | headless PPM frame dump |
| `--dump-composite PATH` | dump post-AGC composite as f32 (debug) |
| `--spectrum` | print PSD and exit |

### Keys / on-screen display

- `q` / ESC quit, `a` next channel (E2→E1→F1→F3→F4→F5), `A` (shift) gain auto/fast/manual,
  `l`/`L` LNA, `g`/`G` VGA,
  `b` RF amp, `c` color/gray, `o` OSD on/off, `s` screenshot, `h` help
- `←`/`→` tune ±50 kHz, `↑`/`↓` ±1 MHz, `r` CRT emulation,
  `v` IQ record start/stop (10 MSPS ≈ 20 MB/s — keep clips short)
- **Top left (big green)**: nearest FPV channel name (`F4`)
- **Top line (yellow)**: `V:OK H:OK 59.94FPS 5800.39 F4 45MS AUTO L40 V20 AMP`
  — sync locks, decoded field rate, AFC-tracked VTX frequency, latency,
  gain state; a red `CLIP` appears on ADC clipping. In `FAST` gain mode the
  line ends with `W<n> <p>%`: gain writes so far and the share of USB
  transfers whose median |IQ| sat in the healthy band

### Fast gain (`--gain fast`, experimental)

A test port of the gain control in [C5VRX](https://github.com/Twotoz/C5VRX)
(Direct Gain V3-V5), the ESP32-C5 FPV receiver, to compare against the
default peak AGC (`src/dsp/gain_ctl.hpp`, unit tests in
`tests/test_gain_ctl.cpp`):

| | `auto` (default) | `fast` |
|---|---|---|
| Measures | ADC peak | P50/P95 of \|IQ\|, rail fraction, lag-1 carrier coherence |
| Cadence | 500 ms | every USB transfer (~13 ms at 10 MSPS) |
| In range | hold | zero-write hold while P50 is 24-48 |
| Out of range | VGA ±2 dB (or LNA ±8 dB) per step | one predictive jump to P50 = 34 |
| Overload | −6 / −12 dB | emergency drop, never damped |
| No carrier | ramps up | listens at high gain; noise never pulls it down |
| Anti-hunt | — | two quick reversals → 4 consecutive windows needed for 1.5 s |

Writes are logged as `FAST AGC: ...`. Thresholds are first guesses for 8-bit
samples; the carrier-coherence threshold in particular needs checking on
real hardware.

## How it works

```
HackRF One (10 MSPS, tuned to the channel; AFC re-centers on the VTX)
  → complex DC blocker
  → 4.9 MHz complex channel LPF (flat through the chroma upper sideband)
  → FM quadrature discriminator (AVX2; sync-tip-positive polarity)
  → AGC (per-line sync-tip / back-porch tracking = pedestal clamp)
  → sync separation on a dedicated 1 MHz low-pass stream (~7 dB noise margin)
  → flywheel line PLL with interlace-aware half-line re-anchoring
  → 3.58 MHz chroma BPF, per-line burst measurement, U/V demod
  → YUV→RGB 640×480 (fields line-doubled, ~59.94 updates/s)
  → triple buffer → SDL2 display
```

Compared to the Famicom original: FM instead of AM detection, two-stage
AFC (mean-frequency acquisition + occupied-band-midpoint tracking with
feed-forward level compensation into the AGC), automatic RF gain from ADC
peak statistics, a narrowband sync path for weak signals, and real-camera
interlace handling. The DSP hot paths (FIR kernels, discriminator,
atan2) are AVX2-vectorized; the full chain runs ~2.4× real time at
10 MSPS on an i7-9700K.

## Tests

```sh
./build/Release/synth_fm            # FM color bars → decode → assert RGB
./build/Release/synth_fm bars.cs8   # write synthetic IQ for E2E tests
ctest --test-dir build -C Release
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| Snow, never syncs | check the VTX channel; watch the console for `AFC (coarse)` pulling in; try `--spectrum` |
| Black-and-white picture that looks inverted | non-standard VTX polarity → `--invert` |
| Weak / sparkly at range | use a circular-polarized 5.8 GHz antenna; keep the RF amp on |
| Distorted up close | auto gain backs off, but very close in also toggle the amp off (`b`) |
| No color | signal too weak for burst detection, or `--rate` below 8 MSPS |
| Video latency climbing | CPU can't keep up — lower `--rate` (8e6 / 6e6) |

## License / disclaimer

GPL-3.0-only — see [LICENSE](LICENSE).

- Based on [famicom-rf-hackrf-decoder](https://github.com/GOROman/famicom-rf-hackrf-decoder)
  by GOROman. That code is MIT-licensed; its copyright and permission notice
  are kept in [LICENSES/MIT-famicom-rf-hackrf-decoder.txt](LICENSES/MIT-famicom-rf-hackrf-decoder.txt)
  and still apply to the parts taken from it.
- The experimental fast gain control (`--gain fast`, `src/dsp/gain_ctl.hpp`)
  and the C5VRX receive-chain emulation (`--c5`) adapt the Direct Gain
  V3–V5 gain control and receive-chain design of
  [C5VRX](https://github.com/Twotoz/C5VRX) by Twotoz and the C5VRX
  contributors (GPL-3.0-only). Visit the
  [official C5VRX website](https://twotoz.github.io/C5VRX/) for the original
  project and its Discord invite. This is not an official C5VRX release.
- `third_party/SDL2` keeps its own license (zlib, `third_party/SDL2/LICENSE.txt`).

Receive-only tool. It never transmits with the HackRF One. Receiving FPV
video may require an amateur radio license and/or VTX registration in
your country (it does in Japan) — fly and receive legally.
