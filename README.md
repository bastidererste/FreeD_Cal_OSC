# LinInterpol OSC Starter

Minimal C++ GUI starter app for:

`OSC in -> interpolation -> OSC out`

Built with JUCE + CMake so you can extend it for math-heavy interpolation logic.

This repo now includes 2 desktop apps:

- `LinInterpol OSC Starter`: input -> LUT/interpolation -> output
- `Lens Encoder OSC Sim`: broadcast-style focal/encoder OSC generator

## Features

- GUI fields for:
  - input port + input OSC address
  - output host + output port + output OSC address
  - calibration input range (`Input Min`, `Input Max`)
  - LUT loader (`Load LUT`)
  - smoothing time (ms)
  - output rate (Hz)
- Accepts first OSC argument as float/int/string-number
- Optional LUT calibration before smoothing/output:
  - incoming raw OSC value is normalized from `Input Min..Input Max` to `0..1`
  - normalized position is interpolated across the LUT
  - with ALGLIB Free Edition configured: cubic spline interpolation
  - without ALGLIB: linear interpolation fallback
- Interpolation stage uses an exponential one-pole smoother:
  - `y = y + alpha * (target - y)`
  - `alpha = 1 - exp(-dt / tau)`
- Live plot shows incoming raw value and outgoing interpolated value over time
- Cartesian LUT plot shows:
  - current input point
  - LUT target point at that exact input
  - ALGLIB cubic interpolation curve (for comparison)
  - static LUT curve (no scrolling/autoscale)
- Sends interpolated values continuously to output OSC endpoint

## Build

Requirements:

- CMake 3.22+
- C++20 compiler
- JUCE source:
  - Option A: internet access on first configure (JUCE fetched from GitHub)
  - Option B: local JUCE checkout via `-DJUCE_SOURCE_DIR=/path/to/JUCE`

```bash
cmake -S . -B build
cmake --build build -j
```

Using a local JUCE checkout:

```bash
cmake -S . -B build -DJUCE_SOURCE_DIR=/path/to/JUCE
cmake --build build -j
```

Enable ALGLIB Free Edition cubic interpolation:

```bash
cmake -S . -B build \
  -DJUCE_SOURCE_DIR=/path/to/JUCE \
  -DALGLIB_SOURCE_DIR=/path/to/alglib
cmake --build build -j
```

## Run

macOS app bundles (default locations):

```bash
open build/LinInterpolOscGui_artefacts/LinInterpol\ OSC\ Starter.app
open build/LensEncoderOscSim_artefacts/Lens\ Encoder\ OSC\ Sim.app
```

If you generate Xcode projects, app bundles may be inside a config folder (`Debug` / `Release`).

## Lens Encoder OSC Sim

The second app simulates a broadcast lens zoom encoder with:

- direct zoom position input (`0..1`)
- non-linear encoder mapping (`gamma`)
- backlash and noise in encoder counts

Default OSC output:

- host: `127.0.0.1`
- port: `9100`
- address: `/lens/encoder`
- args:
  - `int`: encoder count (first arg, useful for your existing parser)
  - `float`: focal length mm
  - `float`: zoom position `0..1`

## LUT Format

Load a `.txt` or `.csv` file with at least `4` points in one of these formats:

- Output-only format (legacy):
  - one list of numeric values
  - input axis is derived from `Input Min..Input Max`
- Explicit input->output format (recommended for lens->FOV):
  - one `input,output` pair per line
  - input axis comes from the first column
  - output axis comes from the second column

Values can be comma, semicolon, whitespace, or newline separated.

Examples:

```text
# output-only
0.0, 0.2, 0.45, 0.8, 1.0

# explicit input,output pairs
0,105
4096,70
8192,42
12288,20
16383,5
```

Example file in this repo:

- `examples/lut_lens_to_fov_0_16383_to_5_105_10pts.csv`

## Next Extension Points

- Replace the one-pole smoother with your interpolation model in `timerCallback()` in `src/Main.cpp`
- Add per-message timestamps / sequence IDs for advanced interpolation/extrapolation
- Add multi-channel input (bundle/messages with multiple args)
