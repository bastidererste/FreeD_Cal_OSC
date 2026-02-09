# FreeD Cal OSC

GUI tools for calibrating FreeD lens data and sending corrected values over OSC.

This repo includes two desktop apps:

- `LinInterpol OSC Starter`: FreeD in -> calibrate FOV + focus deviation -> OSC out
- `Lens Encoder OSC Sim`: FreeD-style lens simulator for testing

## Features

- FreeD input with configurable OSC address/port
- Calibrate FOV and focus deviation from LUTs
- OSC output with configurable host/port/address
- ALGLIB Free Edition:
  - cubic spline interpolation (1D)
  - bicubic interpolation (2D)
  - linear fallback when ALGLIB is not configured
- Optional smoothing and fixed output rate
- Live plots for incoming vs calibrated output

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

Windows (Visual Studio 2022):

- Install CMake (or enable **CMake tools for Windows** in Visual Studio Installer) and reopen your terminal so `cmake` is on PATH.
- Use a **x64 Native Tools Command Prompt for VS 2022** (recommended) or PowerShell.

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DALGLIB_SOURCE_DIR="C:\path\to\alglib-cpp"
cmake --build build --config Release
```

## Run

macOS app bundles (default locations):

```bash
open build/LinInterpolOscGui_artefacts/LinInterpol\ OSC\ Starter.app
open build/LensEncoderOscSim_artefacts/Lens\ Encoder\ OSC\ Sim.app
```

If you generate Xcode projects, app bundles may be inside a config folder (`Debug` / `Release`).

## Lens Encoder OSC Sim

Simulator for testing FreeD calibration without hardware.

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

