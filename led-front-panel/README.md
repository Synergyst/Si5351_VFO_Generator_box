# led-front-panel

This directory contains code, build tools, and supporting files for the LED front panel component of the Si5351 VFO Generator Box project.

## Contents

- [`VFO_Generator_WaveShare_RP2040Zero_ONLY.ino`](VFO_Generator_WaveShare_RP2040Zero_ONLY.ino): Arduino sketch for the WaveShare RP2040 Zero microcontroller, implementing VFO (Variable Frequency Oscillator) logic and interfacing with the MCU-controlled/OLED portion of front panel of the box.
- [`Cargo.toml`](Cargo.toml): Rust package manifest for the `led_mplex` project. Declares dependencies including GPIO control, error handling, command-line parsing, regular expressions, and async runtime support.
- [`build.sh`](build.sh): Shell script to build Rust binaries `led_mplex` and `led_os_agent` in release mode using Cargo.
- [`uf2_dropbox_watcher.js`](uf2_dropbox_watcher.js): Node.js script, for monitoring UF2 firmware files and automating uploads/flashing the MCU (see file for details).
- [`arduino-tools/`](arduino-tools/): Directory for Windows Arduino IDE-related utilities and scripts (PuTTY SCP not included. Please download on your own and place in `C:\arduino-tools` with the batch script).
- [`etc/`](etc/): Directory for Systemd services.
- [`src/`](src/): Source code directory for the Rust LED multiplexing agent and counterpart application.

## Build Instructions

To build the Rust binaries for the LED panel:

```
bash ./build.sh
```

## Dependencies

Rust code in this directory uses:

- `gpio-cdev`: For GPIO control
- `anyhow`: Error handling
- `ctrlc`: Signal handling (Ctrl-C)
- `tokio`: Async runtime
- `clap`: Command-line argument parsing
- `regex`: Regular expressions

See [`Cargo.toml`](Cargo.toml) for details.

## Purpose

This folder provides the interface, firmware, and support utilities for driving and managing the LED front panel, including microcontroller code, Rust-based multiplexing agents, and auxiliary scripts for development and deployment.
