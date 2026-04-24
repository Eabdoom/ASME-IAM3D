# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino/Teensy firmware for an ASME IAM3D competition robot — a remote-controlled excavator. All files are standalone `.ino` sketches. No build system, no tests. Compiled and uploaded via **Arduino IDE + Teensyduino**.

## Active File

**`Allcontrolsv2.ino`** is the only file being actively developed. All other `.ino` files are legacy prototypes for earlier hardware configurations — do not touch them unless asked.

## Hardware Platform

- **Board**: Teensy 4.1
- **RC**: ELRS receiver on Serial1 via `AlfredoCRSF` library
- **Drive**: 4-pin H-bridge, 10-bit PWM at 20kHz
- **Arm servo**: LX16A smart servo (ID 1) on Serial5, TX=pin 20 only (half-duplex, pin 21 not used)
- **PWM servos**: Rotation (pin 11), Bucket (pin 12) via `Servo` library
- **Actuators**: Arm lift (pins 22/23), Claw (pins 9/10) — digital H-bridge, no PWM

## Verified Pin Map (Allcontrolsv2.ino)

These were confirmed by a physical pin-fire test on the actual hardware.

| Pin | Function | Type |
|-----|----------|------|
| 2 | Right motor — backward | PWM 20kHz |
| 3 | Right motor — forward | PWM 20kHz |
| 4 | Left motor — backward | PWM 20kHz |
| 5 | Left motor — forward | PWM 20kHz |
| 9 | Claw — open | Digital |
| 10 | Claw — close | Digital |
| 11 | Rotation servo | Servo (50Hz) |
| 12 | Bucket servo | Servo (50Hz) |
| 20 | LX16A TX (half-duplex) | Serial5 TX |
| 22 | Arm lift — up | Digital |
| 23 | Arm lift — down | Digital |

## CRSF Channel Map (Allcontrolsv2.ino)

| CH | Function | Notes |
|----|----------|-------|
| 3 | Drive throttle | Left stick vertical. Set `THROTTLE_INVERTED = true` if forward/backward is reversed |
| 4 | Drive steering | Left stick horizontal |
| 6 | Claw open/close | Switch: high = open, low = close |
| 7 | Arm lift | Switch: high = up, low = down |
| 8 | Bucket dump | SE button: high = dump (10°), off = hold (170°) |
| 9 | LX16A arm servo | 3-pos switch: low = extend, mid = hold, high = retract |
| 11 | Rotation servo | Analog — maps to 0–180° |

CRSF raw range: 989–2012, center ≈ 1500. Switch thresholds: LOW < 1300, HIGH > 1700.

## Core Functions

**`channelNorm(ch)`** — normalizes CRSF raw to −1…+1 with 4% deadband. Returns 0.0 on failsafe (raw == 0).

**`read3PosChannel(ch)`** — returns `1` (high), `0` (mid/stop), `-1` (low) for switch channels.

**`driveMotor(forwardPin, backwardPin, norm)`** — 10-bit PWM on H-bridge pair. Inactive pin is forced to digital LOW (not PWM 0) to ensure clean H-bridge state. Has change-detection to avoid hammering registers every loop.

**`driveActuator(posPin, negPin, state)`** — digital H-bridge: state `1` = extend, `-1` = retract, `0` = stop.

**LX16A arm servo** — incremental stepping (1° per 60ms) from a 3-pos switch. Does not home on startup to avoid violent jumps.

**Bucket servo** — moves at 1°/30ms (~33°/sec). SE button high sweeps to `BUCKET_DROP_POS`, off returns to `BUCKET_START_POS`.

**Rotation servo** — moves at 1°/15ms, smoothing stick input.

## Key Tuning Constants

| Constant | Default | Purpose |
|----------|---------|---------|
| `THROTTLE_INVERTED` | `false` | Flip if robot drives backward when pushing forward |
| `BUCKET_START_POS` | `170` | Bucket rest angle |
| `BUCKET_DROP_POS` | `10` | Bucket dump angle |
| `ARM_BUS_STEP` | `1` | LX16A degrees per step — decrease to smooth |
| `ARM_BUS_MS` | `60` | LX16A step interval ms — increase to slow |

## Build & Upload

1. Open `.ino` in Arduino IDE with Teensyduino installed.
2. **Tools → Board → Teensy 4.1**
3. **Tools → USB Type → Serial**
4. Upload. Serial Monitor at **115200 baud**.

Serial debug commands (type in Serial Monitor):
- `H` — toggle LX16A arm servo between 70° and 90°
- `P` — print all 16 CRSF channel raw values
- `X` — stop all drive motors

## Required Libraries

- `AlfredoCRSF` — ELRS/CRSF protocol
- `LX16A-bus` (alecxcode) — LX16A half-duplex serial servo
- `Servo` — built-in Teensy/Arduino PWM servo library
