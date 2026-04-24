# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino/Teensy firmware repository for an ASME IAM3D robotics competition robot — a remote-controlled excavator-style machine. All files are `.ino` sketches; there is no build system, test framework, or package manager. Code is compiled and uploaded via the **Arduino IDE** or **Teensyduino** toolchain.

## Hardware Platform

- **Primary target**: Teensy 4.1 (ARM Cortex-M7, 600 MHz)
- **Legacy/alternate targets**: Teensy 4.0, Teensy 3.6, Arduino Uno R3

Key hardware:
- **Drive**: 4 H-bridge motor channels (LF/RF/LB/RB), 10-bit PWM at 20 kHz
- **RC receiver**: ELRS/CRSF receiver on Serial1 at `CRSF_BAUDRATE` (~420 kbps), read via `AlfredoCRSF` library
- **Arm bus servo**: LX16A serial servo (ID 1) on Serial5 (TX=pin 20, RX=pin 21) via `LX16A-bus` library, 0–240° range
- **PWM servos**: Rotation servo (pin 11) and bucket servo (pin 12) via standard `Servo` library
- **Linear actuators**: Arm lift (pins 7/8) and claw (pins 9/10) driven as H-bridges with `digitalWrite`

## File Inventory

| File | Purpose | Target |
|---|---|---|
| `Allcontrolsv2.ino` | **Main integration firmware** — all subsystems unified | Teensy 4.1 |
| `All_Controls.ino` | Earlier combined version with two LX16A servos | Teensy 4.1 |
| `Claw + movement.ino` | Drive + claw actuator, no arm servo | Teensy 4.0 |
| `ArmServo(Teensy4.0).ino` | Drive + single LX16A arm servo | Teensy 4.0 |
| `ArmServo(Arduino).ino` | LX16A sweep test only | Arduino Uno |
| `servo41.ino` | LX16A auto-sweep test on Serial2 | Teensy 4.1 |
| `MotorControlArduino.ino` | Drive only, CRSF + keyboard fallback, 8-bit PWM | Arduino Uno R3 |
| `Motor Control (Teensy 4.0)` | Drive only | Teensy 4.0 |
| `teensy 3.6 motor control.ino` | Drive only | Teensy 3.6 |
| `sketch_nov11b.ino` | CRSF packet-received test sketch | Arduino Mega/etc |

## CRSF Channel Map (Allcontrolsv2.ino)

| Channel | Function |
|---|---|
| CH2 | Throttle (forward/back) |
| CH4 | Steering (left/right) |
| CH6 | Arm lift actuator AND claw actuator (shared — 3-pos switch) |
| CH9 | LX16A arm bus servo (3-pos switch: low=extend, mid=hold, high=retract) |
| CH11 | Rotation PWM servo (analog stick mapped to 0–180°) |

CRSF raw values: 989–2012 µs, center ≈ 1500. Switch thresholds: LOW < 1300, HIGH > 1700.

## Core Patterns

**`channelNorm(ch)`** — normalizes CRSF raw to −1…+1 with 4% deadband. Returns 0.0 on failsafe (raw == 0).

**`read3PosChannel(ch)`** — returns 0 (low), 1 (mid/stop), or 2 (high) for switch channels.

**`driveMotor(forwardPin, backwardPin, norm)`** — maps normalized float to 10-bit PWM on an H-bridge pair.

**`driveActuator(pinA, pinB, state)`** — drives a linear actuator: state 0 = extend, 1 = stop, 2 = retract.

**LX16A arm bus servo** is stepped incrementally each loop tick (rate-limited by `ARM_BUS_MS = 60 ms`) rather than mapped directly, giving smooth motion from a 3-pos switch.

**Rotation servo** is also rate-limited (one degree per 15 ms) to smooth stick input.

## Build & Upload

There is no Makefile. Use the Arduino IDE with Teensyduino installed:
1. Open the target `.ino` in Arduino IDE.
2. Set **Tools → Board** to the correct Teensy or Arduino model.
3. Set **Tools → USB Type** to "Serial" for debug output.
4. Click **Upload** (or Sketch → Upload).

For serial debug output, open **Tools → Serial Monitor** at **115200 baud**. The main loop prints status every 250 ms.

**Debug command**: sending `H` or `h` via Serial Monitor toggles the LX16A arm bus servo between 70° and 90° for bench testing without a connected receiver.

## Required Libraries

Install via Arduino Library Manager or manually:
- `AlfredoCRSF` — ELRS/CRSF receiver protocol
- `LX16A-bus` (alecxcode) — half-duplex serial bus for LX16A smart servos
- `Servo` — built-in Arduino/Teensy library for PWM servos

## Known Issues / Design Notes

- CH6 is assigned to **both** `ARM_LIFT_CRSF_CH` and `CLAW_CRSF_CH` in `Allcontrolsv2.ino` — this is a known conflict; they cannot be independently controlled as-is.
- The bucket servo has no assigned control channel; it is held at `BUCKET_START_POS = 170°` during operation.
- The LX16A arm servo is **not** moved to a park position on startup (a deliberate fix to prevent violent jumps); `armBusAngle` starts at 0 but the physical servo is wherever it last stopped.
- `analogWriteResolution(10)` is Teensy-specific; Arduino Uno uses 8-bit PWM — `MotorControlArduino.ino` accounts for this.
