# ASME IAM3D Robot — Pin & Connector Reference

Firmware: `Allcontrolsv2.ino` | Board: **Teensy 4.1**

---

## Teensy 4.1 Pin Map

| Pin | Connected To | Signal Type | Notes |
|-----|-------------|-------------|-------|
| 0 | ELRS Receiver (out → TX) | Serial1 RX | CRSF input |
| 1 | ELRS Receiver (in ← RX) | Serial1 TX | CRSF output |
| 2 | H-Bridge RIGHT — IN2 | PWM 10-bit 20kHz | Right Backward |
| 3 | H-Bridge RIGHT — IN1 | PWM 10-bit 20kHz | Right Forward |
| 4 | H-Bridge LEFT — IN2 | PWM 10-bit 20kHz | Left Backward |
| 5 | H-Bridge LEFT — IN1 | PWM 10-bit 20kHz | Left Forward |
| 7 | Rotation Servo — signal | PWM 50Hz (Servo) | Moves smoothly (1°/15ms) instead of snapping |
| 9 | Claw Actuator — IN1 | Digital OUT | Claw Open |
| 10 | Claw Actuator — IN2 | Digital OUT | Claw Close |
| 12 | Bucket Servo — signal | PWM 50Hz (Servo) | SE button controlled |
| 20 | LX16A Bus Servo — TX | Serial5 TX | Half-duplex, single wire only |
| 22 | Arm Lift Actuator — IN1 | Digital OUT | Arm Up |
| 23 | Arm Lift Actuator — IN2 | Digital OUT | Arm Down |

---

## CRSF Channel Map (EdgeTX)

| CH | Function | Physical Control | Values |
|----|----------|-----------------|--------|
| 3 | Drive — Throttle | Left stick vertical | −1 (back) → +1 (forward) |
| 4 | Drive — Steering | Left stick horizontal | −1 (left) → +1 (right) |
| 6 | Claw open / close | Switch | High = Open, Low = Close |
| 7 | Arm lift up / down | Switch | High = Up, Low = Down |
| 8 | Bucket dump | SE button | High = Dump, else = Hold |
| 9 | LX16A Arm Servo | 3-pos switch | Low = extend, High = retract |
| 11 | Rotation Servo position | Knob or stick | Maps to 0–180° |

---

## H-Bridge Wiring (Drive Motors)

Both left-side motors are wired in parallel to one H-bridge channel; same for right side.

| Side | Forward pin | Backward pin |
|------|------------|--------------|
| Left | Pin 5 | Pin 4 |
| Right | Pin 3 | Pin 2 |

Motor PWM: 10-bit resolution (0–1023), 20 kHz switching frequency (silent to human hearing).

---

## LX16A Smart Servo (Arm Bus)

| Property | Value |
|----------|-------|
| Servo ID | 1 |
| Interface | Serial5 half-duplex |
| TX pin | 20 |
| RX pin | 21 |
| Baud rate | 115200 |
| Angle range | 0–240° |

---

## Bucket Servo Positions

| State | Angle | Trigger |
|-------|-------|---------|
| Rest / Hold | 170° (`BUCKET_START_POS`) | SE switch off |
| Dump | 10° (`BUCKET_DROP_POS`) | SE switch high |

Servo moves at 1°/30ms (~33°/sec, ~5 seconds full travel). Adjust `BUCKET_DROP_POS` in code if the dump angle needs tuning.

---

## FlexPWM Timer Layout (Teensy 4.1)

Understanding this prevents oscillation caused by servo/motor timer conflicts.

| Pin | FlexPWM Module & Submodule | Used For |
|-----|---------------------------|---------|
| 2 | FlexPWM4, SM2, B | Right Backward motor |
| 3 | FlexPWM4, SM2, A | Right Forward motor |
| 4 | FlexPWM2, SM0, A | Left Backward motor |
| 5 | FlexPWM2, SM1, A | Left Forward motor |
| 7 | FlexPWM1 | Rotation Servo ✅ no conflict |
| 12 | FlexPWM1, SM3 | Bucket Servo ✅ no conflict |

> **Rule:** Servos and motors on the **same submodule** conflict because the Servo library forces 50Hz, overriding the motor's 20kHz. Pin 7 (FlexPWM1) and pin 12 (FlexPWM1 SM3) are on completely different modules from the motor pins (FlexPWM2 / FlexPWM4), so there is no conflict.

---

## Software Tuning Constants

| Constant | Location | Purpose |
|----------|----------|---------|
| `THROTTLE_INVERTED` | line ~41 | Set `true` if robot drives backward when pushing forward |
| `BUCKET_DROP_POS` | line ~80 | Dump angle in degrees (default 10°) |
| `BUCKET_START_POS` | line ~79 | Rest angle in degrees (default 170°) |
| `ARM_BUS_MS` | line ~57 | LX16A step interval — increase to slow the arm |
| `ARM_BUS_STEP` | line ~56 | LX16A degrees per step — decrease to smooth the arm |
