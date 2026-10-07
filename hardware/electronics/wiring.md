# Cubilox Mk1 – Wiring

This document lists every electrical connection of the Cubilox Mk1. It matches the
schematic [`cubilox_mk1_schematic.pdf`](cubilox_mk1_schematic.pdf) and the pin definitions in
[`firmware/cubilox_mk1/config.h`](../../firmware/cubilox_mk1/config.h).

Notes on the schematic:

- The motors are numbered 1–3 in the schematic. In the firmware they are named after the IMU axis
  they rotate about: **Nidec24H1 = motor X, Nidec24H2 = motor Z, Nidec24H3 = motor Y**.
- The transistor symbol in the schematic is labelled 2N2219; the built cube uses a **2N2222**
  (both are NPN transistors and work the same way in this circuit).

## 1. Power supply

| Connection | From | To |
|---|---|---|
| Battery + (3S LiPo, 11.1 V nominal, 12.6 V full) | XT30 connector | +12 V rail → buck converter input, red wire (VIN) of all three motors, voltage divider |
| Battery − | XT30 connector | common GND rail → buck converter, all motors (black), ESP32 GND, IMU GND |
| Buck converter output (5 V) | buck converter | ESP32 VIN, white wire (5 V logic) of all three motors, buzzer |
| C4, 1000 µF electrolytic | +12 V rail ↔ GND | directly at the battery input |
| C1–C3, 100 µF electrolytic | +12 V rail ↔ GND | one at the supply of each motor |

## 2. ESP32 pin assignment

| Signal | GPIO | Notes |
|---|---|---|
| Motor X – PWM | 32 | |
| Motor X – direction | 4 | |
| Motor X – encoder A | 35 | input-only pin, no internal pull resistor |
| Motor X – encoder B | 33 | |
| Motor Y – PWM | 14 | |
| Motor Y – direction | 13 | |
| Motor Y – encoder A | 36 | input-only pin, no internal pull resistor |
| Motor Y – encoder B | 19 | |
| Motor Z – PWM | 25 | |
| Motor Z – direction | 27 | |
| Motor Z – encoder A | 34 | input-only pin, no internal pull resistor |
| Motor Z – encoder B | 18 | |
| Brake (all three motors) | 26 | one signal drives the three yellow brake wires; HIGH = free, LOW = brake |
| IMU – SDA | 21 | I²C |
| IMU – SCL | 22 | I²C |
| Buzzer | 23 | via 1 kΩ base resistor to the 2N2222 |
| Battery voltage (ADC) | 39 | input-only pin, voltage divider 33 kΩ / 10 kΩ |
| 3.3 V | 3V3 | supplies the IMU |
| VIN | VIN | 5 V from the buck converter |
| GND | GND | common GND rail |

Pins that are deliberately not used:

- GPIO 0, 2, 5, 12, 15 – strapping pins, external loads can disturb the boot process
- GPIO 6–11 – connected to the internal flash memory
- GPIO 1, 3 – UART0 (USB serial for flashing and the command interface)

The encoder outputs on the input-only pins 34, 35 and 36 work without external pull-up resistors in
this build. If an encoder ever counts irregularly, a 10 kΩ pull-up to 3.3 V on that line is the
first thing to try.

## 3. Motor connector (Nidec 24H, 8 pins, identical for all three motors)

| Pin | Wire colour | Function | Connected to |
|---|---|---|---|
| 1 | red | VIN (battery voltage) | +12 V rail |
| 2 | black | GND | GND rail |
| 3 | yellow | brake | GPIO 26 (shared) |
| 4 | green | PWM | motor-specific GPIO |
| 5 | blue | direction | motor-specific GPIO |
| 6 | white | 5 V logic supply | buck converter output |
| 7 | orange | encoder A | motor-specific GPIO |
| 8 | brown | encoder B | motor-specific GPIO |

The wire colours can differ between suppliers – the **pin order** is what matters.

The PWM input is inverted: the firmware writes `255 - |speed|`, so a duty of 255 means "stop".

## 4. IMU (MPU-6500 board)

| Pin | Connected to |
|---|---|
| VCC | ESP32 3.3 V |
| GND | GND rail |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

## 5. Buzzer driver

| Part / pin | Connected to |
|---|---|
| ESP32 GPIO 23 | R1 (1 kΩ) → base of Q1 (2N2222) |
| Q1 emitter | GND |
| Q1 collector | buzzer − |
| Buzzer + | 5 V rail |

The buzzer is an active 5 V buzzer, so it only needs to be switched on and off.

## 6. Battery voltage measurement

| Part | Connection |
|---|---|
| R2 = 33 kΩ (20 kΩ + 10 kΩ + 3.3 kΩ in series) | +12 V rail → measuring point |
| R3 = 10 kΩ | measuring point → GND |
| C5 = 100 nF ceramic | measuring point → GND |
| measuring point | GPIO 39 |

Check: with a full battery (12.6 V) the measuring point is at
12.6 V × 10 / (33 + 10) ≈ 2.93 V, safely below the 3.3 V limit of the ESP32.
The firmware multiplies the measured pin voltage by (33 + 10) / 10 = 4.3.
