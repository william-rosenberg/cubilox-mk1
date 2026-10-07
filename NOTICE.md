# Notice – licenses and third-party software

## Licenses of this repository

Copyright (c) 2026 William Rosenberg.

| Part | Folder | License |
|---|---|---|
| Firmware and other source code | `firmware/` | MIT – see [`LICENSE`](LICENSE) |
| Schematic and wiring documentation | `hardware/` | CERN-OHL-P-2.0 – see [`hardware/LICENSE`](hardware/LICENSE) |
| Documentation (LaTeX sources, figures, PDF) | `docs/` | CC BY 4.0 – see [`docs/LICENSE`](docs/LICENSE) |
| Photos, renders and animations | `media/` | CC BY 4.0 – see [`media/LICENSE`](media/LICENSE) |
| Bill of materials | `BOM.csv` | CC BY 4.0 |

## Third-party software used by the firmware

The firmware is written entirely by the author. No third-party source code is included in this
repository. To build it, the following libraries are installed separately through the Arduino IDE:

| Component | Used for | License | Source |
|---|---|---|---|
| MPU6050_light (tested with 1.2.1) by Romain Fétick | reading the IMU (accelerometer, gyroscope, tilt angle) | MIT | https://github.com/rfetick/MPU6050_light |
| Arduino core for the ESP32 (tested with 3.3.10) by Espressif Systems | Arduino API, `Wire` (I²C), LEDC PWM, ADC | LGPL-2.1 | https://github.com/espressif/arduino-esp32 |
| `BluetoothSerial` library (part of the ESP32 Arduino core) | command interface over Bluetooth Classic | Apache-2.0 | https://github.com/espressif/arduino-esp32 |
| `Preferences` library (part of the ESP32 Arduino core) | storing gains and calibration in flash | Apache-2.0 | https://github.com/espressif/arduino-esp32 |
| ESP-IDF (underlying framework of the ESP32 Arduino core) | system software, Bluetooth stack | Apache-2.0 | https://github.com/espressif/esp-idf |

### Compatibility

- MIT and Apache-2.0 are permissive licenses and are compatible with the MIT-licensed firmware.
- The ESP32 Arduino core is licensed under LGPL-2.1. This repository only contains the firmware
  source code, which uses the core through its public API, so the firmware itself can be
  distributed under MIT. If a compiled firmware binary is distributed, the LGPL-2.1 terms of the
  core apply to that binary (in particular, recipients must be able to relink it with a modified
  version of the core – publishing the source code, as done here, makes this possible).

## Inspiration

The Cubilox Mk1 is inspired by the self-balancing cube of **remrc**
(https://github.com/remrc/Self-Balancing-Cube) and uses the same type of motor (Nidec 24H).
No code, CAD data or schematics were copied from that project. The scientific origin of
reaction-wheel balancing cubes is the **Cubli** developed at ETH Zurich.
