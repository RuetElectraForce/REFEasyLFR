# REFEasyLFR

**Version:** 1.0.0  
**Release date:** 2025-06-23  

A modular Arduino library for building and testing line‐following robots.  
Provides:

- **SensorReader** (with MUX or direct analog reads)  
- **MotorDriver** (supports BTS and L298 drivers)  
- **LineController** (dynamic PID control)  
- **REFEasyLFR** An overhead facade for one-line integration of the whole library

---

## Features

- **Two sensor modes**:  
  - **MUX**–driven 16-channel analog multiplexing  
  - **DIRECT** analog reads on up to 8 IR sensors (16 IR sensors can be read also with Arduino Mega (ATMega2560))
- **Two motor drivers**:  
  - **BTS7960B** (4 PWM lines)  
  - **L298N** (2 direction + 1 PWM per motor)  
- **Dynamic gain based PID**: auto‐scales P and D gains by error magnitude  
- **Calibration**: automated min/max capture & EEPROM storage. (Uses flash memory to imitate EEPROM feature for PI PICO.)
- **Facade API**: single `REFEasyLFR` object with clear setters  
- **Standalone use**: use `SensorReader.h` or `MotorDriver.h` for testing each module individually. 

---

## Supported Platforms

Tested on any Arduino‐compatible boards (Uno, Nano, Mega, etc.).  
Also supports Raspberry Pi Pico (RP2040) when using the Arduino-Pico core.

---

## Installation
1. Download the [latest release archive from GitHub](https://github.com/pololu/qtr-sensors-arduino/releases) and decompress it.
2. Rename the folder to "REFEasyLFR".
3. Drag the "REFEasyLFR" folder into the "libraries" directory inside your Arduino sketchbook directory. You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE. If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You can access them from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "REFEasyLFR". If you cannot find these examples, the library was probably installed incorrectly and you should retry the installation instructions above.

## Version history
* 1.0.0 (2025-06-23): Initial release of library on GitHub (with Arduino 1.8.xx or over compatibility).
