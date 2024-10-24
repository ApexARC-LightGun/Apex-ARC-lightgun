# ApexArc Adaptive Retro Controller

## Overview
ApexArc is an open-source IR-based light gun controller that connects to computers via Bluetooth Low Energy (BLE). It emulates a mouse for compatibility with most shooting games and features multiple firing modes, haptic feedback, and web-based configuration.

## Features
- **Multiple Firing Modes**
  - Single Shot: One click per trigger pull
  - Burst Fire: Configurable number of rapid clicks
  - Full Auto: Continuous firing with adjustable rate
- **Bluetooth Low Energy Connectivity**
  - Shows up as a standard HID mouse device
  - Plug-and-play compatibility with most systems
- **IR Position Tracking**
  - Precise position tracking using IR camera
  - Screen calibration system for accuracy
  - Configurable smoothing for steady aim
- **Web Configuration Interface**
  - Easy setup through built-in web server
  - Adjustable settings for all features
  - Real-time feedback testing
- **Haptic Feedback System**
  - Configurable feedback duration and delay
  - Can be disabled if desired

## Hardware Requirements
- ESP32 microcontroller
- DFRobot IR Position Camera
- Trigger button (default: GPIO 12)
- Reload button (default: GPIO 14)
- Feedback device (LED/vibration motor) (default: GPIO 27)

## Software Dependencies
- WiFi.h (ESP32 WiFi functionality)
- ESPAsyncWebServer (Asynchronous Web Server)
- LITTLEFS (File system)
- ArduinoJson (JSON parsing)
- NimBLEDevice (Bluetooth Low Energy)
- DFRobotIRPosition (IR Camera interface)

## Configuration
### Default Settings
- BLE Device Name: "ApexArc"
- WiFi SSID: "ApexArc"
- WiFi Password: None (empty)
- Screen Resolution: 1920x1080
- Smoothing Factor: 5

### Configurable Parameters
- Trigger mode (Single/Burst/Full Auto)
- Burst fire settings (count and delay)
- Full auto rate of fire
- Feedback duration and delay
- Screen resolution
- GPIO pin assignments
- Movement smoothing factor

## Setup Instructions
1. Flash the code to your ESP32
2. Power on the device
3. Connect to the "ApexArc" WiFi network
4. Navigate to the web interface (192.168.4.1)
5. Configure your desired settings
6. Perform screen calibration
7. Connect via Bluetooth to your computer

## Calibration
The device requires a one-time screen calibration process:
1. Access the web interface
2. Enter calibration mode
3. Point at each corner of your screen when prompted
4. Calibration data is automatically saved

## Usage
1. Power on the device
2. Connect via Bluetooth to your computer
3. Launch your game
4. Point at the screen and use the trigger to fire
5. Use reload button for right-click actions

## Troubleshooting
- If position tracking seems off, recalibrate the device
- Check logs through web interface for debugging
- Ensure IR camera has clear view of screen
- Try adjusting smoothing factor if movement is jittery
- Verify feedback pin connection if haptics aren't working

## Building the Project
### Prerequisites
- Arduino IDE or PlatformIO
- ESP32 board support package
- Required libraries installed
- USB cable for programming

### Compilation
1. Install all required libraries
2. Select ESP32 board in Arduino IDE
3. Set appropriate upload speed and port
4. Compile and upload

## Contributing
Feel free to submit issues and pull requests for:
- Bug fixes
- New features
- Documentation improvements
- Performance optimizations

## License
This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License**. 
You may not use the code for commercial purposes. Modifications and personal use are allowed, as long as proper attribution is given and shared under the same license.
For more details, see the [LICENSE](./LICENSE) file or visit [Creative Commons](https://creativecommons.org/licenses/by-nc-sa/4.0/).

## Acknowledgments
- Uses the NimBLE library for BLE functionality
- DFRobot for the IR position camera
- ESP32 community for various examples and inspiration
