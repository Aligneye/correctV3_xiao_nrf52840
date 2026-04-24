# AlignEye

XIAO nRF52840-based posture monitoring and training device.

## Hardware

- Board: Seeed XIAO nRF52840
- Sensor: Adafruit LIS3DH Accelerometer
- Framework: Arduino (PlatformIO)

## Features

- Real-time posture tracking
- Posture training with customizable delay settings
- Vibration therapy
- Bluetooth Low Energy (BLE) connectivity
- Battery monitoring
- Auto-off functionality
- Calibration system
- Deep sleep power management

## VS Code Setup

1. Install the **PlatformIO IDE** extension in VS Code.
2. Open this project folder in VS Code.
3. Build using PlatformIO:
   - `PlatformIO: Build` (or run `pio run`)
4. Upload firmware:
   - `PlatformIO: Upload` (or run `pio run --target upload`)
5. Open serial monitor:
   - `PlatformIO: Monitor` at `115200`

## Building

This project uses PlatformIO. To build:

```bash
pio run
```

To upload to device:

```bash
pio run --target upload
```

## Configuration

Edit `include/config.h` to modify pin assignments and other settings.

## License

[Add your license here]
