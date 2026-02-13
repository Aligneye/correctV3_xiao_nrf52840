# AlignEye

ESP32-based posture monitoring and training device.

## Hardware

- Board: Seeed XIAO ESP32-C3
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
