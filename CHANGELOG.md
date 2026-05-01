# Changelog

All notable changes to OpenDualMotorDriver are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
once a stable release is cut.

## [Unreleased]

### Added

- Initial public release of the OpenDualMotorDriver design.
- 4-layer PCB based on the Raspberry Pi RP2350 (Pico 2) and Texas Instruments DRV8412.
- Per-bridge current sensing with two ACS722 hall sensors and a divided VIN ADC.
- AS5600 magnetic encoder support on I²C0 with multi-turn position tracking.
- Cascaded position → speed → output PID controller running at 4 ms.
- ASCII and binary command APIs over USB CDC, UART, and an I²C slave at `0x16`.
- PCA9633 RGB status LED with idle, active, and fault patterns.
- PySide6 desktop GUI with manual drive, telemetry plotting, and closed-loop tuning.
- Full ASCII/binary command reference documented in `Firmware/API_REFERENCE.md`.
- Long-form build write-up in `Docs/blog-post.md`.
