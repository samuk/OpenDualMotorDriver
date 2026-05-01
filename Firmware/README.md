# Firmware

Arduino-flavored C++ firmware for the OpenDualMotorDriver, targeting the Raspberry Pi RP2350 (Pico 2) on the [Earle Philhower arduino-pico core](https://github.com/earlephilhower/arduino-pico).

The full ASCII and binary command set is documented in [`API_REFERENCE.md`](API_REFERENCE.md). This file just covers how the firmware is laid out and how to build it.

## Module layout

```
Firmware/
├── API_REFERENCE.md             Command and telemetry API reference
└── PicoDualMotorDriver/
    ├── PicoDualMotorDriver.ino  setup() / loop() — wires the modules together
    ├── BoardConfig.h            Pin map, baud rates, PWM/PID/IO defaults
    ├── DualMotorController.{h,cpp}
    ├── As5600Encoder.{h,cpp}
    ├── PidController.h          Header-only PID with conditional anti-windup
    ├── ClosedLoopController.{h,cpp}
    ├── CommandProcessor.{h,cpp} ASCII + binary parsers, I²C slave glue
    └── StatusLed.{h,cpp}        PCA9633 RGB status LED driver
```

Each module has a single responsibility:

- **`DualMotorController`** owns the DRV8412 bridges. It computes PWM duty from a signed permille command, polls the FAULT and OTW open-drain inputs, samples ACS722 currents and the VIN divider, and tracks per-bridge enable state and direction inversion.
- **`As5600Encoder`** reads the raw 12-bit angle, applies a software zero offset and direction inversion, unwraps the angle into a signed multi-turn count, and produces a filtered velocity estimate at the closed-loop period.
- **`PidController`** is a header-only PID with output clamping, an integral clamp, and conditional anti-windup that only integrates when the output is not saturated, or when the error is moving the controller out of saturation.
- **`ClosedLoopController`** owns one of the two bridges at a time. It runs in three modes — manual, speed PID, and position PID — and cascades the position loop into the speed loop (so the position PID's output is a speed reference).
- **`CommandProcessor`** parses ASCII commands (`M…` codes), binary frames starting with `0xA5`, and I²C slave traffic. It emits one-shot replies, optional periodic text/binary reports on USB and UART, and stores the last response for I²C reads.
- **`StatusLed`** drives a PCA9633 with three RGB blink patterns: idle, "bridge enabled", and "fault active".

## Building and flashing

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or 2.x).
2. Add this URL to *File → Preferences → Additional Board Manager URLs*:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. Open *Tools → Board → Boards Manager*, search for **Raspberry Pi Pico/RP2040/RP2350**, and install it.
4. Open `PicoDualMotorDriver/PicoDualMotorDriver.ino`.
5. Pick **Raspberry Pi Pico 2** as the board (or whichever RP2350 module you populated).
6. Hold `BOOTSEL`, plug the board into USB, release `BOOTSEL`, and click **Upload** in the IDE.

After boot the firmware prints a banner on USB CDC and on UART:

```
PicoDualMotorDriver ready. Use M100 for help.
```

## Quick smoke test over serial

Open a serial terminal at 115200 baud (or use the GUI under `Software/gui/`) and try:

```text
M100              # short help
M101              # one-shot human-readable status
M200 H0 S1        # enable bridge 0
M203 H0 S250      # 25.0% manual drive on bridge 0
M203 H0 S0        # stop
M201              # stop all bridges
```

For closed-loop walkthroughs (speed PID, position PID, encoder zeroing, motor direction inversion) see the worked examples at the bottom of [`API_REFERENCE.md`](API_REFERENCE.md).

## License

This firmware is released under the MIT license — see the top-level [`../LICENSE`](../LICENSE).
