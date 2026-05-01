# OpenDualMotorDriver — Desktop GUI

A PySide6 desktop control application for the OpenDualMotorDriver firmware. It speaks the same ASCII and binary protocol documented in [`../../Firmware/API_REFERENCE.md`](../../Firmware/API_REFERENCE.md), so anything you can do over a raw serial terminal you can do here too — just with sliders, plots, and buttons.

## Features

- USB serial autodetect with manual port and baud overrides.
- Manual drive sliders for both H-bridges with permille readout.
- Live oscilloscope-style plots for VIN, per-bridge currents, and per-bridge commanded power.
- Fault, OTW, and status-LED indicators wired to the firmware status flags.
- Closed-loop tab with mode selector (manual / speed / position), reference fields in counts/rpm/degrees/turns, PID gain entry, and `M210` "zero current position" button.
- Periodic binary status frames are decoded and pushed into the plot buffers; periodic text reports stay readable in the log pane.

## Install

```bash
pip install -r requirements.txt
```

PySide6 already ships with the serial and chart modules used here, so a separate `pyserial` install is not needed.

## Run

From this folder:

```bash
python pico_motor_driver_gui.py
```

Optional arguments:

```bash
python pico_motor_driver_gui.py --port COM5 --baud 115200 --window 60
```

| Flag | Meaning |
|---|---|
| `--port` | Serial port to connect to (e.g. `COM5` on Windows, `/dev/ttyACM0` on Linux, `/dev/cu.usbmodemXXXX` on macOS). |
| `--baud` | Serial baud rate. Defaults to `115200` to match `BoardConfig::USB_SERIAL_BAUD` in firmware. |
| `--window` | Visible window length, in seconds, for the live plots. |

## Files

| File | What it is |
|---|---|
| `pico_motor_driver_gui.py` | The PySide6 main window, plot panels, and command/event glue. |
| `pico_motor_driver_protocol.py` | Pure-Python encoder/decoder for the ASCII commands and the `0xA5`-framed binary protocol. Use this on its own if you want to script the board without the GUI. |
| `requirements.txt` | Pinned-loose dependency list (currently just `PySide6`). |

## Scripting the board without the GUI

`pico_motor_driver_protocol.py` is GUI-free. You can import it and write your own automation:

```python
from pico_motor_driver_protocol import open_port, send_text, send_binary, parse_status_frame

port = open_port("/dev/ttyACM0", 115200)
send_text(port, "M200 H0 S1")            # enable bridge 0
send_text(port, "M203 H0 S250")          # 25.0% manual drive
```

(See the protocol module for the full set of helpers and the binary status-frame layout.)

## License

Released under the MIT license — see the top-level [`../../LICENSE`](../../LICENSE).
