# Hardware

This directory holds the design files needed to fabricate and assemble the OpenDualMotorDriver PCB.

## Layout

```
Hardware/
├── Schematics/      Page-by-page schematic exports (PNG)
├── PCB-Renders/     Layer-by-layer board renders and 3D views (PNG)
├── Gerbers/         Manufacturer-ready gerber + drill files (zip)
```

## Schematic pages

| Page | File | What it covers |
|---|---|---|
| 1 | `Schematics/1_PowerElectronics.PNG` | Input filtering, bulk capacitance, and the DRV8412 power section. |
| 2 | `Schematics/2_MotorDriver.PNG` | DRV8412 PWM/reset wiring, ACS722 hall current sensing for both bridges, motor connectors. |
| 3 | `Schematics/3_RP2350.PNG` | Raspberry Pi Pico 2 (RP2350) carrier, AS5600 encoder header, PCA9633 status LED, software-switched I²C pull-ups, UART/SPI/I²C slave headers, VIN divider. |

## Stackup and PCB renders

The design is a 4-layer board. Layer renders live in `PCB-Renders/`:

| File | Layer |
|---|---|
| `1_TopLayer.PNG` | L1 — top copper |
| `2_GNDLayer1.PNG` | L2 — internal ground plane |
| `3_GNDLayer2.PNG` | L3 — internal ground plane |
| `4_BottomLayer.PNG` | L4 — bottom copper |
| `PCB_top.PNG` / `PCB_bottom.PNG` | 2D top and bottom views |
| `PCB_3D_1.PNG` / `PCB_3D_2.PNG` | 3D renders |

## Gerbers

Production-ready gerber + drill files are zipped under `Gerbers/`. They can be uploaded directly to JLCPCB, PCBWay, OSH Park, or any other fab that accepts standard gerber bundles.

## Bill of materials

The BOM is provided on Google Sheet Link: [BOM](https://docs.google.com/spreadsheets/d/1Zc9ybkn5q93EQncn_3QRWnhDOZEBM622GhTFV9P49-4/edit?gid=0#gid=0)

## Assembly notes

A few practical pointers for first-time builders:

- The DRV8412 is a thermally enhanced PowerPAD package. Solder the pad to the copper pour underneath it; without that thermal connection the device will throttle the bridges via the OTW input.
- The board includes a footprint for an inline magnet directly above the AS5600. If you fit the encoder, place a diametrically magnetized magnet on the motor shaft so the field sweeps across the AS5600 die as the motor turns.
- The two software-switched I²C pull-ups default to enabled. If you connect another bus master (or a sensor add-on with its own pull-ups), you can disable them in firmware via `M104`/`M260` or the `0x04` binary command.
- Both UART and the I²C slave on `0x16` are wired to dedicated headers. They are not isolated, so plan your harness accordingly if the driver will sit at a different ground potential from the host.
