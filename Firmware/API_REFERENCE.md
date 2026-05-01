# Pico Dual Motor Driver API Reference

This document describes the current firmware command and telemetry API implemented by:

- `CommandProcessor.h`
- `CommandProcessor.cpp`
- `DualMotorController.h`
- `DualMotorController.cpp`
- `ClosedLoopController.h`
- `ClosedLoopController.cpp`
- `As5600Encoder.h`
- `As5600Encoder.cpp`
- `BoardConfig.h`

It is written against the current code in this repository, not an older GUI protocol.

## 1. Overview

The firmware exposes three control/telemetry paths:

- USB serial text and binary protocol
- UART serial text and binary protocol
- I2C slave command/response protocol on `0x16`

There are two motor bridges:

- `H0`
- `H1`

Only one bridge at a time can be the active closed-loop bridge. Closed-loop commands that select a different bridge can stop the previously selected bridge and clear loop state.

## 2. Important Defaults And Limits

From `BoardConfig.h`:

- USB serial baud: `115200`
- UART baud: `115200`
- I2C slave address: `0x16`
- AS5600 address: `0x36`
- PWM frequency: `20000 Hz`
- PWM active ceiling: `950 / 1000` = `95% duty`
- Closed-loop control period: `1 ms`
- Max closed-loop output command: `+/-1000 permille`
- Max speed reference: `+/-262144 counts/s`
- Max position reference: `+/-1000000 counts`
- AS5600 counts per rev: `4096`
- Encoder reduced resolution range: `10..12 bits`
- Driver fault clear holdoff after fault assert: `1000 ms`
- Current sensor model used by firmware math: ACS722, zero at `1650 mV`, sensitivity `132 mV/A`

Derived useful units:

- `1000 permille` = full commanded output
- `4096 counts` = `1 turn` = `360 degrees`
- `262144 counts/s` = `3840 rpm`
- `1000000 counts` = about `244.141 turns` = `87890.625 degrees`

## 3. Transport Rules

### 3.1 ASCII Transport

- Commands are case-insensitive.
- Parameter prefixes are case-insensitive.
- Separators can be:
  - space
  - tab
  - comma
- `\r` is ignored.
- `\n` terminates the command.
- Maximum text command length is `119` visible characters plus terminator.
- If a line exceeds the limit, the firmware returns:
  - `error command too long`

ASCII replies are text and end with `\r\n`.

### 3.2 Binary Transport

Binary frames are:

```text
[0] start byte = 0xA5
[1] length     = command byte + payload bytes
[2] command
[3..n] payload
[last] XOR checksum of bytes [1] through [last-1]
```

All multi-byte integer and float fields are little-endian.

If a binary request is invalid, the response is:

```text
start=0xA5, command=0x7F, payload=[request_command, error_code]
```

### 3.3 I2C Behavior

- The firmware creates an I2C slave on `0x16`.
- You can send either text or binary commands to it.
- The last generated response is stored internally.
- An I2C read returns the most recent response buffer.
- There is no push-stream over I2C. Periodic reports only go to USB/UART.

## 4. Core Concepts

### 4.1 Bridge Selection

The closed-loop controller owns one active bridge at a time.

Commands that select a bridge for closed-loop context include:

- `M202`
- `M204`
- `M205`
- `M206`
- `M207` with `H`
- `M208` with `H`
- `M210` with `H`
- `M211` with `H`
- Binary `0x12`, `0x14`, `0x15`, `0x16`, `0x17`, `0x20`

If a different closed-loop bridge is selected:

- the previously selected bridge is stopped
- the loop is forced to manual
- manual command is cleared
- speed reference is cleared
- position reference is cleared
- the loop is reset

### 4.2 Loop Modes

- `0` = `manual`
- `1` = `speed`
- `2` = `position`

Important behavior:

- Any manual-drive command applied to the active closed-loop bridge forces that loop back to manual mode and resets it.
- The position controller is cascaded:
  - outer loop: position PID
  - inner loop: speed PID

### 4.3 Units

- Manual/output/bridge power commands use `permille`
  - `-1000..1000`
- Speed references use either:
  - counts/s
  - rpm
- Position references use either:
  - counts
  - degrees
  - turns
- Status text reports PID gains in milli-units
  - example `200/20/0` means `0.200 / 0.020 / 0.000`

## 5. ASCII Command API

Each command below shows syntax, behavior, response, and important notes.

### 5.1 `M100` / `M115` / `HELP` / `?`

Purpose:

- Return short built-in help text.

Syntax:

```text
M100
M115
HELP
?
```

Response:

- One-line summary of the command set.

### 5.2 `M101` / `M114`

Purpose:

- Return one-shot human-readable status.

Syntax:

```text
M101
M114
```

Response format:

```text
ok up=<ms> err=<0|1> fault=<0|1> otw=<0|1> holdoff=<ms> led=<0|1> fast=<0|1>
en=[<0|1>,<0|1>] pwr=[<permille>,<permille>] duty=[<0..950>,<0..950>] dir=[<0|1>,<0|1>]
vin=<volts> i1=<amps> i2=<amps> pullups=[<0|1>,<0|1>]
loop={H<bridge> mode=<manual|speed|position> en=<0|1> enc=<0|1> raw=<0..4095> red=<reduced>
pos=<counts> spd=<counts/s> refS=<counts/s> refP=<counts> out=<permille> man=<permille>
pidS=<milli kp>/<milli ki>/<milli kd> pidP=<milli kp>/<milli ki>/<milli kd>}
```

Use this when you want the easiest way to inspect the whole board state.

### 5.3 `M102` / `M120`

Purpose:

- Return one-shot compact binary status.

Syntax:

```text
M102
M120
```

Response:

- Binary status frame with command `0x81`.

Binary payload layout is documented in section 7.

### 5.4 `M103` / `M154` / `M155`

Purpose:

- Configure periodic text or binary reports on USB/UART.

Syntax:

```text
M103 T<0|1> S<interval_ms> P<port_mask>
M154 S<interval_ms> P<port_mask>    ; text alias
M155 S<interval_ms> P<port_mask>    ; binary alias
```

Parameters:

- `T`
  - `0` = text
  - `1` = binary
- `S`
  - interval in ms
  - `0` disables the report stream
- `P`
  - port mask
  - `1` = USB
  - `2` = UART
  - `3` = USB + UART
  - `0` = allowed, but effectively no serial port is selected

Responses:

- `ok text report S<ms> P<mask>`
- `ok binary report S<ms> P<mask>`

Errors:

- `error invalid T parameter`
- `error invalid P parameter`

Notes:

- Text and binary report timers are independent.
- Reports are emitted only on USB/UART, not I2C.

### 5.5 `M104` / `M260`

Purpose:

- Query or set board-controlled I2C pull-up enables.

Syntax:

```text
M104 B<0|1>
M104 B<0|1> S<0|1>
```

Parameters:

- `B`
  - `0` = I2C0 pull-up switch
  - `1` = I2C1 pull-up switch
- `S`
  - omitted = query current state
  - `0` = disable
  - `1` = enable

Response:

- `ok I2C<bus> pullups=<0|1>`

Errors:

- `error invalid B parameter`

### 5.6 `M105` / `M280`

Purpose:

- Query or set the manual software error latch.

Syntax:

```text
M105
M105 S<0|1>
```

Behavior:

- This toggles the software-controlled error source used in the overall `err` status flag.

Response:

- `ok manual_error=<0|1>`

### 5.7 `M106` / `M121`

Purpose:

- Clear DRV fault state by pulsing the bridge reset lines.

Syntax:

```text
M106
M106 H0
M106 H1
M106 H-1
M106 H255
```

Behavior:

- No `H` means all bridges.
- `H-1` and `H255` also mean all bridges.
- Clearing is blocked until the fault holdoff timer has expired.

Responses:

- `ok faults cleared on all bridges`
- `ok faults cleared on H<n>`

Errors:

- `error invalid H parameter`
- `error fault clear holdoff=<ms>ms`

### 5.8 `M200` / `M17` / `M18`

Purpose:

- Enable or disable motor bridges.

Syntax:

```text
M200 S<0|1>
M200 H0 S<0|1>
M200 H1 S<0|1>
M17
M17 H0
M17 H1
M18
M18 H0
M18 H1
```

Parameters:

- `H`
  - omitted or `-1` or `255` means all bridges
- `S`
  - `0` = disable
  - `1` = enable

Responses:

- `ok enabled all bridges`
- `ok disabled all bridges`
- `ok H<n> enabled`
- `ok H<n> disabled`

Errors:

- `error invalid H parameter`
- `error missing S parameter`
- `error bridge enable blocked, holdoff=<ms>ms`
- `error H<n> blocked, holdoff=<ms>ms`

Notes:

- Enabling is blocked during driver fault holdoff.
- Disabling a bridge clears its commanded power.

### 5.9 `M201` / `M5`

Purpose:

- Stop one bridge or all bridges.

Syntax:

```text
M201
M201 H0
M201 H1
M5
M5 H0
M5 H1
```

Responses:

- `ok stopped all bridges`
- `ok stopped H<n>`

Notes:

- Stopping the active closed-loop bridge disables the closed-loop controller.
- `M201` does not clear faults.

### 5.10 `M202`

Purpose:

- Select the closed-loop bridge and set loop mode.

Syntax:

```text
M202 H<0|1> C<0|1|2>
```

Parameters:

- `H`
  - target closed-loop bridge
- `C`
  - `0` = manual
  - `1` = speed PID
  - `2` = position PID

Response:

- `ok H<n> mode=<manual|speed|position>`

Errors:

- `error missing or invalid H parameter`
- `error invalid C parameter`

Side effects:

- Selects the closed-loop bridge
- synchronizes loop enabled state from actual bridge enabled state
- resets the closed-loop controller

### 5.11 `M203` / `G1` / `M3` / `M4`

Purpose:

- Manual drive command for one bridge.

Syntax:

```text
M203 H<0|1> S<-1000..1000>
G1   H<0|1> S<-1000..1000>
M3   H<0|1> S<value>   ; force positive direction
M4   H<0|1> S<value>   ; force negative direction
```

Behavior:

- `M203` and `G1` use the signed value directly.
- `M3` forces the result positive.
- `M4` forces the result negative.
- Values are clamped to `-1000..1000`.

Response:

- `ok H<n> manual=<permille>`

Errors:

- `error missing or invalid H parameter`
- `error missing S parameter`
- `error H<n> disabled, use M200 H<n> S1 first`

Important notes:

- Non-zero manual drive requires the bridge to be enabled.
- `S0` succeeds even if the bridge is disabled.
- If the target bridge is the active closed-loop bridge, this forces the loop back to manual mode and resets it.

### 5.12 `M204`

Purpose:

- Set speed reference for the selected closed-loop bridge.

Syntax:

```text
M204 H<0|1> R<value> U<0|1>
```

Parameters:

- `R`
  - numeric reference value
- `U`
  - `0` = counts/s
  - `1` = rpm
  - omitted defaults to `0`

Response:

- `ok H<n> speed_ref=<milli-units> unit=<mcps|mrpm>`

Errors:

- `error missing or invalid H parameter`
- `error missing R parameter`
- `error invalid U parameter`

Notes:

- The bridge becomes the active closed-loop bridge.
- The applied value is clamped to the firmware speed limit.
- Unlike `M205`, the reply reports the applied clamped value.

### 5.13 `M205`

Purpose:

- Set position reference for the selected closed-loop bridge.

Syntax:

```text
M205 H<0|1> R<value> U<0|1|2>
```

Parameters:

- `U=0` = counts
- `U=1` = degrees
- `U=2` = turns
- omitted defaults to `0`

Response:

- `ok H<n> position_ref=<milli-units> unit=<mcount|mdeg|mturn>`

Errors:

- `error missing or invalid H parameter`
- `error missing R parameter`
- `error invalid U parameter`

Notes:

- The bridge becomes the active closed-loop bridge.
- Internally the reference is clamped to the max position range.
- The reply echoes the requested value, not the clamped stored value.

### 5.14 `M206`

Purpose:

- Set PID gains.

Syntax:

```text
M206 H<0|1> L<0|1> P<kp> I<ki> D<kd>
```

Parameters:

- `L`
  - `0` = speed PID
  - `1` = position PID

Response:

- `ok H<n> speed_pid=<milli kp>/<milli ki>/<milli kd>`
- `ok H<n> position_pid=<milli kp>/<milli ki>/<milli kd>`

Errors:

- `error missing or invalid H parameter`
- `error invalid L parameter`
- `error missing P/I/D parameter`

Side effects:

- Selects the closed-loop bridge
- resets the closed-loop controller

### 5.15 `M207`

Purpose:

- Reset the closed-loop controller state.

Syntax:

```text
M207
M207 H0
M207 H1
```

Behavior:

- No `H` uses the current closed-loop bridge.
- With `H`, the bridge is selected first, then the loop is reset.

Response:

- `ok loop reset H<n>`

Notes:

- The reset clears PID runtime state.
- Stored gains and references remain as currently configured.

### 5.16 `M208`

Purpose:

- Query or modify encoder configuration.

Syntax:

```text
M208
M208 H<0|1>
M208 H<0|1> B<10..12> Z<0..4095> V<0|1>
```

Parameters:

- `H`
  - optional bridge selection for closed-loop context
- `B`
  - reduced encoder bits
- `Z`
  - zero offset in raw AS5600 counts
- `V`
  - `0` = normal direction
  - `1` = inverted direction

Response:

- `ok encoder H<n> bits=<bits> zero=<offset> invert=<0|1>`

Errors:

- `error invalid B parameter`
- `error invalid Z parameter`

Important notes:

- This is not a passive read-only query.
- Even `M208` with no changing parameters re-applies the current encoder config and resets the loop.
- The encoder is global, not per-bridge. `H` only changes which bridge owns the closed-loop context.

### 5.17 `M209`

Purpose:

- Query or set binary fast-ACK mode.

Syntax:

```text
M209
M209 S<0|1>
```

Response:

- `ok fast_binary=<0|1>`

Important note:

- This affects binary write-command responses only.
- When enabled, binary write commands return a compact one-byte ACK payload instead of a full status payload.
- ASCII replies remain normal text.

### 5.18 `M210`

Purpose:

- Make the current encoder angle become zero.

Syntax:

```text
M210
M210 H0
M210 H1
```

Behavior:

- No `H` uses the current closed-loop bridge.
- With `H`, that bridge is selected first.
- The firmware:
  - samples the encoder
  - sets zero offset to the current native angle
  - forces the current measured position to `0`
  - sets position reference to `0`
  - resets the loop

Response:

- `ok encoder H<n> zeroed zero=<offset> pos=0`

Errors:

- `error invalid H parameter`
- `error encoder zero failed`

### 5.19 `M211`

Purpose:

- Query or set per-bridge motor direction inversion.

Syntax:

```text
M211 H0
M211 H0 S0
M211 H0 S1
M211 H1
M211 H1 S0
M211 H1 S1
```

Parameters:

- `H`
  - target bridge
- `S`
  - omitted = query current setting
  - `0` = normal polarity
  - `1` = inverted polarity

Response:

- `ok H<n> motor_invert=<0|1>`

Errors:

- `error missing or invalid H parameter`

### 5.20 Unknown Or Malformed ASCII Commands

Possible generic text errors:

- `error empty command`
- `error command too long`
- `error unknown command`

## 6. Binary Command API

Binary responses use:

- `request_command | 0x80` for ACK/full-status replies
- `0x7F` for errors

If fast binary mode is disabled:

- write commands return full status payloads

If fast binary mode is enabled:

- write commands return a 1-byte ACK payload:
  - `0x00` = success status byte

### 6.1 `0x01` Get Status

Payload:

- none

Response:

- full status frame, command `0x81`

### 6.2 `0x02` Set Fast Mode

Payload:

```text
[0] enable (0 or non-zero)
```

Notes:

- If you enable fast mode with this command, the response may already be a compact ACK because the mode change happens before the response is built.

### 6.3 `0x03` Set Report

Payload:

```text
[0] report_type   0=text, 1=binary
[1] port_mask     bit0=USB, bit1=UART
[2] interval_ms LSB
[3] interval_ms MSB
```

### 6.4 `0x04` Set Pullups

Payload:

```text
[0] bus_index   0 or 1
[1] enable      0 or non-zero
```

### 6.5 `0x05` Set Manual Error

Payload:

```text
[0] enable   0 or non-zero
```

### 6.6 `0x06` Clear Faults

Payload:

- length `0` = all bridges
- length `1`
  - `0xFF` = all bridges
  - `0` = H0
  - `1` = H1

Errors:

- `busy` if fault holdoff has not expired

### 6.7 `0x10` Set Bridge Enable

Payload:

```text
[0] bridge   0, 1, or 0xFF for all
[1] enable   0 or non-zero
```

Errors:

- `busy` if enabling is blocked by fault holdoff

### 6.8 `0x11` Stop

Payload:

- length `0` = all bridges
- length `1`
  - `0xFF` = all bridges
  - `0` = H0
  - `1` = H1

### 6.9 `0x12` Set Mode

Payload:

```text
[0] bridge   0 or 1
[1] mode     0=manual, 1=speed, 2=position
```

Side effects:

- selects closed-loop bridge
- synchronizes enable state
- resets the loop

### 6.10 `0x13` Set Manual Drive

Payload:

```text
[0] bridge
[1] command_permille LSB  int16
[2] command_permille MSB
```

Errors:

- `bridge disabled` for non-zero command on disabled bridge

### 6.11 `0x14` Set Speed Reference

Payload:

```text
[0] bridge
[1..4] speed_counts_per_sec int32 LE
```

Units are always counts/s in binary mode.

### 6.12 `0x15` Set Position Reference

Payload:

```text
[0] bridge
[1..4] position_counts int32 LE
```

Units are always counts in binary mode.

### 6.13 `0x16` Set PID Gains

Payload:

```text
[0] bridge
[1] loop_selector   0=speed, 1=position
[2..5]  kp float32 LE
[6..9]  ki float32 LE
[10..13] kd float32 LE
```

### 6.14 `0x17` Reset Loop

Payload:

- length `0` = current closed-loop bridge
- length `1`
  - `0` = H0
  - `1` = H1

### 6.15 `0x18` Set Encoder Config

Payload:

```text
[0] bits        10..12, or 0xFF to leave unchanged
[1] zero LSB
[2] zero MSB    0xFFFF to leave unchanged
[3] invert      0, 1, or 0xFF to leave unchanged
```

Notes:

- There is no bridge byte here.
- This affects the single shared encoder.
- The loop is reset after applying the config.

### 6.16 `0x19` Set Bridge Direction

Payload:

```text
[0] bridge   0 or 1
[1] invert   0=normal, 1=inverted
```

### 6.17 `0x20` Fast Setpoint

Purpose:

- One compact command to select bridge, choose mode, optionally change enable state, and apply a setpoint.

Payload:

```text
[0] bridge         0 or 1
[1] mode           0=manual, 1=speed, 2=position
[2] enable_action  0=no change, 1=disable, 2=enable
[3..6] value int32 LE
```

Value meaning:

- mode `0` manual: `value` = manual permille
- mode `1` speed: `value` = speed reference in counts/s
- mode `2` position: `value` = position reference in counts

Notes:

- If `enable_action=2`, enable happens before applying the setpoint.
- Manual mode still fails with `bridge disabled` if the bridge ends up disabled and the manual value is non-zero.

### 6.17 `0x21` Fast Manual Dual

Payload:

```text
[0..1] H0 command int16 LE
[2..3] H1 command int16 LE
```

Errors:

- `bridge disabled` if either non-zero command targets a disabled bridge

### 6.18 Binary Error Codes

Binary error response format:

```text
command = 0x7F
payload[0] = original request command
payload[1] = error code
```

Error codes:

- `1` = bad length
- `2` = bad checksum
- `3` = unknown command
- `4` = invalid argument
- `5` = bridge disabled
- `6` = busy

Meaning of `busy` in current firmware:

- enabling blocked by fault holdoff
- fault clear blocked by fault holdoff

## 7. Binary Status Payload

The binary status response payload is 46 bytes and uses this layout:

| Offset | Size | Type | Field |
|---|---:|---|---|
| 0 | 1 | `uint8` | status flags |
| 1 | 4 | `uint32` | uptime ms |
| 5 | 2 | `int16` | bridge 0 power permille |
| 7 | 2 | `int16` | bridge 1 power permille |
| 9 | 2 | `uint16` | bridge 0 duty |
| 11 | 2 | `uint16` | bridge 1 duty |
| 13 | 2 | `uint16` | VIN mV |
| 15 | 2 | `int16` | current 0 mA |
| 17 | 2 | `int16` | current 1 mA |
| 19 | 2 | `uint16` | fault holdoff ms |
| 21 | 1 | `uint8` | loop bridge index |
| 22 | 1 | `uint8` | loop mode |
| 23 | 1 | `uint8` | loop flags |
| 24 | 1 | `uint8` | encoder reduced resolution bits |
| 25 | 2 | `uint16` | encoder native counts |
| 27 | 2 | `uint16` | encoder reduced counts |
| 29 | 4 | `int32` | measured position counts |
| 33 | 2 | `int16` | measured speed counts/s |
| 35 | 2 | `int16` | manual command permille |
| 37 | 2 | `int16` | loop output command permille |
| 39 | 4 | `int32` | reference position counts |
| 43 | 2 | `int16` | reference speed counts/s |
| 45 | 1 | `uint8` | bridge direction flags |

### 7.1 Status Flag Bits

Status flag byte bits:

- `0x01` error active
- `0x02` driver fault active
- `0x04` over-temperature warning active
- `0x08` status LED controller ready
- `0x10` bridge 0 enabled
- `0x20` bridge 1 enabled
- `0x40` I2C0 pull-ups enabled
- `0x80` I2C1 pull-ups enabled

Loop flag byte bits:

- `0x01` closed-loop bridge enabled
- `0x02` encoder healthy
- `0x04` fast binary mode enabled
- `0x08` loop error condition

Direction flag byte bits:

- `0x01` bridge 0 direction inverted
- `0x02` bridge 1 direction inverted

Current loop error condition:

- closed-loop enabled
- mode is not manual
- encoder is not healthy

## 8. Text Status Field Definitions

Global fields:

- `up`
  - uptime in ms
- `err`
  - aggregate error bit
  - set when any of these are true:
    - manual error latch asserted
    - driver fault active
    - status LED controller not ready
    - loop enabled in speed/position mode while encoder is unhealthy
- `fault`
  - DRV fault input active
- `otw`
  - DRV over-temperature warning input active
- `holdoff`
  - remaining fault clear/enable holdoff in ms
- `led`
  - PCA9633 status LED controller healthy
- `fast`
  - binary fast-ACK mode enabled
- `en`
  - actual bridge enable states
- `pwr`
  - commanded bridge power in permille
- `duty`
  - actual PWM duty applied to each bridge
- `dir`
  - per-bridge motor inversion state
  - `0` = normal
  - `1` = inverted
- `vin`
  - VIN measurement after divider scaling
- `i1`, `i2`
  - current in amps from ACS722 conversion
- `pullups`
  - I2C pull-up switch states

Loop block fields:

- `H`
  - currently selected closed-loop bridge
- `mode`
  - `manual`, `speed`, or `position`
- `en`
  - loop enabled state for the selected bridge
- `enc`
  - encoder healthy flag
- `raw`
  - native AS5600 raw angle counts
- `red`
  - reduced-resolution count value
- `pos`
  - measured position counts, multi-turn
- `spd`
  - measured speed counts/s
- `refS`
  - stored speed reference counts/s
- `refP`
  - stored position reference counts
- `out`
  - loop output command in permille
- `man`
  - stored manual command in permille
- `pidS`
  - speed PID gains in milli-units
- `pidP`
  - position PID gains in milli-units

## 9. Operational Notes That Matter In Practice

### 9.1 Closed-Loop Bridge Ownership

Only one bridge can be under closed-loop control at a time. If you send a closed-loop command for the other bridge, the firmware will switch ownership and stop the previous closed-loop bridge.

### 9.2 Manual Commands Override Closed Loop

If you send manual drive to the active closed-loop bridge:

- mode is forced to manual
- manual command is stored
- loop is reset

### 9.3 `M208` Is Destructive To Runtime State

Even when you use `M208` just to "query" encoder settings, it re-applies the current config and resets the loop. Treat it like a write.

### 9.4 `M210` Re-Bases Position

`M210` is the clean way to redefine the current actuator location as zero so you do not need to keep working with large accumulated degree references.

### 9.5 `M211` Fixes Motor Polarity In Software

`M211` lets you flip motor polarity in firmware so positive command and positive encoder motion agree without rewiring the motor leads.

### 9.6 Measurement Meaning

- `pwr` is the commanded signed bridge power
- `duty` is the actual PWM compare value after the 95% cap
- `dir` is the per-bridge motor inversion flag
- `out` is the current closed-loop output command
- `man` is the stored manual command for the active loop bridge

### 9.7 Fault Holdoff

After a fault assertion:

- fault clear can be rejected for up to `1000 ms`
- bridge enable can also be rejected during that same holdoff window

## 10. Minimal Working Command Sequences

### Manual Drive

```text
M200 H0 S1
M203 H0 S250
M203 H0 S0
```

### Speed PID

```text
M200 H0 S1
M206 H0 L0 P0.2 I0.02 D0
M202 H0 C1
M204 H0 R1500 U0
```

### Position PID

```text
M200 H0 S1
M208 H0 B10 Z0 V0
M206 H0 L1 P0.1 I0 D0
M206 H0 L0 P0.2 I0.02 D0
M202 H0 C2
M205 H0 R90 U1
```

### Re-Zero Current Position

```text
M210 H0
```

This file should be treated as the reference for the current repository state.
