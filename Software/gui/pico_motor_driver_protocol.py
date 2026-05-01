"""Protocol helpers for the Pico dual motor driver GUI."""

from __future__ import annotations

from dataclasses import dataclass
import re
import struct
from typing import Optional


START_BYTE = 0xA5
RESP_STATUS = 0x81
RESP_ERROR = 0x7F
AS5600_COUNTS_PER_REV = 4096

PORT_USB = 0x01
PORT_UART = 0x02

MODE_MANUAL = 0
MODE_SPEED = 1
MODE_POSITION = 2

MODE_NAMES = {
    MODE_MANUAL: "Manual",
    MODE_SPEED: "Speed PID",
    MODE_POSITION: "Position PID",
}

STATUS_PAYLOAD_STRUCT = struct.Struct("<BIhhHHHhhHBBBBHHihhhihB")

_STATUS_TEXT_RE = re.compile(
    r"^ok "
    r"up=(?P<uptime>\d+)ms "
    r"err=(?P<err>[01]) "
    r"fault=(?P<fault>[01]) "
    r"otw=(?P<otw>[01]) "
    r"holdoff=(?P<holdoff>\d+)ms "
    r"led=(?P<led>[01]) "
    r"fast=(?P<fast>[01]) "
    r"en=\[(?P<en0>[01]),(?P<en1>[01])\] "
    r"pwr=\[(?P<pwr0>-?\d+),(?P<pwr1>-?\d+)\] "
    r"duty=\[(?P<duty0>\d+),(?P<duty1>\d+)\] "
    r"dir=\[(?P<dir0>[01]),(?P<dir1>[01])\] "
    r"vin=(?P<vin_whole>\d+)\.(?P<vin_frac>\d{3})V "
    r"i1=(?P<i1_sign>-?)(?P<i1_whole>\d+)\.(?P<i1_frac>\d{3})A "
    r"i2=(?P<i2_sign>-?)(?P<i2_whole>\d+)\.(?P<i2_frac>\d{3})A "
    r"pullups=\[(?P<pu0>[01]),(?P<pu1>[01])\] "
    r"loop=\{H(?P<loop_bridge>\d+) "
    r"mode=(?P<mode>manual|speed|position) "
    r"en=(?P<loop_enabled>[01]) "
    r"enc=(?P<enc_healthy>[01]) "
    r"raw=(?P<raw>\d+) "
    r"red=(?P<red>\d+) "
    r"pos=(?P<pos>-?\d+) "
    r"spd=(?P<spd>-?\d+) "
    r"refS=(?P<ref_s>-?\d+) "
    r"refP=(?P<ref_p>-?\d+) "
    r"out=(?P<out>-?\d+) "
    r"man=(?P<man>-?\d+) "
    r"pidS=(?P<speed_pid>-?\d+/-?\d+/-?\d+) "
    r"pidP=(?P<position_pid>-?\d+/-?\d+/-?\d+)\}$"
)


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def parse_key_value_fields(text: str) -> dict[str, str]:
    """Best-effort field extraction for firmware ACK lines."""

    fields: dict[str, str] = {}
    tokens = text.strip().split()
    for token in tokens:
        upper = token.upper()
        if "=" not in token:
            if len(token) > 1 and upper[0] == "H" and token[1:].lstrip("-").isdigit():
                fields["bridge"] = token[1:]
                fields["h"] = token[1:]
            elif len(token) > 1 and upper[0] == "S" and token[1:].lstrip("-").isdigit():
                fields["s"] = token[1:]
            elif len(token) > 1 and upper[0] == "P" and token[1:].lstrip("-").isdigit():
                fields["p"] = token[1:]
            elif token == "enabled":
                fields["enabled"] = "1"
            elif token == "disabled":
                fields["enabled"] = "0"
            continue

        key, value = token.split("=", 1)
        key = key.lower()
        fields[key] = value

        if key == "mode":
            fields["loop_mode"] = value
        elif key == "manual":
            fields["manual_command"] = value
        elif key == "speed_ref":
            fields["speed_ref"] = value
        elif key == "position_ref":
            fields["position_ref"] = value
        elif key == "fast_binary":
            fields["fast_binary"] = value
        elif key == "invert":
            fields["invert"] = value
        elif key == "motor_invert":
            fields["motor_invert"] = value

    if "speed_pid" in fields:
        parts = fields["speed_pid"].split("/")
        if len(parts) == 3:
            fields["speed_kp"], fields["speed_ki"], fields["speed_kd"] = parts

    if "position_pid" in fields:
        parts = fields["position_pid"].split("/")
        if len(parts) == 3:
            fields["position_kp"], fields["position_ki"], fields["position_kd"] = parts

    return fields


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, f"Mode {mode}")


def mode_from_text(text: str) -> int:
    mapping = {
        "manual": MODE_MANUAL,
        "speed": MODE_SPEED,
        "position": MODE_POSITION,
    }
    return mapping.get(text.lower(), MODE_MANUAL)


def _parse_pid_triplet(text: str) -> tuple[float, float, float]:
    kp_text, ki_text, kd_text = text.split("/")
    return (float(kp_text) / 1000.0, float(ki_text) / 1000.0, float(kd_text) / 1000.0)


def _current_ma(sign: str, whole: str, frac: str) -> int:
    magnitude = int(whole) * 1000 + int(frac)
    return -magnitude if sign == "-" else magnitude


@dataclass(slots=True)
class TelemetrySample:
    response_command: int
    flags: int
    uptime_ms: int
    bridge_power: tuple[int, int]
    bridge_duty: tuple[int, int]
    bridge_direction_inverted: tuple[bool, bool]
    vin_mv: int
    current_ma: tuple[int, int]
    fault_holdoff_ms: int
    loop_bridge_index: int
    loop_mode: int
    loop_flags: int
    encoder_resolution_bits: Optional[int]
    encoder_native_counts: int
    encoder_reduced_counts: int
    measured_position_counts: int
    measured_speed_counts_per_sec: float
    manual_command_permille: int
    output_command_permille: int
    reference_position_counts: int
    reference_speed_counts_per_sec: float
    speed_pid_gains: Optional[tuple[float, float, float]] = None
    position_pid_gains: Optional[tuple[float, float, float]] = None
    raw_payload: bytes = b""
    source: str = "binary"

    @property
    def error_active(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def fault_active(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def otw_active(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def led_ready(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def bridge_enabled(self) -> tuple[bool, bool]:
        return (bool(self.flags & 0x10), bool(self.flags & 0x20))

    @property
    def pullups_enabled(self) -> tuple[bool, bool]:
        return (bool(self.flags & 0x40), bool(self.flags & 0x80))

    @property
    def loop_enabled(self) -> bool:
        return bool(self.loop_flags & 0x01)

    @property
    def encoder_healthy(self) -> bool:
        return bool(self.loop_flags & 0x02)

    @property
    def fast_binary_mode(self) -> bool:
        return bool(self.loop_flags & 0x04)

    @property
    def loop_error(self) -> bool:
        return bool(self.loop_flags & 0x08)

    @property
    def loop_mode_name(self) -> str:
        return mode_name(self.loop_mode)

    @property
    def vin_v(self) -> float:
        return self.vin_mv / 1000.0

    def summary(self) -> str:
        enabled0, enabled1 = self.bridge_enabled
        return (
            f"up={self.uptime_ms}ms err={int(self.error_active)} fault={int(self.fault_active)} "
            f"otw={int(self.otw_active)} en=[{int(enabled0)},{int(enabled1)}] "
            f"pwr=[{self.bridge_power[0]},{self.bridge_power[1]}] "
            f"dir=[{int(self.bridge_direction_inverted[0])},{int(self.bridge_direction_inverted[1])}] "
            f"duty=[{self.bridge_duty[0]},{self.bridge_duty[1]}] "
            f"vin={self.vin_v:.3f}V "
            f"loop=H{self.loop_bridge_index}:{self.loop_mode_name} "
            f"ref=({self.reference_position_counts},{self.reference_speed_counts_per_sec:.0f}) "
            f"meas=({self.measured_position_counts},{self.measured_speed_counts_per_sec:.0f})"
        )


@dataclass(slots=True)
class ClosedLoopSample:
    bridge_index: int
    enabled: bool
    mode: int
    encoder_healthy: bool
    manual_permille: int
    output_permille: int
    ref_position_counts: int
    measured_position_counts: int
    ref_speed_counts_per_sec: float
    measured_speed_counts_per_sec: float
    encoder_native_counts: int
    encoder_reduced_counts: int
    encoder_resolution_bits: Optional[int]
    fault_holdoff_ms: int
    fast_binary_mode: bool
    speed_pid_gains: Optional[tuple[float, float, float]] = None
    position_pid_gains: Optional[tuple[float, float, float]] = None
    raw_line: str = ""
    source: str = "binary"

    def mode_name(self) -> str:
        return mode_name(self.mode)

    @classmethod
    def from_telemetry(cls, sample: TelemetrySample) -> "ClosedLoopSample":
        return cls(
            bridge_index=sample.loop_bridge_index,
            enabled=sample.loop_enabled,
            mode=sample.loop_mode,
            encoder_healthy=sample.encoder_healthy,
            manual_permille=sample.manual_command_permille,
            output_permille=sample.output_command_permille,
            ref_position_counts=sample.reference_position_counts,
            measured_position_counts=sample.measured_position_counts,
            ref_speed_counts_per_sec=sample.reference_speed_counts_per_sec,
            measured_speed_counts_per_sec=sample.measured_speed_counts_per_sec,
            encoder_native_counts=sample.encoder_native_counts,
            encoder_reduced_counts=sample.encoder_reduced_counts,
            encoder_resolution_bits=sample.encoder_resolution_bits,
            fault_holdoff_ms=sample.fault_holdoff_ms,
            fast_binary_mode=sample.fast_binary_mode,
            speed_pid_gains=sample.speed_pid_gains,
            position_pid_gains=sample.position_pid_gains,
            raw_line=sample.raw_payload.decode("utf-8", errors="replace") if sample.source == "text" else "",
            source=sample.source,
        )


@dataclass(slots=True)
class BinaryPacket:
    command: int
    payload: bytes
    checksum_ok: bool
    frame: bytes


def build_binary_frame(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF:
        raise ValueError("command must fit in one byte")
    if len(payload) > 254:
        raise ValueError("payload too large for firmware framing")

    frame = bytearray((START_BYTE, 1 + len(payload), command & 0xFF))
    frame.extend(payload)
    frame.append(xor_checksum(frame[1:]))
    return bytes(frame)


class FirmwareCommands:
    """Builds the commands supported by the current firmware."""

    @staticmethod
    def ascii_line(text: str) -> str:
        return text.strip()

    @staticmethod
    def request_help() -> str:
        return "M100"

    @staticmethod
    def request_status_text() -> str:
        return "M101"

    @staticmethod
    def request_status_binary() -> str:
        return "M102"

    @staticmethod
    def enable_bridge(bridge_index: Optional[int], enable: bool) -> str:
        if bridge_index is None:
            return f"M200 S{1 if enable else 0}"
        return f"M200 H{bridge_index} S{1 if enable else 0}"

    @staticmethod
    def stop(bridge_index: Optional[int]) -> str:
        if bridge_index is None:
            return "M201"
        return f"M201 H{bridge_index}"

    @staticmethod
    def clear_faults(bridge_index: Optional[int]) -> str:
        if bridge_index is None:
            return "M106"
        return f"M106 H{bridge_index}"

    @staticmethod
    def set_report(binary: bool, interval_ms: int, port_mask: int) -> str:
        report_type = 1 if binary else 0
        return f"M103 T{report_type} S{max(0, int(interval_ms))} P{clamp_int(int(port_mask), 0, 3)}"

    @staticmethod
    def set_pullup(bus_index: int, enable: bool) -> str:
        bus_index = clamp_int(int(bus_index), 0, 1)
        return f"M104 B{bus_index} S{1 if enable else 0}"

    @staticmethod
    def set_manual_error(enable: bool) -> str:
        return f"M105 S{1 if enable else 0}"

    @staticmethod
    def set_loop_mode(bridge_index: int, mode: int) -> str:
        return f"M202 H{bridge_index} C{clamp_int(int(mode), 0, 2)}"

    @staticmethod
    def set_manual_drive(bridge_index: int, command_permille: int) -> str:
        return f"M203 H{bridge_index} S{clamp_int(int(command_permille), -1000, 1000)}"

    @staticmethod
    def set_speed_reference_counts(bridge_index: int, counts_per_sec: float) -> str:
        return f"M204 H{bridge_index} R{counts_per_sec:g} U0"

    @staticmethod
    def set_speed_reference_rpm(bridge_index: int, rpm: float) -> str:
        return f"M204 H{bridge_index} R{rpm:g} U1"

    @staticmethod
    def set_position_reference_counts(bridge_index: int, counts: int) -> str:
        return f"M205 H{bridge_index} R{int(counts)} U0"

    @staticmethod
    def set_position_reference_degrees(bridge_index: int, degrees: float) -> str:
        return f"M205 H{bridge_index} R{degrees:g} U1"

    @staticmethod
    def set_position_reference_turns(bridge_index: int, turns: float) -> str:
        return f"M205 H{bridge_index} R{turns:g} U2"

    @staticmethod
    def set_pid_gains(bridge_index: int, loop_selector: int, kp: float, ki: float, kd: float) -> str:
        return f"M206 H{bridge_index} L{clamp_int(int(loop_selector), 0, 1)} P{kp:g} I{ki:g} D{kd:g}"

    @staticmethod
    def reset_loop(bridge_index: Optional[int]) -> str:
        if bridge_index is None:
            return "M207"
        return f"M207 H{bridge_index}"

    @staticmethod
    def zero_current_position(bridge_index: Optional[int]) -> str:
        if bridge_index is None:
            return "M210"
        return f"M210 H{int(bridge_index)}"

    @staticmethod
    def set_motor_direction(bridge_index: int, inverted: bool) -> str:
        return f"M211 H{int(bridge_index)} S{1 if inverted else 0}"

    @staticmethod
    def set_encoder_config(
        bridge_index: Optional[int],
        resolution_bits: Optional[int],
        zero_offset: Optional[int],
        inverted: Optional[bool],
    ) -> str:
        parts = ["M208"]
        if bridge_index is not None:
            parts.append(f"H{int(bridge_index)}")
        if resolution_bits is not None:
            parts.append(f"B{int(resolution_bits)}")
        if zero_offset is not None:
            parts.append(f"Z{int(zero_offset)}")
        if inverted is not None:
            parts.append(f"V{1 if inverted else 0}")
        return " ".join(parts)

    @staticmethod
    def set_fast_binary_mode(enabled: Optional[bool]) -> str:
        if enabled is None:
            return "M209"
        return f"M209 S{1 if enabled else 0}"


def parse_binary_packet(frame: bytes) -> BinaryPacket:
    if len(frame) < 4:
        raise ValueError("binary frame too short")
    if frame[0] != START_BYTE:
        raise ValueError("missing binary frame start byte")

    expected_length = frame[1] + 3
    if len(frame) != expected_length:
        raise ValueError("binary frame length mismatch")

    checksum = xor_checksum(frame[1:-1])
    return BinaryPacket(
        command=frame[2],
        payload=frame[3:-1],
        checksum_ok=(checksum == frame[-1]),
        frame=frame,
    )


def parse_binary_status(packet: BinaryPacket) -> TelemetrySample:
    if not packet.checksum_ok:
        raise ValueError("status packet checksum failed")
    if len(packet.payload) != STATUS_PAYLOAD_STRUCT.size:
        raise ValueError(
            f"status payload length mismatch: expected {STATUS_PAYLOAD_STRUCT.size}, got {len(packet.payload)}"
        )

    unpacked = STATUS_PAYLOAD_STRUCT.unpack(packet.payload)
    return TelemetrySample(
        response_command=packet.command,
        flags=unpacked[0],
        uptime_ms=unpacked[1],
        bridge_power=(unpacked[2], unpacked[3]),
        bridge_duty=(unpacked[4], unpacked[5]),
        bridge_direction_inverted=(bool(unpacked[22] & 0x01), bool(unpacked[22] & 0x02)),
        vin_mv=unpacked[6],
        current_ma=(unpacked[7], unpacked[8]),
        fault_holdoff_ms=unpacked[9],
        loop_bridge_index=unpacked[10],
        loop_mode=unpacked[11],
        loop_flags=unpacked[12],
        encoder_resolution_bits=unpacked[13],
        encoder_native_counts=unpacked[14],
        encoder_reduced_counts=unpacked[15],
        measured_position_counts=unpacked[16],
        measured_speed_counts_per_sec=float(unpacked[17]),
        manual_command_permille=unpacked[18],
        output_command_permille=unpacked[19],
        reference_position_counts=unpacked[20],
        reference_speed_counts_per_sec=float(unpacked[21]),
        raw_payload=packet.payload,
        source="binary",
    )


def parse_status_text(line: str) -> Optional[TelemetrySample]:
    match = _STATUS_TEXT_RE.match(line.strip())
    if match is None:
        return None

    data = match.groupdict()
    flags = 0
    flags |= int(data["err"]) & 0x01
    flags |= (int(data["fault"]) & 0x01) << 1
    flags |= (int(data["otw"]) & 0x01) << 2
    flags |= (int(data["led"]) & 0x01) << 3
    flags |= (int(data["en0"]) & 0x01) << 4
    flags |= (int(data["en1"]) & 0x01) << 5
    flags |= (int(data["pu0"]) & 0x01) << 6
    flags |= (int(data["pu1"]) & 0x01) << 7

    loop_flags = 0
    if int(data["loop_enabled"]):
        loop_flags |= 0x01
    if int(data["enc_healthy"]):
        loop_flags |= 0x02
    if int(data["fast"]):
        loop_flags |= 0x04
    if int(data["loop_enabled"]) and data["mode"] != "manual" and not int(data["enc_healthy"]):
        loop_flags |= 0x08

    return TelemetrySample(
        response_command=RESP_STATUS,
        flags=flags,
        uptime_ms=int(data["uptime"]),
        bridge_power=(int(data["pwr0"]), int(data["pwr1"])),
        bridge_duty=(int(data["duty0"]), int(data["duty1"])),
        bridge_direction_inverted=(bool(int(data["dir0"])), bool(int(data["dir1"]))),
        vin_mv=int(data["vin_whole"]) * 1000 + int(data["vin_frac"]),
        current_ma=(
            _current_ma(data["i1_sign"], data["i1_whole"], data["i1_frac"]),
            _current_ma(data["i2_sign"], data["i2_whole"], data["i2_frac"]),
        ),
        fault_holdoff_ms=int(data["holdoff"]),
        loop_bridge_index=int(data["loop_bridge"]),
        loop_mode=mode_from_text(data["mode"]),
        loop_flags=loop_flags,
        encoder_resolution_bits=None,
        encoder_native_counts=int(data["raw"]),
        encoder_reduced_counts=int(data["red"]),
        measured_position_counts=int(data["pos"]),
        measured_speed_counts_per_sec=float(int(data["spd"])),
        manual_command_permille=int(data["man"]),
        output_command_permille=int(data["out"]),
        reference_position_counts=int(data["ref_p"]),
        reference_speed_counts_per_sec=float(int(data["ref_s"])),
        speed_pid_gains=_parse_pid_triplet(data["speed_pid"]),
        position_pid_gains=_parse_pid_triplet(data["position_pid"]),
        raw_payload=line.encode("utf-8", errors="replace"),
        source="text",
    )


def parse_closed_loop_text(line: str) -> Optional[ClosedLoopSample]:
    return None


def describe_binary_error(request_command: int, error_code: int) -> str:
    error_names = {
        1: "bad length",
        2: "bad checksum",
        3: "unknown command",
        4: "invalid argument",
        5: "bridge disabled",
        6: "busy",
    }
    name = error_names.get(error_code, f"error {error_code}")
    return f"binary error for 0x{request_command:02X}: {name}"
