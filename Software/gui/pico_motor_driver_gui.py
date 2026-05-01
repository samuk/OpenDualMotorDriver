"""Desktop GUI for the Pico dual motor driver firmware."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
import time
from typing import Callable, Optional

from PySide6 import QtCharts, QtCore, QtGui, QtWidgets
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

from pico_motor_driver_protocol import (
    AS5600_COUNTS_PER_REV,
    ClosedLoopSample,
    FirmwareCommands,
    PORT_UART,
    PORT_USB,
    RESP_ERROR,
    TelemetrySample,
    clamp_int,
    describe_binary_error,
    parse_binary_packet,
    parse_binary_status,
    parse_key_value_fields,
    parse_status_text,
)


WINDOW_SECONDS = 60.0
DEFAULT_BINARY_REPORT_MS = 100
DEFAULT_TEXT_REPORT_MS = 250
DEFAULT_BAUD = 115200
DEFAULT_MAX_SPEED_REF_COUNTS_PER_SEC = 262144.0
DEFAULT_MAX_SPEED_REF_RPM = DEFAULT_MAX_SPEED_REF_COUNTS_PER_SEC * 60.0 / AS5600_COUNTS_PER_REV
SELF_CHECK_POWER_PERMILLE = 300
SELF_CHECK_DRIVE_MS = 450
SELF_CHECK_POLL_MS = 75
SELF_CHECK_SETTLE_MS = 120
SELF_CHECK_MIN_POSITION_COUNTS = 24
SELF_CHECK_MIN_SPEED_COUNTS_PER_SEC = 20.0


def now_stamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def apply_dark_theme(app: QtWidgets.QApplication) -> None:
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#0f1317"))
    palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor("#e6edf3"))
    palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#12171d"))
    palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor("#181e25"))
    palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#e6edf3"))
    palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor("#1b222a"))
    palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#e6edf3"))
    palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor("#3aaed8"))
    palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor("#091014"))
    app.setPalette(palette)
    app.setStyleSheet(
        """
        QWidget { color: #e6edf3; font-size: 10pt; }
        QMainWindow, QDialog { background: #0f1317; }
        QGroupBox {
            border: 1px solid #2a323b; border-radius: 10px;
            margin-top: 10px; padding-top: 8px; background: #151b21;
        }
        QGroupBox::title {
            subcontrol-origin: margin; left: 12px; padding: 0 4px;
            color: #8fd7ff; font-weight: 600;
        }
        QLabel[chip="true"] {
            border-radius: 8px; padding: 4px 8px; font-weight: 700; min-height: 18px;
        }
        QLabel[active="true"] { background: #1f5f45; color: #c9ffe2; }
        QLabel[active="false"] { background: #353d46; color: #d5dde6; }
        QLabel[value="true"] { color: #6ee7a6; font-weight: 700; }
        QLabel[value="false"] { color: #ff8a8a; font-weight: 700; }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {
            background: #10151a; border: 1px solid #303842; border-radius: 6px;
            padding: 4px 6px; selection-background-color: #3aaed8;
        }
        QPushButton {
            background: #22303b; border: 1px solid #31404c; border-radius: 6px;
            padding: 6px 10px;
        }
        QPushButton:hover { background: #2a3a48; }
        QPushButton:pressed { background: #16212b; }
        QPushButton:disabled { color: #8090a0; background: #1a2027; border-color: #2a323a; }
        QTabWidget::pane {
            border: 1px solid #2a323b; border-radius: 8px; top: -1px; background: #13181e;
        }
        QTabBar::tab {
            background: #1a2129; border: 1px solid #2a323b; border-bottom: none;
            border-top-left-radius: 6px; border-top-right-radius: 6px;
            padding: 6px 10px; margin-right: 2px;
        }
        QTabBar::tab:selected { background: #24313c; color: #d8f0ff; }
        QPlainTextEdit { font-family: Consolas, "Cascadia Mono", monospace; font-size: 9pt; }
        """
    )


def _apply_label_value_style(widget: QtWidgets.QLabel, good: Optional[bool]) -> None:
    if good is None:
        widget.setProperty("value", "")
    else:
        widget.setProperty("value", good)
    widget.style().unpolish(widget)
    widget.style().polish(widget)


def _set_chip(widget: QtWidgets.QLabel, active: bool, active_text: str, inactive_text: str) -> None:
    widget.setText(active_text if active else inactive_text)
    widget.setProperty("active", active)
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class TelemetryHistory:
    def __init__(self, window_seconds: float = WINDOW_SECONDS, max_points: int = 1200) -> None:
        self.window_seconds = window_seconds
        self.samples: deque[tuple[float, TelemetrySample]] = deque(maxlen=max_points)

    def add(self, sample: TelemetrySample) -> None:
        timestamp = time.monotonic()
        self.samples.append((timestamp, sample))
        cutoff = timestamp - self.window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def clear(self) -> None:
        self.samples.clear()

    def as_list(self) -> list[tuple[float, TelemetrySample]]:
        return list(self.samples)


@dataclass(slots=True)
class DirectionSelfCheckSession:
    bridge_index: int
    original_inverted: bool
    current_inverted: bool
    toggled_during_check: bool = False
    phase: str = "idle"
    baseline_position_counts: int = 0
    samples: list[TelemetrySample] = field(default_factory=list)
    poll_timer: Optional[QtCore.QTimer] = None


@dataclass(slots=True)
class MetricSpec:
    name: str
    color: str
    getter: Callable[[TelemetrySample], float]


class StatusChip(QtWidgets.QLabel):
    def __init__(self, title: str, active_text: str = "ON", inactive_text: str = "OFF") -> None:
        super().__init__()
        self.setProperty("chip", True)
        self._active_text = active_text
        self._inactive_text = inactive_text
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(72)
        self.setToolTip(title)
        _set_chip(self, False, active_text, inactive_text)

    def set_active(self, active: bool, active_text: Optional[str] = None, inactive_text: Optional[str] = None) -> None:
        if active_text is not None:
            self._active_text = active_text
        if inactive_text is not None:
            self._inactive_text = inactive_text
        _set_chip(self, active, self._active_text, self._inactive_text)


class ReadoutLabel(QtWidgets.QLabel):
    def __init__(self, text: str = "-") -> None:
        super().__init__(text)
        font = QtGui.QFont("Consolas")
        font.setPointSize(10)
        font.setBold(True)
        self.setFont(font)
        self.setMinimumWidth(110)

    def set_value(self, text: str, good: Optional[bool] = None) -> None:
        self.setText(text)
        _apply_label_value_style(self, good)


class ChartCard(QtWidgets.QGroupBox):
    def __init__(
        self,
        title: str,
        metric_specs: list[MetricSpec],
        window_seconds: float = WINDOW_SECONDS,
        fixed_y_range: Optional[tuple[float, float]] = None,
    ) -> None:
        super().__init__(title)
        self.metric_specs = metric_specs
        self.window_seconds = window_seconds
        self.fixed_y_range = fixed_y_range

        self.chart = QtCharts.QChart()
        self.chart.setBackgroundBrush(QtGui.QBrush(QtGui.QColor("#13181e")))
        self.chart.legend().setVisible(True)
        self.chart.legend().setAlignment(QtCore.Qt.AlignmentFlag.AlignBottom)
        self.chart.setAnimationOptions(QtCharts.QChart.AnimationOption.NoAnimation)

        self.series: dict[str, QtCharts.QLineSeries] = {}
        for spec in metric_specs:
            series = QtCharts.QLineSeries(self.chart)
            series.setName(spec.name)
            pen = QtGui.QPen(QtGui.QColor(spec.color))
            pen.setWidthF(2.0)
            series.setPen(pen)
            self.chart.addSeries(series)
            self.series[spec.name] = series

        self.axis_x = QtCharts.QValueAxis()
        self.axis_x.setRange(-self.window_seconds, 0.0)
        self.axis_x.setLabelFormat("%.0f")
        self.axis_x.setGridLineVisible(True)
        self.axis_x.setTitleText("Seconds")

        self.axis_y = QtCharts.QValueAxis()
        self.axis_y.setGridLineVisible(True)
        self.chart.addAxis(self.axis_x, QtCore.Qt.AlignmentFlag.AlignBottom)
        self.chart.addAxis(self.axis_y, QtCore.Qt.AlignmentFlag.AlignLeft)
        for series in self.series.values():
            series.attachAxis(self.axis_x)
            series.attachAxis(self.axis_y)

        self.view = QtCharts.QChartView(self.chart)
        self.view.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.view)

    def update_from_history(self, samples: list[tuple[float, TelemetrySample]]) -> None:
        if not samples:
            for series in self.series.values():
                series.clear()
            return

        latest_time = samples[-1][0]
        y_values: list[float] = []

        for spec in self.metric_specs:
            points = []
            for timestamp, sample in samples:
                x = timestamp - latest_time
                y = spec.getter(sample)
                points.append(QtCore.QPointF(x, y))
                y_values.append(y)
            self.series[spec.name].replace(points)

        self.axis_x.setRange(-self.window_seconds, 0.0)

        if self.fixed_y_range is not None:
            self.axis_y.setRange(self.fixed_y_range[0], self.fixed_y_range[1])
            return

        if not y_values:
            return

        y_min = min(y_values)
        y_max = max(y_values)
        if y_min == y_max:
            span = abs(y_min) * 0.1 or 1.0
            y_min -= span
            y_max += span
        else:
            margin = max((y_max - y_min) * 0.12, 1.0)
            y_min -= margin
            y_max += margin
        self.axis_y.setRange(y_min, y_max)


class EncoderResponsePanel(QtWidgets.QGroupBox):
    def __init__(self) -> None:
        super().__init__("AS5600 Response (I2C0)")

        self.view_mode = QtWidgets.QComboBox()
        self.view_mode.addItems(["Position", "Speed"])

        self.position_plot = ChartCard(
            "Position Reference vs Measurement (deg)",
            [
                MetricSpec("Ref Pos", "#3aaed8", lambda s: float(s.reference_position_counts) * 360.0 / AS5600_COUNTS_PER_REV),
                MetricSpec(
                    "Meas Pos",
                    "#f59e0b",
                    lambda s: float(s.measured_position_counts) * 360.0 / AS5600_COUNTS_PER_REV,
                ),
            ],
        )
        self.speed_plot = ChartCard(
            "Speed Reference vs Measurement",
            [
                MetricSpec("Ref Speed", "#6ee7a6", lambda s: float(s.reference_speed_counts_per_sec)),
                MetricSpec("Meas Speed", "#f472b6", lambda s: float(s.measured_speed_counts_per_sec)),
            ],
        )

        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.position_plot)
        self.stack.addWidget(self.speed_plot)

        self.view_mode.currentIndexChanged.connect(self.stack.setCurrentIndex)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("View"))
        controls.addWidget(self.view_mode)
        controls.addStretch(1)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.stack)

    def update_from_history(self, history: list[tuple[float, TelemetrySample]]) -> None:
        self.position_plot.update_from_history(history)
        self.speed_plot.update_from_history(history)


class TelemetryScopePanel(QtWidgets.QWidget):
    clearRequested = QtCore.Signal()

    def __init__(self) -> None:
        super().__init__()

        self.encoder_plot = EncoderResponsePanel()
        self.command_plot = ChartCard(
            "Power Command and PWM Duty",
            [
                MetricSpec("Power H0", "#3aaed8", lambda s: float(s.bridge_power[0])),
                MetricSpec("Power H1", "#f59e0b", lambda s: float(s.bridge_power[1])),
                MetricSpec("Duty H0", "#6ee7a6", lambda s: float(s.bridge_duty[0])),
                MetricSpec("Duty H1", "#f472b6", lambda s: float(s.bridge_duty[1])),
            ],
            fixed_y_range=(-1050.0, 1050.0),
        )
        self.current_plot = ChartCard(
            "ACS Current (mA)",
            [
                MetricSpec("Current H0", "#6ee7a6", lambda s: float(s.current_ma[0])),
                MetricSpec("Current H1", "#f59e0b", lambda s: float(s.current_ma[1])),
            ],
        )
        self.clear_button = QtWidgets.QPushButton("Clear Scope Data")
        self.clear_button.clicked.connect(self.clearRequested.emit)

        self.encoder_plot.setMinimumHeight(360)
        self.command_plot.setMinimumHeight(220)
        self.current_plot.setMinimumHeight(220)

        self.bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.bottom_splitter.setChildrenCollapsible(False)
        self.bottom_splitter.addWidget(self.command_plot)
        self.bottom_splitter.addWidget(self.current_plot)
        self.bottom_splitter.setStretchFactor(0, 1)
        self.bottom_splitter.setStretchFactor(1, 1)

        self.scope_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.scope_splitter.setChildrenCollapsible(False)
        self.scope_splitter.addWidget(self.encoder_plot)
        self.scope_splitter.addWidget(self.bottom_splitter)
        self.scope_splitter.setStretchFactor(0, 2)
        self.scope_splitter.setStretchFactor(1, 1)

        controls = QtWidgets.QHBoxLayout()
        controls.addStretch(1)
        controls.addWidget(self.clear_button)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.scope_splitter, 1)

        QtCore.QTimer.singleShot(0, self._set_default_sizes)

    def update_from_history(self, history: list[tuple[float, TelemetrySample]]) -> None:
        self.encoder_plot.update_from_history(history)
        self.command_plot.update_from_history(history)
        self.current_plot.update_from_history(history)

    def _set_default_sizes(self) -> None:
        self.scope_splitter.setSizes([700, 340])
        self.bottom_splitter.setSizes([1, 1])


class StatusPanel(QtWidgets.QGroupBox):
    def __init__(self) -> None:
        super().__init__("Live Status")
        self.setMinimumWidth(380)

        self.chip_error = StatusChip("Error", "ERROR", "OK")
        self.chip_fault = StatusChip("Fault", "FAULT", "CLEAR")
        self.chip_otw = StatusChip("Overtemp", "OTW", "OK")
        self.chip_led = StatusChip("LED Ready", "READY", "NOT READY")
        self.chip_b0 = StatusChip("Bridge 0", "EN", "DIS")
        self.chip_b1 = StatusChip("Bridge 1", "EN", "DIS")
        self.chip_pu0 = StatusChip("I2C0 Pullups", "ON", "OFF")
        self.chip_pu1 = StatusChip("I2C1 Pullups", "ON", "OFF")

        self.uptime = ReadoutLabel()
        self.vin = ReadoutLabel()
        self.currents = ReadoutLabel()
        self.commands = ReadoutLabel()
        self.duties = ReadoutLabel()
        self.directions = ReadoutLabel()
        self.holdoff = ReadoutLabel()
        self.loop = ReadoutLabel()
        self.source = ReadoutLabel()

        grid = QtWidgets.QGridLayout(self)
        grid.addWidget(QtWidgets.QLabel("Error"), 0, 0)
        grid.addWidget(self.chip_error, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Fault"), 0, 2)
        grid.addWidget(self.chip_fault, 0, 3)
        grid.addWidget(QtWidgets.QLabel("OTW"), 0, 4)
        grid.addWidget(self.chip_otw, 0, 5)
        grid.addWidget(QtWidgets.QLabel("LED"), 0, 6)
        grid.addWidget(self.chip_led, 0, 7)
        grid.addWidget(QtWidgets.QLabel("Bridge 0"), 1, 0)
        grid.addWidget(self.chip_b0, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Bridge 1"), 1, 2)
        grid.addWidget(self.chip_b1, 1, 3)
        grid.addWidget(QtWidgets.QLabel("Pullups"), 1, 4)
        grid.addWidget(self.chip_pu0, 1, 5)
        grid.addWidget(self.chip_pu1, 1, 6)

        form = QtWidgets.QFormLayout()
        form.addRow("Uptime", self.uptime)
        form.addRow("VIN", self.vin)
        form.addRow("Currents", self.currents)
        form.addRow("Commands", self.commands)
        form.addRow("Duty", self.duties)
        form.addRow("Direction", self.directions)
        form.addRow("Holdoff", self.holdoff)
        form.addRow("Loop", self.loop)
        form.addRow("Source", self.source)
        grid.addLayout(form, 2, 0, 1, 8)
        grid.setColumnStretch(7, 1)
        self.set_sample(None)

    def set_sample(self, sample: Optional[TelemetrySample]) -> None:
        if sample is None:
            for chip in (
                self.chip_error,
                self.chip_fault,
                self.chip_otw,
                self.chip_led,
                self.chip_b0,
                self.chip_b1,
                self.chip_pu0,
                self.chip_pu1,
            ):
                chip.set_active(False)
            self.uptime.set_value("-")
            self.vin.set_value("-")
            self.currents.set_value("-")
            self.commands.set_value("-")
            self.duties.set_value("-")
            self.directions.set_value("-")
            self.holdoff.set_value("-")
            self.loop.set_value("-")
            self.source.set_value("-")
            return

        enabled0, enabled1 = sample.bridge_enabled
        pullup0, pullup1 = sample.pullups_enabled
        self.chip_error.set_active(sample.error_active)
        self.chip_fault.set_active(sample.fault_active)
        self.chip_otw.set_active(sample.otw_active)
        self.chip_led.set_active(sample.led_ready)
        self.chip_b0.set_active(enabled0)
        self.chip_b1.set_active(enabled1)
        self.chip_pu0.set_active(pullup0)
        self.chip_pu1.set_active(pullup1)
        self.uptime.set_value(f"{sample.uptime_ms / 1000.0:.2f} s")
        self.vin.set_value(f"{sample.vin_mv / 1000.0:.3f} V")
        self.currents.set_value(f"H0 {sample.current_ma[0] / 1000.0:.3f} A | H1 {sample.current_ma[1] / 1000.0:.3f} A")
        self.commands.set_value(f"H0 {sample.bridge_power[0]:+d} | H1 {sample.bridge_power[1]:+d}")
        self.duties.set_value(f"H0 {sample.bridge_duty[0]} | H1 {sample.bridge_duty[1]}")
        self.directions.set_value(
            f"H0 {'INV' if sample.bridge_direction_inverted[0] else 'NOR'} | "
            f"H1 {'INV' if sample.bridge_direction_inverted[1] else 'NOR'}"
        )
        self.holdoff.set_value(f"{sample.fault_holdoff_ms} ms")
        self.loop.set_value(
            f"H{sample.loop_bridge_index} {sample.loop_mode_name} | "
            f"enc={int(sample.encoder_healthy)} fast={int(sample.fast_binary_mode)}"
        )
        self.source.set_value(sample.source.upper())


class BridgeControlCard(QtWidgets.QGroupBox):
    driveRequested = QtCore.Signal(int, int)
    enableRequested = QtCore.Signal(int, bool)
    stopRequested = QtCore.Signal(int)
    clearFaultRequested = QtCore.Signal(int)
    directionToggleRequested = QtCore.Signal(int)
    selfCheckRequested = QtCore.Signal(int)

    def __init__(self, bridge_index: int) -> None:
        super().__init__(f"Bridge {bridge_index}")
        self.bridge_index = bridge_index

        self.enabled_chip = StatusChip("Enabled state", "ENABLED", "DISABLED")
        self.direction_chip = StatusChip("Direction", "INVERTED", "NORMAL")
        self.power_value = ReadoutLabel("+0")
        self.duty_value = ReadoutLabel("0")
        self.current_value = ReadoutLabel("0.000 A")

        self.power_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.power_slider.setRange(-1000, 1000)
        self.power_slider.setSingleStep(10)
        self.power_slider.setPageStep(100)
        self.power_slider.setValue(0)
        self.power_slider.valueChanged.connect(self._sync_from_slider)

        self.power_spin = QtWidgets.QSpinBox()
        self.power_spin.setRange(-1000, 1000)
        self.power_spin.setSingleStep(10)
        self.power_spin.setValue(0)
        self.power_spin.valueChanged.connect(self._sync_from_spin)

        self.enable_button = QtWidgets.QPushButton("Enable")
        self.disable_button = QtWidgets.QPushButton("Disable")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.drive_button = QtWidgets.QPushButton("Drive")
        self.clear_fault_button = QtWidgets.QPushButton("Clear Fault")
        self.toggle_direction_button = QtWidgets.QPushButton("Toggle Direction")
        self.self_check_button = QtWidgets.QPushButton("Self-Check Direction")

        self.enable_button.clicked.connect(lambda: self.enableRequested.emit(self.bridge_index, True))
        self.disable_button.clicked.connect(lambda: self.enableRequested.emit(self.bridge_index, False))
        self.stop_button.clicked.connect(lambda: self.stopRequested.emit(self.bridge_index))
        self.drive_button.clicked.connect(self._emit_drive)
        self.clear_fault_button.clicked.connect(lambda: self.clearFaultRequested.emit(self.bridge_index))
        self.toggle_direction_button.clicked.connect(lambda: self.directionToggleRequested.emit(self.bridge_index))
        self.self_check_button.clicked.connect(lambda: self.selfCheckRequested.emit(self.bridge_index))

        top = QtWidgets.QGridLayout()
        top.addWidget(QtWidgets.QLabel("State"), 0, 0)
        top.addWidget(self.enabled_chip, 0, 1)
        top.addWidget(QtWidgets.QLabel("Direction"), 0, 2)
        top.addWidget(self.direction_chip, 0, 3)
        top.addWidget(QtWidgets.QLabel("Power"), 1, 0)
        top.addWidget(self.power_slider, 1, 1, 1, 4)
        top.addWidget(self.power_spin, 1, 5)
        top.addWidget(self.drive_button, 2, 1)
        top.addWidget(self.stop_button, 2, 2)
        top.addWidget(self.enable_button, 2, 3)
        top.addWidget(self.disable_button, 2, 4)
        top.addWidget(self.clear_fault_button, 2, 5)
        top.addWidget(self.toggle_direction_button, 3, 1, 1, 2)
        top.addWidget(self.self_check_button, 3, 3, 1, 3)

        stats = QtWidgets.QFormLayout()
        stats.addRow("Command", self.power_value)
        stats.addRow("Duty", self.duty_value)
        stats.addRow("Current", self.current_value)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(stats)

    def _sync_from_slider(self, value: int) -> None:
        if self.power_spin.value() != value:
            blocker = QtCore.QSignalBlocker(self.power_spin)
            self.power_spin.setValue(value)
            del blocker

    def _sync_from_spin(self, value: int) -> None:
        if self.power_slider.value() != value:
            blocker = QtCore.QSignalBlocker(self.power_slider)
            self.power_slider.setValue(value)
            del blocker

    def _emit_drive(self) -> None:
        self.driveRequested.emit(self.bridge_index, int(self.power_spin.value()))

    def set_sample(self, sample: Optional[TelemetrySample]) -> None:
        if sample is None:
            self.enabled_chip.set_active(False)
            self.direction_chip.set_active(False, "INVERTED", "NORMAL")
            self.power_value.set_value("+0")
            self.duty_value.set_value("0")
            self.current_value.set_value("0.000 A")
            return

        enabled = sample.bridge_enabled[self.bridge_index]
        self.enabled_chip.set_active(enabled)
        self.direction_chip.set_active(sample.bridge_direction_inverted[self.bridge_index], "INVERTED", "NORMAL")
        self.power_value.set_value(f"{sample.bridge_power[self.bridge_index]:+d}")
        self.duty_value.set_value(str(sample.bridge_duty[self.bridge_index]))
        self.current_value.set_value(f"{sample.current_ma[self.bridge_index] / 1000.0:.3f} A")

    def set_self_check_running(self, running: bool) -> None:
        self.toggle_direction_button.setEnabled(not running)
        self.self_check_button.setEnabled(not running)
        self.self_check_button.setText("Self-Check Running..." if running else "Self-Check Direction")


class ReportsPanel(QtWidgets.QGroupBox):
    pollNowRequested = QtCore.Signal()
    binaryReportRequested = QtCore.Signal(bool, int, int)
    textReportRequested = QtCore.Signal(bool, int, int)
    manualErrorRequested = QtCore.Signal(bool)
    pullupRequested = QtCore.Signal(int, bool)
    rawCommandRequested = QtCore.Signal(str)
    textStatusRequested = QtCore.Signal()
    telemetryLoggingRequested = QtCore.Signal(bool)
    autoPollRequested = QtCore.Signal(bool, int)

    def __init__(self) -> None:
        super().__init__("Reports and I/O")

        self.poll_button = QtWidgets.QPushButton("Poll M102")
        self.text_status_button = QtWidgets.QPushButton("Poll M101")
        self.binary_enabled = QtWidgets.QCheckBox("Binary report active")
        self.binary_interval = QtWidgets.QSpinBox()
        self.binary_interval.setRange(0, 60000)
        self.binary_interval.setSuffix(" ms")
        self.binary_interval.setValue(DEFAULT_BINARY_REPORT_MS)
        self.binary_usb = QtWidgets.QCheckBox("USB")
        self.binary_uart = QtWidgets.QCheckBox("UART")
        self.binary_usb.setChecked(True)

        self.text_enabled = QtWidgets.QCheckBox("Text report active")
        self.text_interval = QtWidgets.QSpinBox()
        self.text_interval.setRange(0, 60000)
        self.text_interval.setSuffix(" ms")
        self.text_interval.setValue(DEFAULT_TEXT_REPORT_MS)
        self.text_usb = QtWidgets.QCheckBox("USB")
        self.text_uart = QtWidgets.QCheckBox("UART")
        self.text_usb.setChecked(True)

        self.auto_poll = QtWidgets.QCheckBox("Auto-poll locally with M101")
        self.auto_poll_interval = QtWidgets.QSpinBox()
        self.auto_poll_interval.setRange(20, 60000)
        self.auto_poll_interval.setSuffix(" ms")
        self.auto_poll_interval.setValue(100)

        self.log_telemetry = QtWidgets.QCheckBox("Log telemetry packets")
        self.manual_error = QtWidgets.QCheckBox("Manual error")
        self.pullup0 = QtWidgets.QCheckBox("I2C0 pullups")
        self.pullup1 = QtWidgets.QCheckBox("I2C1 pullups")
        self.raw_command = QtWidgets.QLineEdit()
        self.raw_command.setPlaceholderText("Type any ASCII command here, for example M101 or M201 H0")
        self.send_raw_button = QtWidgets.QPushButton("Send")

        self.poll_button.clicked.connect(self.pollNowRequested.emit)
        self.text_status_button.clicked.connect(self.textStatusRequested.emit)
        self.binary_enabled.toggled.connect(self._emit_binary_settings)
        self.binary_interval.valueChanged.connect(self._emit_binary_settings)
        self.binary_usb.toggled.connect(self._emit_binary_settings)
        self.binary_uart.toggled.connect(self._emit_binary_settings)
        self.text_enabled.toggled.connect(self._emit_text_settings)
        self.text_interval.valueChanged.connect(self._emit_text_settings)
        self.text_usb.toggled.connect(self._emit_text_settings)
        self.text_uart.toggled.connect(self._emit_text_settings)
        self.auto_poll.toggled.connect(self._emit_auto_poll)
        self.auto_poll_interval.valueChanged.connect(self._emit_auto_poll)
        self.log_telemetry.toggled.connect(self.telemetryLoggingRequested.emit)
        self.manual_error.toggled.connect(self.manualErrorRequested.emit)
        self.pullup0.toggled.connect(lambda checked: self.pullupRequested.emit(0, checked))
        self.pullup1.toggled.connect(lambda checked: self.pullupRequested.emit(1, checked))
        self.send_raw_button.clicked.connect(self._emit_raw)
        self.raw_command.returnPressed.connect(self._emit_raw)

        binary_group = QtWidgets.QGroupBox("Binary Status Stream")
        binary_layout = QtWidgets.QGridLayout(binary_group)
        binary_layout.addWidget(self.binary_enabled, 0, 0, 1, 2)
        binary_layout.addWidget(QtWidgets.QLabel("Interval"), 1, 0)
        binary_layout.addWidget(self.binary_interval, 1, 1)
        binary_layout.addWidget(QtWidgets.QLabel("Ports"), 2, 0)
        port_row = QtWidgets.QHBoxLayout()
        port_row.addWidget(self.binary_usb)
        port_row.addWidget(self.binary_uart)
        port_row.addStretch(1)
        binary_layout.addLayout(port_row, 2, 1)

        text_group = QtWidgets.QGroupBox("Text Status Stream")
        text_layout = QtWidgets.QGridLayout(text_group)
        text_layout.addWidget(self.text_enabled, 0, 0, 1, 2)
        text_layout.addWidget(QtWidgets.QLabel("Interval"), 1, 0)
        text_layout.addWidget(self.text_interval, 1, 1)
        text_layout.addWidget(QtWidgets.QLabel("Ports"), 2, 0)
        text_port_row = QtWidgets.QHBoxLayout()
        text_port_row.addWidget(self.text_usb)
        text_port_row.addWidget(self.text_uart)
        text_port_row.addStretch(1)
        text_layout.addLayout(text_port_row, 2, 1)

        poll_group = QtWidgets.QGroupBox("Polling")
        poll_layout = QtWidgets.QGridLayout(poll_group)
        poll_layout.addWidget(self.poll_button, 0, 0)
        poll_layout.addWidget(self.text_status_button, 0, 1)
        poll_layout.addWidget(self.auto_poll, 1, 0, 1, 2)
        poll_layout.addWidget(QtWidgets.QLabel("Auto-poll interval"), 2, 0)
        poll_layout.addWidget(self.auto_poll_interval, 2, 1)

        io_group = QtWidgets.QGroupBox("I/O Controls")
        io_layout = QtWidgets.QGridLayout(io_group)
        io_layout.addWidget(self.log_telemetry, 0, 0, 1, 2)
        io_layout.addWidget(self.manual_error, 1, 0, 1, 2)
        io_layout.addWidget(self.pullup0, 2, 0)
        io_layout.addWidget(self.pullup1, 2, 1)

        raw_group = QtWidgets.QGroupBox("Raw Command")
        raw_layout = QtWidgets.QHBoxLayout(raw_group)
        raw_layout.addWidget(self.raw_command, 1)
        raw_layout.addWidget(self.send_raw_button)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(binary_group)
        layout.addWidget(text_group)
        layout.addWidget(poll_group)
        layout.addWidget(io_group)
        layout.addWidget(raw_group)
        layout.addStretch(1)

    def _emit_binary_settings(self) -> None:
        enabled = self.binary_enabled.isChecked()
        interval = int(self.binary_interval.value())
        port_mask = (PORT_USB if self.binary_usb.isChecked() else 0) | (PORT_UART if self.binary_uart.isChecked() else 0)
        self.binaryReportRequested.emit(enabled, interval, port_mask)

    def _emit_text_settings(self) -> None:
        enabled = self.text_enabled.isChecked()
        interval = int(self.text_interval.value())
        port_mask = (PORT_USB if self.text_usb.isChecked() else 0) | (PORT_UART if self.text_uart.isChecked() else 0)
        self.textReportRequested.emit(enabled, interval, port_mask)

    def _emit_auto_poll(self) -> None:
        self.autoPollRequested.emit(self.auto_poll.isChecked(), int(self.auto_poll_interval.value()))

    def _emit_raw(self) -> None:
        text = self.raw_command.text().strip()
        if text:
            self.rawCommandRequested.emit(text)
            self.raw_command.clear()

    def set_connected(self, connected: bool) -> None:
        for widget in (
            self.poll_button,
            self.text_status_button,
            self.binary_enabled,
            self.binary_interval,
            self.binary_usb,
            self.binary_uart,
            self.text_enabled,
            self.text_interval,
            self.text_usb,
            self.text_uart,
            self.auto_poll,
            self.auto_poll_interval,
            self.log_telemetry,
            self.manual_error,
            self.pullup0,
            self.pullup1,
            self.raw_command,
            self.send_raw_button,
        ):
            widget.setEnabled(connected)

    def set_manual_error_state(self, value: bool) -> None:
        blocker = QtCore.QSignalBlocker(self.manual_error)
        self.manual_error.setChecked(value)
        del blocker

    def set_pullups_state(self, bus0: bool, bus1: bool) -> None:
        blocker0 = QtCore.QSignalBlocker(self.pullup0)
        self.pullup0.setChecked(bus0)
        del blocker0
        blocker1 = QtCore.QSignalBlocker(self.pullup1)
        self.pullup1.setChecked(bus1)
        del blocker1


class FutureClosedLoopPanel(QtWidgets.QGroupBox):
    def __init__(self) -> None:
        super().__init__("Closed-Loop Control")

        self.history: deque[tuple[float, ClosedLoopSample]] = deque(maxlen=1200)
        self._dirty_fields: set[str] = set()

        self.query_status_button = QtWidgets.QPushButton("Refresh Status (M101)")
        self.query_config_button = QtWidgets.QPushButton("Query Encoder (M208)")
        self.reset_button = QtWidgets.QPushButton("Reset Loop (M207)")
        self.zero_position_button = QtWidgets.QPushButton("Zero Position (M210)")

        self.bridge = QtWidgets.QComboBox()
        self.bridge.addItems(["Bridge 0", "Bridge 1"])
        self.enabled = QtWidgets.QCheckBox("Bridge Enabled")
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["Manual", "Speed PID", "Position PID"])
        self.manual_command = QtWidgets.QSpinBox()
        self.manual_command.setRange(-1000, 1000)
        self.manual_command.setSingleStep(10)

        self.speed_counts = QtWidgets.QDoubleSpinBox()
        self.speed_counts.setRange(-DEFAULT_MAX_SPEED_REF_COUNTS_PER_SEC, DEFAULT_MAX_SPEED_REF_COUNTS_PER_SEC)
        self.speed_counts.setDecimals(3)
        self.speed_counts.setSuffix(" counts/s")
        self.speed_rpm = QtWidgets.QDoubleSpinBox()
        self.speed_rpm.setRange(-DEFAULT_MAX_SPEED_REF_RPM, DEFAULT_MAX_SPEED_REF_RPM)
        self.speed_rpm.setDecimals(3)
        self.speed_rpm.setSuffix(" rpm")

        self.position_counts = QtWidgets.QSpinBox()
        self.position_counts.setRange(-1000000, 1000000)
        self.position_turns = QtWidgets.QDoubleSpinBox()
        self.position_turns.setRange(-1000.0, 1000.0)
        self.position_turns.setDecimals(4)
        self.position_turns.setSuffix(" turns")
        self.position_degrees = QtWidgets.QDoubleSpinBox()
        self.position_degrees.setRange(-360000.0, 360000.0)
        self.position_degrees.setDecimals(3)
        self.position_degrees.setSuffix(" deg")

        self.speed_kp = QtWidgets.QDoubleSpinBox()
        self.speed_ki = QtWidgets.QDoubleSpinBox()
        self.speed_kd = QtWidgets.QDoubleSpinBox()
        self.position_kp = QtWidgets.QDoubleSpinBox()
        self.position_ki = QtWidgets.QDoubleSpinBox()
        self.position_kd = QtWidgets.QDoubleSpinBox()
        for spin in (self.speed_kp, self.speed_ki, self.speed_kd, self.position_kp, self.position_ki, self.position_kd):
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(5)
            spin.setSingleStep(0.01)

        self.encoder_resolution = QtWidgets.QSpinBox()
        self.encoder_resolution.setRange(10, 12)
        self.encoder_zero_offset = QtWidgets.QSpinBox()
        self.encoder_zero_offset.setRange(0, 4095)
        self.encoder_inverted = QtWidgets.QCheckBox("Inverted")

        self.report_enabled = QtWidgets.QCheckBox("Fast binary ACK mode (M209)")
        self.report_interval = QtWidgets.QSpinBox()
        self.report_interval.setRange(0, 60000)
        self.report_interval.setSuffix(" ms")
        self.report_interval.setValue(100)
        self.report_usb = QtWidgets.QCheckBox("USB")
        self.report_uart = QtWidgets.QCheckBox("UART")
        self.report_usb.setChecked(True)

        self.apply_bridge_button = QtWidgets.QPushButton("Select Bridge (M208)")
        self.apply_enable_button = QtWidgets.QPushButton("Apply Enable (M200)")
        self.apply_mode_button = QtWidgets.QPushButton("Apply Mode (M202)")
        self.apply_manual_button = QtWidgets.QPushButton("Force Manual (M203)")
        self.apply_speed_counts_button = QtWidgets.QPushButton("Send Speed Ref (M204 counts/s)")
        self.apply_speed_rpm_button = QtWidgets.QPushButton("Send Speed Ref (M204 rpm)")
        self.apply_position_counts_button = QtWidgets.QPushButton("Send Position Ref (M205 counts)")
        self.apply_position_turns_button = QtWidgets.QPushButton("Send Position Ref (M205 turns)")
        self.apply_position_degrees_button = QtWidgets.QPushButton("Send Position Ref (M205 degrees)")
        self.apply_speed_gain_button = QtWidgets.QPushButton("Apply Speed PID (M206)")
        self.apply_position_gain_button = QtWidgets.QPushButton("Apply Position PID (M206)")
        self.apply_encoder_button = QtWidgets.QPushButton("Apply Encoder (M208)")
        self.query_speed_gain_button = QtWidgets.QPushButton("Refresh M101")
        self.query_position_gain_button = QtWidgets.QPushButton("Refresh M101")

        self.status_bridge = StatusChip("Bridge", "H0", "H1")
        self.status_enabled = StatusChip("Enabled", "EN", "DIS")
        self.status_mode = StatusChip("Mode", "MAN", "MAN")
        self.status_encoder = StatusChip("Encoder", "OK", "BAD")
        self.status_manual = ReadoutLabel("0")
        self.status_output = ReadoutLabel("0")
        self.status_ref_pos = ReadoutLabel("0")
        self.status_meas_pos = ReadoutLabel("0")
        self.status_ref_speed = ReadoutLabel("0")
        self.status_meas_speed = ReadoutLabel("0")
        self.status_raw = ReadoutLabel("0")
        self.status_red = ReadoutLabel("0")

        self.position_plot = ChartCard(
            "Position Reference vs Measurement (deg)",
            [
                MetricSpec("Ref Pos", "#3aaed8", lambda s: float(s.ref_position_counts) * 360.0 / AS5600_COUNTS_PER_REV),
                MetricSpec(
                    "Meas Pos",
                    "#f59e0b",
                    lambda s: float(s.measured_position_counts) * 360.0 / AS5600_COUNTS_PER_REV,
                ),
            ],
        )
        self.speed_plot = ChartCard(
            "Speed Reference vs Measurement",
            [
                MetricSpec("Ref Speed", "#6ee7a6", lambda s: float(s.ref_speed_counts_per_sec)),
                MetricSpec("Meas Speed", "#f472b6", lambda s: float(s.measured_speed_counts_per_sec)),
            ],
        )

        self.query_status_button.clicked.connect(self._noop)
        self.query_config_button.clicked.connect(self._noop)
        self.reset_button.clicked.connect(self._noop)
        self.zero_position_button.clicked.connect(self._noop)
        self.apply_bridge_button.clicked.connect(self._noop)
        self.apply_enable_button.clicked.connect(self._noop)
        self.apply_mode_button.clicked.connect(self._noop)
        self.apply_manual_button.clicked.connect(self._noop)
        self.apply_speed_counts_button.clicked.connect(self._noop)
        self.apply_speed_rpm_button.clicked.connect(self._noop)
        self.apply_position_counts_button.clicked.connect(self._noop)
        self.apply_position_turns_button.clicked.connect(self._noop)
        self.apply_position_degrees_button.clicked.connect(self._noop)
        self.apply_speed_gain_button.clicked.connect(self._noop)
        self.apply_position_gain_button.clicked.connect(self._noop)
        self.apply_encoder_button.clicked.connect(self._noop)
        self.query_speed_gain_button.clicked.connect(self._noop)
        self.query_position_gain_button.clicked.connect(self._noop)
        self.report_enabled.toggled.connect(self._noop)
        self.report_interval.valueChanged.connect(self._noop)
        self.report_usb.toggled.connect(self._noop)
        self.report_uart.toggled.connect(self._noop)
        self.bridge.currentIndexChanged.connect(lambda _value: self._mark_dirty("bridge"))
        self.enabled.toggled.connect(lambda _checked: self._mark_dirty("enabled"))
        self.mode.currentIndexChanged.connect(lambda _value: self._mark_dirty("mode"))
        self.manual_command.valueChanged.connect(lambda _value: self._mark_dirty("manual"))
        self.speed_counts.valueChanged.connect(lambda _value: self._mark_dirty("speed_ref"))
        self.speed_rpm.valueChanged.connect(lambda _value: self._mark_dirty("speed_ref"))
        self.position_degrees.valueChanged.connect(lambda _value: self._mark_dirty("position_ref"))
        self.speed_kp.valueChanged.connect(lambda _value: self._mark_dirty("speed_pid"))
        self.speed_ki.valueChanged.connect(lambda _value: self._mark_dirty("speed_pid"))
        self.speed_kd.valueChanged.connect(lambda _value: self._mark_dirty("speed_pid"))
        self.position_kp.valueChanged.connect(lambda _value: self._mark_dirty("position_pid"))
        self.position_ki.valueChanged.connect(lambda _value: self._mark_dirty("position_pid"))
        self.position_kd.valueChanged.connect(lambda _value: self._mark_dirty("position_pid"))
        self.encoder_resolution.valueChanged.connect(lambda _value: self._mark_dirty("encoder"))
        self.encoder_zero_offset.valueChanged.connect(lambda _value: self._mark_dirty("encoder"))
        self.encoder_inverted.toggled.connect(lambda _checked: self._mark_dirty("encoder"))
        self.report_enabled.toggled.connect(lambda _checked: self._mark_dirty("fast_binary"))
        self.speed_counts.valueChanged.connect(self._sync_speed_from_counts)
        self.speed_rpm.valueChanged.connect(self._sync_speed_from_rpm)
        self.position_counts.valueChanged.connect(self._sync_position_from_counts)
        self.position_turns.valueChanged.connect(self._sync_position_from_turns)
        self.position_degrees.valueChanged.connect(self._sync_position_from_degrees)

        ctrl_group = QtWidgets.QGroupBox("Controller")
        ctrl_form = QtWidgets.QFormLayout(ctrl_group)
        ctrl_form.addRow("Bridge", self.bridge)
        ctrl_form.addRow("Enabled", self.enabled)
        ctrl_form.addRow("Mode", self.mode)
        ctrl_form.addRow("Manual command", self.manual_command)
        ctrl_button_row = QtWidgets.QHBoxLayout()
        for button in (self.query_status_button, self.query_config_button, self.reset_button, self.zero_position_button):
            ctrl_button_row.addWidget(button)
        ctrl_button_row.addStretch(1)
        ctrl_form.addRow(ctrl_button_row)
        apply_row = QtWidgets.QHBoxLayout()
        for button in (self.apply_bridge_button, self.apply_enable_button, self.apply_mode_button, self.apply_manual_button):
            apply_row.addWidget(button)
        apply_row.addStretch(1)
        ctrl_form.addRow(apply_row)
        manual_note = QtWidgets.QLabel("Any M203 manual-drive command forces the selected closed-loop bridge back to Manual mode.")
        manual_note.setWordWrap(True)
        manual_note.setStyleSheet("color: #9aa7b4;")
        ctrl_form.addRow(manual_note)

        setpoint_group = QtWidgets.QGroupBox("PID References")
        setpoint_form = QtWidgets.QGridLayout(setpoint_group)
        setpoint_form.addWidget(QtWidgets.QLabel("Speed ref counts/s"), 0, 0)
        setpoint_form.addWidget(self.speed_counts, 0, 1)
        setpoint_form.addWidget(self.apply_speed_counts_button, 0, 2)
        setpoint_form.addWidget(QtWidgets.QLabel("Speed ref rpm"), 1, 0)
        setpoint_form.addWidget(self.speed_rpm, 1, 1)
        setpoint_form.addWidget(self.apply_speed_rpm_button, 1, 2)
        setpoint_form.addWidget(QtWidgets.QLabel("Position ref degrees"), 2, 0)
        setpoint_form.addWidget(self.position_degrees, 2, 1)
        setpoint_form.addWidget(self.apply_position_degrees_button, 2, 2)
        reference_note = QtWidgets.QLabel(
            "These controls send the actual PID references: M204 for Speed PID and M205 for Position PID."
        )
        reference_note.setWordWrap(True)
        reference_note.setStyleSheet("color: #9aa7b4;")
        setpoint_form.addWidget(reference_note, 3, 0, 1, 3)

        gain_group = QtWidgets.QGroupBox("Gains")
        gain_form = QtWidgets.QGridLayout(gain_group)
        gain_form.addWidget(QtWidgets.QLabel("Speed Kp"), 0, 0)
        gain_form.addWidget(self.speed_kp, 0, 1)
        gain_form.addWidget(QtWidgets.QLabel("Speed Ki"), 0, 2)
        gain_form.addWidget(self.speed_ki, 0, 3)
        gain_form.addWidget(QtWidgets.QLabel("Speed Kd"), 0, 4)
        gain_form.addWidget(self.speed_kd, 0, 5)
        gain_form.addWidget(self.query_speed_gain_button, 1, 0, 1, 3)
        gain_form.addWidget(self.apply_speed_gain_button, 1, 3, 1, 3)
        gain_form.addWidget(QtWidgets.QLabel("Position Kp"), 2, 0)
        gain_form.addWidget(self.position_kp, 2, 1)
        gain_form.addWidget(QtWidgets.QLabel("Position Ki"), 2, 2)
        gain_form.addWidget(self.position_ki, 2, 3)
        gain_form.addWidget(QtWidgets.QLabel("Position Kd"), 2, 4)
        gain_form.addWidget(self.position_kd, 2, 5)
        gain_form.addWidget(self.query_position_gain_button, 3, 0, 1, 3)
        gain_form.addWidget(self.apply_position_gain_button, 3, 3, 1, 3)

        encoder_group = QtWidgets.QGroupBox("Encoder")
        encoder_form = QtWidgets.QGridLayout(encoder_group)
        encoder_form.addWidget(QtWidgets.QLabel("Resolution bits"), 0, 0)
        encoder_form.addWidget(self.encoder_resolution, 0, 1)
        encoder_form.addWidget(QtWidgets.QLabel("Zero offset"), 0, 2)
        encoder_form.addWidget(self.encoder_zero_offset, 0, 3)
        encoder_form.addWidget(self.encoder_inverted, 0, 4)
        encoder_form.addWidget(self.apply_encoder_button, 1, 0, 1, 5)

        report_group = QtWidgets.QGroupBox("Binary ACK Mode")
        report_form = QtWidgets.QGridLayout(report_group)
        report_form.addWidget(self.report_enabled, 0, 0, 1, 2)
        fast_note = QtWidgets.QLabel("When enabled, write commands return compact binary ACKs instead of full status frames.")
        fast_note.setWordWrap(True)
        fast_note.setStyleSheet("color: #9aa7b4;")
        report_form.addWidget(fast_note, 1, 0, 1, 2)
        self.report_interval.hide()
        self.report_usb.hide()
        self.report_uart.hide()

        status_group = QtWidgets.QGroupBox("Status")
        status_form = QtWidgets.QGridLayout(status_group)
        status_form.addWidget(QtWidgets.QLabel("Bridge"), 0, 0)
        status_form.addWidget(self.status_bridge, 0, 1)
        status_form.addWidget(QtWidgets.QLabel("Enabled"), 0, 2)
        status_form.addWidget(self.status_enabled, 0, 3)
        status_form.addWidget(QtWidgets.QLabel("Mode"), 0, 4)
        status_form.addWidget(self.status_mode, 0, 5)
        status_form.addWidget(QtWidgets.QLabel("Encoder"), 0, 6)
        status_form.addWidget(self.status_encoder, 0, 7)
        row = 1
        for label, widget in (
            ("Manual", self.status_manual),
            ("Output", self.status_output),
            ("Ref pos (deg)", self.status_ref_pos),
            ("Meas pos (deg)", self.status_meas_pos),
            ("Ref speed", self.status_ref_speed),
            ("Meas speed", self.status_meas_speed),
            ("Raw", self.status_raw),
            ("Reduced", self.status_red),
        ):
            status_form.addWidget(QtWidgets.QLabel(label), row, 0)
            status_form.addWidget(widget, row, 1)
            row += 1

        note = QtWidgets.QLabel(
            "This panel matches the current firmware: M102 for compact live status, M101 for full text status, "
            "M200 for bridge enable, M202 to M208 for loop setup, M204/M205 for speed and position references, "
            "M209 for fast binary ACK mode, and M210 to zero the current position."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #9aa7b4;")
        sequence_note = QtWidgets.QLabel(
            "Speed PID setup: enable bridge, apply Speed PID gains, switch mode to Speed PID, then send a speed reference. "
            "Avoid any manual-drive command while tuning."
        )
        sequence_note.setWordWrap(True)
        sequence_note.setStyleSheet("color: #9aa7b4;")

        top_grid = QtWidgets.QGridLayout()
        top_grid.addWidget(ctrl_group, 0, 0)
        top_grid.addWidget(setpoint_group, 0, 1)
        top_grid.addWidget(gain_group, 1, 0)
        top_grid.addWidget(encoder_group, 1, 1)
        top_grid.addWidget(report_group, 2, 0)
        top_grid.addWidget(status_group, 2, 1)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top_grid)
        layout.addWidget(note)
        layout.addWidget(sequence_note)
        layout.addWidget(self.position_plot)
        layout.addWidget(self.speed_plot)
        layout.addStretch(1)

    def _noop(self, *_args: object) -> None:
        return

    def _mark_dirty(self, *names: str) -> None:
        for name in names:
            self._dirty_fields.add(name)

    def _clear_dirty(self, *names: str) -> None:
        for name in names:
            self._dirty_fields.discard(name)

    def _is_dirty(self, name: str) -> bool:
        return name in self._dirty_fields

    def _sync_speed_from_counts(self, value: float) -> None:
        blocker = QtCore.QSignalBlocker(self.speed_rpm)
        self.speed_rpm.setValue(self._counts_to_rpm(value))
        del blocker

    def _sync_speed_from_rpm(self, value: float) -> None:
        blocker = QtCore.QSignalBlocker(self.speed_counts)
        self.speed_counts.setValue(self._rpm_to_counts(value))
        del blocker

    def _sync_position_from_counts(self, value: int) -> None:
        counts = float(value)
        blocker_turns = QtCore.QSignalBlocker(self.position_turns)
        blocker_degrees = QtCore.QSignalBlocker(self.position_degrees)
        self.position_turns.setValue(self._counts_to_turns(counts))
        self.position_degrees.setValue(self._counts_to_degrees(counts))
        del blocker_turns
        del blocker_degrees

    def _sync_position_from_turns(self, value: float) -> None:
        counts = self._turns_to_counts(value)
        blocker_counts = QtCore.QSignalBlocker(self.position_counts)
        blocker_degrees = QtCore.QSignalBlocker(self.position_degrees)
        self.position_counts.setValue(int(round(counts)))
        self.position_degrees.setValue(self._turns_to_degrees(value))
        del blocker_counts
        del blocker_degrees

    def _sync_position_from_degrees(self, value: float) -> None:
        counts = self._degrees_to_counts(value)
        blocker_counts = QtCore.QSignalBlocker(self.position_counts)
        blocker_turns = QtCore.QSignalBlocker(self.position_turns)
        self.position_counts.setValue(int(round(counts)))
        self.position_turns.setValue(self._degrees_to_turns(value))
        del blocker_counts
        del blocker_turns

    @staticmethod
    def _widget_is_active(widget: QtWidgets.QWidget) -> bool:
        focus_widget = QtWidgets.QApplication.focusWidget()
        return bool(focus_widget and (focus_widget is widget or widget.isAncestorOf(focus_widget)))

    def _set_combo_index_if_idle(self, widget: QtWidgets.QComboBox, field_name: str, index: int) -> None:
        if not self._widget_is_active(widget) and not self._is_dirty(field_name):
            self._set_combo_index(widget, index)

    def _set_checked_if_idle(self, widget: QtWidgets.QCheckBox, field_name: str, checked: bool) -> None:
        if not self._widget_is_active(widget) and not self._is_dirty(field_name):
            self._set_checked(widget, checked)

    def _set_value_if_idle(self, widget: QtWidgets.QWidget, field_name: str, value: float | int) -> None:
        if self._widget_is_active(widget) or self._is_dirty(field_name):
            return
        blocker = QtCore.QSignalBlocker(widget)
        widget.setValue(value)
        del blocker

    def set_sample(self, sample: Optional[ClosedLoopSample]) -> None:
        if sample is None:
            self.history.clear()
            self._dirty_fields.clear()
            self.status_bridge.set_active(False, "--", "--")
            self.status_enabled.set_active(False)
            self.status_mode.set_active(False, "MAN", "MAN")
            self.status_encoder.set_active(False)
            self.status_manual.set_value("-")
            self.status_output.set_value("-")
            self.status_ref_pos.set_value("-")
            self.status_meas_pos.set_value("-")
            self.status_ref_speed.set_value("-")
            self.status_meas_speed.set_value("-")
            self.status_raw.set_value("-")
            self.status_red.set_value("-")
            self.position_plot.update_from_history([])
            self.speed_plot.update_from_history([])
            return

        self.history.append((time.monotonic(), sample))
        cutoff = self.history[-1][0] - WINDOW_SECONDS
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

        self.status_bridge.set_active(True, f"H{sample.bridge_index}", f"H{sample.bridge_index}")
        self.status_enabled.set_active(sample.enabled, "EN", "DIS")
        self.status_mode.set_active(True, sample.mode_name(), sample.mode_name())
        self.status_encoder.set_active(sample.encoder_healthy)
        self.status_manual.set_value(str(sample.manual_permille))
        self.status_output.set_value(str(sample.output_permille))
        self.status_ref_pos.set_value(f"{self._counts_to_degrees(sample.ref_position_counts):.3f} deg")
        self.status_meas_pos.set_value(f"{self._counts_to_degrees(sample.measured_position_counts):.3f} deg")
        self.status_ref_speed.set_value(f"{sample.ref_speed_counts_per_sec:.3f}")
        self.status_meas_speed.set_value(f"{sample.measured_speed_counts_per_sec:.3f}")
        self.status_raw.set_value(str(sample.encoder_native_counts))
        self.status_red.set_value(str(sample.encoder_reduced_counts))

        self._set_combo_index_if_idle(self.bridge, "bridge", sample.bridge_index)
        self._set_checked_if_idle(self.enabled, "enabled", sample.enabled)
        self._set_combo_index_if_idle(self.mode, "mode", sample.mode)
        self._set_value_if_idle(self.manual_command, "manual", sample.manual_permille)
        self._set_value_if_idle(self.speed_counts, "speed_ref", sample.ref_speed_counts_per_sec)
        self._set_value_if_idle(self.speed_rpm, "speed_ref", self._counts_to_rpm(sample.ref_speed_counts_per_sec))
        self._set_value_if_idle(self.position_counts, "position_ref", int(round(sample.ref_position_counts)))
        self._set_value_if_idle(self.position_turns, "position_ref", self._counts_to_turns(sample.ref_position_counts))
        self._set_value_if_idle(self.position_degrees, "position_ref", self._counts_to_degrees(sample.ref_position_counts))
        if sample.encoder_resolution_bits is not None:
            self._set_value_if_idle(self.encoder_resolution, "encoder", sample.encoder_resolution_bits)
        self._set_checked_if_idle(self.report_enabled, "fast_binary", sample.fast_binary_mode)
        if sample.speed_pid_gains is not None:
            self._set_value_if_idle(self.speed_kp, "speed_pid", sample.speed_pid_gains[0])
            self._set_value_if_idle(self.speed_ki, "speed_pid", sample.speed_pid_gains[1])
            self._set_value_if_idle(self.speed_kd, "speed_pid", sample.speed_pid_gains[2])
        if sample.position_pid_gains is not None:
            self._set_value_if_idle(self.position_kp, "position_pid", sample.position_pid_gains[0])
            self._set_value_if_idle(self.position_ki, "position_pid", sample.position_pid_gains[1])
            self._set_value_if_idle(self.position_kd, "position_pid", sample.position_pid_gains[2])

        self.position_plot.update_from_history(list(self.history))
        self.speed_plot.update_from_history(list(self.history))

    def apply_ack(self, kind: str, fields: dict[str, str]) -> None:
        self._apply_common_fields(fields)

        if kind in {"bridge", "encoder"}:
            self._apply_bridge_field(fields)
        if kind in {"enabled"}:
            self._apply_enabled_field(fields)
        if kind in {"mode"}:
            self._apply_mode_field(fields)
        if kind in {"manual"}:
            self._apply_manual_field(fields)
        if kind in {"speed_counts", "speed_rpm"}:
            self._apply_speed_fields(fields)
        if kind in {"position_counts", "position_turns", "position_degrees"}:
            self._apply_position_fields(fields)
        if kind == "speed_pid":
            self._apply_speed_gain_fields(fields)
        if kind == "position_pid":
            self._apply_position_gain_fields(fields)
        if kind == "encoder":
            self._apply_encoder_fields(fields)
        if kind == "fast_binary":
            self._apply_fast_binary_field(fields)
        if kind == "zero_position":
            self._clear_dirty("position_ref")
            self.history.clear()
            self.position_plot.update_from_history([])
            self.speed_plot.update_from_history([])

        if kind == "bridge":
            self._clear_dirty("bridge")
        elif kind == "enabled":
            self._clear_dirty("enabled")
        elif kind == "mode":
            self._clear_dirty("mode")
        elif kind == "manual":
            self._clear_dirty("manual", "mode")
        elif kind in {"speed_counts", "speed_rpm"}:
            self._clear_dirty("speed_ref")
        elif kind in {"position_counts", "position_turns", "position_degrees"}:
            self._clear_dirty("position_ref")
        elif kind == "speed_pid":
            self._clear_dirty("speed_pid")
        elif kind == "position_pid":
            self._clear_dirty("position_pid")
        elif kind == "encoder":
            self._clear_dirty("bridge", "encoder")
        elif kind == "fast_binary":
            self._clear_dirty("fast_binary")

    def _apply_common_fields(self, fields: dict[str, str]) -> None:
        self._apply_bridge_field(fields)
        self._apply_enabled_field(fields)
        self._apply_mode_field(fields)
        self._apply_manual_field(fields)
        if "raw" in fields:
            self.status_raw.set_value(fields["raw"])
        if "red" in fields:
            self.status_red.set_value(fields["red"])

    def _apply_bridge_field(self, fields: dict[str, str]) -> None:
        value = self._extract_int(fields, ("bridge", "h", "b"))
        if value is not None:
            self._set_combo_index(self.bridge, value)

    def _apply_enabled_field(self, fields: dict[str, str]) -> None:
        enabled = self._extract_bool(fields, ("enabled", "en"))
        if enabled is not None:
            self._set_checked(self.enabled, enabled)

    def _apply_mode_field(self, fields: dict[str, str]) -> None:
        mode_value = self._extract_value(fields, ("loop_mode", "mode", "m"))
        if mode_value is None:
            return
        if mode_value.isdigit():
            self._set_combo_index(self.mode, int(mode_value))
            return
        mode_index = {"manual": 0, "speed": 1, "position": 2}.get(mode_value.lower())
        if mode_index is not None:
            self._set_combo_index(self.mode, mode_index)

    def _apply_manual_field(self, fields: dict[str, str]) -> None:
        manual = self._extract_int(fields, ("manual_command", "manual", "man"))
        if manual is not None:
            self.manual_command.setValue(manual)
            self.status_manual.set_value(str(manual))

    def _apply_speed_fields(self, fields: dict[str, str]) -> None:
        if "speed_ref" in fields:
            raw_value = self._extract_float(fields, ("speed_ref",))
            unit = fields.get("unit", "")
            if raw_value is not None:
                if unit == "mcps":
                    counts = raw_value / 1000.0
                    self.speed_counts.setValue(counts)
                    self.speed_rpm.setValue(self._counts_to_rpm(counts))
                    return
                if unit == "mrpm":
                    rpm = raw_value / 1000.0
                    self.speed_rpm.setValue(rpm)
                    self.speed_counts.setValue(self._rpm_to_counts(rpm))
                    return

        counts = self._extract_float(fields, ("counts", "rs"))
        if counts is not None:
            self.speed_counts.setValue(counts)
            self.speed_rpm.setValue(self._counts_to_rpm(counts))

        rpm = self._extract_float(fields, ("rpm",))
        if rpm is not None:
            self.speed_rpm.setValue(rpm)
            self.speed_counts.setValue(self._rpm_to_counts(rpm))

    def _apply_position_fields(self, fields: dict[str, str]) -> None:
        if "position_ref" in fields:
            raw_value = self._extract_float(fields, ("position_ref",))
            unit = fields.get("unit", "")
            if raw_value is not None:
                if unit == "mcount":
                    counts = raw_value / 1000.0
                    self.position_counts.setValue(int(round(counts)))
                    self.position_turns.setValue(self._counts_to_turns(counts))
                    self.position_degrees.setValue(self._counts_to_degrees(counts))
                    return
                if unit == "mdeg":
                    degrees = raw_value / 1000.0
                    self.position_degrees.setValue(degrees)
                    counts = self._degrees_to_counts(degrees)
                    self.position_counts.setValue(int(round(counts)))
                    self.position_turns.setValue(self._degrees_to_turns(degrees))
                    return
                if unit == "mturn":
                    turns = raw_value / 1000.0
                    self.position_turns.setValue(turns)
                    counts = self._turns_to_counts(turns)
                    self.position_counts.setValue(int(round(counts)))
                    self.position_degrees.setValue(self._turns_to_degrees(turns))
                    return

        counts = self._extract_float(fields, ("counts", "rp"))
        if counts is not None:
            self.position_counts.setValue(int(round(counts)))
            self.position_turns.setValue(self._counts_to_turns(counts))
            self.position_degrees.setValue(self._counts_to_degrees(counts))

        turns = self._extract_float(fields, ("turns",))
        if turns is not None:
            self.position_turns.setValue(turns)
            counts = self._turns_to_counts(turns)
            self.position_counts.setValue(int(round(counts)))
            self.position_degrees.setValue(self._turns_to_degrees(turns))

        degrees = self._extract_float(fields, ("degrees",))
        if degrees is not None:
            self.position_degrees.setValue(degrees)
            counts = self._degrees_to_counts(degrees)
            self.position_counts.setValue(int(round(counts)))
            self.position_turns.setValue(self._degrees_to_turns(degrees))

    def _apply_speed_gain_fields(self, fields: dict[str, str]) -> None:
        kp = self._extract_float(fields, ("speed_kp", "kp"))
        ki = self._extract_float(fields, ("speed_ki", "ki"))
        kd = self._extract_float(fields, ("speed_kd", "kd"))
        if kp is not None:
            self.speed_kp.setValue(kp / 1000.0 if abs(kp) > 10.0 else kp)
        if ki is not None:
            self.speed_ki.setValue(ki / 1000.0 if abs(ki) > 10.0 else ki)
        if kd is not None:
            self.speed_kd.setValue(kd / 1000.0 if abs(kd) > 10.0 else kd)

    def _apply_position_gain_fields(self, fields: dict[str, str]) -> None:
        kp = self._extract_float(fields, ("position_kp", "kp"))
        ki = self._extract_float(fields, ("position_ki", "ki"))
        kd = self._extract_float(fields, ("position_kd", "kd"))
        if kp is not None:
            self.position_kp.setValue(kp / 1000.0 if abs(kp) > 10.0 else kp)
        if ki is not None:
            self.position_ki.setValue(ki / 1000.0 if abs(ki) > 10.0 else ki)
        if kd is not None:
            self.position_kd.setValue(kd / 1000.0 if abs(kd) > 10.0 else kd)

    def _apply_encoder_fields(self, fields: dict[str, str]) -> None:
        bits = self._extract_int(fields, ("bits",))
        zero = self._extract_int(fields, ("zero",))
        inverted = self._extract_bool(fields, ("invert",))
        if bits is not None:
            self.encoder_resolution.setValue(bits)
        if zero is not None:
            self.encoder_zero_offset.setValue(zero)
        if inverted is not None:
            self.encoder_inverted.setChecked(inverted)

    def _apply_fast_binary_field(self, fields: dict[str, str]) -> None:
        enabled = self._extract_bool(fields, ("fast_binary", "enabled"))
        if enabled is not None:
            self._set_checked(self.report_enabled, enabled)

    def _extract_int(self, fields: dict[str, str], names: tuple[str, ...]) -> Optional[int]:
        value = self._extract_value(fields, names)
        if value is None:
            return None
        try:
            return int(float(value))
        except ValueError:
            return None

    def _extract_float(self, fields: dict[str, str], names: tuple[str, ...]) -> Optional[float]:
        value = self._extract_value(fields, names)
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    def _extract_bool(self, fields: dict[str, str], names: tuple[str, ...]) -> Optional[bool]:
        value = self._extract_value(fields, names)
        if value is None:
            return None
        return value.lower() not in {"0", "false", "off", "no"}

    def _extract_value(self, fields: dict[str, str], names: tuple[str, ...]) -> Optional[str]:
        for name in names:
            if name in fields:
                return fields[name]
        return None

    def _set_checked(self, widget: QtWidgets.QCheckBox, checked: bool) -> None:
        blocker = QtCore.QSignalBlocker(widget)
        widget.setChecked(checked)
        del blocker

    def _set_combo_index(self, widget: QtWidgets.QComboBox, index: int) -> None:
        index = clamp_int(index, 0, widget.count() - 1)
        blocker = QtCore.QSignalBlocker(widget)
        widget.setCurrentIndex(index)
        del blocker

    @staticmethod
    def _safe_int(text: str, fallback: int) -> int:
        try:
            return int(float(text))
        except ValueError:
            return fallback

    @staticmethod
    def _safe_bool(text: str, fallback: bool) -> bool:
        lowered = text.lower()
        if lowered in {"0", "false", "off", "no"}:
            return False
        if lowered in {"1", "true", "on", "yes"}:
            return True
        return fallback

    @staticmethod
    def _counts_to_rpm(counts: float) -> float:
        return counts * 60.0 / AS5600_COUNTS_PER_REV

    @staticmethod
    def _rpm_to_counts(rpm: float) -> float:
        return rpm * AS5600_COUNTS_PER_REV / 60.0

    @staticmethod
    def _counts_to_turns(counts: float) -> float:
        return counts / AS5600_COUNTS_PER_REV

    @staticmethod
    def _turns_to_counts(turns: float) -> float:
        return turns * AS5600_COUNTS_PER_REV

    @staticmethod
    def _counts_to_degrees(counts: float) -> float:
        return counts * 360.0 / AS5600_COUNTS_PER_REV

    @staticmethod
    def _degrees_to_counts(degrees: float) -> float:
        return degrees * AS5600_COUNTS_PER_REV / 360.0

    @staticmethod
    def _turns_to_degrees(turns: float) -> float:
        return turns * 360.0

    @staticmethod
    def _degrees_to_turns(degrees: float) -> float:
        return degrees / 360.0


class CommandLog(QtWidgets.QGroupBox):
    def __init__(self) -> None:
        super().__init__("Command and Telemetry Log")
        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(4000)
        font = QtGui.QFont("Consolas")
        font.setPointSize(9)
        self.text.setFont(font)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.text)

    def append(self, kind: str, message: str) -> None:
        self.text.appendPlainText(f"{now_stamp()} [{kind}] {message}")
        scrollbar = self.text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class SerialLink(QtCore.QObject):
    connectedChanged = QtCore.Signal(bool)
    disconnected = QtCore.Signal(str)
    telemetryReceived = QtCore.Signal(object)
    textReceived = QtCore.Signal(str)
    logMessage = QtCore.Signal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._port = QSerialPort(self)
        self._rx_buffer = bytearray()
        self._port.readyRead.connect(self._read_ready)
        self._port.errorOccurred.connect(self._on_error)

    @staticmethod
    def available_ports() -> list[tuple[str, str]]:
        ports: list[tuple[str, str]] = []
        for info in QSerialPortInfo.availablePorts():
            label = info.portName()
            description = info.description().strip()
            if description:
                label = f"{label} - {description}"
            ports.append((info.portName(), label))
        return ports

    def is_connected(self) -> bool:
        return self._port.isOpen()

    def connect_port(self, port_name: str, baud_rate: int) -> tuple[bool, str]:
        if self._port.isOpen():
            self._port.close()

        self._rx_buffer.clear()
        self._port.setPortName(port_name)
        self._port.setBaudRate(baud_rate)
        self._port.setDataBits(QSerialPort.DataBits.Data8)
        self._port.setParity(QSerialPort.Parity.NoParity)
        self._port.setStopBits(QSerialPort.StopBits.OneStop)
        self._port.setFlowControl(QSerialPort.FlowControl.NoFlowControl)

        if not self._port.open(QtCore.QIODevice.OpenModeFlag.ReadWrite):
            return False, self._port.errorString()

        self._port.setDataTerminalReady(True)
        self._port.setRequestToSend(True)
        self.connectedChanged.emit(True)
        return True, f"Connected to {port_name} @ {baud_rate}"

    def disconnect_port(self) -> None:
        if self._port.isOpen():
            self._port.close()
        self._rx_buffer.clear()
        self.connectedChanged.emit(False)

    def send_line(self, line: str) -> None:
        if not self._port.isOpen():
            raise RuntimeError("serial port is not connected")

        text = line.strip()
        if not text:
            return
        data = (text + "\r\n").encode("ascii", errors="replace")
        self._port.write(data)

    def _read_ready(self) -> None:
        if not self._port.isOpen():
            return

        data = bytes(self._port.readAll())
        if not data:
            return

        self._rx_buffer.extend(data)
        self._process_buffer()

    def _process_buffer(self) -> None:
        while self._rx_buffer:
            if self._rx_buffer[0] == 0xA5:
                if len(self._rx_buffer) < 2:
                    return
                expected = self._rx_buffer[1] + 3
                if expected < 4 or expected > 255:
                    del self._rx_buffer[0]
                    continue
                if len(self._rx_buffer) < expected:
                    return

                frame = bytes(self._rx_buffer[:expected])
                del self._rx_buffer[:expected]
                try:
                    packet = parse_binary_packet(frame)
                except ValueError as exc:
                    self.logMessage.emit(f"Binary parse error: {exc}")
                    continue

                if not packet.checksum_ok:
                    self.logMessage.emit("Binary checksum mismatch")
                    continue

                if packet.command == RESP_ERROR:
                    if len(packet.payload) >= 2:
                        self.logMessage.emit(describe_binary_error(packet.payload[0], packet.payload[1]))
                    else:
                        self.logMessage.emit(f"Binary error packet: {packet.payload.hex(' ')}")
                else:
                    try:
                        sample = parse_binary_status(packet)
                    except ValueError:
                        if len(packet.payload) == 1:
                            self.logMessage.emit(
                                f"Binary ACK 0x{packet.command:02X}: status=0x{packet.payload[0]:02X}"
                            )
                        else:
                            self.logMessage.emit(f"Binary packet 0x{packet.command:02X}: {packet.payload.hex(' ')}")
                    else:
                        self.telemetryReceived.emit(sample)
                continue

            newline = self._rx_buffer.find(b"\n")
            if newline < 0:
                return

            raw_line = bytes(self._rx_buffer[: newline + 1])
            del self._rx_buffer[: newline + 1]
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                self.textReceived.emit(line)

    def _on_error(self, error: QSerialPort.SerialPortError) -> None:
        if error in (QSerialPort.SerialPortError.NoError, QSerialPort.SerialPortError.TimeoutError):
            return
        if error == QSerialPort.SerialPortError.ResourceError and self._port.isOpen():
            message = self._port.errorString() or "serial resource error"
            self.logMessage.emit(message)
            self.disconnected.emit(message)
            self.disconnect_port()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, initial_port: Optional[str] = None, initial_baud: int = DEFAULT_BAUD, window_seconds: float = WINDOW_SECONDS) -> None:
        super().__init__()
        self.setWindowTitle("Pico Dual Motor Driver GUI")
        self.resize(1720, 1080)

        self.commands = FirmwareCommands()
        self.serial = SerialLink(self)
        self.history = TelemetryHistory(window_seconds=window_seconds)
        self.latest_sample: Optional[TelemetrySample] = None
        self.latest_closed_loop_sample: Optional[ClosedLoopSample] = None
        self._closed_loop_pending: deque[str] = deque()
        self._bridge_direction_inverted = [False, False]
        self._direction_self_check: Optional[DirectionSelfCheckSession] = None
        self.last_telemetry_log_ms = 0

        self.auto_poll_timer = QtCore.QTimer(self)
        self.auto_poll_timer.timeout.connect(self._auto_poll_tick)

        self._build_ui()

        self.serial.connectedChanged.connect(self._on_connected_changed)
        self.serial.disconnected.connect(self._on_disconnected)
        self.serial.telemetryReceived.connect(self._on_telemetry)
        self.serial.textReceived.connect(self._on_text_line)
        self.serial.logMessage.connect(lambda message: self.log.append("SER", message))
        self.scope.clearRequested.connect(self._clear_scope_history)

        self._refresh_ports()
        self._restore_defaults(initial_port, initial_baud)
        self._apply_connected_state(False)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)

        root.addWidget(self._build_connection_bar())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        top_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)

        self.tabs = QtWidgets.QTabWidget()
        self.live_tab = self._scrollable(self._build_live_tab())
        self.reports = ReportsPanel()
        self.reports_tab = self._scrollable(self.reports)
        self.closed_loop = FutureClosedLoopPanel()
        self.closed_loop_tab = self._scrollable(self.closed_loop)
        self.tabs.addTab(self.live_tab, "Live Control")
        self.tabs.addTab(self.reports_tab, "Reports")
        self.tabs.addTab(self.closed_loop_tab, "Closed Loop")

        self.scope = TelemetryScopePanel()
        top_splitter.addWidget(self.tabs)
        top_splitter.addWidget(self.scope)
        top_splitter.setStretchFactor(0, 0)
        top_splitter.setStretchFactor(1, 1)

        self.log = CommandLog()
        splitter.addWidget(top_splitter)
        splitter.addWidget(self.log)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter)

        self.setCentralWidget(central)
        self.statusBar().showMessage("Disconnected")

        self._connect_reports_controls()
        self._connect_closed_loop_controls()

    def _build_connection_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        layout = QtWidgets.QHBoxLayout(bar)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh Ports")
        self.baud_combo = QtWidgets.QComboBox()
        for baud in ("115200", "230400", "460800", "921600", "1000000"):
            self.baud_combo.addItem(baud, int(baud))
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.status_indicator = StatusChip("Connection", "CONNECTED", "DISCONNECTED")
        self.status_indicator.setMinimumWidth(120)
        self.status_indicator.set_active(False, "CONNECTED", "DISCONNECTED")

        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)

        layout.addWidget(QtWidgets.QLabel("Port"))
        layout.addWidget(self.port_combo, 1)
        layout.addWidget(self.refresh_button)
        layout.addSpacing(8)
        layout.addWidget(QtWidgets.QLabel("Baud"))
        layout.addWidget(self.baud_combo)
        layout.addWidget(self.connect_button)
        layout.addStretch(1)
        layout.addWidget(self.status_indicator)
        return bar

    def _build_live_tab(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        self.status_panel = StatusPanel()
        self.bridge_cards = [BridgeControlCard(0), BridgeControlCard(1)]
        for card in self.bridge_cards:
            card.driveRequested.connect(self._drive_bridge)
            card.enableRequested.connect(self._enable_bridge)
            card.stopRequested.connect(self._stop_bridge)
            card.clearFaultRequested.connect(self._clear_faults)
            card.directionToggleRequested.connect(self._toggle_bridge_direction)
            card.selfCheckRequested.connect(self._start_direction_self_check)

        global_group = QtWidgets.QGroupBox("Global Actions")
        global_layout = QtWidgets.QGridLayout(global_group)
        self.enable_all_button = QtWidgets.QPushButton("Enable All")
        self.disable_all_button = QtWidgets.QPushButton("Disable All")
        self.stop_all_button = QtWidgets.QPushButton("Stop All")
        self.clear_all_button = QtWidgets.QPushButton("Clear Faults")
        self.one_shot_status_button = QtWidgets.QPushButton("Request M102")
        self.text_status_button = QtWidgets.QPushButton("Request M101")
        self.enable_all_button.clicked.connect(lambda: self._enable_bridge(None, True))
        self.disable_all_button.clicked.connect(lambda: self._enable_bridge(None, False))
        self.stop_all_button.clicked.connect(lambda: self._stop_bridge(None))
        self.clear_all_button.clicked.connect(lambda: self._clear_faults(None))
        self.one_shot_status_button.clicked.connect(self._poll_now)
        self.text_status_button.clicked.connect(self._request_text_status)

        global_layout.addWidget(self.enable_all_button, 0, 0)
        global_layout.addWidget(self.disable_all_button, 0, 1)
        global_layout.addWidget(self.stop_all_button, 0, 2)
        global_layout.addWidget(self.clear_all_button, 0, 3)
        global_layout.addWidget(self.one_shot_status_button, 1, 0)
        global_layout.addWidget(self.text_status_button, 1, 1)

        layout.addWidget(self.status_panel)
        layout.addWidget(self.bridge_cards[0])
        layout.addWidget(self.bridge_cards[1])
        layout.addWidget(global_group)
        layout.addStretch(1)
        return panel

    def _scrollable(self, widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        area.setWidget(widget)
        return area

    def _connect_reports_controls(self) -> None:
        self.reports.pollNowRequested.connect(self._poll_now)
        self.reports.textStatusRequested.connect(self._request_text_status)
        self.reports.binaryReportRequested.connect(self._set_binary_report)
        self.reports.textReportRequested.connect(self._set_text_report)
        self.reports.manualErrorRequested.connect(self._set_manual_error)
        self.reports.pullupRequested.connect(self._set_pullup)
        self.reports.rawCommandRequested.connect(self._send_raw_command)
        self.reports.telemetryLoggingRequested.connect(self._set_telemetry_logging)
        self.reports.autoPollRequested.connect(self._set_auto_poll)

    def _connect_closed_loop_controls(self) -> None:
        self.closed_loop.query_status_button.clicked.connect(self._query_closed_loop_status)
        self.closed_loop.query_config_button.clicked.connect(self._query_closed_loop_encoder_config)
        self.closed_loop.reset_button.clicked.connect(self._reset_closed_loop_state)
        self.closed_loop.zero_position_button.clicked.connect(self._zero_closed_loop_position)
        self.closed_loop.apply_bridge_button.clicked.connect(self._apply_closed_loop_bridge)
        self.closed_loop.apply_enable_button.clicked.connect(self._apply_closed_loop_enabled)
        self.closed_loop.apply_mode_button.clicked.connect(self._apply_closed_loop_mode)
        self.closed_loop.apply_manual_button.clicked.connect(self._apply_closed_loop_manual)
        self.closed_loop.apply_speed_counts_button.clicked.connect(self._apply_closed_loop_speed_counts)
        self.closed_loop.apply_speed_rpm_button.clicked.connect(self._apply_closed_loop_speed_rpm)
        self.closed_loop.apply_position_counts_button.clicked.connect(self._apply_closed_loop_position_counts)
        self.closed_loop.apply_position_turns_button.clicked.connect(self._apply_closed_loop_position_turns)
        self.closed_loop.apply_position_degrees_button.clicked.connect(self._apply_closed_loop_position_degrees)
        self.closed_loop.apply_speed_gain_button.clicked.connect(self._apply_closed_loop_speed_gains)
        self.closed_loop.apply_position_gain_button.clicked.connect(self._apply_closed_loop_position_gains)
        self.closed_loop.apply_encoder_button.clicked.connect(self._apply_closed_loop_encoder_config)
        self.closed_loop.query_speed_gain_button.clicked.connect(self._query_closed_loop_speed_pid)
        self.closed_loop.query_position_gain_button.clicked.connect(self._query_closed_loop_position_pid)
        self.closed_loop.report_enabled.toggled.connect(self._set_closed_loop_fast_binary_mode)

    def _restore_defaults(self, initial_port: Optional[str], initial_baud: int) -> None:
        if initial_baud:
            index = self.baud_combo.findData(int(initial_baud))
            if index >= 0:
                self.baud_combo.setCurrentIndex(index)
        if initial_port:
            self._select_port_name(initial_port)

    def _refresh_ports(self) -> None:
        selected = self.port_combo.currentData()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        ports = SerialLink.available_ports()
        for port_name, label in ports:
            self.port_combo.addItem(label, port_name)
        if not ports:
            self.port_combo.addItem("<no serial ports found>", "")
        if selected:
            self._select_port_name(str(selected))
        self.port_combo.blockSignals(False)

    def _select_port_name(self, port_name: str) -> None:
        for index in range(self.port_combo.count()):
            if self.port_combo.itemData(index) == port_name:
                self.port_combo.setCurrentIndex(index)
                return

    def _apply_connected_state(self, connected: bool) -> None:
        self.reports.set_connected(connected)
        self.closed_loop.setEnabled(connected)
        for card in self.bridge_cards:
            for widget in (
                card.enable_button,
                card.disable_button,
                card.stop_button,
                card.drive_button,
                card.clear_fault_button,
                card.toggle_direction_button,
                card.self_check_button,
                card.power_slider,
                card.power_spin,
            ):
                widget.setEnabled(connected)
        for button in (
            self.enable_all_button,
            self.disable_all_button,
            self.stop_all_button,
            self.clear_all_button,
            self.one_shot_status_button,
            self.text_status_button,
        ):
            button.setEnabled(connected)
        if not connected:
            self.reports.set_manual_error_state(False)
            self.reports.set_pullups_state(False, False)
            self.closed_loop.set_sample(None)

    def _toggle_connection(self) -> None:
        if self.serial.is_connected():
            self.serial.disconnect_port()
            return

        port_name = str(self.port_combo.currentData() or "")
        baud_rate = int(self.baud_combo.currentData() or DEFAULT_BAUD)
        if not port_name:
            self.log.append("ERR", "No serial port selected")
            return

        ok, message = self.serial.connect_port(port_name, baud_rate)
        if ok:
            self.log.append("SYS", message)
            self.connect_button.setText("Disconnect")
            self.status_indicator.set_active(True, "CONNECTED", "DISCONNECTED")
            self.statusBar().showMessage(message)
            self._apply_connected_state(True)
            self._schedule_if_connected(250, self._request_text_status)
            self._schedule_if_connected(450, self._refresh_closed_loop_state)
        else:
            self.log.append("ERR", f"Unable to open {port_name}: {message}")
            self.statusBar().showMessage(message)

    def _on_connected_changed(self, connected: bool) -> None:
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.status_indicator.set_active(connected, "CONNECTED", "DISCONNECTED")
        self.statusBar().showMessage("Connected" if connected else "Disconnected")
        self._apply_connected_state(connected)
        if not connected:
            self.latest_sample = None
            self.latest_closed_loop_sample = None
            self._closed_loop_pending.clear()
            self._cancel_direction_self_check("Disconnected")
            self.history.clear()
            self.status_panel.set_sample(None)
            for card in self.bridge_cards:
                card.set_sample(None)
            self.scope.update_from_history([])
            self.closed_loop.set_sample(None)
            self.auto_poll_timer.stop()

    def _on_disconnected(self, message: str) -> None:
        self.log.append("ERR", message)

    def _clear_scope_history(self) -> None:
        self.history.clear()
        self.scope.update_from_history([])
        self.log.append("SYS", "Scope history cleared")

    def _send_command(self, command: str) -> bool:
        if not self.serial.is_connected():
            self.log.append("ERR", f"Not connected: {command}")
            return False
        try:
            self.serial.send_line(command)
        except Exception as exc:
            self.log.append("ERR", f"Failed to send {command}: {exc}")
            return False
        self.log.append("TX", command)
        return True

    def _request_text_status(self) -> None:
        self._send_command(self.commands.request_status_text())

    def _auto_poll_tick(self) -> None:
        self._request_text_status()

    def _poll_now(self) -> None:
        self._send_command(self.commands.request_status_binary())

    def _set_binary_report(self, enabled: bool, interval_ms: int, port_mask: int) -> None:
        if not self.serial.is_connected():
            return
        if enabled:
            if not port_mask:
                self.log.append("ERR", "Binary report enabled without a port selected")
                return
            self._send_command(self.commands.set_report(True, interval_ms, port_mask))
            self.auto_poll_timer.stop()
            self.reports.auto_poll.setChecked(False)
            self.log.append("SYS", f"Binary report enabled: {interval_ms} ms, mask {port_mask}")
        else:
            self._send_command(self.commands.set_report(True, 0, 0))
            self.log.append("SYS", "Binary report disabled")

    def _set_text_report(self, enabled: bool, interval_ms: int, port_mask: int) -> None:
        if not self.serial.is_connected():
            return
        if enabled:
            if not port_mask:
                self.log.append("ERR", "Text report enabled without a port selected")
                return
            self._send_command(self.commands.set_report(False, interval_ms, port_mask))
            self.log.append("SYS", f"Text report enabled: {interval_ms} ms, mask {port_mask}")
        else:
            self._send_command(self.commands.set_report(False, 0, 0))
            self.log.append("SYS", "Text report disabled")

    def _set_auto_poll(self, enabled: bool, interval_ms: int) -> None:
        if not self.serial.is_connected():
            return
        if enabled:
            self.auto_poll_timer.start(max(20, interval_ms))
            self.log.append("SYS", f"Local auto-poll enabled with M101: {interval_ms} ms")
        else:
            self.auto_poll_timer.stop()
            self.log.append("SYS", "Local auto-poll disabled")

    def _set_telemetry_logging(self, enabled: bool) -> None:
        self.log.append("SYS", f"Telemetry logging {'enabled' if enabled else 'disabled'}")

    def _set_manual_error(self, enabled: bool) -> None:
        self._send_command(self.commands.set_manual_error(enabled))
        self._schedule_if_connected(80, self._request_text_status)
        self.log.append("SYS", f"Manual error {'enabled' if enabled else 'cleared'}")

    def _set_pullup(self, bus_index: int, enabled: bool) -> None:
        self._send_command(self.commands.set_pullup(bus_index, enabled))
        self._schedule_if_connected(80, self._request_text_status)
        self.log.append("SYS", f"I2C{bus_index} pullups {'enabled' if enabled else 'disabled'}")

    def _send_raw_command(self, text: str) -> None:
        self._send_command(text)

    def _selected_closed_loop_bridge(self) -> int:
        return clamp_int(self.closed_loop.bridge.currentIndex(), 0, self.closed_loop.bridge.count() - 1)

    def _schedule_if_connected(self, delay_ms: int, callback: Callable[[], None]) -> None:
        def invoke() -> None:
            if self.serial.is_connected():
                callback()

        QtCore.QTimer.singleShot(delay_ms, invoke)

    def _enable_bridge(self, bridge_index: Optional[int], enable: bool) -> None:
        if self._send_command(self.commands.enable_bridge(bridge_index, enable)):
            self._schedule_if_connected(80, self._request_text_status)

    def _drive_bridge(self, bridge_index: int, power_permille: int) -> None:
        if self._send_command(self.commands.set_manual_drive(bridge_index, power_permille)):
            self._schedule_if_connected(80, self._request_text_status)

    def _stop_bridge(self, bridge_index: Optional[int]) -> None:
        if self._send_command(self.commands.stop(bridge_index)):
            self._schedule_if_connected(80, self._request_text_status)

    def _clear_faults(self, bridge_index: Optional[int]) -> None:
        if self._send_command(self.commands.clear_faults(bridge_index)):
            self._schedule_if_connected(80, self._request_text_status)

    def _toggle_bridge_direction(self, bridge_index: int) -> None:
        target_inverted = not self._bridge_direction_inverted[bridge_index]
        if self._send_command(self.commands.set_motor_direction(bridge_index, target_inverted)):
            self.log.append(
                "SYS",
                f"H{bridge_index} motor direction -> {'inverted' if target_inverted else 'normal'}",
            )
            self._schedule_if_connected(80, self._request_text_status)

    def _start_direction_self_check(self, bridge_index: int) -> None:
        if not self.serial.is_connected():
            self.log.append("ERR", f"H{bridge_index} self-check requires an active serial connection")
            return
        if self._direction_self_check is not None:
            self.log.append("ERR", "A direction self-check is already running")
            return

        session = DirectionSelfCheckSession(
            bridge_index=bridge_index,
            original_inverted=self._bridge_direction_inverted[bridge_index],
            current_inverted=self._bridge_direction_inverted[bridge_index],
        )
        session.poll_timer = QtCore.QTimer(self)
        session.poll_timer.timeout.connect(self._request_text_status)
        self._direction_self_check = session
        self.bridge_cards[bridge_index].set_self_check_running(True)
        self.log.append("SYS", f"Starting H{bridge_index} direction self-check")
        self._direction_self_check_prepare_run()

    def _direction_self_check_prepare_run(self) -> None:
        session = self._direction_self_check
        if session is None:
            return

        session.phase = "preparing"
        session.samples.clear()
        bridge_index = session.bridge_index
        self._send_command(self.commands.set_loop_mode(bridge_index, 0))
        self._schedule_if_connected(50, self._request_text_status)
        self._schedule_if_connected(110, self._direction_self_check_capture_baseline)
        self._schedule_if_connected(140, lambda: self._send_command(self.commands.enable_bridge(bridge_index, True)))
        self._schedule_if_connected(200, self._direction_self_check_begin_drive)

    def _direction_self_check_capture_baseline(self) -> None:
        session = self._direction_self_check
        if session is None:
            return

        if (
            self.latest_sample is not None
            and self.latest_sample.loop_bridge_index == session.bridge_index
            and self.latest_sample.encoder_healthy
        ):
            session.baseline_position_counts = self.latest_sample.measured_position_counts
        else:
            session.baseline_position_counts = 0

    def _direction_self_check_begin_drive(self) -> None:
        session = self._direction_self_check
        if session is None:
            return

        session.phase = "measuring"
        session.samples.clear()
        if session.poll_timer is not None:
            session.poll_timer.start(SELF_CHECK_POLL_MS)
        self._send_command(self.commands.set_manual_drive(session.bridge_index, SELF_CHECK_POWER_PERMILLE))
        self._schedule_if_connected(SELF_CHECK_DRIVE_MS, self._direction_self_check_finish_drive)

    def _direction_self_check_finish_drive(self) -> None:
        session = self._direction_self_check
        if session is None:
            return

        session.phase = "stopping"
        if session.poll_timer is not None:
            session.poll_timer.stop()
        self._send_command(self.commands.stop(session.bridge_index))
        self._schedule_if_connected(SELF_CHECK_SETTLE_MS, self._direction_self_check_evaluate)

    def _direction_self_check_evaluate(self) -> None:
        session = self._direction_self_check
        if session is None:
            return

        result, detail = self._evaluate_direction_self_check(session.baseline_position_counts, session.samples)
        bridge_index = session.bridge_index

        if result == "positive":
            message = (
                f"H{bridge_index} direction verified as positive "
                f"({'inverted' if session.current_inverted else 'normal'})"
            )
            self.log.append("SYS", message)
            self._direction_self_check_disable_bridge(final_message=message)
            return

        if result == "negative" and not session.toggled_during_check:
            session.toggled_during_check = True
            session.current_inverted = not session.current_inverted
            self.log.append(
                "SYS",
                f"H{bridge_index} moved negative under positive command, flipping direction and verifying again",
            )
            self._send_command(self.commands.set_motor_direction(bridge_index, session.current_inverted))
            self._schedule_if_connected(80, self._request_text_status)
            self._schedule_if_connected(140, self._direction_self_check_prepare_run)
            return

        if session.toggled_during_check and session.current_inverted != session.original_inverted:
            self._send_command(self.commands.set_motor_direction(bridge_index, session.original_inverted))
            self._schedule_if_connected(80, self._request_text_status)

        self._direction_self_check_disable_bridge(final_error=f"H{bridge_index} self-check failed: {detail}")

    def _direction_self_check_disable_bridge(
        self,
        *,
        final_message: Optional[str] = None,
        final_error: Optional[str] = None,
    ) -> None:
        session = self._direction_self_check
        if session is None:
            return

        session.phase = "cleanup"
        self._send_command(self.commands.enable_bridge(session.bridge_index, False))
        self._schedule_if_connected(
            100,
            lambda: self._finish_direction_self_check(final_message=final_message, final_error=final_error),
        )

    def _finish_direction_self_check(
        self,
        *,
        final_message: Optional[str] = None,
        final_error: Optional[str] = None,
    ) -> None:
        session = self._direction_self_check
        if session is None:
            return

        bridge_index = session.bridge_index
        if session.poll_timer is not None:
            session.poll_timer.stop()
            session.poll_timer.deleteLater()
        self.bridge_cards[bridge_index].set_self_check_running(False)
        self._direction_self_check = None
        self._schedule_if_connected(50, self._request_text_status)
        if final_error is not None:
            self.log.append("ERR", final_error)
        elif final_message is not None:
            self.log.append("SYS", final_message)

    def _cancel_direction_self_check(self, reason: str) -> None:
        session = self._direction_self_check
        if session is None:
            return

        if session.poll_timer is not None:
            session.poll_timer.stop()
            session.poll_timer.deleteLater()
        self.bridge_cards[session.bridge_index].set_self_check_running(False)
        if not self.serial.is_connected():
            self.bridge_cards[session.bridge_index].toggle_direction_button.setEnabled(False)
            self.bridge_cards[session.bridge_index].self_check_button.setEnabled(False)
        self._direction_self_check = None
        self.log.append("ERR", f"Direction self-check cancelled: {reason}")

    @staticmethod
    def _evaluate_direction_self_check(
        baseline_position_counts: int,
        samples: list[TelemetrySample],
    ) -> tuple[str, str]:
        if not samples:
            return ("invalid", "no encoder samples were received")

        final_sample = samples[-1]
        final_position = final_sample.measured_position_counts - baseline_position_counts
        recent_samples = samples[-3:] if len(samples) >= 3 else samples
        average_speed = sum(sample.measured_speed_counts_per_sec for sample in recent_samples) / len(recent_samples)

        if abs(final_position) < SELF_CHECK_MIN_POSITION_COUNTS:
            return ("invalid", f"position change too small ({final_position} counts)")
        if abs(average_speed) < SELF_CHECK_MIN_SPEED_COUNTS_PER_SEC:
            return ("invalid", f"speed magnitude too small ({average_speed:.1f} counts/s)")

        position_sign = 1 if final_position > 0 else -1
        speed_sign = 1 if average_speed > 0 else -1
        if position_sign != speed_sign:
            return (
                "invalid",
                f"position and speed disagree (pos={final_position}, speed={average_speed:.1f})",
            )

        if position_sign > 0:
            return ("positive", f"positive motion confirmed (pos={final_position}, speed={average_speed:.1f})")
        return ("negative", f"negative motion detected (pos={final_position}, speed={average_speed:.1f})")

    def _query_closed_loop_status(self) -> None:
        self._request_text_status()

    def _query_closed_loop_encoder_config(self) -> None:
        self._send_closed_loop_command(
            "encoder",
            self.commands.set_encoder_config(None, None, None, None),
            refresh_binary=False,
        )

    def _reset_closed_loop_state(self) -> None:
        self._send_closed_loop_command("reset", self.commands.reset_loop(self._selected_closed_loop_bridge()))

    def _zero_closed_loop_position(self) -> None:
        self._send_closed_loop_command(
            "zero_position",
            self.commands.zero_current_position(self._selected_closed_loop_bridge()),
        )

    def _query_closed_loop_speed_pid(self) -> None:
        self._request_text_status()

    def _query_closed_loop_position_pid(self) -> None:
        self._request_text_status()

    def _query_closed_loop_fast_binary_mode(self) -> None:
        self._send_closed_loop_command("fast_binary", self.commands.set_fast_binary_mode(None), refresh_binary=False)

    def _apply_closed_loop_bridge(self) -> None:
        self._send_closed_loop_command(
            "encoder",
            self.commands.set_encoder_config(self._selected_closed_loop_bridge(), None, None, None),
        )

    def _apply_closed_loop_enabled(self) -> None:
        self._send_closed_loop_command(
            "enabled",
            self.commands.enable_bridge(self._selected_closed_loop_bridge(), self.closed_loop.enabled.isChecked()),
        )

    def _apply_closed_loop_mode(self) -> None:
        self._send_closed_loop_command(
            "mode",
            self.commands.set_loop_mode(self._selected_closed_loop_bridge(), self.closed_loop.mode.currentIndex()),
        )

    def _apply_closed_loop_manual(self) -> None:
        self._send_closed_loop_command(
            "manual",
            self.commands.set_manual_drive(self._selected_closed_loop_bridge(), self.closed_loop.manual_command.value()),
        )

    def _apply_closed_loop_speed_counts(self) -> None:
        self._send_closed_loop_command(
            "speed_counts",
            self.commands.set_speed_reference_counts(self._selected_closed_loop_bridge(), self.closed_loop.speed_counts.value()),
        )

    def _apply_closed_loop_speed_rpm(self) -> None:
        self._send_closed_loop_command(
            "speed_rpm",
            self.commands.set_speed_reference_rpm(self._selected_closed_loop_bridge(), self.closed_loop.speed_rpm.value()),
        )

    def _apply_closed_loop_position_counts(self) -> None:
        self._send_closed_loop_command(
            "position_counts",
            self.commands.set_position_reference_counts(
                self._selected_closed_loop_bridge(), self.closed_loop.position_counts.value()
            ),
        )

    def _apply_closed_loop_position_turns(self) -> None:
        self._send_closed_loop_command(
            "position_turns",
            self.commands.set_position_reference_turns(
                self._selected_closed_loop_bridge(), self.closed_loop.position_turns.value()
            ),
        )

    def _apply_closed_loop_position_degrees(self) -> None:
        self._send_closed_loop_command(
            "position_degrees",
            self.commands.set_position_reference_degrees(
                self._selected_closed_loop_bridge(), self.closed_loop.position_degrees.value()
            ),
        )

    def _apply_closed_loop_speed_gains(self) -> None:
        self._send_closed_loop_command(
            "speed_pid",
            self.commands.set_pid_gains(
                self._selected_closed_loop_bridge(),
                0,
                self.closed_loop.speed_kp.value(),
                self.closed_loop.speed_ki.value(),
                self.closed_loop.speed_kd.value(),
            ),
        )

    def _apply_closed_loop_position_gains(self) -> None:
        self._send_closed_loop_command(
            "position_pid",
            self.commands.set_pid_gains(
                self._selected_closed_loop_bridge(),
                1,
                self.closed_loop.position_kp.value(),
                self.closed_loop.position_ki.value(),
                self.closed_loop.position_kd.value(),
            ),
        )

    def _apply_closed_loop_encoder_config(self) -> None:
        self._send_closed_loop_command(
            "encoder",
            self.commands.set_encoder_config(
                self._selected_closed_loop_bridge(),
                self.closed_loop.encoder_resolution.value(),
                self.closed_loop.encoder_zero_offset.value(),
                self.closed_loop.encoder_inverted.isChecked(),
            ),
        )

    def _set_closed_loop_fast_binary_mode(self, *_args: object) -> None:
        self._send_closed_loop_command(
            "fast_binary",
            self.commands.set_fast_binary_mode(self.closed_loop.report_enabled.isChecked()),
            refresh_binary=False,
            refresh_text=True,
        )

    def _refresh_closed_loop_state(self) -> None:
        if not self.serial.is_connected():
            return
        self._request_text_status()
        self._send_closed_loop_command(
            "fast_binary",
            self.commands.set_fast_binary_mode(None),
            refresh_binary=False,
            refresh_text=False,
        )

    def _send_closed_loop_command(
        self,
        kind: str,
        command: str,
        *,
        refresh_binary: bool = False,
        refresh_text: bool = True,
    ) -> bool:
        if self._send_command(command):
            self._closed_loop_pending.append(kind)
            if refresh_binary:
                self._schedule_if_connected(40, self._poll_now)
            if refresh_text:
                self._schedule_if_connected(80, self._request_text_status)
            return True
        return False

    def _on_text_line(self, line: str) -> None:
        sample = parse_status_text(line)
        if sample is not None:
            self._consume_sample(sample)
            return

        if line.startswith("ok"):
            kind = self._closed_loop_pending.popleft() if self._closed_loop_pending else ""
            if kind:
                fields = parse_key_value_fields(line)
                self.closed_loop.apply_ack(kind, fields)
            self.log.append("OK", line)
        elif line.startswith("error"):
            if self._closed_loop_pending:
                self._closed_loop_pending.popleft()
            self.log.append("ERR", line)
        else:
            self.log.append("RX", line)

    def _on_telemetry(self, sample: TelemetrySample) -> None:
        self._consume_sample(sample)

    def _consume_sample(self, sample: TelemetrySample) -> None:
        self.latest_sample = sample
        self._bridge_direction_inverted[0] = sample.bridge_direction_inverted[0]
        self._bridge_direction_inverted[1] = sample.bridge_direction_inverted[1]
        self.latest_closed_loop_sample = ClosedLoopSample.from_telemetry(sample)
        self.history.add(sample)
        history = self.history.as_list()
        self.status_panel.set_sample(sample)
        for card in self.bridge_cards:
            card.set_sample(sample)
        self.scope.update_from_history(history)
        self.closed_loop.set_sample(self.latest_closed_loop_sample)

        session = self._direction_self_check
        if session is not None:
            session.current_inverted = sample.bridge_direction_inverted[session.bridge_index]
            if (
                session.phase == "measuring"
                and sample.loop_bridge_index == session.bridge_index
                and sample.encoder_healthy
            ):
                session.samples.append(sample)

        if self.reports.log_telemetry.isChecked():
            now_ms = int(time.monotonic() * 1000)
            if now_ms - self.last_telemetry_log_ms > 1000:
                self.log.append("TEL", sample.summary())
                self.last_telemetry_log_ms = now_ms

    def _consume_closed_loop_sample(self, sample: ClosedLoopSample) -> None:
        self.latest_closed_loop_sample = sample
        self.closed_loop.set_sample(sample)
        if self.reports.log_telemetry.isChecked():
            now_ms = int(time.monotonic() * 1000)
            if now_ms - self.last_telemetry_log_ms > 1000:
                self.log.append("TEL", sample.raw_line)
                self.last_telemetry_log_ms = now_ms

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # type: ignore[override]
        if self.serial.is_connected():
            self.serial.disconnect_port()
        super().closeEvent(event)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pico dual motor driver GUI")
    parser.add_argument("--port", help="Serial port name, for example COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--window", type=float, default=WINDOW_SECONDS, help="Scope window in seconds")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    app = QtWidgets.QApplication([])
    apply_dark_theme(app)
    window = MainWindow(initial_port=args.port, initial_baud=args.baud, window_seconds=args.window)
    window.show()
    if args.port:
        QtCore.QTimer.singleShot(0, window._toggle_connection)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
