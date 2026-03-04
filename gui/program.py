#!/usr/bin/env python3
"""Inverted Pendulum GUI — real-time telemetry and controller configuration."""

import csv
import struct
import sys
from collections import deque

# QApplication must exist before pyqtgraph (or any QWidget) is imported
from PyQt5.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QVBoxLayout, QWidget,
)
import serial

# ================= CONFIG =================
PORT     = "/dev/ttyACM0"
BAUD     = 115200
MAX_PLOT = 10000  # samples kept in plot buffers (10 s at 1 kHz)
PLOT_WIN = 10.0   # seconds of data shown in the plot

# ---- Packet layout (must match firmware struct Sample) ----
# header(2) t_us(4) angle_deg(4) pend_vel_deg_s(4)
# cart_mm(4) cart_vel_mm_s(4) accel_mm_s2(4) feedback(1)  = 27 bytes
PACKET_FMT  = "<H I f f f f f B"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

CART_MIN_MM = -200.0
CART_MAX_MM =  200.0


# ================= SERIAL =================
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.001)
except serial.SerialException as e:
    print(f"[WARNING] Could not open {PORT}: {e}", file=sys.stderr)
    ser = None


def send(cmd: str):
    if ser and ser.is_open:
        ser.write((cmd + "\n").encode())


# ================= DATA BUFFERS =================
t_buf        = deque(maxlen=MAX_PLOT)
angle_buf    = deque(maxlen=MAX_PLOT)
pend_vel_buf = deque(maxlen=MAX_PLOT)
cart_buf     = deque(maxlen=MAX_PLOT)
cart_vel_buf = deque(maxlen=MAX_PLOT)
accel_buf    = deque(maxlen=MAX_PLOT)
fb_buf       = deque(maxlen=MAX_PLOT)

log_rows: list = []   # rows saved on "Save CSV"


# ================= PACKET READER =================
def read_packets() -> list:
    """Drain serial buffer and return list of decoded packets."""
    pkts = []
    if not ser or not ser.is_open:
        return pkts
    while ser.in_waiting >= PACKET_SIZE:
        b = ser.read(1)
        if not b or b[0] != 0x55:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != 0xAA:
            continue
        rest = ser.read(PACKET_SIZE - 2)
        if len(rest) != PACKET_SIZE - 2:
            continue
        try:
            pkts.append(struct.unpack(PACKET_FMT, b + b2 + rest))
        except struct.error:
            pass
    return pkts


# ================= MAIN WINDOW =================
class MainWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum Control")
        self._build_ui()
        self._connect_signals()

        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(20)   # 50 Hz UI refresh

    HOMING_VEL    = 50.0   # mm/s used by "Go to Zero"
    HOMING_TOL_MM = 2.0    # stop when |cart| < this

    # ------------------------------------------------------------------
    def _build_ui(self):
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self._angle_offset = 0.0   # 0 = downward reference, 180 = upright reference

        root = QHBoxLayout(self)

        # ---- LEFT: stacked plots ----
        plots_col = QVBoxLayout()
        root.addLayout(plots_col, stretch=3)

        # Angle plot header row (title + centre selector)
        angle_hdr = QHBoxLayout()
        angle_hdr.addWidget(QLabel("<b>Pendulum Angle [deg]</b>"))
        angle_hdr.addStretch()
        angle_hdr.addWidget(QLabel("Center on:"))
        self.angle_center_combo = QComboBox()
        self.angle_center_combo.addItems(["0° (downward)", "180° (upright)"])
        angle_hdr.addWidget(self.angle_center_combo)
        plots_col.addLayout(angle_hdr)

        self.angle_plot = pg.PlotWidget()
        self.angle_plot.setYRange(-20, 20)
        self.angle_plot.showGrid(x=True, y=True, alpha=0.3)
        self.angle_plot.setLabel("left", "Angle [deg]")
        self.angle_plot.setLabel("bottom", "Time [s]")
        self.angle_curve = self.angle_plot.plot(pen=pg.mkPen("b", width=1.5))
        self._angle_zeroline = self.angle_plot.addLine(y=0, pen=pg.mkPen("k", style=Qt.DashLine))

        self.cart_plot = pg.PlotWidget(title="Cart Position [mm]")
        self.cart_plot.setYRange(CART_MIN_MM, CART_MAX_MM)
        self.cart_plot.showGrid(x=True, y=True, alpha=0.3)
        self.cart_plot.setLabel("left", "Position [mm]")
        self.cart_plot.setLabel("bottom", "Time [s]")
        self.cart_curve = self.cart_plot.plot(pen=pg.mkPen("r", width=1.5))
        for lim in [CART_MIN_MM, CART_MAX_MM]:
            self.cart_plot.addLine(y=lim, pen=pg.mkPen("gray", style=Qt.DashLine, width=0.8))

        plots_col.addWidget(self.angle_plot)
        plots_col.addWidget(self.cart_plot)

        # ---- RIGHT: control panels ----
        ctrl_col = QVBoxLayout()
        ctrl_col.setSpacing(8)
        root.addLayout(ctrl_col, stretch=1)

        ctrl_col.addWidget(self._make_controller_group())
        ctrl_col.addWidget(self._make_setpoints_group())
        ctrl_col.addWidget(self._make_motor_group())
        ctrl_col.addWidget(self._make_data_group())
        ctrl_col.addStretch()

    # ------------------------------------------------------------------
    def _make_controller_group(self) -> QGroupBox:
        grp = QGroupBox("Controller")
        lay = QVBoxLayout(grp)

        # Mode selector
        row = QHBoxLayout()
        row.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["P", "PD", "LQR"])
        self.mode_combo.setCurrentIndex(1)
        row.addWidget(self.mode_combo)
        lay.addLayout(row)

        # P / PD gains
        form = QFormLayout()
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setRange(-10000, 10000)
        self.spin_kp.setDecimals(3)
        self.spin_kp.setSingleStep(1.0)
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setRange(-10000, 10000)
        self.spin_kd.setDecimals(3)
        self.spin_kd.setSingleStep(0.1)
        form.addRow("Kp:", self.spin_kp)
        form.addRow("Kd:", self.spin_kd)
        lay.addLayout(form)

        # LQR gains
        lay.addWidget(QLabel("LQR  K = [x, ẋ, φ, φ̇]:"))
        lqr_grid = QFormLayout()
        labels = ["K[x]", "K[ẋ]", "K[φ]", "K[φ̇]"]
        self.spin_k = []
        for lbl in labels:
            sp = QDoubleSpinBox()
            sp.setRange(-1e6, 1e6)
            sp.setDecimals(4)
            sp.setSingleStep(1.0)
            lqr_grid.addRow(lbl + ":", sp)
            self.spin_k.append(sp)
        lay.addLayout(lqr_grid)

        self.btn_apply_gains = QPushButton("Apply Gains")
        lay.addWidget(self.btn_apply_gains)

        return grp

    # ------------------------------------------------------------------
    def _make_setpoints_group(self) -> QGroupBox:
        grp = QGroupBox("Setpoints")
        form = QFormLayout(grp)

        self.spin_theta_ref = QDoubleSpinBox()
        self.spin_theta_ref.setRange(-360, 360)
        self.spin_theta_ref.setDecimals(1)
        self.spin_theta_ref.setSuffix(" deg")

        self.spin_x_ref = QDoubleSpinBox()
        self.spin_x_ref.setRange(CART_MIN_MM, CART_MAX_MM)
        self.spin_x_ref.setDecimals(1)
        self.spin_x_ref.setSuffix(" mm")

        form.addRow("θ_ref:", self.spin_theta_ref)
        form.addRow("x_ref:", self.spin_x_ref)

        self.btn_apply_sp = QPushButton("Apply Setpoints")
        form.addRow(self.btn_apply_sp)

        return grp

    # ------------------------------------------------------------------
    def _make_motor_group(self) -> QGroupBox:
        grp = QGroupBox("Motor Control")
        lay = QVBoxLayout(grp)

        row1 = QHBoxLayout()
        self.btn_motor = QPushButton()
        self.btn_motor.setProperty("stateName", "Motor")
        self.btn_fb    = QPushButton()
        self.btn_fb.setProperty("stateName", "Feedback")
        self._set_btn_state(self.btn_motor, False)
        self._set_btn_state(self.btn_fb,    False)
        row1.addWidget(self.btn_motor)
        row1.addWidget(self.btn_fb)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        self.btn_zero_cart = QPushButton("Zero Cart")
        self.btn_zero_enc  = QPushButton("Zero Pendulum")
        row2.addWidget(self.btn_zero_cart)
        row2.addWidget(self.btn_zero_enc)
        lay.addLayout(row2)

        self.btn_go_zero = QPushButton("Go to Zero")
        lay.addWidget(self.btn_go_zero)

        lay.addWidget(QLabel("Open-loop velocity [mm/s]:"))
        slider_row = QHBoxLayout()
        self.vel_slider = QSlider(Qt.Horizontal)
        self.vel_slider.setRange(-300, 300)
        self.vel_slider.setValue(0)
        self.vel_label = QLabel("  0 mm/s")
        self.vel_label.setMinimumWidth(60)
        slider_row.addWidget(self.vel_slider)
        slider_row.addWidget(self.vel_label)
        lay.addLayout(slider_row)

        return grp

    # ------------------------------------------------------------------
    def _make_data_group(self) -> QGroupBox:
        grp = QGroupBox("Data")
        lay = QVBoxLayout(grp)
        self.btn_save = QPushButton("Save CSV")
        lay.addWidget(self.btn_save)
        return grp

    # ------------------------------------------------------------------
    def _connect_signals(self):
        self.btn_apply_gains.clicked.connect(self._apply_gains)
        self.btn_apply_sp.clicked.connect(self._apply_setpoints)
        self.btn_motor.clicked.connect(self._toggle_motor)
        self.btn_fb.clicked.connect(self._toggle_fb)
        self.btn_zero_cart.clicked.connect(lambda: send("ZERO CART"))
        self.btn_zero_enc.clicked.connect(lambda: send("ZERO ENC"))
        self.btn_go_zero.clicked.connect(self._start_homing)
        self.vel_slider.valueChanged.connect(self._on_vel_slider)
        self.vel_slider.sliderReleased.connect(self._on_vel_released)
        self.btn_save.clicked.connect(self._save_csv)
        self.angle_center_combo.currentIndexChanged.connect(self._on_angle_center_changed)

        # Homing timer (fires at 20 Hz while active)
        self._homing_timer = QTimer()
        self._homing_timer.setInterval(50)
        self._homing_timer.timeout.connect(self._homing_step)

    # ------------------------------------------------------------------
    def _apply_gains(self):
        mode = self.mode_combo.currentText()
        send(f"MODE {mode}")
        send(f"SET KP {self.spin_kp.value():.4f}")
        send(f"SET KD {self.spin_kd.value():.4f}")
        k = [sp.value() for sp in self.spin_k]
        send(f"SET K {k[0]:.4f} {k[1]:.4f} {k[2]:.4f} {k[3]:.4f}")

    def _apply_setpoints(self):
        send(f"SET THETA {self.spin_theta_ref.value():.2f}")
        send(f"SET X {self.spin_x_ref.value():.2f}")

    @staticmethod
    def _set_btn_state(btn: QPushButton, on: bool):
        """Update a Motor/Feedback toggle button to reflect current state."""
        name = btn.property("stateName") or ""
        if on:
            btn.setText(f"{name} ON")
            btn.setStyleSheet("font-weight: bold; color: white;"
                              "background-color: #2a7a2a;")
        else:
            btn.setText(f"{name} OFF")
            btn.setStyleSheet("font-weight: bold; color: white;"
                              "background-color: #aa2222;")

    def _toggle_motor(self):
        on = "OFF" in self.btn_motor.text()   # currently OFF → turn ON
        send("MOTOR ON" if on else "MOTOR OFF")
        self._set_btn_state(self.btn_motor, on)

    def _toggle_fb(self):
        on = "OFF" in self.btn_fb.text()
        send("FB ON" if on else "FB OFF")
        self._set_btn_state(self.btn_fb, on)

    def _on_vel_slider(self, v: int):
        self.vel_label.setText(f"{v:4d} mm/s")
        send(f"VEL {v}")

    def _on_vel_released(self):
        self.vel_slider.setValue(0)
        send("VEL 0")

    def _start_homing(self):
        if not cart_buf:
            return
        self._homing_timer.start()
        self.btn_go_zero.setText("Homing…")
        self.btn_go_zero.setEnabled(False)

    def _homing_step(self):
        if not cart_buf:
            self._stop_homing()
            return
        pos = cart_buf[-1]
        if abs(pos) < self.HOMING_TOL_MM:
            self._stop_homing()
        elif pos > 0:
            send(f"VEL -{self.HOMING_VEL:.0f}")
        else:
            send(f"VEL {self.HOMING_VEL:.0f}")

    def _stop_homing(self):
        self._homing_timer.stop()
        send("VEL 0")
        self.btn_go_zero.setText("Go to Zero")
        self.btn_go_zero.setEnabled(True)
        # Homing runs open-loop; ensure feedback button shows OFF
        self._set_btn_state(self.btn_fb, False)

    def _on_angle_center_changed(self, index: int):
        self._angle_offset = 180.0 if index == 1 else 0.0

    def _save_csv(self):
        fname, _ = QFileDialog.getSaveFileName(
            self, "Save Data", "data.csv", "CSV files (*.csv)")
        if not fname:
            return
        columns = ["t_us", "t_s", "angle_deg", "pend_vel_deg_s",
                   "cart_mm", "cart_vel_mm_s", "accel_mm_s2", "feedback_on"]
        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(columns)
            w.writerows(log_rows)
        print(f"Saved {len(log_rows)} rows → {fname}")

    # ------------------------------------------------------------------
    def _update(self):
        pkts = read_packets()
        for pkt in pkts:
            _, t_us, pend_deg, pend_vel, cart_mm, cart_vel, accel, fb = pkt
            t_s = t_us / 1e6
            t_buf.append(t_s)
            angle_buf.append(pend_deg)
            pend_vel_buf.append(pend_vel)
            cart_buf.append(cart_mm)
            cart_vel_buf.append(cart_vel)
            accel_buf.append(accel)
            fb_buf.append(fb)
            log_rows.append([t_us, t_s, pend_deg, pend_vel,
                              cart_mm, cart_vel, accel, int(fb)])

        if not t_buf:
            return

        t = list(t_buf)
        t_lo = max(t[-1] - PLOT_WIN, t[0])
        t_hi = t[-1]

        offset = self._angle_offset
        angle_display = [a - offset for a in angle_buf]
        self.angle_curve.setData(t, angle_display)
        self.cart_curve.setData(t, list(cart_buf))
        self.angle_plot.setXRange(t_lo, t_hi, padding=0)
        self.cart_plot.setXRange(t_lo, t_hi, padding=0)


# ================= ENTRY POINT =================
if __name__ == "__main__":
    win = MainWindow()
    win.resize(1200, 700)
    win.show()
    sys.exit(_app.exec_())
