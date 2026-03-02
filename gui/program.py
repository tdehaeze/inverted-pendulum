import serial, struct, time
from collections import deque
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ================= CONFIG =================
PORT = "/dev/ttyACM0"
BAUD = 115200

PACKET_FMT  = "<H I f f f B"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

MAX_SAMPLES = 10000  # 10 seconds @1kHz

CART_MIN_MM = -200
CART_MAX_MM = 200

# ================= SERIAL =================
ser = serial.Serial(PORT, BAUD, timeout=0.001)

def send(cmd):
    ser.write((cmd + "\n").encode())

# ================= DATA =================
t_buf = deque(maxlen=MAX_SAMPLES)
angle_buf = deque(maxlen=MAX_SAMPLES)
cart_buf = deque(maxlen=MAX_SAMPLES)
vel_buf = deque(maxlen=MAX_SAMPLES)
fb_buf = deque(maxlen=MAX_SAMPLES)
log = []

# ================= GUI =================
app = QApplication([])
win = QWidget()
layout = QVBoxLayout(win)

# ---- Plots ----
anglePlot = pg.PlotWidget(title="Pendulum Angle [deg]")
anglePlot.setYRange(-180, 180)
angleCurve = anglePlot.plot()

cartPlot = pg.PlotWidget(title="Cart Position [mm]")
cartPlot.setYRange(CART_MIN_MM, CART_MAX_MM)
cartCurve = cartPlot.plot()

layout.addWidget(anglePlot)
layout.addWidget(cartPlot)

# ---- Controls ----
ctrl = QHBoxLayout()

slider = QSlider()
slider.setOrientation(Qt.Horizontal)
slider.setRange(-100, 100)
slider.sliderReleased.connect(lambda: send("VEL 0"))
slider.valueChanged.connect(lambda v: send(f"VEL {v}"))

btn_motor = QPushButton("Motor ON")
btn_fb = QPushButton("Feedback OFF")

def toggle_motor():
    if btn_motor.text() == "Motor ON":
        send("MOTOR ON")
        btn_motor.setText("Motor OFF")
    else:
        send("MOTOR OFF")
        btn_motor.setText("Motor ON")

def toggle_fb():
    if btn_fb.text() == "Feedback OFF":
        send("FB ON")
        btn_fb.setText("Feedback ON")
    else:
        send("FB OFF")
        btn_fb.setText("Feedback OFF")

btn_motor.clicked.connect(toggle_motor)
btn_fb.clicked.connect(toggle_fb)

btn_zero_cart = QPushButton("Zero Cart")
btn_zero_enc = QPushButton("Zero Pendulum")
btn_zero_cart.clicked.connect(lambda: send("ZERO CART"))
btn_zero_enc.clicked.connect(lambda: send("ZERO ENC"))

btn_save = QPushButton("Save Data")

def saveData():
    fname, _ = QFileDialog.getSaveFileName(win, "Save Data", "", "*.txt")
    if not fname: return
    with open(fname, "w") as f:
        f.write("t_ms angle_deg cart_mm vel feedback\n")
        for r in log:
            f.write(r + "\n")

btn_save.clicked.connect(saveData)

ctrl.addWidget(slider)
ctrl.addWidget(btn_motor)
ctrl.addWidget(btn_fb)
ctrl.addWidget(btn_zero_cart)
ctrl.addWidget(btn_zero_enc)
ctrl.addWidget(btn_save)

layout.addLayout(ctrl)

def read_packet():
    while True:
        # Search for header
        b = ser.read(2)
        if len(b) < 2:
            return None
        if b == b'\x55\xAA':   # little-endian 0xAA55
            payload = ser.read(PACKET_SIZE - 2)
            if len(payload) != PACKET_SIZE - 2:
                return None
            return struct.unpack(PACKET_FMT, b + payload)

# ================= UPDATE =================
def update():
    while ser.in_waiting >= PACKET_SIZE:
        pkt = read_packet()
        if pkt is None:
            return

        _, t_us, pend_deg, cart_mm, cart_vel, fb = pkt

        t_buf.append(t_us)
        angle_buf.append(pend_deg)
        cart_buf.append(cart_mm)
        vel_buf.append(cart_vel)
        fb_buf.append(fb)

        log.append(f"{t_us} {pend_deg:.3f} {cart_mm:.3f} {cart_vel:.3f} {fb}")

    angleCurve.setData(angle_buf)
    cartCurve.setData(cart_buf)

timer = QTimer()
timer.timeout.connect(update)
timer.start(20)

# ================= RUN =================
win.show()
app.exec_()
