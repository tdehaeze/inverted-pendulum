# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

An inverted pendulum stabilization system with three components:
- **model/**: Python control simulation and algorithm design
- **gui/**: PyQt5 real-time visualization and hardware control interface
- **teensy/**: Embedded firmware for Teensy 4.1 microcontroller

## Commands

### Teensy Firmware (PlatformIO)
```bash
cd teensy/
pio run -e teensy41           # Build
pio run -e teensy41 -t upload # Flash to device
pio device monitor            # Serial monitor (115200 baud)
```

### Model Simulation
```bash
source model/.venv/bin/activate
python model/inverted_pendulum.py
```

### GUI Application
```bash
source gui/.venv/bin/activate
python gui/program.py
```

## Architecture

### model/inverted_pendulum.py
Physics simulation and control design. Key classes:
- `PendulumParameters`: system parameters (mass, length, gravity, damping)
- `InvertedPendulumPlant`: nonlinear dynamics with linearization around the upright position
- `Simulator`: time-step integration for closed-loop simulation
- `PIDController` / `LQRController`: two control strategies (LQR is optimal, requires scipy/control)

### gui/program.py
PyQt5 app connecting to Teensy via USB serial (`/dev/ttyACM0`, 115200 baud). Receives 27-byte packed telemetry packets and displays real-time plots. Panels: Controller (mode P/PD/LQR, Kp/Kd, LQR K vector), Setpoints (θ_ref, x_ref), Motor Control (motor/feedback toggles, zero buttons, open-loop velocity slider), Data (save CSV).

**Serial commands sent to Teensy:**
- `MOTOR ON/OFF`, `FB ON/OFF`, `ZERO CART`, `ZERO ENC`
- `VEL <mm/s>` — open-loop velocity
- `MODE P|PD|LQR` — controller mode
- `SET KP <val>`, `SET KD <val>` — PD gains
- `SET K <k0> <k1> <k2> <k3>` — LQR gains [x, ẋ, φ, φ̇] in SI units
- `SET THETA <deg>`, `SET X <mm>` — setpoints

**Telemetry packet** (`<H I f f f f f B`, 27 bytes): header(0xAA55), t_us, angle_deg, pend_vel_deg_s, cart_mm, cart_vel_mm_s, accel_cmd_mm_s2, feedback_on.

### teensy/src/main.cpp
1 kHz control loop via `IntervalTimer`. Hardware:
- **Stepper motor**: DRV8825 driver, pins 10 (STEP) / 11 (DIR), 32 microsteps, 5 µm/microstep
- **Encoder**: Quadrature, pins 2 & 3, 4096 steps/rev → 0.088°/step
- **Safety**: hard stop at ±200 mm, soft limit at ±160 mm (blocks accel toward boundary)

Plant input is cart **acceleration** (m/s²), integrated to velocity in the ISR. Controllers: P (`u = -Kp·φ`), PD (`u = -Kp·φ - Kd·φ̇`), LQR (`u = -K·[x, ẋ, φ, φ̇]`). All gains are in SI units. Pendulum angular velocity estimated by finite difference of encoder reading at 1 kHz. Telemetry buffered in 512-sample ring buffer.

## Dependencies

- **model**: Python 3.14, numpy, matplotlib, scipy, control
- **gui**: Python 3.13, PyQt5, pyqtgraph, pyserial, numpy, colorama
- **teensy**: PlatformIO, Arduino framework, Encoder library (Paul Stoffregen), IntervalTimer
