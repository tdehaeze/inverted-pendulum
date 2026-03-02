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
PyQt5 app connecting to Teensy via USB serial (`/dev/ttyACM0`, 115200 baud). Receives packed binary telemetry structs (18 bytes: header, timestamp, angles, positions, feedback state) and displays real-time plots. Sends ASCII commands: `MOTOR ON/OFF`, `FB ON/OFF`, `ZERO CART`, `ZERO ENC`, `VEL <value>`.

### teensy/src/main.cpp
1 kHz control loop via `IntervalTimer`. Hardware:
- **Stepper motor**: DRV8825 driver, pins 10 (STEP) / 11 (DIR), 32 microsteps, 5 µm/microstep
- **Encoder**: Quadrature, pins 2 & 3, 4096 steps/rev → 0.088°/step
- **Safety limits**: cart travel ±200 mm

Telemetry is buffered in a 512-sample ring buffer and flushed over serial between control cycles. Closed-loop control is not yet implemented in firmware — the linearized state-space model exists in the Python simulation for algorithm development before porting to C++.

## Dependencies

- **model**: Python 3.14, numpy, matplotlib, scipy, control
- **gui**: Python 3.13, PyQt5, pyqtgraph, pyserial, numpy, colorama
- **teensy**: PlatformIO, Arduino framework, Encoder library (Paul Stoffregen), IntervalTimer
