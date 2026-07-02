# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`lifeOs` is a two-part hardware project: an ESP32 firmware that reads an MPU6050
IMU and drives servos, and a PySide6 desktop GUI that visualizes the orientation,
plots the data, and commands the servos. The link between them is **two-way** and
the transport is **swappable** (WiFi/UDP or Bluetooth).

- `lifeOs.ino` — ESP32 firmware. Reads the MPU6050 via its on-chip **DMP**
  (fused orientation quaternion) on an interrupt-driven FreeRTOS task, drives up
  to `NUM_SERVOS` servos, and streams telemetry + receives commands over the
  selected transport.
- `gui.py` — PySide6 app with four tabs: **Connect**, **Monitor**, **Visualizer**,
  **Servos**.
- `CleanInput.py`, `live_charts.py` — packet cleaning / unit conversion and the
  scrolling charts used by the Monitor tab.
- `web/` — vendored three.js scene for the Visualizer (`index.html`, `main.js`,
  `hand_model.js` + vendored `three.min.js`, `OrbitControls.js`, `qwebchannel.js`).
- `reciever.py` — minimal terminal receiver (WiFi/UDP mode). `bt_receiver.py` —
  minimal terminal receiver for the Bluetooth link.

## Wire protocol (transport-independent)

One newline-delimited ASCII line per sample, ~50 Hz. Same format on every
transport, so only the byte source changes on the PC side:

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,s0:<i>,s1:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z), floats.
- `ax..gz` — raw accel/gyro counts (converted to g / °/s on the PC).
- `s0..s1` — **servo angles the ESP32 currently holds, echoed back** so the Servos
  visualizer reflects real device state.

Commands PC→ESP32 use the same key:val form: `s0:90,s1:45`.

## Transports (the `link*` seam)

The firmware isolates the transport behind `linkBegin()` / `linkConnected()` /
`linkSend()` / `linkPoll()`. Pick one with `#define USE_BLUETOOTH` at the top of
`lifeOs.ino`:

- **`1` — Bluetooth Classic SPP** (default). ESP32 pairs as `lifeos` and appears
  as a COM port on the laptop. Keeps the laptop's WiFi/internet free and ignores
  the local network entirely (no router, no client isolation, no mDNS). This is
  the reliable path on managed/isolated networks. WiFi is compiled out.
- **`0` — WiFi / UDP**. ESP32 advertises `lifeos.local` / `_lifeos._udp` via mDNS.
  The PC discovers it (or connects by IP), sends a `hello` on `CMD_PORT`, and the
  ESP32 learns the PC's IP from that packet (nothing hardcoded — a DHCP change no
  longer breaks the link) and streams telemetry to `UDP_PORT`. **Requires a
  network that allows peer-to-peer traffic**; many managed WiFi networks
  (client/AP isolation) block this — symptom: ping works but UDP never arrives.

Roadmap: a runtime connection-method selector in the Connect tab mirroring this
firmware seam (WiFi / Bluetooth), and later a cloud-relay transport for reach.

## GUI tabs

- **Connect** (first tab) — WiFi mode: mDNS device list + Refresh + Connect, a
  manual "connect by IP / hostname" fallback, and a "make default" checkbox
  persisted to `connect_config.json` (auto-connects next launch).
- **Monitor** — connection status, per-axis raw/converted table, live charts,
  Reset/Recalibrate, UDP port diagnostics.
- **Visualizer** — three.js model (procedural hand from `hand_model.js`) driven by
  the quaternion over QWebChannel; **Zero / Level** cancels resting offset.
- **Servos** — slider + spinbox per servo, Upload button, and read-only `QDial`
  gauges that follow the **echoed** angles (true device state).

## GPIO / wiring (ESP-WROOM-32)

| Signal | ESP32 pin | Notes |
| --- | --- | --- |
| MPU6050 SDA | GPIO21 | default I2C SDA |
| MPU6050 SCL | GPIO22 | default I2C SCL |
| MPU6050 INT | GPIO4 | DMP data-ready interrupt (`INTERRUPT_PIN`) |
| MPU6050 VCC | 3V3 | |
| MPU6050 GND | GND | |
| MPU6050 AD0 | GND | selects I2C address `0x68` |
| Servo 0 signal | GPIO13 | `SERVO_PINS[0]` |
| Servo 1 signal | GPIO25 | `SERVO_PINS[1]` |

Servos are powered from an **external 5–6V supply** (not the ESP32), with its
**ground tied to the ESP32 ground** (common ground required). Add a bulk cap
(470–1000 µF) across the servo rail to stop current spikes browning out the ESP32.
Servo pins avoid the MPU pins (4/21/22), flash (6–11), input-only (34/35/36/39),
and strapping pins (0/2/5/12/15).

## Critical configuration

- `USE_BLUETOOTH` in `lifeOs.ino` — transport selector (see above).
- `secrets.h` (gitignored; copy from `secrets.example.h`) — WiFi mode needs
  `WIFI_SSID` / `WIFI_PASSWORD`. `PC_IP` is **no longer used** (learned at runtime).
- Ports: `UDP_PORT` 5005 (telemetry), `CMD_PORT` 5006 (commands/hello) — must match
  the constants in `gui.py`. `NUM_SERVOS` / `SERVO_PINS` must match
  `NUM_SERVOS` / `SERVO_GPIOS` in `gui.py`.

## Build / run

**Firmware:** Arduino IDE / arduino-cli with the ESP32 board package. Libraries:
"MPU6050" by Electronic Cats (bundles I2Cdev + MPU6050_6Axis_MotionApps20) and
"ESP32Servo". `ESP32Servo`, `WiFi`/`ESPmDNS`, and `BluetoothSerial` come with the
core. **For Bluetooth mode set Tools → Partition Scheme to "Huge APP" / "Minimal
SPIFFS"** — the BT stack + DMP blob overflow the default partition. Serial monitor
at 115200; keep the sensor still for ~2 s on boot (DMP calibration).

**PC:**
```
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt   # PySide6 (pinned 6.8.0.2), zeroconf, pyserial
python gui.py
```
Quick link checks without the GUI: `python reciever.py` (WiFi) or
`python bt_receiver.py [COMx]` (Bluetooth — pair `lifeos` first).

## Notes / gotchas

- Yaw drifts slowly (no magnetometer); roll/pitch are stable. Use Zero / Level.
- `README.md` predates the two-way / servo / Bluetooth work and is partly stale;
  this file is the current reference.
- PC-side tools target Windows (port diagnostics use `netstat`/`tasklist`;
  `reciever.py` uses `msvcrt`).
