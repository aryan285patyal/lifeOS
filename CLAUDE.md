# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`lifeOs` is a two-part hardware project: an ESP32 firmware that reads an MPU6050
IMU and drives servos, and a PySide6 desktop GUI that visualizes orientation,
plots the data, commands the servos, and manages connectivity. The ESP32 runs
**two channels at once**:

- **Bluetooth Classic SPP (always on)** — sensor telemetry out (quaternion + raw
  counts + servo echo + health flags), servo commands in, an `id?` identity
  reply, and Wi-Fi **provisioning** in. Network-independent; keeps the laptop's
  Wi-Fi free. This is the link the Monitor/Visualizer/Servos tabs read from.
- **Wi-Fi station (on demand)** — high-bandwidth **video** over UDP. Brought up
  only after the laptop sends Wi-Fi credentials + its own current IP over
  Bluetooth, so a changing laptop DHCP address is handled every session and
  nothing is hardcoded. (Camera is future work; a synthetic frame stream tests
  the path today.)

Files:
- `lifeOs.ino` — firmware (MPU6050 DMP + servos + BT sensor link + Wi-Fi video).
- `gui.py` — PySide6 app: **Connect**, **Monitor**, **Visualizer**, **Servos**
  tabs, plus a bottom-left status bar.
- `CleanInput.py`, `live_charts.py` — packet cleaning / unit conversion + charts.
- `web/` — vendored three.js scene for the Visualizer.
- `reciever.py` — legacy terminal receiver (old Wi-Fi/UDP sensor mode).
- `bt_receiver.py` — terminal receiver for the Bluetooth sensor link.
- `wifi_video_test.py` — provisions over BT and measures the Wi-Fi video stream.
- `designDecisions.md` — rationale for the architecture.

## Wire protocol

**Sensor telemetry (Bluetooth), one newline-delimited line per sample (~50 Hz):**

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,s0:<i>,s1:<i>,dmp:<i>,wf:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z), floats.
- `ax..gz` — raw accel/gyro counts (converted on the PC).
- `s0..s1` — servo angles the ESP32 holds (echoed back).
- `dmp` — 1 if the DMP is producing orientation (drives the MPU status dot).
- `wf` — 1 if Wi-Fi video is streaming (drives the Wi-Fi status dot).

`q*` parse as float, everything else int.

**Control lines PC→ESP32 (Bluetooth):**
- `s0:90,s1:45` — servo command.
- `id?` → ESP32 replies `id:lifeos,proto:1,servos:2` (used to auto-detect the
  lifeOs COM port).
- `wifi:<ssid>|<password>|<laptop_ip>` — provision Wi-Fi; the ESP32 joins that
  network and streams video UDP to `<laptop_ip>:5010`. Replies `wifi:connecting`
  then `wifi:connected,<esp_ip>` over Bluetooth.

**Video (Wi-Fi UDP → port 5010):** synthetic `vid:<seq>:`-prefixed 1 KB packets.

## GUI

- **Connect tab** — two sections (no transport dropdown):
  - *Bluetooth (sensor & servos)*: COM port list, **Find lifeOs port**
    (auto-detects by sending `id?`/reading telemetry), Connect, Save as default.
  - *Wi-Fi (video)*: SSID / password / laptop-IP fields (IP auto-filled),
    Connect (sends creds over the active BT link), Save as default.
  - Saved defaults auto-connect BT and auto-provision video on launch.
- **Monitor / Visualizer / Servos** — read through a `ConnectionManager` and are
  transport-agnostic.
- **Hand Model tab** — dropdown of the PC's video input devices (via
  QtMultimedia `QMediaDevices`); Apply streams the selected camera into a small
  preview box (`QCamera` → `QMediaCaptureSession` → `QVideoWidget`). Planned:
  the ESP32's Wi-Fi UDP video as another selectable source, then hand tracking.
- **Status bar (bottom-left)** — colored `StatusDot` boxes (BT blue, Wi-Fi green,
  Servo1/Servo2 red, MPU yellow) showing ✓/✗ by whether that source had a live
  signal in the last second (all derived from the fresh BT telemetry line).

## Code structure (gui.py)

`Link` (base) → `BluetoothLink` (pyserial COM) and `WifiLink` (legacy UDP sensor,
retained but not used by the two-section Connect tab). `ConnectionManager` holds
the active link and proxies `snapshot()/is_connected()/send_servos()/
description()`; switching links stops the old one. `PortProber` runs COM probing
off the GUI thread. `DeviceDiscovery` (mDNS) is retained but unused now that
Wi-Fi is video-only and provisioned over Bluetooth.

## GPIO / wiring (ESP-WROOM-32)

| Wire | ESP32 pin |
| --- | --- |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| MPU6050 INT | GPIO4 (`INTERRUPT_PIN`) |
| MPU6050 VCC / GND | 3V3 / GND |
| MPU6050 AD0 | GND (address `0x68`) |
| Servo 0 signal | GPIO13 (`SERVO_PINS[0]`) |
| Servo 1 signal | GPIO25 (`SERVO_PINS[1]`) |

Power servos from an **external 5–6V supply** (not the ESP32), ground tied to the
ESP32 ground; add a 470–1000 µF bulk cap across the servo rail. If the MPU reads
`MPU6050 connection FAILED`, it's almost always an I2C wiring/power issue (re-seat
SDA/SCL/VCC/GND, confirm AD0→GND).

## Critical configuration

- Ports: `VIDEO_PORT` 5010 (Wi-Fi video). `UDP_PORT` 5005 / `CMD_PORT` 5006 remain
  for the legacy Wi-Fi sensor path only. Constants must match between `lifeOs.ino`
  and `gui.py`.
- `NUM_SERVOS` / `SERVO_PINS` (firmware) must match `NUM_SERVOS` / `SERVO_GPIOS`
  (gui.py).
- Wi-Fi creds are provided at runtime over Bluetooth; nothing is in `secrets.h`
  anymore (the firmware no longer includes it).

## Build / run

**Firmware:** Arduino IDE / arduino-cli with the ESP32 board package. Libraries:
"MPU6050" by Electronic Cats + "ESP32Servo" (WiFi/WiFiUdp/BluetoothSerial ship
with the core). **Set Tools → Partition Scheme → "Huge APP"** — Bluetooth **and**
Wi-Fi **and** the DMP blob overflow the default partition. Serial at 115200; keep
the sensor still ~2 s on boot (DMP calibration). Note BT+Wi-Fi coexistence shares
one radio, so Wi-Fi throughput is lower than Wi-Fi alone.

**PC:**
```
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt   # PySide6 6.8.0.2, zeroconf, pyserial
python gui.py
```
Typical flow: Connect tab → Bluetooth → Find lifeOs port → Connect; then Wi-Fi
section → SSID/password → Connect. Verify video with
`python wifi_video_test.py [--com COMx --ssid X --password Y]` (allow inbound UDP
5010 in the firewall).

## Notes

- Yaw drifts (no magnetometer); roll/pitch stable. Use the Visualizer's
  Zero / Level.
- The saved Wi-Fi password lives in `connect_config.json` (gitignored, plaintext).
- PC tools target Windows (`netstat`/`tasklist`; `reciever.py` uses `msvcrt`).
- PySide6 is pinned to 6.8.0.2 (newer needs an MSVC runtime some Python builds
  lack).
