# lifeOS

A small hardware telemetry project that streams motion data from an
**MPU6050 6-axis IMU** to a PC over WiFi and visualizes it live.

```
MPU6050  ──I2C──►  ESP32  ──UDP/WiFi──►  PC (receiver / GUI)
```

The MPU6050's on-chip **DMP** (Digital Motion Processor) produces a fused
orientation **quaternion**; the ESP32 forwards that plus the raw accel/gyro
counts as ASCII UDP packets. On the PC side you can watch the raw stream in a
terminal or run a Qt GUI with two tabs: a **Monitor** (calibrated values, live
charts, connection status) and a **Visualizer** (a real-time 3D model that
rotates to match the device's orientation).

## Components

| File | Role |
| --- | --- |
| `lifeOs.ino` | ESP32 firmware. Reads the MPU6050 DMP and streams a quaternion + raw counts over UDP (~50 Hz). |
| `reciever.py` | Minimal terminal receiver. Prints each packet; press `r` to zero the sensor. |
| `gui.py` | PySide6 desktop app: **Monitor** tab (table, charts, port diagnostics) + **Visualizer** tab (3D orientation). |
| `CleanInput.py` | Parses/cleans the packet stream and converts raw counts to physical units (g, °/s). |
| `live_charts.py` | Scrolling Qt charts for accelerometer and gyroscope axes. |
| `web/` | Vendored three.js scene (`index.html`, `main.js`, `three.min.js`, `OrbitControls.js`, `qwebchannel.js`) for the Visualizer. |
| `secrets.example.h` | Template for WiFi/PC-IP secrets (copy to `secrets.h`, which is gitignored). |
| `requirements.txt` | Python dependencies for the PC-side tools. |

The wire format is a single ASCII string per packet — a fused quaternion
(`q0..q3`, floats: w, x, y, z) followed by **raw sensor counts**:

```
q0:<float>,q1:<float>,q2:<float>,q3:<float>,ax:<int>,ay:<int>,az:<int>,gx:<int>,gy:<int>,gz:<int>
```

The Visualizer uses the quaternion directly; the Monitor still converts the raw
accel/gyro counts to g / °/s on the PC side (`CleanInput.convert`).

## Hardware

- ESP-WROOM-32 (dual-core ESP32) dev board
- MPU6050 IMU breakout (I2C address `0x68`)

| MPU6050 | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 (default I2C SDA) |
| SCL | GPIO22 (default I2C SCL) |
| **INT** | **GPIO4** — DMP data-ready interrupt (new wire; no pull resistor needed) |

The `INT` wire is required: the firmware drains the DMP FIFO on this interrupt.

## Firmware setup

1. Open `lifeOs.ino` in the Arduino IDE (or `arduino-cli`) with the **ESP32 board package** installed.
2. Install the **"MPU6050" library by Electronic Cats** via the Library Manager. It bundles `I2Cdev` and `MPU6050_6Axis_MotionApps20`, which drive the DMP. (WiFi/UDP use the built-in ESP32 core libraries.)
3. Create your secrets file from the template (the sketch `#include`s `secrets.h`, which is gitignored and never committed):

   ```bash
   cp secrets.example.h secrets.h
   ```

   Then edit `secrets.h` with your real values:
   - `WIFI_SSID` / `WIFI_PASSWORD` — your WiFi network.
   - `PC_IP` — the receiving PC's current LAN IP (run `ipconfig` on the PC). **This changes between networks/DHCP leases and is the most common reason packets don't arrive.**

   `UDP_PORT` (default `5005`) stays in the sketch and must match the PC side.
4. Select the ESP32 board + serial port and upload.
5. Open the serial monitor at **115200 baud**. On boot the DMP calibrates — **keep the sensor still and level for ~2 seconds** — then it prints `DMP ready` and the ESP32's IP once WiFi connects.

The ESP32 and the PC must be on the **same network** for UDP to route.

### Firmware design notes

- **Dual-core split:** a dedicated FreeRTOS task pinned to **core 0** drains the DMP
  FIFO promptly (interrupt-driven on GPIO4, with FIFO-overflow resync); WiFi/UDP
  transmit runs on **core 1**. This keeps the IMU read path off the network path so a
  future on-board camera (JPEG encode) can't starve the FIFO.
- **Orientation accuracy:** roll/pitch are absolute and stable; **yaw drifts slowly**
  because the MPU6050 has no magnetometer. Use the Visualizer's **Zero / Level** button
  to reset the resting pose.

## PC setup

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
pip install -r requirements.txt
```

> `PySide6` is pinned to `6.8.0.2`; newer versions need a newer MSVC runtime than
> some Python builds ship with. See the note in `requirements.txt`.

### Terminal receiver

```bash
python reciever.py
```

Prints each decoded packet. Press `r` to recalibrate (zero) the sensor while
holding it still; `Ctrl+C` to quit.

> `reciever.py` uses `msvcrt` for the keypress handling and is **Windows-only**.

### GUI

```bash
python gui.py
```

- **Monitor tab:** live connection status, a per-axis table (raw + converted),
  scrolling charts, a **Reset / Recalibrate** button, and a UDP port diagnostics
  helper.
- **Visualizer tab:** a 3D labeled box (TOP/BOTTOM/LEFT/RIGHT/FRONT/BACK) that
  rotates in real time with the device. Drag to orbit the camera, scroll to zoom,
  and press **Zero / Level** to set the current pose as upright. The 3D view is
  rendered with three.js inside a `QWebEngineView`; no extra pip install is needed
  (QtWebEngine ships with PySide6, and three.js is vendored under `web/`).

The `web/` model is built in one place (`createModel()` in `web/main.js`) so the
box can later be swapped for a glTF mannequin head / hand model.

## Notes

- UDP port `5005` must be free on the PC. The GUI includes a "Check Port" helper
  that lists any process holding it and offers a `taskkill` command.
- The PC-side tools (`reciever.py`, parts of `gui.py`) target Windows.
