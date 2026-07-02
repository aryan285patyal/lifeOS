# lifeOS

A small hardware project: an **ESP32 + MPU6050 IMU** streams fused orientation
(and drives servos) to a PC, which visualizes the motion in 3D and commands the
servos back. The ESP32 runs **two channels at once**:

```
                 ┌──────────────── Bluetooth SPP (control) ───────────────┐
MPU6050 ──I2C──► ESP32   sensor telemetry + servo echo  ──►  PC (PySide6 GUI)
 servos ◄─PWM──   │      ◄── servo commands + Wi-Fi provisioning ──────────┘
                 └──────────── Wi-Fi/UDP (video, on demand) ──────────────► PC
```

Bluetooth is the always-on control link (sensor + servos). Once the laptop sends
its Wi-Fi SSID/password and current IP **over Bluetooth**, the ESP32 joins Wi-Fi
and streams high-bandwidth **video** by UDP. Sending the laptop's IP each session
means a changing DHCP address never breaks the link, and nothing is hardcoded.
(No camera yet — a synthetic frame stream tests the Wi-Fi path.)

## Wire protocol

**Sensor telemetry (Bluetooth), one line per sample (~50 Hz):**

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,s0:<i>,s1:<i>,dmp:<i>,wf:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z).
- `ax..gz` — raw accel/gyro counts (converted on the PC).
- `s0..s1` — servo angles the ESP32 holds (echoed back).
- `dmp` — DMP producing orientation (1/0); `wf` — Wi-Fi video streaming (1/0).
  These drive the MPU and Wi-Fi status indicators.

**Control PC→ESP32 (Bluetooth):** `s0:90,s1:45` (servo), `id?` → `id:lifeos,...`
(identity, for COM auto-detect), `wifi:<ssid>|<password>|<laptop_ip>` (provision
Wi-Fi video).

## Components

| File | Role |
| --- | --- |
| `lifeOs.ino` | ESP32 firmware. MPU6050 DMP + servos, Bluetooth sensor/control link, Wi-Fi video on demand. |
| `gui.py` | PySide6 app: Connect / Monitor / Visualizer / Servos / Hand Model tabs + status bar. |
| `CleanInput.py` | Cleans the packet stream, converts raw counts to physical units. |
| `live_charts.py` | Scrolling Qt charts for the Monitor tab. |
| `web/` | Vendored three.js scene for the Visualizer. |
| `bt_receiver.py` | Minimal terminal receiver for the Bluetooth sensor link. |
| `wifi_video_test.py` | Provisions Wi-Fi over Bluetooth and measures the video UDP stream. |
| `reciever.py` | Legacy terminal receiver for the old Wi-Fi/UDP sensor mode. |
| `designDecisions.md` | Why the architecture is the way it is. |

## Hardware / wiring (ESP-WROOM-32)

| Wire | ESP32 pin |
| --- | --- |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| MPU6050 INT | GPIO4 |
| MPU6050 VCC / GND | 3V3 / GND |
| MPU6050 AD0 | GND (address `0x68`) |
| Servo 0 signal | GPIO13 |
| Servo 1 signal | GPIO25 |

Power servos from an **external 5–6V supply** (not the ESP32), ground tied to the
ESP32 ground; add a 470–1000 µF bulk cap across the servo rail to prevent current
spikes browning out the board. The `INT` wire is required (the firmware drains the
DMP FIFO on it).

## Firmware setup

1. Open `lifeOs.ino` in the Arduino IDE / arduino-cli with the **ESP32 board
   package**.
2. Install **"MPU6050" by Electronic Cats** + **"ESP32Servo"**
   (WiFi/WiFiUdp/BluetoothSerial ship with the core).
3. **Set Tools → Partition Scheme → "Huge APP"** (or "Minimal SPIFFS") — Bluetooth
   **+** Wi-Fi **+** the DMP blob overflow the default partition.
4. Upload. Open the serial monitor at **115200**; keep the sensor still ~2 s on
   boot (DMP calibration). No Wi-Fi credentials are needed at build time — they
   arrive over Bluetooth from the GUI.

## PC setup

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt   # PySide6 (pinned 6.8.0.2), zeroconf, pyserial
python gui.py
```

- **Connect** — two sections. *Bluetooth (sensor & servos)*: pick the paired
  `lifeos` COM port (**Find lifeOs port** auto-detects it), Connect, Save default.
  *Wi-Fi (video)*: enter SSID/password, confirm the auto-filled laptop IP,
  Connect (creds sent over Bluetooth), Save default.
- **Monitor** — connection status, raw/converted table, live charts, Reset /
  Recalibrate.
- **Visualizer** — a 3D model that rotates with the device; **Zero / Level**
  cancels the resting pose (three.js in a `QWebEngineView`).
- **Servos** — sliders/spinboxes to set angles + dials that follow the echoed
  angles (true device state).
- **Status bar (bottom-left)** — colored ✓/✗ boxes for **BT** (blue), **WiFi**
  (green), **S1/S2** (red), **MPU** (yellow), live if that source had a signal in
  the last second.

Verify the Wi-Fi video path:
```bash
python wifi_video_test.py --com COM7 --ssid MyWifi --password secret
# or, if already provisioned via the GUI:
python wifi_video_test.py
```
(Allow inbound **UDP 5010** in the firewall if you see zero packets.)

## Notes

- Yaw drifts slowly (no magnetometer); roll/pitch are stable. Use Zero / Level.
- BT + Wi-Fi share one radio, so Wi-Fi throughput is lower than Wi-Fi alone.
- The saved Wi-Fi password sits in `connect_config.json` (gitignored, plaintext).
- PC-side tools target Windows. See `CLAUDE.md` for the working reference and
  `designDecisions.md` for rationale.
