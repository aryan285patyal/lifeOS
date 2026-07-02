# lifeOS

A small hardware project: an **ESP32 + MPU6050 IMU** streams fused orientation
(and drives servos) to a PC, which visualizes the motion in 3D and commands the
servos back. The link is **two-way** and the transport is **swappable** between
**Bluetooth** and **Wi-Fi**.

```
MPU6050 ──I2C──► ESP32 ──(Bluetooth SPP  or  Wi-Fi/UDP)──► PC (PySide6 GUI)
 servos ◄─PWM──   ▲  ◄───────── servo commands ──────────────┘
```

The MPU6050's on-chip **DMP** produces a fused orientation **quaternion**; the
ESP32 forwards that plus raw accel/gyro counts and the current servo angles as
one ASCII stream. The PC GUI shows a live 3D model, plots the data, and sends
servo targets; the ESP32 echoes the applied angles so the servo view always
reflects real device state.

## Wire protocol (same on every transport)

One line per sample (~50 Hz):

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,s0:<i>,s1:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z), floats.
- `ax..gz` — raw accel/gyro counts (converted to g / °/s on the PC).
- `s0..s1` — servo angles the ESP32 currently holds (echoed back).

Commands PC→ESP32 use the same form: `s0:90,s1:45`.

## Components

| File | Role |
| --- | --- |
| `lifeOs.ino` | ESP32 firmware. MPU6050 DMP + servos, streaming/commands over a swappable link (`USE_BLUETOOTH`). |
| `gui.py` | PySide6 app: **Connect**, **Monitor**, **Visualizer**, **Servos** tabs, with a Bluetooth/Wi-Fi transport selector. |
| `CleanInput.py` | Cleans the packet stream and converts raw counts to physical units. |
| `live_charts.py` | Scrolling Qt charts for the Monitor tab. |
| `web/` | Vendored three.js scene for the Visualizer (`index.html`, `main.js`, `hand_model.js`, + vendored libs). |
| `reciever.py` | Minimal terminal receiver for Wi-Fi/UDP mode. |
| `bt_receiver.py` | Minimal terminal receiver for the Bluetooth link. |
| `secrets.example.h` | Template for Wi-Fi credentials (copy to `secrets.h`, gitignored). |
| `designDecisions.md` | Why the architecture is the way it is (transports, protocol, servos, roadmap). |

## Transports

Set at the top of `lifeOs.ino`:

- **`#define USE_BLUETOOTH 1`** (default) — **Bluetooth Classic SPP**. The ESP32
  pairs as `lifeos` and shows up as a COM port. Keeps the laptop's Wi-Fi/internet
  free and ignores the local network entirely — the reliable choice on managed /
  isolated Wi-Fi (apartment, campus, corporate).
- **`#define USE_BLUETOOTH 0`** — **Wi-Fi / UDP** with mDNS discovery
  (`lifeos.local` / `_lifeos._udp`). The PC discovers the device (or connects by
  IP), sends a `hello`, and the ESP32 learns the PC's IP from it (nothing
  hardcoded). Requires a network that allows peer-to-peer traffic; many managed
  networks block it (symptom: ping works but UDP never arrives).

The GUI's Connect tab lets you pick either transport at runtime.

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

Power servos from an **external 5–6V supply** (not the ESP32), with its **ground
tied to the ESP32 ground**. Add a bulk cap (470–1000 µF) across the servo rail to
prevent current spikes browning out the board. The `INT` wire is required (the
firmware drains the DMP FIFO on it).

## Firmware setup

1. Open `lifeOs.ino` in the Arduino IDE / arduino-cli with the **ESP32 board
   package**.
2. Install libraries: **"MPU6050" by Electronic Cats** (bundles I2Cdev +
   MPU6050_6Axis_MotionApps20) and **"ESP32Servo"**. `WiFi`/`ESPmDNS`/
   `BluetoothSerial` ship with the core.
3. Pick a transport with `USE_BLUETOOTH`. For **Bluetooth**, set **Tools →
   Partition Scheme → "Huge APP"** (or "Minimal SPIFFS") — the BT stack + DMP
   blob overflow the default partition.
4. For **Wi-Fi** mode: `cp secrets.example.h secrets.h` and set `WIFI_SSID` /
   `WIFI_PASSWORD` (Bluetooth mode needs no secrets; `PC_IP` is no longer used).
5. Upload. Open the serial monitor at **115200**; keep the sensor still for ~2 s
   on boot (DMP calibration).

## PC setup

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt   # PySide6 (pinned 6.8.0.2), zeroconf, pyserial
python gui.py
```

- **Connect** — choose Bluetooth (pick the paired `lifeos` COM port) or Wi-Fi
  (mDNS list or manual IP/`lifeos.local`); "Make this my default" auto-connects
  next launch.
- **Monitor** — connection status, raw/converted table, live charts, Reset /
  Recalibrate, and (Wi-Fi) a UDP port diagnostics helper.
- **Visualizer** — a 3D model that rotates with the device; **Zero / Level**
  cancels the resting pose. Rendered with three.js in a `QWebEngineView`.
- **Servos** — sliders/spinboxes to set angles + read-only dials that follow the
  **echoed** angles (true device state).

Quick link checks without the GUI: `python reciever.py` (Wi-Fi) or
`python bt_receiver.py [COMx]` (Bluetooth — pair `lifeos` first).

## Notes

- Yaw drifts slowly (no magnetometer); roll/pitch are stable. Use Zero / Level.
- PC-side tools target Windows (`netstat`/`tasklist` diagnostics; `reciever.py`
  uses `msvcrt`).
- See `CLAUDE.md` for the working reference and `designDecisions.md` for rationale.
