# lifeOS

A small hardware project: an **ESP32 + MPU6050 IMU** streams fused orientation
(and drives servos) to a PC, which visualizes the motion in 3D and commands the
servos back. **Two boards are supported** (compile-time `#define` in
`lifeOs.ino`, runtime Board dropdown in the GUI):

- **ESP-WROOM-32** — the sensor/servo feed is **Bluetooth Classic SPP**
  (always on, network-independent).
- **ESP32-S3-CAM** (GoouuuTech, ESP32-S3-WROOM-1 **N16R8**, OV3660 camera,
  dual USB-C, microSD) — the S3 has **no Bluetooth Classic**, so the feed
  starts on **USB serial** (the CH343 "COM" USB-C port) and can then be moved
  onto **Wi-Fi UDP** with the GUI's **Go wireless** button.

```
                 ┌────────── feed: BT SPP (WROOM-32) / USB→Wi-Fi UDP (S3) ──┐
MPU6050 ──I2C──► ESP32   sensor telemetry + servo echo  ──►  PC (PySide6 GUI)
 servos ◄─PWM──   │      ◄── servo commands + Wi-Fi provisioning ───────────┘
                 └──────────── Wi-Fi/UDP (video, on demand) ───────────────► PC
```

Either way the laptop sends its Wi-Fi SSID/password and **current IP** over the
feed link; the ESP32 joins Wi-Fi and streams high-bandwidth **video** by UDP.
Sending the laptop's IP each session means a changing DHCP address never breaks
the link, and nothing is hardcoded. On the S3, `feed:wifi` ("Go wireless")
additionally moves telemetry + servo commands onto Wi-Fi UDP so the USB cable
can be unplugged. (No real camera output yet — a synthetic frame stream tests
the Wi-Fi path; the S3's OV3660 is the planned source.)

## Wire protocol

Identical bytes on every transport (BT SPP / USB serial / Wi-Fi UDP).

**Sensor telemetry, one line per sample (~50 Hz):**

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,tp:<i>,s0:<i>,s1:<i>,e0:<i>,e1:<i>,dmp:<i>,wf:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z).
- `ax..gz` — raw accel/gyro counts (converted on the PC); `tp` — die temp counts.
- `s0..s1` — servo angles the ESP32 holds (echoed back); `e0..e1` — 1 if that
  servo is enabled (0 = detached/limp).
- `dmp` — DMP producing orientation (1/0); `wf` — Wi-Fi video streaming (1/0).
  These drive the MPU and Wi-Fi status indicators.

**Control PC→ESP32:** `s0:90,s1:45` (servo), `e0:0` (disable/limp a servo),
`id?` → `id:lifeos,proto:1,servos:2,board:<b>` (identity, for COM auto-detect),
`cal` (gyro bias recal), `acal:set,...`/`acal:clear` (6-position accel cal),
`wifi:<ssid>|<password>|<laptop_ip>` (provision Wi-Fi), `wifi:off`, and on the
S3: `feed:wifi` / `feed:usb` (move the sensor/servo feed onto Wi-Fi UDP / back).

## Components

| File | Role |
| --- | --- |
| `lifeOs.ino` | ESP32 firmware. MPU6050 DMP + servos, Bluetooth sensor/control link, Wi-Fi video on demand. |
| `gui.py` | PySide6 app: Connect / Monitor / Servos / Hand Model tabs + status bar (3D visualizer lives in Monitor). |
| `CleanInput.py` | Cleans the packet stream, converts raw counts to physical units. |
| `live_charts.py` | Scrolling Qt charts for the Monitor tab. |
| `web/` | Vendored three.js scene for the Monitor tab's 3D view. |
| `bt_receiver.py` | Minimal terminal receiver for the Bluetooth sensor link. |
| `wifi_video_test.py` | Provisions Wi-Fi over Bluetooth and measures the video UDP stream. |
| `reciever.py` | Legacy terminal receiver for the old Wi-Fi/UDP sensor mode. |
| `designDecisions.md` | Why the architecture is the way it is. |

## Hardware / wiring

| Wire | ESP-WROOM-32 | ESP32-S3-CAM |
| --- | --- | --- |
| MPU6050 SDA | GPIO21 | GPIO21 |
| MPU6050 SCL | GPIO22 | **GPIO14** |
| MPU6050 INT | GPIO4 | **GPIO47** |
| MPU6050 VCC / GND | 3V3 / GND | 3V3 / GND |
| MPU6050 AD0 | GND (address `0x68`) | GND (address `0x68`) |
| Servo 0 signal | GPIO13 | **GPIO1** |
| Servo 1 signal | GPIO25 | **GPIO2** |

The S3-CAM's only free GPIOs are **1, 2, 3, 14, 21, 47** — the OV3660 camera
owns 4–13/15–18, the microSD slot 38–40, the WS2812 LED 48, native USB 19/20,
and 0/45/46 are strapping pins. GPIO3 is the one spare. The old WROOM-32 map
physically cannot work there (GPIO22/25 don't exist on the S3; 4 and 13 are
camera pins).

Power servos from an **external 5–6V supply** (not the ESP32), ground tied to the
ESP32 ground; add a 470–1000 µF bulk cap across the servo rail to prevent current
spikes browning out the board. The `INT` wire is required (the firmware drains the
DMP FIFO on it).

**S3-CAM antenna:** the board has both a PCB antenna and an IPEX socket — clip
the bundled external antenna on for noticeably better Wi-Fi range/throughput
(it's what carries the video). If it makes no difference, the tiny 0 Ω RF-path
resistor near the socket is still pointing at the PCB trace.

**S3-CAM USB ports:** the **"COM"** USB-C goes through the CH343 UART bridge —
this is the flash/monitor/feed port the GUI connects to. The other ("USB") is
the S3's native USB OTG.

## Firmware setup

1. Open `lifeOs.ino` and pick the board at the top: `#define BOARD_S3CAM` or
   `#define BOARD_WROOM32` (exactly one).
2. Arduino IDE / arduino-cli with the **ESP32 board package**; install
   **"MPU6050" by Electronic Cats** + **"ESP32Servo"** (WiFi/WiFiUdp/
   BluetoothSerial ship with the core).
3. Board settings:
   - **WROOM-32:** board "ESP32 Dev Module", **Partition Scheme → "Huge APP"**
     (BT + Wi-Fi + the DMP blob overflow the default partition).
   - **S3-CAM:** board "ESP32S3 Dev Module", **Flash Size 16MB**, **PSRAM
     "OPI PSRAM"**; the default partition fits (no BT Classic stack).
4. Upload (S3: via the **COM** USB-C port). Open the serial monitor at
   **115200**; keep the sensor still ~2 s on boot (DMP calibration). No Wi-Fi
   credentials are needed at build time — they arrive from the GUI.

## PC setup

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt   # PySide6 (pinned 6.8.0.2), zeroconf, pyserial
python gui.py
```

- **Connect** — a **Board dropdown** picks the flow (persisted):
  - *ESP-WROOM-32*: pair `lifeos` in Windows Bluetooth, pick the COM port
    (**Find lifeOs port** auto-detects it), Connect; then Wi-Fi section for
    video (creds sent over Bluetooth).
  - *ESP32-S3-CAM*: plug the board's **COM** USB-C in, Find lifeOs port,
    Connect (this is the "pairing"); Wi-Fi section sends creds over USB; once
    `wifi:connected` arrives, **Go wireless** moves the sensor/servo feed onto
    Wi-Fi UDP — the USB cable can then be unplugged.
- **Monitor** — connection status, raw/converted table, live charts, a 3D model
  that rotates with the device (three.js in a `QWebEngineView`; **Zero / Level**
  cancels the resting pose), Reset / Recalibrate.
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
- Radios share the 2.4 GHz band: on the WROOM-32, BT + Wi-Fi coexistence lowers
  Wi-Fi throughput; the S3 (BLE only, unused) doesn't pay that cost.
- The saved Wi-Fi password sits in `connect_config.json` (gitignored, plaintext).
- PC-side tools target Windows. See `CLAUDE.md` for the working reference and
  `designDecisions.md` for rationale.
