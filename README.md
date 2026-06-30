# lifeOS

A small hardware telemetry project that streams motion data from an
**MPU6050 6-axis IMU** to a PC over WiFi and visualizes it live.

```
MPU6050  ──I2C──►  ESP32  ──UDP/WiFi──►  PC (receiver / GUI)
```

The ESP32 reads raw accelerometer and gyroscope counts from the MPU6050 and
broadcasts them as ASCII UDP packets. On the PC side you can either watch the
raw stream in a terminal or run a Qt GUI that shows calibrated values, live
charts, and connection status.

## Components

| File | Role |
| --- | --- |
| `lifeOs.ino` | ESP32 firmware. Reads the MPU6050 over I2C and sends one UDP packet per sample (default 10 Hz). |
| `reciever.py` | Minimal terminal receiver. Prints each packet; press `r` to zero the sensor. |
| `gui.py` | PySide6 desktop app: connection status, per-axis table, live charts, port diagnostics. |
| `CleanInput.py` | Parses/cleans the packet stream and converts raw counts to physical units (g, °/s). |
| `live_charts.py` | Scrolling Qt charts for accelerometer and gyroscope axes. |
| `requirements.txt` | Python dependencies for the PC-side tools. |

The wire format is a single ASCII string per packet:

```
ax:<int>,ay:<int>,az:<int>,gx:<int>,gy:<int>,gz:<int>
```

Values are **raw sensor counts**. Conversion to g / °/s happens on the PC side
(`CleanInput.convert`).

## Hardware

- ESP32 dev board
- MPU6050 IMU breakout (I2C address `0x68`)
- Wiring: `SDA → GPIO21`, `SCL → GPIO22`, `VCC → 3V3`, `GND → GND` (defaults for most ESP32 boards)

## Firmware setup

1. Open `lifeOs.ino` in the Arduino IDE (or `arduino-cli`) with the **ESP32 board package** installed.
2. Create your secrets file from the template (the sketch `#include`s `secrets.h`, which is gitignored and never committed):

   ```bash
   cp secrets.example.h secrets.h
   ```

   Then edit `secrets.h` with your real values:
   - `WIFI_SSID` / `WIFI_PASSWORD` — your WiFi network.
   - `PC_IP` — the receiving PC's current LAN IP (run `ipconfig` on the PC). **This changes between networks/DHCP leases and is the most common reason packets don't arrive.**

   `UDP_PORT` (default `5005`) stays in the sketch and must match the PC side.
3. Select the ESP32 board + serial port and upload.
4. Open the serial monitor at **115200 baud** — the ESP32 prints its own IP once WiFi connects.

The ESP32 and the PC must be on the **same network** for UDP to route.

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

Shows live connection status, a per-axis table (raw + converted), scrolling
charts, a **Reset / Recalibrate** button, and a UDP port diagnostics helper.

## Notes

- UDP port `5005` must be free on the PC. The GUI includes a "Check Port" helper
  that lists any process holding it and offers a `taskkill` command.
- The PC-side tools (`reciever.py`, parts of `gui.py`) target Windows.
