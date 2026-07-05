# GUI Functioning (`gui.py`)

`gui.py` is a PySide6 desktop app for the lifeOs ESP32. It talks to the device
over **Bluetooth Classic SPP** (a paired COM port) for sensor telemetry, servo
control, and Wi-Fi provisioning, and receives high-bandwidth **video over
Wi-Fi UDP** (port 5010) once provisioned. The window is four full-width tabs —
**Connect**, **Monitor** (with the 3D visualizer embedded), **Servos**,
**Hand Model** — plus a status bar of per-source connection dots.

## Transport layer

### `Link` (base class)

One parsed-sample store shared by both transports, guarded by a
`threading.Lock` because readers run off the GUI thread:

- `latest`: dict of the most recent telemetry line, parsed by `parse_line()`
  (`q*` keys as float, everything else as int).
- `last_seen`: `time.monotonic()` of the last good line. `is_connected()` is
  inferred from freshness: `True` only if a line arrived within the last 1 s.
- `controls`: non-telemetry replies (`cal:*`, `wifi:*`, `id:*`) arrive on the
  same byte stream; lines that fail telemetry parsing are stashed per prefix
  with a timestamp and read back via `control(prefix)`.
- `send_line(text)`: one raw protocol line to the device (`cal`, `e0:0`,
  `wifi:off`, ...). `send_servos(angles)` builds the `s0:..,s1:..` line.

### `BluetoothLink`

The active transport. Opens the COM port at 115200 (pyserial, imported
lazily), spawns a daemon reader thread that `readline()`s and `_ingest()`s
each line. `provision_video(ssid, password, ip)` sends
`wifi:<ssid>|<pass>|<laptop_ip>` so the ESP32 joins the network and streams
video UDP back to the laptop. `stop()` closes the port — which actively tears
down the SPP connection (the ESP32 sees `hasClient()` go false).

### `WifiLink` (legacy)

The old UDP sensor transport (telemetry in on 5005, commands out to 5006).
Retained for reference but not reachable from the two-section Connect tab.
`DeviceDiscovery` (mDNS browser for `_lifeos._udp`) is likewise retained but
unused now that Wi-Fi is video-only and provisioned over Bluetooth.

### `ConnectionManager`

Holds the active `Link` and proxies everything the tabs call —
`snapshot() / is_connected() / send_servos() / send_line() / control() /
description()` — so the Monitor and Servos tabs are transport-agnostic.
`set_active(link)` stops the previous link (freeing its COM port/socket);
`disconnect()` drops the active link explicitly.

## Connect tab (`ConnectTab`)

Two sections, no transport dropdown:

- **Bluetooth — sensor & servos.** A COM-port list, **Refresh ports**, **Find
  lifeOs port** (a `PortProber` thread opens each port, sends `id?`, and looks
  for the `id:lifeos` reply or live telemetry, reporting back via a Qt
  signal), **Connect**, **Disconnect** (closes the COM port → active SPP
  teardown), **Save as default**.
- **Wi-Fi — video.** SSID / password / laptop-IP fields (IP auto-detected via
  a routing-table trick in `get_local_ip()`), **Connect** (sends the creds
  over the active Bluetooth link), **Disconnect (stop video)** (sends
  `wifi:off`; the ESP32 stops streaming and powers its radio down), **Save as
  default**.

A 1 s `QTimer` drives auto-connect: the saved COM port is opened once it
appears, and saved Wi-Fi creds are auto-provisioned once Bluetooth is up.
Defaults persist in `connect_config.json` (gitignored; the Wi-Fi password is
stored in plaintext).

## Monitor tab (`MonitorWindow`)

A 100 ms `QTimer` calls `refresh()`, which snapshots the active link and
updates everything on the GUI thread.

- **Sensor table** — rows interleaved per axis: `ax, gx, ay, gy, az, gz,
  temp`. Columns: raw **Value**, **Converted** (`CleanInput.convert`: g,
  °/s, and °C via the MPU6050 die-temp formula `raw/340 + 36.53`), and
  **History** — a per-row `Sparkline` (live_charts.py) of the last 100
  converted samples, autoscaled with a per-family minimum span, colored x red
  / y green / z blue / temp yellow. Converted cells are green when fresh, red
  when held over (`CleanInput` flags "assumed" samples), grey before any data.
  The `tp` field is optional, so older firmware still works (temp row shows
  `-`).
- **Relative-state table** — Roll / Pitch / Yaw computed from the live DMP
  quaternion relative to the captured zero pose, and **Delta Temp**, the die
  temperature change since the session's first reading (a self-heating
  indicator: recalibrate once it flattens).
- **Reset / Recalibrate** — sends `cal` so the ESP32 re-runs its accel/gyro
  bias calibration at the source, waits for `cal:done` (10 s timeout, falls
  back gracefully on error or old firmware), then averages the next 20 fresh
  packets into a receiver-side zero for the raw counts and captures the
  current orientation as the angle table's zero pose. Temperature is never
  zeroed (`CALIB_SENSORS` vs `SENSORS`).
- **3D visualizer** — the `VisualizerPanel` described below sits at the bottom
  of the tab, under the relative-state table.

## 3D visualizer (`VisualizerPanel`, embedded in Monitor)

A `QWebEngineView` hosting the vendored three.js scene (`web/`), bridged over
`QWebChannel` (`OrientationBridge`: an `orientation` signal pushing quaternions
at ~30 Hz and a `zeroRequested` signal from the button).

Before each push, Python applies **yaw de-drift**: yaw has no absolute
reference (no magnetometer), so while the raw counts say the sensor is
physically still (gyro < ~3.7 °/s per axis and |accel| ≈ 1 g for ≥ 0.5 s), any
yaw change *is* drift — it's frozen out and the creep rate is learned with a
slow EMA, which keeps being subtracted while the sensor moves. Roll/pitch pass
through untouched.

In JavaScript (`web/main.js`), the device quaternion is remapped from the
DMP's Z-up frame to three.js Y-up, and **Zero** cancels heading only (a
swing-twist decomposition about the vertical), so pressing it while tilted
never bakes in a false "level".

## Servos tab (`ServoTab`)

Per servo: an **Input** dropdown (Manual / Roll / Pitch / Yaw), a slider +
spinbox, an **Enabled/Disabled** toggle, and a read-only dial. A 50 ms timer
refreshes state.

- **Manual** — slider release (or **Upload to ESP32**) sends the angles.
- **Mimic** — a servo bound to Roll/Pitch/Yaw continuously tracks that Euler
  component of the live quaternion (`quat_to_euler` → `axis_to_servo`,
  ±90° mapped onto 0–180, yaw halved), throttled to changes.
- **Enable/disable** — sends `e<i>:0/1`; disabling detaches the servo pin on
  the ESP32 (no PWM pulses, the horn goes limp) — an active off. The dials
  always show the ESP32's *echoed* angles (its real held position), with
  "(off)" when the echoed `e*` flag says detached.

## Hand Model tab (`HandModelTab`)

A dropdown of the PC's video input devices (QtMultimedia `QMediaDevices`);
**Apply** streams the selected camera into a preview box (`QCamera` →
`QMediaCaptureSession` → `QVideoWidget`). QtMultimedia is imported lazily —
if it's missing, the rest of the GUI still runs and this tab degrades to a
message. Planned: the ESP32's Wi-Fi UDP video as another selectable source,
then hand tracking.

## Status bar

Five fixed-color `StatusDot` boxes (BT blue, Wi-Fi green, Servo 1/2 red, MPU
yellow) showing ✓/✗. All states derive from the fresh Bluetooth telemetry
line: `wf` lights Wi-Fi, `e0/e1` + `s0/s1` light the servos (a detached servo
counts as not live), `dmp` + a quaternion light MPU. Everything is ✗ when
telemetry is stale (> 1 s).

## Threading model

Qt widgets are only touched on the GUI thread. Background work — the serial
reader, port probing, (legacy) UDP listening — runs on daemon threads that
either update lock-guarded state polled by `QTimer`s or report back through Qt
signals (which queue safely onto the GUI thread).

## Running

```
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt   # PySide6 6.8.0.2 (pinned), zeroconf, pyserial
python gui.py
```

`smoke_gui.py` is a headless sanity check of the Monitor tables (row order,
conversions, colors, sparkline feeding, relative angles, Delta Temp);
`live_charts.py` and `CleanInput.py` each run a self-test when executed
directly.
