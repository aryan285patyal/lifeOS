# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`lifeOs` is a two-part hardware project: an ESP32 firmware that reads an MPU6050
IMU and drives servos, and a PySide6 desktop GUI that visualizes orientation,
plots the data, commands the servos, and manages connectivity. **Two boards are
supported** — a compile-time `#define` at the top of `lifeOs.ino`
(`BOARD_S3CAM` / `BOARD_WROOM32`) plus a runtime Board dropdown in the GUI:

- **ESP-WROOM-32** — the sensor/servo **feed** is Bluetooth Classic SPP
  (always on). Network-independent; keeps the laptop's Wi-Fi free.
- **ESP32-S3-CAM** (GoouuuTech, ESP32-S3-WROOM-1 N16R8, OV3660 camera, dual
  USB-C, microSD, WS2812 on 48) — the S3 silicon has **no Bluetooth Classic**,
  so the feed starts on **USB serial** (the CH343 "COM" USB-C port, same
  newline protocol) and moves onto **Wi-Fi UDP** via `feed:wifi` (GUI "Go
  wireless": telemetry → laptop:5005, commands in on 5006 — the revived
  `WifiLink` path), letting the board run untethered.

On either board, **Wi-Fi station (on demand)** carries high-bandwidth **video**
over UDP: brought up only after the laptop sends Wi-Fi credentials + its own
current IP over the feed link, so a changing laptop DHCP address is handled
every session and nothing is hardcoded. On the S3-CAM the OV3660 streams real
JPEG frames (chunked `vf:` packets); the synthetic `vid:` stream remains the
fallback (WROOM-32 always; S3 if camera init fails).

Files:
- `lifeOs.ino` — firmware (MPU6050 DMP + servos + BT sensor link + Wi-Fi video).
- `gui.py` — PySide6 app: **Connect**, **Monitor** (with embedded 3D
  visualizer), **Servos**, **Hand Model** tabs, plus a bottom-left status bar.
- `CleanInput.py`, `live_charts.py` — packet cleaning / unit conversion + the
  per-sensor `Sparkline` widgets used in the Monitor table.
- `session_log.py` — per-run log file (see **Logging** below); `log/` holds the
  output, one dated folder per day (gitignored).
- `web/` — vendored three.js scene for the Monitor tab's 3D view.
- `reciever.py` — legacy terminal receiver (old Wi-Fi/UDP sensor mode).
- `bt_receiver.py` — terminal receiver for the Bluetooth sensor link.
- `wifi_video_test.py` — provisions over BT and measures the Wi-Fi video stream.
- `designDecisions.md` — rationale for the architecture.
- `progress.md` — changelog: every change with description, reason, timestamp,
  and a pushed-to-GitHub flag.
- `to-do.md` — living task list.
- `fixes.md` — production backlog: diagnosed-but-deferred issues, with the
  debugging already done and candidate fixes.

## Wire protocol

**Sensor telemetry (Bluetooth), one newline-delimited line per sample (~50 Hz):**

```
q0:<f>,q1:<f>,q2:<f>,q3:<f>,ax:<i>,ay:<i>,az:<i>,gx:<i>,gy:<i>,gz:<i>,tp:<i>,s0:<i>,s1:<i>,e0:<i>,e1:<i>,dmp:<i>,wf:<i>,rs:<i>
```

- `q0..q3` — fused DMP quaternion (w, x, y, z), floats.
- `ax..gz` — raw accel/gyro counts read from the sensor's **data registers**
  (converted on the PC). Never read these from the DMP FIFO: its "accel" bytes
  are a DMP-internal filtered quantity that only equals gravity when flat and
  collapses toward zero when rotated (measured ~0.03g total while inverted).
- `tp` — raw die-temperature counts (°C = raw/340 + 36.53, converted on the
  PC). Optional: the GUI tolerates firmware that doesn't send it.
- `s0..s1` — servo angles the ESP32 holds (echoed back).
- `e0..e1` — 1 if that servo is enabled (attached); 0 = detached/limp.
- `dmp` — 1 if the DMP is producing orientation (drives the MPU status dot).
- `wf` — 1 if Wi-Fi video is streaming (drives the Wi-Fi status dot).
- `rs` — Wi-Fi RSSI in dBm, sampled at 1 Hz on the ESP32 (0 = radio off /
  not connected). Optional like `tp`; drives the color-coded RSSI readout in
  the status bar (green ≥ −60, amber to −70, red below).

`q*` parse as float, everything else int. Non-telemetry replies (`cal:*`,
`acal:*`, `wifi:*`, `id:*`) arrive on the same stream; `Link._ingest` stashes
them per prefix for `Link.control()`.

**Control lines PC→ESP32 (Bluetooth):**
- `s0:90,s1:45` — servo command.
- `e0:1,e1:0` — enable/disable individual servos (0 detaches the pin → limp;
  re-enable restores the last commanded angle).
- `cal` — re-run the MPU6050 **gyro** bias calibration (hold the sensor still,
  any orientation; replies `cal:start` then `cal:done`; telemetry pauses ~2 s).
  Accel offsets are never touched: the library's `CalibrateAccel()` is broken
  (Electronic Cats v1.4.4 drives flat Z to 2g instead of 1g — measured; the
  resulting +1g Z offset bias is what made DMP roll/pitch decay toward wrong
  angles). Accel bias comes only from `acal:set` or factory trim; boot no
  longer runs `CalibrateAccel` either.
- `acal:set,<bx>,<by>,<bz>` — six-position accel calibration: per-axis bias in
  raw ±2g counts (solved by the GUI wizard). Converted to offset-register units
  (÷8, bit 0 of each register preserved — factory temp-compensation bit),
  written to the MPU so the DMP fuses against true gravity, and persisted in
  ESP32 NVS (re-applied every boot, which then skips `CalibrateAccel` and runs
  `CalibrateGyro` only). Replies `acal:ok,<ox>,<oy>,<oz>` (register values) or
  `acal:error,...`.
- `acal:clear` — drop the stored offsets (reply `acal:cleared`); next boot
  reverts to the automatic flat calibration.
- `id?` → ESP32 replies `id:lifeos,proto:1,servos:2,board:<s3cam|wroom32>`
  (used to auto-detect the lifeOs COM port).
- `wifi:<ssid>|<password>|<laptop_ip>` — provision Wi-Fi; the ESP32 joins that
  network and streams video UDP to `<laptop_ip>:5010`. Replies `wifi:connecting`
  then `wifi:connected,<esp_ip>` over the feed link.
- `wifi:off` — active video/Wi-Fi teardown: stops the stream and turns the
  radio off until re-provisioned. Replies `wifi:off,ok`. On the S3 this also
  drops a Wi-Fi feed back to USB.
- `feed:wifi` / `feed:usb` — **S3 only**: move the sensor/servo feed onto Wi-Fi
  UDP (requires provisioned Wi-Fi; replies `feed:wifi,ok` or
  `feed:error,no-wifi`) or back to USB serial (`feed:usb,ok`). While the feed
  is Wi-Fi, telemetry goes to `<laptop_ip>:5005` (`UDP_PORT`) and a command
  socket on 5006 (`CMD_PORT`) accepts `hello` (re-learn the laptop IP — sent by
  `WifiLink.register_peer`) plus all the control lines above. The WROOM-32
  replies `feed:error,unsupported`.
- `debug` — toggle telemetry echo + 1 Hz status on the USB serial monitor
  (handled as a normal control line; on the WROOM-32 also accepted directly on
  the USB monitor).

**Video (Wi-Fi UDP → port 5010):** on the S3-CAM, OV3660 JPEG frames (VGA,
quality 12, ~10 fps, sensor-compressed) as chunks
`vf:<frame_id>:<chunk_idx>/<chunk_count>:<binary JPEG bytes>` (~1200-byte
payloads); the receiver reassembles by frame id, latest frame wins, incomplete
frames dropped (designDecisions §18). Camera init failure → `cam:error,<code>`
once + fallback to the synthetic `vid:<seq>:`-prefixed 1 KB packets (which the
camera-less WROOM-32 always sends); success replies `cam:ok`. Camera pins are
the stock ESP32S3-EYE map.

## GUI

- **Connect tab** — a **Board dropdown** at the top (`ESP-WROOM-32 (Bluetooth)`
  / `ESP32-S3-CAM (USB + Wi-Fi)`, persisted immediately in
  `connect_config.json` under `board`) retitles the two sections for the
  selected flow:
  - *Bluetooth / USB (sensor & servos)*: COM port list, **Find lifeOs port**
    (auto-detects by sending `id?`/reading telemetry — works identically for a
    paired-BT COM port and the S3's CH343 USB COM port), Connect, Disconnect,
    Save as default. On the S3, Connect also sends `feed:usb` to reclaim the
    feed if it was wireless.
  - *Wi-Fi (video; on the S3 also the wireless feed)*: SSID / password /
    laptop-IP fields (IP auto-filled), Connect (sends creds over the active
    serial link), Disconnect (sends `wifi:off`), **Go wireless** (S3 only:
    sends `feed:wifi`, then swaps the active link for a `WifiLink` aimed at
    the ESP32's IP from the `wifi:connected` reply — USB can then be
    unplugged), Save as default.
  - Saved defaults auto-connect the serial link and auto-provision video on
    launch.
- **Monitor / Servos** — read through a `ConnectionManager` and are
  transport-agnostic.
  - Monitor's table rows are interleaved per axis — ax, gx, ay, gy, az, gz,
    temp — with a 4th **History** column: one autoscaled `Sparkline` per row
    (last 100 samples; x red, y green, z blue, temp yellow) instead of separate
    accel/gyro charts. Temperature is display-only (never zeroed by
    Reset / Recalibrate — see `CALIB_SENSORS` vs `SENSORS`). Below it, a second
    table shows roll/pitch/yaw **relative to the zero pose** (captured when
    Reset / Recalibrate locks in; before that, relative to the DMP startup
    pose) plus **Delta Temp**, the die-temperature change since the first
    reading of the session (recalibrate once it flattens). Its third **Flip**
    column has a checkbox per row that negates that row's value (mounting
    orientation); the roll/pitch/yaw flips also mirror the 3D view, and the
    set persists in `calibration.json` (`value_flips`).
  - Monitor's **Reset / Recalibrate** sends `cal` (device bias recalibration),
    waits for `cal:done` (timeout `DEVICE_CAL_TIMEOUT`), then averages
    `CALIB_SAMPLES` packets into a receiver-side zero for the raw counts.
  - Monitor's **6-Point Calibration…** opens `SixPointCalDialog`: it walks the
    six faces in a fixed order, prompting one at a time — the user places that
    face up, presses **Calibrate** (enabled only while the prompted face is
    detected up and still, `SIXCAL_*` constants), and readings are averaged
    for `SIXCAL_CAPTURE_SECS` (10 s; movement aborts that face for a retry)
    before the next face is prompted — then it solves per-axis accel
    bias `(r₊+r₋)/2` and scale `(r₊−r₋)/(2·16384)`, sends the bias via
    `acal:set`, and saves the scale to `calibration.json` (gitignored), which
    `CleanInput.set_accel_scale` applies to converted g values at startup.
    Fixes the roll/pitch decay-to-wrong-angle bug (tilted `CalibrateAccel`
    corrupting offsets → DMP pulled toward a biased gravity vector).
  - The 3D view (`VisualizerPanel`, embedded in Monitor below the relative-state
    table) de-drifts yaw: while the raw counts say the sensor is still, yaw
    change is treated as drift, frozen out, and the creep rate is learned (and
    subtracted during motion). Its **Zero** button cancels heading only
    (swing-twist), keeping gravity-true roll/pitch.
  - Servos tab has a per-servo **Enabled/Disabled** toggle (`e<i>:0/1`, detach =
    limp) next to the slider; dial labels show "(off)" from the echoed `e*`.
- **Hand Model tab** — source dropdown: **"ESP32 camera (Wi-Fi UDP :5010)"**
  (always row 0; `VideoReceiver` reassembles the `vf:` JPEG chunks and a QLabel
  paints the latest frame at ≤15 Hz, with an fps/KB status line that also says
  when only the synthetic stream is arriving) plus the PC's video input devices
  (QtMultimedia `QMediaDevices`; Apply streams the picked camera via `QCamera`
  → `QMediaCaptureSession` → `QVideoWidget`). The ESP32 source works without
  QtMultimedia. Planned: hand tracking on the selected source.
- **Status bar (bottom-left)** — colored `StatusDot` boxes (BT blue, Wi-Fi green,
  Servo1/Servo2 red, MPU yellow) showing ✓/✗ by whether that source had a live
  signal in the last second (all derived from the fresh BT telemetry line).
- **Window sizing** — launches maximized (normal frame, title-bar buttons
  visible); "restore down" always lands on `RESTORED_SIZE` clamped to the
  current monitor and centered (`changeEvent` override), never Qt's remembered
  geometry — guards against the off-screen-title-bar glitch.

## Logging (read this first when a value misbehaves)

Every GUI run writes `log/<YYYY-MM-DD>/log-<YYYY-MM-DD>_<HH-MM-SS>.txt`
(`session_log.py`, gitignored). It is the artifact to read instead of asking the
user to reproduce a bug live — the serial-monitor panel hides telemetry, holds
only 2000 lines, and stops draining while hidden.

Format `HH:MM:SS.mmm  TAG  VIA  payload`, chronological, one line per event:

| Tag | Meaning |
| --- | --- |
| `RX` | telemetry accepted by `_ingest` (verbatim, so `parse_line()` reads it back) |
| `CTL` | control reply (`cal:`, `acal:`, `wifi:`, `id:`, `feed:`) |
| `RAW` | other board output (boot banner, `cam_hal:` prints) |
| `DROP` | line **rejected** by `_ingest`, with the failing check: `fragment` (missing keys) / `bad-quat` / `bad-counts` |
| `TX` | line sent to the board (Wi-Fi password redacted; `<< SEND FAILED` / `<< no link` on failure) |
| `PC` | GUI event: button press, link swap, zero locked in, flips changed |
| `STATE` | 1 Hz visualizer de-drift internals (`rpy`, `yaw_off`, `drift`, `still`, `glitches`) + a line whenever the glitch guard trips |
| `VID` | 1 Hz video health (size, fps, KB/frame, frames abandoned mid-reassembly) |
| `OUT` / `ERR` | stdout / stderr, uncaught tracebacks (any thread), Qt messages |

The header records the git SHA, board, baud, laptop IP, `connect_config.json`
(password redacted) and `calibration.json` — a wrong zero or stale scale factor
looks exactly like a bad sensor otherwise. The footer gives per-tag counts and
the drop rate. Logging is hooked at the **source** (`Link._ingest`,
`Link.send_line`, `ConnectionManager.set_active`, `ui_log`), never at the panel;
`session_log.log()` is a no-op until `start()` (called from `main()`), so
`smoke_gui.py` writes nothing. designDecisions §21.

## Code structure (gui.py)

`Link` (base) → `BluetoothLink` (pyserial COM — used for both the WROOM-32's
paired-BT COM port and the S3's USB COM port) and `WifiLink` (UDP telemetry in
on 5005, control lines out to 5006 — the S3's wireless feed after Go wireless).
`ConnectionManager` holds the active link and proxies `snapshot()/
is_connected()/send_servos()/send_line()/description()`; switching links stops
the old one. `PortProber` runs COM probing off the GUI thread. `DeviceDiscovery`
(mDNS) is retained but unused (the ESP32's IP now comes from the
`wifi:connected` reply).

## GPIO / wiring (per board, set by the `BOARD_*` #define)

| Wire | ESP-WROOM-32 | ESP32-S3-CAM |
| --- | --- | --- |
| MPU6050 SDA (`PIN_SDA`) | GPIO21 | GPIO21 |
| MPU6050 SCL (`PIN_SCL`) | GPIO22 | GPIO14 |
| MPU6050 INT (`INTERRUPT_PIN`) | GPIO4 | GPIO47 |
| MPU6050 VCC / GND | 3V3 / GND | 3V3 / GND |
| MPU6050 AD0 | GND (address `0x68`) | GND (address `0x68`) |
| Servo 0 signal (`SERVO_PINS[0]`) | GPIO13 | GPIO1 |
| Servo 1 signal (`SERVO_PINS[1]`) | GPIO25 | GPIO2 |

The S3-CAM's only free GPIOs are 1, 2, 3, 14, 21, 47 (camera 4–13/15–18, SD
38–40, WS2812 LED 48, native USB 19/20, strapping 0/45/46); GPIO3 is the spare.
`Wire.begin(PIN_SDA, PIN_SCL)` is explicit — the S3's Arduino I2C defaults
(SDA 8 / SCL 9) are camera data pins.

Power servos from an **external 5–6V supply** (not the ESP32), ground tied to the
ESP32 ground; add a 470–1000 µF bulk cap across the servo rail. If the MPU reads
`MPU6050 connection FAILED`, it's almost always an I2C wiring/power issue (re-seat
SDA/SCL/VCC/GND, confirm AD0→GND).

## Critical configuration

- Ports: `VIDEO_PORT` 5010 (Wi-Fi video); `UDP_PORT` 5005 / `CMD_PORT` 5006
  carry the S3's wireless sensor/servo feed (`feed:wifi` mode; same ports the
  legacy Wi-Fi sensor path used). Constants must match between `lifeOs.ino`
  and `gui.py`.
- `NUM_SERVOS` / `SERVO_PINS` (firmware) must match `NUM_SERVOS` / `SERVO_GPIOS`
  (gui.py).
- Wi-Fi creds are provided at runtime over Bluetooth; nothing is in `secrets.h`
  anymore (the firmware no longer includes it).

## Build / run

**The ESP32-S3-CAM is the only actively developed board** (user decision
2026-07-08): do not modify or compile-verify the `BOARD_WROOM32` variant
anymore — its code stays as frozen legacy. Keep `#define BOARD_S3CAM` active.

**Firmware:** pick the board at the top of `lifeOs.ino` (`#define BOARD_S3CAM`
or `BOARD_WROOM32`), then Arduino IDE / arduino-cli with the ESP32 board
package. Libraries: "MPU6050" by Electronic Cats + "ESP32Servo" (WiFi/WiFiUdp/
BluetoothSerial ship with the core). Board settings:
- **WROOM-32:** board "ESP32 Dev Module" (`esp32:esp32:esp32`), **Partition
  Scheme → "Huge APP"** — Bluetooth + Wi-Fi + the DMP blob overflow the
  default. BT+Wi-Fi coexistence shares one radio, so Wi-Fi throughput drops.
- **S3-CAM:** board "ESP32S3 Dev Module"
  (`esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi`); the default 16 MB partition
  fits (no BT Classic stack). Flash/monitor via the **COM** USB-C port
  (CH343).
Serial at **921600** (`SERIAL_BAUD` in gui.py must match `Serial.begin` in the
firmware); keep the sensor still ~2 s on boot (DMP calibration).

**PC:**
```
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt   # PySide6 6.8.0.2, zeroconf, pyserial
python gui.py
```
Typical flow: Connect tab → pick the Board → (pair BT / plug in USB) → Find
lifeOs port → Connect; then Wi-Fi section → SSID/password → Connect; on the S3,
**Go wireless** once `wifi:connected` shows, then unplug USB. Verify video with
`python wifi_video_test.py [--com COMx --ssid X --password Y]` (allow inbound
UDP 5010 in the firewall; the wireless feed needs inbound UDP 5005 too).

## Documentation upkeep (required with every change)

- **`designDecisions.md`** — whenever a design decision is locked in (behavior,
  UX flow, persistence location, protocol, sizing policy, ...), append a
  numbered **Decision / Why** entry (newest at the bottom), including what it
  supersedes. Settled questions must not be relitigated from scratch.
- **`progress.md`** — append an entry for every change: a short **What**, a
  one-line **Why**, a timestamp heading, and a **Pushed** flag. Use
  `no (uncommitted)` until the work lands on GitHub, then flip it to
  `yes (<sha>)`.
- **`to-do.md`** — keep current: add items as they come up, check them off
  (with a date) when done.
- **`fixes.md`** — the production backlog: issues diagnosed but deliberately
  deferred. Each entry: symptom, root cause as established, **all debugging
  already done** (so it is never redone from scratch), the current
  workaround, and candidate production fixes. Entries are added only on the
  user's decision (see below); when one is eventually fixed, annotate it
  (date + progress.md entry) instead of deleting it.

## Staying on the main goal (defer-or-fix triage)

When a bug or limitation turns out to be an environment/robustness issue
rather than the feature being built (network policy quirks, firewall/OS
config, hardware corner cases, "works here but not there" compatibility) —
or when fixing it properly would clearly derail the current goal — **don't
silently keep fighting it.** Say so: give a short description of the issue,
what was already tried, the workaround available now, and a recommendation.
**The user then decides** whether to fix it now or log it to `fixes.md` for
later. Precedent: the 2026-07-08 wireless-feed dropout (fixes.md §1) burned a
session on inter-VLAN UDP filtering that a same-Wi-Fi workaround sidestepped.

## Git conventions

- **Never add a `Co-Authored-By: Claude` trailer** (or any co-author tag) to
  commit messages when committing or pushing — commits must show only the
  user. This is an explicit standing instruction from the user.
- After pushing, flip the affected progress.md entries to `yes (<sha>)`.

## Notes

- Yaw drifts (no magnetometer); roll/pitch stable. Mitigations: the 3D view's
  stillness-based yaw de-drift, its heading-only Zero button, and Monitor's
  Reset/Recalibrate (`cal`). Only a magnetometer would eliminate it.
- The saved Wi-Fi password lives in `connect_config.json` (gitignored, plaintext).
- PC tools target Windows (`netstat`/`tasklist`; `reciever.py` uses `msvcrt`).
- PySide6 is pinned to 6.8.0.2 (newer needs an MSVC runtime some Python builds
  lack).
