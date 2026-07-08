# Progress

Running log of every change made to lifeOs: what was done, why, when, and
whether that work has been pushed to GitHub. Chronological, newest at the
bottom (same convention as designDecisions.md). Append an entry for each
change; when the work lands on GitHub, flip its flag to `yes (<sha>)`.
Entries up to 2026-07-07 were reconstructed from git history, CLAUDE.md,
designDecisions.md, and to-do.md.

---

## 2026-06-29 17:16 — Initial commit: MPU6050 → ESP32 → UDP telemetry
- **What:** First working pipeline: firmware reads the MPU6050 and streams
  telemetry over Wi-Fi UDP; PC side has a terminal receiver (`reciever.py`)
  and the first GUI.
- **Why:** Get sensor data off the device and onto the PC end-to-end before
  building anything fancier.
- **Pushed:** yes (161d162)

## 2026-06-29 21:27 — 3D Visualizer tab + DMP quaternions
- **What:** Firmware switched from raw-only readings to the MPU6050's on-chip
  DMP fused quaternion; GUI gained a three.js 3D orientation view (vendored in
  `web/`, fed over QWebChannel).
- **Why:** Hardware sensor fusion beats PC-side filter tuning (designDecisions
  §1), and a 3D view makes orientation problems visible at a glance.
- **Pushed:** yes (10e6d51)

## 2026-07-01 22:38 — Servos, two-way link, mDNS, swappable transport
- **What:** Servo command + echo protocol (`s0/s1`), the ESP32 learns the PC's
  IP at runtime (`hello`), mDNS discovery (`lifeos.local`), and the `Link`
  abstraction seam on both ends.
- **Why:** Hardcoded IPs broke on every DHCP change (§4); echoed servo angles
  show true device state (§5); the transport seam lets media change without
  touching the protocol (§7).
- **Pushed:** yes (a318ce4)

## 2026-07-01 23:07 — Bluetooth/Wi-Fi transport selector; design docs
- **What:** Bluetooth Classic SPP added as a selectable transport in the
  Connect tab; README and designDecisions.md refreshed.
- **Why:** The apartment Wi-Fi's client isolation blocked peer-to-peer UDP —
  Bluetooth is network-independent and keeps the laptop's Wi-Fi free (§8).
- **Pushed:** yes (a7b80dd)

## 2026-07-02 00:06 — Concurrent BT (sensor/servos) + Wi-Fi (video)
- **What:** The ESP32 now runs both channels at once: Bluetooth for telemetry,
  control, and Wi-Fi provisioning (`wifi:<ssid>|<pass>|<laptop_ip>`); Wi-Fi
  station brought up on demand for UDP video to port 5010 (synthetic frames);
  Connect tab reworked into two sections (no transport dropdown).
- **Why:** Choose the channel by data type: reliable low-bandwidth link always
  on, fat video pipe only when provisioned — and the laptop's changing DHCP
  address is re-sent each session so nothing is hardcoded (§9).
- **Pushed:** yes (5456da5)

## 2026-07-02 00:30 — Status-bar connection indicators
- **What:** Bottom-left `StatusDot` boxes (BT, Wi-Fi, Servo 1/2, MPU) showing
  ✓/✗ from the freshness of the single BT telemetry line; architecture docs
  updated.
- **Why:** One always-on stream already carries every source's health (`dmp`,
  `wf`, `s0/s1`), so a single "fresh within 1 s?" check surfaces the whole
  system's state honestly (§12).
- **Pushed:** yes (de55545)

## 2026-07-02 03:20 — Hand Model tab (webcam preview)
- **What:** New tab listing the PC's video input devices (QtMultimedia) with a
  live preview of the selected camera. Developed on the `hand-detection`
  branch, merged to main via PR #1 (2777d13).
- **Why:** Groundwork for hand tracking; the ESP32's Wi-Fi video stream is
  planned as another selectable source.
- **Pushed:** yes (d889d4b, merged in 2777d13)

## 2026-07-02 04:37 — Device recal, yaw de-drift, servo enables, Monitor rework
- **What:** `cal` command re-runs on-device bias calibration; the 3D view
  freezes out yaw drift while the sensor is still and learns the creep rate;
  per-servo enable/disable (`e0/e1`, detach = limp); Disconnect actively tears
  links down; Monitor table interleaved per axis with `Sparkline` history.
- **Why:** Yaw has no absolute reference (no magnetometer) so drift needs
  mitigation; recalibrating at the source beats display-only zeroing; limp
  servos and real disconnects make the rig safer to handle.
- **Pushed:** yes (319e917)

## 2026-07-02 05:16 — 3D visualizer into Monitor tab; full-width tab bar
- **What:** The visualizer stopped being its own tab and moved below Monitor's
  relative-state table; custom `StretchTabBar` spans the window width.
- **Why:** Orientation, raw counts, and calibration controls belong on one
  screen; even tab widths read better as the app grows (§11).
- **Pushed:** yes (c1d00a4)

## 2026-07-02 05:29 — Fix runaway window growth
- **What:** Stopped the full-width tab bar from re-expanding the window every
  layout pass.
- **Why:** The tab bar's size hint fed back into the window size — the window
  grew without bound.
- **Pushed:** yes (606ae22)

## 2026-07-02 06:12 — Add to-do.md
- **What:** Living task list: calibration roadmap and settings-menu plan.
- **Why:** Keep planned work and its ordering visible in the repo instead of
  in chat history.
- **Pushed:** yes (290a98f)

## 2026-07-04 23:55 — Add gui_functioning.md
- **What:** Walkthrough of gui.py's transports, tabs, and threading.
- **Why:** Orientation doc for readers of the largest file in the repo.
- **Pushed:** yes (600ec2d)

## 2026-07-05 — Six-position accel calibration (wizard + firmware fixes)
- **What:** Monitor's "6-Point Calibration…" wizard (v1: auto-detected each
  face-up pose) solving per-axis accel bias and scale; bias sent via
  `acal:set` to the MPU's hardware offset registers and persisted in ESP32
  NVS; scale saved to `calibration.json` for PC-side correction; `cal` became
  gyro-only. Firmware fixes found by USB debugging: telemetry `ax..az` now
  read from the data registers (`getMotion6`) instead of the DMP FIFO, and the
  library's broken `CalibrateAccel()` (drives flat Z to 2g) removed from boot.
- **Why:** A tilted/broken auto accel calibration planted a +1g Z offset — the
  root cause of roll/pitch decaying toward wrong angles; six-position
  calibration measures true ±1g per axis with no flatness assumption (§13
  context, to-do.md notes).
- **Pushed:** no (uncommitted)

## 2026-07-07 — 6-point calibration: explicit step-per-face flow
- **What:** The wizard now prompts one face at a time in a fixed order; the
  user places the sensor, presses **Calibrate** (enabled only while the
  prompted face is detected up and still), and readings average for 10 s
  before the next face is prompted; movement mid-window aborts that face.
- **Why:** v1's auto-detect made each step implicit — the user couldn't tell
  which face was being measured or when; explicit steps also average 10× more
  samples per face (§13).
- **Pushed:** no (uncommitted)

## 2026-07-07 — Flip column in the relative-state table
- **What:** Third column of Monitor's second table: a checkbox per row (Roll /
  Pitch / Yaw / Delta Temp) that negates that row's displayed value; the angle
  flips also mirror the 3D visualizer; the set persists in `calibration.json`
  (`value_flips`).
- **Why:** Mounting orientation decides each axis's sign — an inverted mount
  reads "tilt up" as negative; a display-side flip fixes it without touching
  firmware (§14).
- **Pushed:** no (uncommitted)

## 2026-07-07 — Launch maximized; standard restore size
- **What:** The GUI opens maximized (normal frame, title-bar buttons visible);
  "restore down" always lands on 720×860 clamped to the current monitor and
  centered, ignoring Qt's remembered geometry.
- **Why:** The window sometimes opened with glitched oversized geometry, hiding
  the minimize/restore/close buttons and the drag area off-screen (§15).
- **Pushed:** no (uncommitted)

## 2026-07-07 — Documentation upkeep rules + this progress log
- **What:** CLAUDE.md now mandates: designDecisions.md entry whenever a
  decision is locked in, and a progress.md entry (what/why/timestamp/pushed
  flag) for every change; progress.md created and back-filled from git
  history, CLAUDE.md, designDecisions.md, and to-do.md.
- **Why:** Several changes had landed without designDecisions.md entries; a
  single changelog with push status shows at a glance what exists and what has
  reached GitHub.
- **Pushed:** no (uncommitted)

## 2026-07-07 — Git convention: no Claude co-author trailer
- **What:** CLAUDE.md now instructs that commits made through Claude must
  never carry a `Co-Authored-By: Claude` trailer, and that progress.md pushed
  flags get flipped after each push.
- **Why:** GitHub renders the trailer as a co-author badge; the user wants
  commits to show only them.
- **Pushed:** no (uncommitted)
