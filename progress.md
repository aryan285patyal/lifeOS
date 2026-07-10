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
- **Pushed:** yes (92fde26)

## 2026-07-07 — 6-point calibration: explicit step-per-face flow
- **What:** The wizard now prompts one face at a time in a fixed order; the
  user places the sensor, presses **Calibrate** (enabled only while the
  prompted face is detected up and still), and readings average for 10 s
  before the next face is prompted; movement mid-window aborts that face.
- **Why:** v1's auto-detect made each step implicit — the user couldn't tell
  which face was being measured or when; explicit steps also average 10× more
  samples per face (§13).
- **Pushed:** yes (92fde26)

## 2026-07-07 — Flip column in the relative-state table
- **What:** Third column of Monitor's second table: a checkbox per row (Roll /
  Pitch / Yaw / Delta Temp) that negates that row's displayed value; the angle
  flips also mirror the 3D visualizer; the set persists in `calibration.json`
  (`value_flips`).
- **Why:** Mounting orientation decides each axis's sign — an inverted mount
  reads "tilt up" as negative; a display-side flip fixes it without touching
  firmware (§14).
- **Pushed:** yes (92fde26)

## 2026-07-07 — Launch maximized; standard restore size
- **What:** The GUI opens maximized (normal frame, title-bar buttons visible);
  "restore down" always lands on 720×860 clamped to the current monitor and
  centered, ignoring Qt's remembered geometry.
- **Why:** The window sometimes opened with glitched oversized geometry, hiding
  the minimize/restore/close buttons and the drag area off-screen (§15).
- **Pushed:** yes (92fde26)

## 2026-07-07 — Documentation upkeep rules + this progress log
- **What:** CLAUDE.md now mandates: designDecisions.md entry whenever a
  decision is locked in, and a progress.md entry (what/why/timestamp/pushed
  flag) for every change; progress.md created and back-filled from git
  history, CLAUDE.md, designDecisions.md, and to-do.md.
- **Why:** Several changes had landed without designDecisions.md entries; a
  single changelog with push status shows at a glance what exists and what has
  reached GitHub.
- **Pushed:** yes (92fde26)

## 2026-07-07 — Git convention: no Claude co-author trailer
- **What:** CLAUDE.md now instructs that commits made through Claude must
  never carry a `Co-Authored-By: Claude` trailer, and that progress.md pushed
  flags get flipped after each push.
- **Why:** GitHub renders the trailer as a co-author badge; the user wants
  commits to show only them.
- **Pushed:** yes (92fde26)

## 2026-07-07 — ESP32-S3-CAM support: board switch, USB pairing, Go wireless
- **What:** Firmware gained a compile-time board switch (`BOARD_S3CAM` /
  `BOARD_WROOM32`: per-board pins + feed transport behind a `linkSendLine`
  seam). On the S3 (no BT Classic) the sensor/servo feed starts on USB serial
  and `feed:wifi` moves it onto Wi-Fi UDP (telemetry → laptop:5005, commands
  on 5006, `hello` re-teaches the IP). GUI: Board dropdown in the Connect tab
  (persisted), per-board section labels, and a **Go wireless** button that
  swaps the serial link for the revived `WifiLink` (which got `send_line`).
  New S3 wiring: SDA 21, SCL 14, INT 47, servos 1/2. README rewritten for both
  boards; CLAUDE.md/designDecisions §16 updated. Both firmware variants
  compile (S3: 16MB/OPI fits default partition; WROOM-32: huge_app).
- **Why:** The new board's camera is the video source the project has been
  stubbing, but its silicon drops Bluetooth Classic and its camera/SD/USB pins
  leave only six free GPIOs — so the link, the pins, and the connect flow all
  had to change together (designDecisions §16).
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Board-aware link naming: no more "Bluetooth COMx" on the S3's USB
- **What:** `BluetoothLink` (gui.py) now takes a `label` ("Bluetooth" /
  "USB") that `description()` uses, and `ConnectTab._connect_bt` sets it from
  the selected board via a new `_link_name()` helper — so the status ticker
  reads "Connected - USB COMx" on the S3 instead of "Connected - Bluetooth
  COMx". Disconnect, save-default, and credential-failure messages use the
  same board-aware wording (the `connect_video` inline pattern was deduped
  into the helper).
- **Why:** The S3's CH343 USB COM port rides the same pyserial link class as
  the WROOM-32's paired-BT COM port, and the hardcoded "Bluetooth" wording
  misnamed the transport whenever the S3 board was selected (user-reported).
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — "Find lifeOs port": scanning arrow on the port being probed
- **What:** While the prober walks the COM ports, the list row currently being
  tried gets a "   <- scanning..." marker (same style as "<- lifeOs").
  Plumbing: `probe_lifeos_port` gained an optional per-port `progress`
  callback, `PortProber` a `probing = Signal(str)` emitted through it, and
  `ConnectTab._on_probe_progress` repopulates the list with the marker
  (cleared again when the probe finishes, found or not).
- **Why:** Probing opens each port for up to ~1.5 s; the only feedback was the
  button saying "Probing...", so the user couldn't tell which port was under
  scan or how far along the sweep was.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Serial-monitor panel: persistent checkbox + terminal on all tabs
- **What:** New "Serial monitor" checkbox on the Connect tab (persisted in
  `connect_config.json` under `serial_monitor`) toggles a terminal-style
  `SerialMonitorPanel` in the bottom ~quarter of the window, below the tab
  widget so it shows on every tab. It displays every line received on the
  active feed link (per-link `raw_log` ring buffer filled in `Link._ingest`,
  polled at 10 Hz via `raw_since`; divider line on link swaps), has a
  "Hide telemetry" filter for the 50 Hz `q0:` lines, and a command input +
  Send that writes raw lines to the ESP32 over the active link.
- **Why:** Go wireless reports success but telemetry dies after unplugging
  USB; there was no way to watch the ESP32's replies or poke it with raw
  commands from the GUI. designDecisions.md §17.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Serial monitor pinned to 1/5 of the window height
- **What:** The panel's height is now set to `window height // 5` in a
  `MonitorWindow.resizeEvent` override (layout stretch changed to tabs-take-
  the-rest), so the proportion holds across resize/maximize/restore.
- **Why:** Layout stretch factors only split leftover space after size hints,
  so the "bottom quarter" drifted with tab content; the user wants an exact,
  window-proportional 1/5. designDecisions.md §17 updated.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Serial monitor: "Hide telemetry" defaults on and persists
- **What:** The panel's Hide-telemetry checkbox now starts checked and its
  state is stored in `connect_config.json` (`hide_telemetry`). The panel
  shares the ConnectTab's config dict (passed into the constructor) so the
  two writers never save stale copies of each other's keys.
- **Why:** The panel exists mainly to read control replies; the 50 Hz
  telemetry firehose should be opt-in, and the choice should survive GUI
  restarts. designDecisions.md §17 updated.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Serial monitor: [PC]/[ESP] origin tags + button-press logging
- **What:** Terminal lines are now tagged by origin - `[ESP]` for lines
  received from the board, `[PC]` for GUI-side events. Every QPushButton
  press in the main window is logged as a `[PC]` line via a module-level
  `ui_log` ring buffer and one generic hook (`_hook_button_logging`), stating
  whether the button is a laptop-side action or commands the ESP32 (an `esp`
  dynamic property set on Wi-Fi Connect/Disconnect, Go wireless,
  Reset/Recalibrate, servo Enabled toggles, Upload to ESP32, and Send).
- **Why:** While debugging the wireless-feed dropout, the terminal should
  read as a single timeline of what the user did and what the board answered,
  with the actor of each line unambiguous.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Serial monitor: transport tags ([USB] / [BT] / [WiFi])
- **What:** Link traffic in the terminal now carries a second tag naming the
  transport it rode: `[ESP][USB]`, `[ESP][BT]`, `[ESP][WiFi]` on received
  lines and `[PC][<tag>] > cmd` on sends, via a new `Link.transport_tag()`
  (BluetoothLink: "USB"/"BT" from its board-aware label; WifiLink: "WiFi").
- **Why:** The wireless-feed debugging hinges on which pipe a line traveled -
  after Go wireless the tag flipping from [USB] to [WiFi] (or not) is the
  evidence.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Wireless-feed dropout diagnosed: Windows Firewall; hint in GUI
- **What:** Root-caused "Go wireless reports success, then no data after USB
  unplug": the ESP32 joins Wi-Fi fine (wifi:connected,172.16.72.41) and the
  laptop's Ethernet subnet (172.17.8.100/20) routes to the ESP's Wi-Fi subnet
  (ping OK both ways, TTL 61) — but the Ethernet connection is profiled
  Public and only anaconda's python.exe has inbound firewall allow rules, so
  the venv-run GUI never receives the ESP's UDP 5005/5010. Added a persistent
  firewall hint (selectable text, both netsh add-rule commands for
  UDP 5005/5010) to the Connect tab's Wi-Fi section.
- **Why:** Any user hitting "wifi:connected but no feed" should find the
  one-time admin-terminal fix right where they're looking; the agent cannot
  add firewall rules itself (permission/elevation).
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Wireless dropout, layer 2: campus network drops inter-VLAN UDP
- **What:** With the lifeOs firewall rules confirmed active (Get-NetFirewallRule),
  an 8 s listener on UDP 5010 received zero packets while the ESP32 pinged
  fine (TTL 61) and claimed to be streaming video - so the Apollo-Resident
  network's inter-VLAN policy passes ICMP but drops unsolicited UDP between
  the Wi-Fi VLAN (172.16.x, ESP) and the wired VLAN (172.17.x, laptop).
  Remedy: laptop on the same Wi-Fi as the ESP. Also fixed the reader-thread
  crash on link swap: `BluetoothLink._run` now catches all exceptions from
  `readline()` - pyserial raises TypeError (not SerialException) when stop()
  closes the port mid-read during Go wireless.
- **Why:** Root-causes "wifi:connected but no feed" beyond the firewall layer;
  the traceback the user hit at every link swap was noise masking real errors.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — fixes.md production backlog + defer-or-fix triage rule
- **What:** New `fixes.md` at the repo root: diagnosed-but-deferred issues,
  each with symptom, root cause, all debugging already done, workaround, and
  candidate production fixes. Entry 1 is the wireless-feed dropout
  (firewall layer fixed; inter-VLAN UDP filtering deferred - workaround:
  laptop on the same Wi-Fi). CLAUDE.md now lists fixes.md in the doc-upkeep
  rules and adds a "Staying on the main goal" section: when an issue is an
  environment/robustness rabbit hole, Claude surfaces it with findings +
  workaround and the user decides fix-now vs. log-to-fixes.md. to-do.md's
  go-wireless item closed as deferred.
- **Why:** Keep sessions pointed at the main goal instead of bug fights,
  without losing the diagnosis work for a future production-hardening pass.
- **Pushed:** yes (3c0cf38)

## 2026-07-08 — Wi-Fi RSSI in telemetry + status-bar readout
- **What:** Firmware samples `WiFi.RSSI()` at 1 Hz (off the 50 Hz hot path)
  and appends `rs:<dBm>` to the telemetry line (0 = radio off); the `debug`
  1 Hz status prints it too. GUI status bar gains an RSSI readout next to the
  dots, color-coded green (>= -60 dBm), amber (to -70), red (below), "--"
  when absent - old firmware without the field just shows "--" (optional
  field, same pattern as `tp`). Both firmware variants compile-verified
  (S3 73% flash, WROOM-32 huge_app 53%). CLAUDE.md/README protocol updated.
- **Why:** Signal strength tells radio problems apart from network problems
  for the wireless feed, and gives the measurement needed for the pending
  IPEX-vs-PCB antenna check (to-do).
- **Pushed:** yes (0ddd169)

## 2026-07-08 — Antenna check done: PCB antenna active; rework deferred
- **What:** Using the new RSSI readout, the touch/detune test confirmed the
  S3-CAM radio is on the onboard PCB antenna (the module's 0 Ohm RF-path
  resistor is not routed to the IPEX socket - the external antenna is
  currently inert). Checked off the RF-path to-do item; the solder rework to
  switch to the external antenna is logged as fixes.md section 2 with the
  verification recipe.
- **Why:** Resolves the open antenna question with a measurement; the rework
  only matters for range/enclosure use, so it is production backlog.
- **Pushed:** yes (0ddd169)

## 2026-07-08 — Real camera video: OV3660 JPEG over UDP + live GUI preview
- **What:** S3-CAM firmware captures OV3660 JPEG (VGA, quality 12, ~10 fps,
  PSRAM frame buffers, lazy init on first stream; ESP32S3-EYE pin map) and
  sends `vf:<frame>:<idx>/<count>:<bytes>` chunks (~1200 B) to laptop:5010;
  `cam:ok`/`cam:error,<code>` reply, synthetic `vid:` fallback kept (and
  WROOM-32 unchanged). GUI: new `VideoReceiver` (latest-wins reassembly,
  loopback-tested byte-exact with simulated loss) and the Hand Model tab now
  has "ESP32 camera (Wi-Fi UDP :5010)" as its first source with a live
  preview + fps/KB status; works without QtMultimedia. wifi_video_test.py
  understands both stream kinds and gained --save. Both firmware variants
  compile (S3 77%). Live check: synthetic stream measured at 50 pkt/s, 0%
  loss now that laptop and ESP share the Wi-Fi (fixes.md §1 workaround).
- **Why:** The whole Wi-Fi path existed to carry real video; this makes the
  camera the payload and gives the Hand Model tab its intended source.
  designDecisions §18.
- **Pushed:** no (uncommitted)

## 2026-07-08 — Camera init: PSRAM detection + QVGA-in-DRAM fallback
- **What:** First flash hit "cam_dma_config: frame buffer malloc failed" -
  the build was flashed without PSRAM enabled, so the VGA frame buffers had
  nowhere to live. cameraInit() now checks psramFound(): without PSRAM it
  retries with QVGA, fb_count 1, CAMERA_FB_IN_DRAM (fits internal RAM), and
  the error path logs psram state + free heap. Boot log says which mode came
  up. Real fix for full VGA: flash with Tools > PSRAM > "OPI PSRAM"
  (arduino-cli FQBN ...:FlashSize=16M,PSRAM=opi).
- **Why:** The camera should degrade to a smaller frame, not die, when the
  flash settings are wrong - and the log should say why.
- **Pushed:** no (uncommitted)

## 2026-07-08 — Camera/MPU core contention fixed; S3 flashed
- **What:** With the camera live, cam_hal FB-OVF spam appeared and the MPU
  went silent: the SDK pins the near-max-priority cam_task AND the Wi-Fi
  stack to core 0, where our priority-3 IMU task starved. IMU task moved to
  core 1 (outranks loop()); camera XCLK 20 -> 10 MHz (sensor fps ~= our
  10 fps send rate, FB-OVF quieted); camera LEDC moved to timer 2/channel 6
  (was clobbering servo 0's channel 0). designDecisions 19 (supersedes the
  core numbers in 2). Both variants compile; S3 flashed over COM7
  (esptool confirmed 8 MB embedded PSRAM).
- **Why:** Camera and MPU must run simultaneously - the camera's core
  pinning is baked into the prebuilt SDK, so our task is the one that moves.
- **Pushed:** no (uncommitted)

## 2026-07-08 — Visualizer/relative-table jumps: spliced telemetry lines fixed
- **What:** With the camera live, the 3D view and the relative-state table
  jumped between huge +/- values while the raw table stayed stable. Cause:
  the cam driver's FB-OVF warnings print from cam_task (core 0) and
  interleave mid-line with telemetry (loop, core 1) on the shared USB
  serial; a fragment cut at a comma boundary still parses as valid key:vals
  and wholesale-replaced Link.latest, so quaternion consumers read missing
  q* keys as zero. Two-sided fix: firmware silences the cam_hal log tag once
  the camera is up (the warnings are just frame-drop notices), and
  Link._ingest now accepts a line as telemetry only if it carries the full
  q0..q3 + ax..gz set (TELEMETRY_KEYS) - fragments can never displace the
  last good sample. S3 re-flashed (COM7).
- **Why:** Two independent writers on one serial line will always interleave
  eventually; the GUI must be robust to torn lines regardless of firmware
  politeness.
- **Pushed:** no (uncommitted)

## 2026-07-08 — Visualizer freakouts + post-unplug yawing fixed; 921600 baud; S3-only
- **What:** Three-layer fix for the remaining orientation glitches:
  (1) Link._ingest now also validates values (unit-norm quaternion,
  int16-range counts) - torn lines that still parse (e.g. "q0:09948" after
  losing a dot) are rejected; (2) the yaw de-drift ignores >45 deg/tick
  steps, gates rate-learning at 5 deg/s, clamps the learned rate at 2 deg/s,
  and _tick freezes with state reset while disconnected (a stale snapshot
  used to keep integrating drift -> endless yawing after unplug); (3) feed
  serial raised 115200 -> 921600 on both sides (SERIAL_BAUD) so telemetry
  drops from ~65% to ~8% line utilization. CLAUDE.md now records the user's
  S3-only decision (WROOM-32 frozen legacy, no more changes/compile checks).
  GUI + S3 firmware compile; flash pending (board was unplugged).
- **Why:** A single corrupt sample poisoned the de-drift integrator and EMA,
  so the view misbehaved long after good data resumed; disconnect exposed
  the open-loop drift integration.
- **Pushed:** no (uncommitted)

## 2026-07-09 17:20 — Per-run session log (`log/<date>/log-<date>_<time>.txt`)
- **What:** New `session_log.py`: every GUI run opens its own timestamped log
  file under `log/<date>/` and records, chronologically, every line received
  from the board (`RX` accepted / `CTL` reply / `RAW` other / `DROP` rejected
  **with the failing check**), every line sent (`TX`, Wi-Fi password redacted),
  GUI events and button presses (`PC`), the visualizer's de-drift state at 1 Hz
  (`STATE`, plus a line each time the glitch guard trips), video-stream health
  (`VID`), and the console — stdout/stderr, uncaught tracebacks from any thread,
  and Qt messages (`OUT`/`ERR`). A header records the git SHA, board, baud,
  laptop IP, `connect_config.json` (redacted) and `calibration.json`; a footer
  summarizes per-tag counts and the drop rate. Hooks live at the source
  (`Link._ingest`, a new `Link.send_line` wrapper over per-transport
  `_write_line`, `ConnectionManager.set_active`, `ui_log`), never in the
  serial-monitor panel. `Link._telemetry_sane` became `_telemetry_fault`,
  returning *why* a line was rejected. `VideoReceiver` now counts frames
  abandoned mid-reassembly. The Connect tab shows the active log path.
  `log/` is gitignored.
- **Why:** Bugs like the visualizer freak-out could only be diagnosed live —
  the panel hides telemetry, keeps 2000 lines, and stops draining when hidden,
  and `_ingest` dropped corrupt lines silently. Now a run leaves an artifact
  that says exactly which values misbehaved and what the de-drift integrator
  was doing at the time. designDecisions §21.
- **Also:** `smoke_gui.py` stubbed out `load_calibration` — it was asserting on
  roll/pitch rows that this machine's saved `value_flips` negate, so the smoke
  test failed on any box with flips enabled (pre-existing, unrelated to this
  change).
- **Pushed:** no (uncommitted)

## 2026-07-09 17:55 — Visualizer freak-out root-caused: misaligned DMP FIFO reads
- **What:** Firmware `imuTask` now reads the DMP via the overflow-proof
  `mpu.dmpGetCurrentFIFOPacket()` (newest packet, drains backlog) and rejects
  any packet whose quaternion isn't unit-norm (0.96..1.04), calling
  `resetFIFO()` to restore packet alignment and bumping a `dmpResyncs` counter
  (`debug` monitor prints `resync=N`). GUI: the `|q|^2` acceptance window
  tightened from 0.5..2.0 to `QUAT_NORM2_MIN/MAX` (0.96..1.04), and
  `VisualizerPanel._dedrift` now returns None on a glitch so `_tick` holds the
  previous frame instead of painting the corrupt orientation.
- **Why:** The first session log (`log/2026-07-09/log-2026-07-09_17-36-33.txt`)
  showed it outright: with the camera off, all 234 accepted lines had
  |q| = 1.0000 and 0.0 deg yaw spread on a stationary board; 4 ms after
  `cam:ok`, 2132 of 2823 board lines were rejected as bad-quat and the 437 that
  passed swept +/-180 deg of yaw. The lines were well formed and `ax..gz` (read
  from the data registers, not the FIFO) stayed correct — so the DMP FIFO read
  phase had shifted inside a packet and never resynced. One sample was the true
  quaternion shifted a whole field right. designDecisions §22.
- **Note:** the GUI window alone is insufficient — replaying the capture through
  it still accepted 165 misaligned-but-near-unit-norm samples. The firmware
  resync is what actually fixes it. (If corruption ever survives it, the next
  step is a gyro-consistency gate: reject a quaternion step that the measured
  gyro rate cannot explain.)
- **Verified on hardware (2026-07-09 18:05):** flashed, then provisioned Wi-Fi
  over serial to bring the camera up. Camera off: 251 lines, |q| 0.9999..1.0000,
  0 non-unit. Camera on: **1247 lines, |q| 0.9999..1.0000, 0 non-unit (0.0%)**,
  yaw spread 0.2 deg on a stationary board (was +/-180 deg). `dbg cam:` reports
  `resync=2` — alignment slipped twice as the camera started, the guard caught
  both, and the counter stayed at 2 thereafter with video steady at 10 fps.
  Serial tearing persists at ~0.3% (4 malformed lines in 1247); those parse as
  `RAW` and are ignored, unchanged by this work.
- **Pushed:** no (uncommitted)
