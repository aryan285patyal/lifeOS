# To-do

Living task list for lifeOs. Keep this updated: add items as they come up,
check them off (and date them) when done.

## Accuracy / calibration

- [ ] **Visualizer source checkboxes (drift diagnosis)** — two checkboxes
  (gyro / accel) selecting the orientation source: both = DMP quaternion
  (current), gyro-only = PC-side integration of raw gyro counts, accel-only =
  tilt from the gravity vector. Goal: confirm roll/pitch decay is the DMP's
  accel correction pulling toward a *biased* gravity direction. Do before the
  six-position cal.
- [ ] **Fix gyro conversion scale?** — `CleanInput.GYRO_LSB_PER_DPS` is 131
  (±250 °/s) but the DMP configures ±2000 °/s (16.4 LSB/°/s) — gui.py's
  stillness threshold already assumes 16.4. Verify with a timed 90° rotation;
  Monitor's °/s readings are likely 8x too large.
- [x] **Six-position accel calibration** *(done 2026-07-05; reworked to an
  explicit step-per-face flow 2026-07-07)* — drone-style: the Monitor tab's
  "6-Point Calibration…" wizard walks the six faces in a fixed order — it
  prompts a face, the user places the sensor and presses **Calibrate**, and
  readings are averaged for 10 s (`SIXCAL_CAPTURE_SECS`) before the next face
  is prompted (pose still verified from raw telemetry; the button stays
  disabled until the prompted face is up). It solves per-axis bias *and*
  scale, sends the
  bias to the firmware (`acal:set` → MPU offset registers, persisted in NVS and
  re-applied at boot instead of the flat-assuming `CalibrateAccel`), and saves
  the scale to `calibration.json` for PC-side correction. Plain `cal` is now
  gyro-only (see below). Debugging on real hardware (2026-07-05, USB `debug`
  echo + pose logging) found and fixed two firmware root causes:
  - telemetry `ax..az` came from the DMP FIFO, which is NOT raw accel (reads
    ~0.03g total when inverted) → now read from the data registers
    (`getMotion6`), so the wizard can actually see each face;
  - the library's `CalibrateAccel()` (Electronic Cats v1.4.4) drives flat Z to
    2g instead of 1g, planting a +1g Z offset bias — the actual cause of the
    roll/pitch decay-to-wrong-angle bug. Removed from boot and from `cal`
    (now gyro-only); accel bias comes from the wizard or factory trim.
  Still to validate on hardware (blocked by the flaky MPU wiring below):
  run the wizard end-to-end, power-cycle, tilt-hold test.
- [ ] **Flaky MPU6050 I2C connection** — `MPU6050 connection FAILED` appears on
  random boots even with the board untouched (seen twice on 2026-07-05;
  re-seating the jumpers fixed it once). Check/solder the module's header pins,
  and add a firmware retry loop for MPU init so a marginal boot doesn't strand
  the sensor until the next reset.
- [ ] **Temperature-based bias refinement** — log at-rest bias per axis
  against die temp (`tp`) across a warm-up, fit a line per axis, subtract
  `bias(T)` continuously. Second-order polish; do after six-position cal.
- [ ] **Servo-rig validation + AI residual model** — mount the MPU6050 on a
  servo for precise ground-truth angles. First use it to *measure* residual
  error after the calibrations above; only if a small repeatable residual
  remains, train a small model on raw-sequence input to correct it
  (an angle→angle model can't work: the same true angle yields different
  readings over time).

## App

- [x] **ESP32-S3-CAM support (board switch + USB pairing + Go wireless)**
  *(done 2026-07-07)* — firmware `BOARD_S3CAM`/`BOARD_WROOM32` #define (pins +
  feed transport), S3 feed = USB serial → `feed:wifi` moves it to Wi-Fi UDP
  (5005/5006); GUI Board dropdown + Go wireless button; new S3 wiring
  SDA 21 / SCL 14 / INT 47 / servos 1, 2; README/CLAUDE.md/designDecisions §16.
  Still to validate on hardware: wire the MPU + servos to the new pins, flash
  (ESP32S3 Dev Module, 16MB, OPI PSRAM), run the USB → wireless flow end-to-end.
- [x] **Real OV3660 camera video** *(done 2026-07-08)* — see the
  "OV3660 camera feed (S3-CAM)" entry at the bottom of this section.
- [ ] **GUI firmware-flash button** — flash the right firmware for the selected
  board from the GUI (arduino-cli/esptool under the hood; fold into the
  Settings menu below). Discuss design first.
- [ ] **In-GUI wiring diagram per board** — show the selected board's MPU/servo
  wiring in the Connect tab so the user never wires against the wrong map.
  Discuss design first.
- [ ] **BLE feed option (S3)** — wireless without a router: NimBLE GATT server
  in firmware + async bleak client link in gui.py. Only worth it if Wi-Fi UDP
  proves unreliable on hostile networks.
- [ ] **ESP-NOW dongle fallback** — repurpose the retired WROOM-32 as a USB
  dongle bridging ESP-NOW radio ↔ COM port for router-free wireless telemetry
  (no video). designDecisions §16.
- [x] **Check the S3-CAM's RF-path selector** *(done 2026-07-08)* — confirmed
  with the RSSI readout + antenna touch test: the radio uses the **onboard
  PCB antenna**; the 0 Ω resistor is still routed away from the IPEX socket,
  so the plugged-in external antenna does nothing. The solder rework to
  switch paths is deferred to fixes.md §2.
- [x] **progress.md changelog + doc-upkeep rules** *(done 2026-07-07)* —
  progress.md logs every change (what / why / timestamp / pushed-to-GitHub
  flag), back-filled from git history and the docs; CLAUDE.md now requires
  updating designDecisions.md whenever a decision is locked in, progress.md on
  every change, and this file continuously.
- [x] **Launch maximized + safe restore size** *(done 2026-07-07)* — the GUI
  opens maximized (title-bar buttons always visible); "restore down" goes to a
  standard 720×860 clamped to the current monitor and centered, ignoring Qt's
  remembered geometry. Fixes the glitch where the window opened oversized with
  the title bar (buttons + drag area) chopped off-screen. designDecisions.md §15.
- [x] **Flip column in the relative-state table** *(done 2026-07-07)* — third
  column of Monitor's second table, one checkbox per row (Roll / Pitch / Yaw /
  Delta Temp) that negates that row's displayed value for flipped mounting
  orientations. The angle flips also mirror the 3D visualizer (quaternion →
  Euler, negate, rebuild); the set persists across runs in `calibration.json`
  (`value_flips`).
- [ ] **Settings menu** — a place for:
  - default settings (what `connect_config.json` holds today, editable in-app),
  - pulling different versions of the code from GitHub (list releases/commits,
    check out a chosen one),
  - flashing firmware to the ESP32 from the GUI (arduino-cli or esptool
    under the hood).
- [x] **Serial-monitor panel** *(done 2026-07-08)* — Connect-tab checkbox
  (persisted) showing a terminal box on the bottom quarter of every tab:
  all lines from the active feed link + raw command input. Built as the
  debugging endpoint for the go-wireless/UDP-feed dropout.
- [x] **Go wireless drops telemetry after USB unplug** *(closed 2026-07-08 —
  deferred to fixes.md §1)* — diagnosed to two layers: laptop firewall
  (fixed: the two netsh UDP rules now shown in the Connect tab) and the
  campus network dropping inter-VLAN UDP (unfixable client-side). Workaround:
  laptop joins the same Wi-Fi as the ESP (set the Laptop IP field to the
  Wi-Fi adapter's address if Ethernet stays plugged). Production hardening
  ideas (subnet-mismatch warning, reachability probe + auto-fallback,
  ESP-NOW dongle) live in fixes.md §1.
- [x] **OV3660 camera feed (S3-CAM)** *(done 2026-07-08)* — real JPEG frames
  (VGA, ~10 fps, sensor-compressed) chunked over UDP 5010 as `vf:` packets;
  `VideoReceiver` + live preview in the Hand Model tab ("ESP32 camera" source,
  fps/KB status); synthetic `vid:` stream kept as fallback; wifi_video_test.py
  measures both and can --save a frame. designDecisions §18. Needs a flash +
  on-hardware check (camera init is the first real test of the EYE pin map).
- [x] **Session logging to disk** *(done 2026-07-09)* — `session_log.py`: one
  file per GUI run under `log/<date>/`, holding every board line (including the
  ones `_ingest` rejects, with the reason), every sent command, button presses,
  the visualizer's de-drift state at 1 Hz, video health, and the console
  (tracebacks + Qt messages). Read this file to find which values misbehaved
  instead of reproducing the bug live. designDecisions §21.
- [ ] **Prune old `log/` folders** — logs are kept forever by choice
  (~20 MB/hour of running). If the disk gets tight, delete old date folders by
  hand, or add a startup prune (age or total-size cap).
- [ ] **Hand tracking on the selected video source** — next step now that the
  Hand Model tab has live frames (ESP32 or PC camera).
