# To-do

Living task list for lifeOs. Keep this updated: add items as they come up,
check them off (and date them) when done.

## Accuracy / calibration

- [ ] **Six-position accel calibration** — drone-style: hold each face down
  (±X, ±Y, ±Z) a few seconds; firmware averages each pose, PC solves per-axis
  bias *and* scale (the current `cal` estimates bias only and assumes the
  sensor is flat, which corrupts offsets when run tilted — the cause of the
  roll/pitch decay-to-wrong-angle bug). Also add a firmware guard so plain
  `cal` refuses to run when the sensor isn't near-flat.
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

- [ ] **Settings menu** — a place for:
  - default settings (what `connect_config.json` holds today, editable in-app),
  - pulling different versions of the code from GitHub (list releases/commits,
    check out a chosen one),
  - flashing firmware to the ESP32 from the GUI (arduino-cli or esptool
    under the hood).
