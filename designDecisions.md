# Design Decisions

A running log of the architectural choices in lifeOS and *why* they were made, so
future work (and future readers) don't relitigate settled questions. Newest
context near the bottom of each section.

---

## 1. Orientation: on-chip DMP quaternion, not PC-side fusion

**Decision:** Read the MPU6050's on-chip **DMP** fused quaternion and send it,
rather than sending only raw accel/gyro and fusing on the PC.

**Why:** The DMP does hardware sensor fusion at a stable rate with drift handling
(except yaw — no magnetometer). It offloads the math, avoids PC-side filter
tuning, and gives a clean quaternion the 3D view can use directly. Raw counts are
still forwarded so the Monitor tab can show/convert them.

**Consequence:** Yaw drifts slowly; the Visualizer's **Zero / Level** cancels the
resting offset.

## 2. Firmware concurrency: dual-core split

**Decision:** A FreeRTOS task pinned to **core 0** drains the DMP FIFO
(interrupt-driven on GPIO4, with overflow resync); `loop()` on **core 1** does
the transmit.

**Why:** Keeps the latency-sensitive IMU FIFO read path off the radio/TX path. A
future **on-board camera** (JPEG encode) on core 1 must never stall the FIFO and
force a DMP resync. This decision is forward-looking for the camera work (§9).

## 3. Wire protocol: transport-independent ASCII key:val

**Decision:** One newline/packet line of `key:value,...`
(`q0..q3,ax..gz,s0..s1`), the same bytes on every transport.

**Why:** Human-readable, trivial to parse, and — crucially — decouples the
protocol from the medium. Adding Bluetooth meant only changing *where bytes come
from*, not the format or the parser. `q*` are floats; everything else is int.

## 4. Two-way link; PC address learned at runtime

**Decision:** No hardcoded PC IP. In Wi-Fi mode the ESP32 stays silent until the
PC sends a `hello` on the command port, then learns the PC's IP from that packet
and streams to it.

**Why:** The original hardcoded `PC_IP` broke on every DHCP lease / network
change (this bit us in practice). Learning it at runtime makes the link
self-configuring. Same reasoning drives mDNS (§6).

## 5. Servos: command + echo (closed-loop-ish display)

**Decision:** The PC sends target angles; the ESP32 applies them **and echoes the
held angles back** in the telemetry stream. The Servos tab's dials follow the
*echoed* values, not the sliders.

**Why:** Hobby servos have no position feedback, so "real position" = "last target
the ESP32 holds." Echoing it makes the visualizer reflect true device state —
robust to a dropped command and correct even across a GUI restart.

**Note:** Servo v1 visualizer is Qt `QDial` gauges (simple, no WebChannel); a 3D
three.js version was deferred as it wasn't worth the complexity for v1.

## 6. Wi-Fi discovery via mDNS

**Decision:** In Wi-Fi mode the ESP32 advertises `lifeos.local` /
`_lifeos._udp`; the PC browses for it (with a manual IP / hostname fallback).

**Why:** Avoids typing IPs and survives DHCP changes. The manual fallback exists
because mDNS multicast is frequently blocked (Windows firewall, virtual adapters,
managed networks) — discovery is a convenience, not a dependency.

## 7. Transport abstraction (the swappable "link" seam)

**Decision:** Firmware isolates the transport behind
`linkBegin/linkConnected/linkSend/linkPoll`; the PC mirrors this with a `Link`
base + `WifiLink`/`BluetoothLink` and a `ConnectionManager` the tabs talk to. The
Connect tab picks the transport at runtime.

**Why:** One protocol, many media. Monitor/Visualizer/Servos are
transport-agnostic — they only call `snapshot()` / `is_connected()` /
`send_servos()`. Switching WiFi↔Bluetooth changes nothing downstream. This is the
extension point for future transports (cloud relay, §10).

**Constraint (historical):** Early on the firmware picked **one transport per
build** via `USE_BLUETOOTH`, because Wi-Fi + BT together is heavy on RAM/flash and
shares the radio. This was **superseded by §9** — the ESP32 now runs Bluetooth and
Wi-Fi *concurrently* (BT for sensor/control, Wi-Fi for video), which needs the
"Huge APP" partition scheme and accepts reduced Wi-Fi throughput under
coexistence. The PC `Link` abstraction remains, but the Connect tab no longer has
a transport selector (see §9).

## 8. Bluetooth Classic SPP as the primary link (over BLE, over Wi-Fi)

**Context:** Testing on a managed apartment Wi-Fi ("Apollo-Resident"), the PC and
ESP32 landed on **different routed subnets** (ping TTL 61 = 3 hops, ~190 ms
latency) with **client isolation** blocking peer-to-peer UDP. Ping worked but our
UDP `hello` never reached the ESP32. This is not fixable from our code, and
selecting a "nearer router" doesn't help (isolation is a WLAN-wide policy).

**Decision:** Make **Bluetooth Classic SPP** the default sensor/control link.

**Why:**
- **Separate radio** → keeps the laptop's single Wi-Fi free for internet (SoftAP
  would have consumed it).
- **Network-independent** → no router, no client isolation, no mDNS; works on any
  hostile/managed network.
- **Lower latency** than the isolated Wi-Fi (~20–40 ms vs ~190 ms) and ample
  throughput for our ~6 KB/s stream.
- **Reuses the existing ASCII protocol** verbatim (just newline-framed) — the PC
  reads a COM port instead of a socket.

**Why not BLE (yet):** BLE is more universal and enables future **Web Bluetooth**
(browser ↔ ESP32, relevant given the web/ visualizer), but has lower throughput
and more complexity (async `bleak` on the PC). SPP is simpler for a Python desktop
app today. Revisit BLE if/when a browser-based client becomes real.

**Cost:** BT stack is large — Bluetooth builds need a bigger partition scheme
("Huge APP" / "Minimal SPIFFS").

## 9. Bluetooth for sensor data, Wi-Fi/UDP for video (BT-provisioned) — BUILT

Two channels, chosen by data type, running concurrently:

- **Bluetooth (SPP) — sensor + control.** IMU quaternion, servo commands/echo,
  health flags, the `id?` identity reply, and Wi-Fi provisioning. Low bandwidth,
  reliable, network-independent; the always-on channel the GUI reads from.
- **Wi-Fi / UDP — video.** High-bandwidth (future camera; synthetic frames for
  now), streamed to the laptop on `VIDEO_PORT` (5010).

**BT-provisioned bootstrap (the key mechanism):** the laptop sends
`wifi:<ssid>|<password>|<laptop_ip>` **over Bluetooth**; the ESP32 then joins
Wi-Fi (station mode) and streams video UDP to that IP, replying `wifi:connected,
<esp_ip>` over Bluetooth. Bluetooth is the reliable out-of-band control link that
brings up the fat Wi-Fi pipe.

**Why station mode + provisioning (not SoftAP):** the user's Wi-Fi *does* carry
UDP fine; the only real problem was that the laptop's DHCP address changes across
boots. Sending the laptop's *current* IP over Bluetooth each session fixes that
directly, keeps the laptop's internet (no SoftAP takeover), and needs no hardcoded
credentials. SoftAP remains the fallback for networks that truly block P2P (§10).

**Health surfacing:** the BT telemetry line carries `dmp` (DMP producing
orientation) and `wf` (Wi-Fi video streaming) flags, so the whole system's state
— including whether the Wi-Fi side is up — is observable from the single BT
stream (see §12).

**Testing without a camera:** the ESP32 sends synthetic sequence-numbered 1 KB
UDP packets; `wifi_video_test.py` provisions over BT and measures throughput /
packet loss.

**Note:** running BT + Wi-Fi together requires the "Huge APP" partition and
reduces Wi-Fi throughput under radio coexistence (acceptable for the test; revisit
if real camera bandwidth demands it — see §10 for SoftAP/cloud fallbacks).

## 10. Distributed-product direction (future)

**Principle:** Never assume the user's network allows peer-to-peer traffic — many
don't. For "works for the most users," offer tiers and fall back:

1. **Direct, network-independent** (Bluetooth now; SoftAP as an option) — lowest
   latency, always works.
2. **USB/serial** — bulletproof, tethered.
3. **Cloud relay (e.g. MQTT)** — both ends dial *outbound* to a broker; passes
   through isolation/NAT (how consumer IoT gets universal reach). Adds a server +
   internet round-trip latency.
4. **LAN station mode (Wi-Fi/UDP)** — only friendly networks.

The `Link` abstraction (§7) is what makes adding a cloud-relay transport later a
localized change.

## 11. UI: full-width tab bar; Connect as the first tab

**Decision:** Custom `StretchTabBar` makes the four tabs span the window width;
Connect is first.

**Why:** Connect is the entry point (you must connect before anything else works),
and even tab widths read better as the app grows.

## 12. Status bar: per-source ✓/✗ from one telemetry stream

**Decision:** A bottom-left status bar with colored boxes — BT (blue), Wi-Fi
(green), Servo 1 / Servo 2 (red), MPU (yellow) — each showing a tick if that
source had a live signal in the **last 1 second**, else a cross.

**Why derived from the BT telemetry line:** everything reports over the one
always-on Bluetooth stream, so a single "fresh within 1s?" check plus a few fields
covers all sources: BT = stream fresh; MPU = fresh + `dmp:1` + quaternion present;
Servo 1/2 = fresh + `s0`/`s1` present; Wi-Fi = fresh + `wf:1`. If BT drops, all go
cross — the honest state (no link ⇒ no knowledge). Hobby servos have no feedback,
so the servo dots reflect "the ESP32 is reporting that servo's angle," not
physical servo health. The source color is always shown (the two red servo boxes
are told apart by their `S1`/`S2` captions); the glyph carries the live/dead
state. `dmp` defaults to present-and-ok for older firmware that predates the flag.

## 13. 6-point calibration: explicit step-per-face, not auto-detect

**Decision:** The wizard walks the six faces in a **fixed order**: it prompts one
face, the user places the sensor and presses **Calibrate**, and readings are
averaged for **10 s** (`SIXCAL_CAPTURE_SECS`) before the next face is prompted.
The button is enabled only while raw telemetry confirms the prompted face is up
and still.

**Why (supersedes the v1 any-order auto-detect):** v1 started capturing as soon
as *any* uncaptured face was held up, which made each step implicit — the user
couldn't tell which face was being measured or when. The explicit
prompt → place → click → capture loop makes every axis's calibration a
deliberate, visible act, and the 10 s window (~500 samples vs v1's ~1 s / 50)
averages more tremor out. Pose detection is kept, but demoted from *trigger* to
*gate*: it can no longer start a capture, only prevent capturing the wrong face;
movement mid-window aborts just that face for a retry.

## 14. Per-axis "Flip" checkboxes (mounting-orientation sign flips)

**Decision:** The Monitor tab's relative-state table has a **Flip** column — one
checkbox per row (Roll / Pitch / Yaw / Delta Temp) that negates that row's
displayed value. The three angle flips also mirror the 3D visualizer; the set
persists in `calibration.json` (`value_flips`).

**Why:** How the MPU is mounted decides each axis's sign; when it's mounted
inverted, "tilt up" reads negative. A per-axis display flip fixes that without
touching firmware or the wire protocol. It lives in `calibration.json` because it
is device/mounting configuration, like the accel scale. The visualizer flip is
applied PC-side — quaternion → Euler, negate the flipped components, rebuild
(`euler_to_quat`) — *before* the bridge emit, so the three.js page needs no
changes and its Zero button still composes on top. Raw counts and the Servos
mimic stay unflipped (they reflect the physical device). Known cost: with a flip
active, the Euler round-trip is degenerate at pitch ±90° (momentary glitch).

## 15. Window sizing: launch maximized; "restore down" = fixed standard size

**Decision:** The GUI launches **maximized** (normal window frame, so
minimize/restore/close stay visible). Pressing "restore down" always lands on
`RESTORED_SIZE` (720×860) clamped to the current monitor's available area and
centered — never the geometry Qt remembers.

**Why:** The window sometimes opened with glitched oversized geometry: the title
bar (and with it the buttons and the drag area) rendered off-screen, so the app
could be neither moved nor closed (same family as the earlier runaway-growth bug,
commit 606ae22). Maximizing is inherently screen-bounded, and overriding the
restore geometry (via `changeEvent` on the maximized→normal transition) guards
the one path that could reintroduce a bad remembered size — including when
restoring on a different monitor than the one launched on.

## 16. ESP32-S3-CAM: USB pairing replaces SPP; "Go wireless" = Wi-Fi UDP feed

**Context:** The project is moving to a GoouuuTech **ESP32-S3-CAM**
(ESP32-S3-WROOM-1 N16R8, OV3660, dual USB-C, microSD) for its onboard camera.
The S3 silicon has **no Bluetooth Classic** — only BLE — so the SPP
sensor/servo link (§8) cannot exist there.

**Decision:** Support both boards from one firmware (`#define BOARD_S3CAM` /
`BOARD_WROOM32` choosing pins + feed transport behind a `linkSendLine` seam)
and a GUI **Board dropdown**. On the S3 the feed flow is: **USB serial first**
(the CH343 COM port; the "pairing" step — `id?` + Wi-Fi provisioning over USB,
exactly what Bluetooth did), then **"Go wireless"** (`feed:wifi`) moves
telemetry + servo control onto **Wi-Fi UDP** (telemetry → laptop:5005,
commands in on 5006 — the legacy `WifiLink` ports revived) so the board runs
untethered. Video (5010) is unchanged; the real OV3660 camera is a follow-up.

**Why USB + Wi-Fi rather than BLE:** a USB COM port carries the existing
newline protocol with ~zero rework on either end (the GUI's pyserial link and
port auto-detect work verbatim), while BLE needs a GATT server + an async
`bleak` client — a rework that buys nothing once the feed can ride Wi-Fi.
Wi-Fi is also the only transport that will ever carry the camera. The
client-isolation risk that killed Wi-Fi-as-primary (§8) is mitigated by USB
always being available as the wired fallback, and an **ESP-NOW dongle**
(the retired WROOM-32 plugged into the laptop's USB, bridging radio→COM) is
the noted future fallback for hostile networks. **nRF24L01 was rejected:** its
SPI needs 5–6 pins and the S3-CAM has exactly six free GPIOs (1, 2, 3, 14, 21,
47 — camera 4–13/15–18, SD 38–40, LED 48, native USB 19/20, strapping
0/45/46), all consumed by the MPU (3) and servos (2).

**Pin choices (S3):** SDA 21 / SCL 14 / INT 47 / servos 1, 2; GPIO3 (weakly
strapping) left as the one spare. `Wire.begin` must pass pins explicitly — the
S3's Arduino I2C defaults (8/9) are camera data lines. The old map can't work:
GPIO22/25 don't exist on the S3, and 4/13 are camera pins.

**Cost:** wireless feed quality now depends on the router (use the external
IPEX antenna); `wifi:off` on the S3 necessarily drops the feed back to USB.

## 17. Serial-monitor panel: a link monitor under the tabs, not a COM-port tool

**Decision:** A "Serial monitor" checkbox on the Connect tab (persisted
immediately in `connect_config.json` under `serial_monitor`) shows a
terminal-style panel pinned to **1/5 of the current window height** (re-pinned
on every resize, so the proportion holds when maximizing/restoring), below the
tab widget so it is visible on every tab. It displays every line received on the
**active feed link** — USB serial, paired-BT COM, or the S3's Wi-Fi UDP feed,
whichever the `ConnectionManager` holds — plus a line input that sends raw
command lines (`id?`, `debug`, `feed:usb`, ...) over that same link. A
"Hide telemetry" checkbox filters the ~50 Hz `q0:...` lines so control
replies stay readable — **on by default** (the panel exists for control
replies; the telemetry firehose is opt-in) and persisted in
`connect_config.json` (`hide_telemetry`) like the panel toggle itself. Implementation: `Link._ingest` keeps a per-link ring
buffer (`raw_log`, 2000 lines) that the panel polls at 10 Hz via
`raw_since(seq)`; a divider line marks every link swap. Every line carries an
origin tag — `[ESP]` for lines received from the board, `[PC]` for
GUI-originated ones — and link traffic adds a transport tag (`[USB]`, `[BT]`,
or `[WiFi]`, from `Link.transport_tag()` on the active link), e.g.
`[ESP][WiFi] wifi:connected,...` or `[PC][USB] > id?`. GUI button presses are logged as `[PC]` lines via a
module-level `ui_log` ring buffer and one generic hook over all
`QPushButton`s in the main window; each line says whether that button is a
laptop-side action or commands the ESP32 (an `esp` dynamic property set where
the ESP-commanding buttons — Wi-Fi Connect/Disconnect, Go wireless,
Reset/Recalibrate, servo Enabled toggles, Upload, Send — are created).
Dialog-local buttons (the 6-point wizard) are outside the hook.

**Why a link monitor, not a serial-port monitor:** the interesting failures
(e.g. Go wireless handing off to a UDP feed that never delivers) live on
whatever transport currently carries the feed — a literal COM-port monitor
would go dark exactly when debugging matters most (USB unplugged, feed on
Wi-Fi). Tapping `Link._ingest` covers every transport with one mechanism and
zero per-transport code. Below-the-tabs placement (instead of a tab of its
own) keeps the stream visible while operating other tabs — watching replies
while pressing Connect/Go wireless is the whole point. Persisting the toggle
matches the board dropdown's behavior (§16): debugging sessions span many GUI
restarts.
