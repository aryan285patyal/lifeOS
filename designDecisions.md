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
