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

**Constraint:** The ESP32 can't cheaply run all transports at once (Wi-Fi + BT
together is heavy on RAM/flash and shares the radio), so the firmware picks **one
transport per build** via `USE_BLUETOOTH`; the PC-side selector is free.

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

## 9. Roadmap: Bluetooth for sensor data, Wi-Fi/UDP for camera data (BT-first sync)

**Planned decision (not yet built):** Keep two channels, chosen by data type:

- **Bluetooth (SPP/BLE) — sensor + control data.** IMU quaternion, servo
  commands/echo. Low bandwidth, latency-tolerant, must be reliable and
  network-independent. This is the always-on control channel.
- **Wi-Fi / UDP — camera data (future).** JPEG frames are high-bandwidth and
  unsuitable for Bluetooth's throughput, so the camera stream rides Wi-Fi UDP.

**BT-first bootstrap:** Bluetooth comes up first and is used to **sync/hand off
the Wi-Fi side** — e.g. exchange/confirm Wi-Fi association and the PC's address,
and coordinate when to start the UDP camera stream — *before* the high-bandwidth
Wi-Fi channel is brought up. Bluetooth is the reliable out-of-band control link;
Wi-Fi is the fat pipe it turns on once both ends agree.

**Why this split:** It plays to each medium's strength (BT = reliable low-rate
control that ignores the network; Wi-Fi = raw bandwidth) and uses the reliable
channel to make the fragile one come up predictably. It also aligns with the
dual-core firmware split (§2): the camera/JPEG work sits on core 1 with the Wi-Fi
TX, while the IMU FIFO stays isolated on core 0.

**Open questions:** camera stream on the same network vs. ESP32 SoftAP for the
camera only; how the PC consumes two simultaneous channels; whether camera Wi-Fi
reintroduces the client-isolation problem (may push toward SoftAP-for-camera or a
cloud relay, §10).

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
