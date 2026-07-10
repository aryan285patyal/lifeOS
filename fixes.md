# Fixes — production backlog

Issues that were diagnosed, then **deliberately deferred** so work stays on
the main goal. Each entry records the symptom, the root cause as far as it
was established, everything already tried (so nobody re-debugs from scratch),
the current workaround, and candidate production fixes. Entries are added
only when the user decides "fix later"; when one is finally fixed, note the
date + progress.md entry here rather than deleting it.

---

## 1. Wireless feed requires laptop and ESP32 on the same network segment

*Logged 2026-07-08. Decision: fix later, if the software goes to production.
Workaround for now: laptop joins the same Wi-Fi as the ESP32.*

**Symptom:** On the S3, "Go wireless" reports success ("Feed is wireless -
telemetry via UDP from <esp_ip>...") but no telemetry or video ever arrives;
after unplugging USB the GUI shows no data. The ESP32 side is healthy the
whole time (`wifi:connected,<ip>`, "streaming video to <laptop_ip>").

**Root cause (two layers, both confirmed 2026-07-08):**

1. **Laptop firewall.** The GUI runs under `.venv\Scripts\python.exe`, which
   had no inbound firewall rule; only anaconda's `python.exe` did (that's why
   the WROOM-era receivers, run under anaconda, worked). On a network
   profiled *Public*, Windows drops unsolicited inbound UDP silently.
   **Fixed permanently** with two port rules (shown in the Connect tab's
   Wi-Fi section):
   `netsh advfirewall firewall add rule name="lifeOs feed UDP 5005" dir=in action=allow protocol=UDP localport=5005`
   `netsh advfirewall firewall add rule name="lifeOs video UDP 5010" dir=in action=allow protocol=UDP localport=5010`
2. **Network topology.** With the firewall rules verified active, still zero
   UDP arrived. Laptop was on Ethernet (172.17.8.100/20 VLAN), ESP32 on
   Apollo-Resident Wi-Fi (172.16.72.41, different VLAN). The campus network
   routes ICMP between the VLANs (ping works both directions, TTL 61) but
   **drops unsolicited inter-VLAN UDP** — a standard stateful/ACL policy on
   residence networks. Nothing on the laptop or ESP32 can change that.

**Debugging done before deferring** (don't repeat):
- Built the serial-monitor panel ([PC]/[ESP] + transport tags) — showed
  provisioning and `wifi:connected` succeed over USB, then silence after the
  link swap to Wi-Fi.
- `ping <esp_ip>` from the laptop: 0% loss, ~4 ms, TTL 61 (≈3 router hops) —
  proves both VLANs route ICMP, ESP alive.
- `Get-NetConnectionProfile`: Ethernet profiled **Public**.
- Firewall rule audit: inbound allow rules existed only for anaconda's
  python.exe → added the two lifeOs port rules above, re-verified with
  `Get-NetFirewallRule -DisplayName "lifeOs*"`.
- 8-second raw UDP listener on 5010 while the ESP said it was streaming
  video: **0 packets** → network-level UDP filtering, not the laptop.
- Also fixed en route: reader-thread `TypeError` traceback on every link
  swap (pyserial raises it when the COM port closes mid-`readline`).

**Candidate production fixes (any/several):**
- **Same-subnet check in the GUI:** before/after Go wireless, compare the
  ESP's IP against the laptop adapters' subnets and warn explicitly
  ("different subnet — this network will likely drop the UDP feed; join the
  same Wi-Fi or use a hotspot") instead of failing silently.
- **Reachability probe:** after `feed:wifi`, wait ~2 s for the first UDP
  packet; if none, auto-fall back to USB (`feed:usb`) and tell the user why
  — never strand the feed on a dead transport.
- **Laptop-initiated / hole-punched flow:** have the laptop's 5005 socket
  send periodic keepalives to the ESP's data port so a stateful inter-VLAN
  firewall sees an outbound-initiated flow (would need the firmware to send
  telemetry from a fixed source port the keepalive targets).
- **TCP fallback** for the feed (stateful firewalls pass laptop-initiated
  TCP), at the cost of latency jitter.
- **ESP-NOW dongle** (to-do.md / designDecisions §16): the retired WROOM-32
  as a USB radio bridge — bypasses infrastructure networks entirely; the
  robust answer for hostile networks.

---

## 2. S3-CAM RF path: switch from PCB antenna to the external IPEX antenna

*Logged 2026-07-08. Decision: solder rework later. Works for now on the
onboard PCB antenna.*

**Finding (2026-07-08):** With the new RSSI readout (`rs` telemetry field),
the touch/detune test on the module's PCB antenna trace moved RSSI, i.e. the
radio is using the **onboard PCB antenna** — the ESP32-S3-WROOM-1's RF-path
selector (0 Ohm resistor at the module's antenna corner) is still routed to
the PCB trace even though the IPEX socket is populated. The plugged-in
external antenna is currently doing nothing. (This resolves the old
"Check the S3-CAM's RF-path selector" to-do item.)

**Deferred fix:** rotate the 0 Ohm resistor (or bridge the correct pads with
a solder blob) at the module's antenna-corner pad triangle: center pad ->
u.FL/IPEX side instead of center pad -> zig-zag trace side. Fine-pitch
work - needs a fine tip / hot tweezers and a steady hand.

**Verification after the rework:** RSSI readout in the same spot should gain
roughly +3..10 dB with the external antenna attached; unplugging the external
antenna should now crash RSSI (10-20+ dB); touching the PCB trace should do
nothing. Don't leave it transmitting long with the IPEX path selected and no
antenna attached (PA stress).

**Why deferred:** the PCB antenna is adequate at current desk range; the
rework only matters for range/enclosure use (production concern, ties into
fixes.md section 1's wireless reliability).

---

## 3. A dropped serial byte can be silently accepted as a valid sample

*Logged 2026-07-09. Decision: fix later. Harmless for the live display today;
the only real exposure is Reset / Recalibrate's 20-sample average.*

**Symptom:** ~0.3% of telemetry lines arrive damaged (4 of 1247 over 25 s,
measured 2026-07-09 after the 921600 baud raise, with the camera streaming).
Most are rejected — but a specific damage mode is accepted with wrong values.

**Root cause / mechanism (established from `log/2026-07-09/*`):** two distinct
mechanisms, with very different consequences.

1. **Interleaved print** — an IDF/firmware log line lands in the middle of a
   telemetry line, splitting it. The first half loses its trailing keys
   (rejected as `DROP fragment`); the second half starts mid-field and won't
   parse (`RAW`). **Both rejected. Safe by construction.**
2. **Dropped byte** — a single character vanishes (seen: `ax848`, colon lost).
   If it lands on a separator, parsing fails and the line is rejected. If it
   lands on a **digit inside a numeric field**, every key and separator remains
   intact, so the line parses and passes the key check. The quaternion is
   protected (the unit-norm test added in designDecisions §22 catches
   `q0:0.9948` → `9948.0`), but **a raw count is not**: `az:1638` instead of
   `az:16384` is in int16 range and is accepted silently.

**Exposure, quantified:**
- Live display: one wrong sample for 20 ms — a sparkline spike. Invisible.
- The 3D view and DMP never see the receiver-side zero, so they're unaffected.
- **Reset / Recalibrate is the real exposure:** `CALIB_SAMPLES = 20`, so one
  corrupted `az` shifts the zero by ~740 counts (~0.045 g) — a persistent
  couple-of-degrees offset in the Monitor table until the user re-zeroes.
- The six-point wizard is effectively immune: ~500 samples per face, so one bad
  sample moves the mean by ~0.002 g.
- Odds are low: the tear must fall inside the 0.4 s window *and* hit a digit
  rather than a separator.

**Debugging done before deferring** (don't repeat):
- Confirmed the tear rate at 921600 with the camera live (4 malformed / 1247).
- Confirmed the tears are **not** caused by `cam_hal: FB-OVF` prints (§4):
  their timestamps never coincide, and one tear predates `cam:ok` entirely.
  Tearing tracks Wi-Fi/IDF logging generally, not the camera.
- Verified which of the four damaged lines would have been accepted: none —
  all four lost a separator or were truncated. The digit-loss case is inferred
  from the same mechanism (`ax848` proves bytes do get dropped), not observed
  being accepted.

**Candidate production fixes (in order of value):**
- **Checksum the telemetry line** — append `ck:<hex>` (XOR/CRC8 of the
  payload), optional so old firmware still parses. Makes *any* corrupt line
  impossible to accept, instead of pattern-matching each symptom one at a time.
  This is the same lesson §22's FIFO bug taught: reject at the source, by
  construction.
- **Median instead of mean** for the receiver-side zero (one line in
  `MonitorWindow.refresh`) — neutralizes any single outlier in the 20 samples.
- Raise `CALIB_SAMPLES`, or discard samples more than ~3 sigma from the window
  median before averaging.
- Stop sharing the UART: on the S3 the feed could run on the native USB CDC
  while IDF logs stay on UART0 (removes mechanism 1 entirely).

---

## 4. `cam_hal: FB-OVF` bursts while the camera streams

*Logged 2026-07-09. Decision: fix later. Benign at current frame rates.*

**Symptom:** `cam_hal: FB-OVF` appears on the feed in occasional bursts of two
while Wi-Fi video is streaming.

**Root cause:** the camera driver finished a DMA frame while both frame buffers
were still checked out — the firmware holds one for the whole duration of
`sendCameraFrame()` (chunking ~10 KB over UDP). With `fb_count = 2` and
`CAMERA_GRAB_LATEST` the driver's defined behavior is to drop that frame and
log FB-OVF. Bursts line up with Wi-Fi stalls, when the UDP send blocks longer
than usual.

**Impact: none observed.** Frames that arrive are complete JPEGs (receiver
verifies magic bytes; `wifi_video_test.py` reports 0% incomplete), and the
stream held a steady 10 fps through FB-OVF bursts during the 2026-07-09
verification. The cost is a dropped frame. It would only matter if FB-OVF went
from occasional to constant, which shows up as fps sagging in the `VID` lines
of the session log.

**Explicitly ruled out:** FB-OVF prints are *not* the cause of the torn serial
lines in §3 — the timestamps never coincide and tearing predates camera init.

**Candidate production fixes:**
- **Copy out and return the buffer immediately:** memcpy the JPEG into a PSRAM
  buffer, `esp_camera_fb_return()` at once, then transmit from the copy — the
  DMA buffer is then held for microseconds instead of the whole UDP send.
- Raise `fb_count` to 3 (PSRAM has ample room) for more slack.
- Lower frame size / raise JPEG quality number if bandwidth is the limit.
