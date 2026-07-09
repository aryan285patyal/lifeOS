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
