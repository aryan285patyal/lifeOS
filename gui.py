import sys
import os
import json
import math
import socket
import threading
import time

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QLineEdit, QPushButton, QTabWidget, QTabBar, QSlider, QSpinBox, QDial,
    QGroupBox, QListWidget, QListWidgetItem, QCheckBox, QComboBox, QStackedWidget,
)
from PySide6.QtCore import QTimer, Qt, QUrl, QObject, Signal
from PySide6.QtGui import QColor
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel
try:
    from PySide6.QtMultimedia import QCamera, QMediaCaptureSession, QMediaDevices
    from PySide6.QtMultimediaWidgets import QVideoWidget
    HAVE_MULTIMEDIA = True
except ImportError:                       # GUI still runs; Hand Model tab degrades
    HAVE_MULTIMEDIA = False
from zeroconf import Zeroconf, ServiceBrowser, ServiceListener

from CleanInput import CleanInput, convert, format_converted
from live_charts import sparkline_for

QUAT = ["q0", "q1", "q2", "q3"]

# Monitor table rows: accel/gyro interleaved per axis, then die temperature.
SENSORS = ["ax", "gx", "ay", "gy", "az", "gz", "tp"]
# The axes Reset / Recalibrate zeroes (temperature has no meaningful zero).
CALIB_SENSORS = ["ax", "ay", "az", "gx", "gy", "gz"]

NUM_SERVOS = 2                        # must match NUM_SERVOS in lifeOs.ino
SERVO_GPIOS = [13, 25]               # for display only; matches SERVO_PINS in firmware
SERVO_KEYS = [f"s{i}" for i in range(NUM_SERVOS)]  # echoed angle fields in telemetry
UDP_PORT = 5005                       # telemetry port (legacy WiFi sensor mode)
CMD_PORT = 5006                       # legacy WiFi command/hello port
VIDEO_PORT = 5010                     # laptop receives Wi-Fi video UDP here (matches lifeOs.ino)

# mDNS service the ESP32 advertises (MDNS.addService("lifeos", "udp", ...)).
SERVICE_TYPE = "_lifeos._udp.local."
# Remembers the user's "default" device between runs.
CONNECT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "connect_config.json")


def load_connect_config():
    try:
        with open(CONNECT_CONFIG) as f:
            return json.load(f)
    except Exception:
        return {}


def save_connect_config(cfg):
    try:
        with open(CONNECT_CONFIG, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


def looks_like_ip(s):
    parts = s.split('.')
    return len(parts) == 4 and all(p.isdigit() and 0 <= int(p) <= 255 for p in parts)


def get_local_ip():
    """The laptop's IP on the interface that reaches the internet (usually Wi-Fi).
    This is the address the ESP32 will send video UDP to."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))       # no packet sent; just selects the route
        return s.getsockname()[0]
    except Exception:
        return ""
    finally:
        s.close()


# Servo "mimic" inputs: the axes the hand orientation can be decomposed into.
MIMIC_INPUTS = ["Manual", "Roll", "Pitch", "Yaw"]


def quat_to_euler(w, x, y, z):
    """Quaternion (w,x,y,z) -> (roll, pitch, yaw) in degrees, matching the
    orientation the Visualizer shows."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def wrap180(deg):
    """Fold an angle/delta into [-180, 180) so wrap-arounds don't explode."""
    return (deg + 180.0) % 360.0 - 180.0


def quat_yaw(w, x, y, z):
    """Heading (degrees) about the DMP world vertical - the drifting axis."""
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def quat_mul(a, b):
    """Hamilton product of two (w,x,y,z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_about_z(deg):
    """Quaternion rotating `deg` about the DMP world vertical (Z-up)."""
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def axis_to_servo(axis, roll, pitch, yaw):
    """Map one Euler axis (degrees) onto a 0-180 servo angle. Roll/pitch (±90)
    map directly with a +90 offset; yaw (±180) is halved so its full range fits.
    Returns None for 'Manual'/unknown."""
    if axis == "Roll":
        v = roll + 90
    elif axis == "Pitch":
        v = pitch + 90
    elif axis == "Yaw":
        v = yaw / 2 + 90
    else:
        return None
    return max(0, min(180, int(round(v))))


def list_serial_ports():
    """[(device, description)] of COM ports, or [] if pyserial isn't installed."""
    try:
        import serial.tools.list_ports as lp
    except Exception:
        return []
    return [(p.device, p.description) for p in lp.comports()]


def probe_lifeos_port(timeout=1.5, skip=None):
    """Open each COM port, ask 'id?', and return the device that answers as a
    lifeOs ESP32 (or is already streaming lifeOs telemetry). None if not found.
    `skip` is a set of devices to leave alone (e.g. one already open by us)."""
    try:
        import serial
    except Exception:
        return None
    skip = skip or set()
    for dev, _desc in list_serial_ports():
        if dev in skip:
            continue
        try:
            with serial.Serial(dev, 115200, timeout=0.3, write_timeout=0.5) as s:
                try:
                    s.write(b"id?\n")
                except Exception:
                    pass
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    raw = s.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="ignore").strip()
                    if line.startswith("id:lifeos") or ("q0:" in line and "s0:" in line):
                        return dev
        except Exception:
            continue   # port busy / not openable / not ours
    return None

CALIB_SAMPLES = 20          # fresh samples averaged into the zero baseline on reset
DEVICE_CAL_TIMEOUT = 10.0   # seconds to wait for the ESP32's cal:done reply

# Stillness detection for the Visualizer's yaw de-drift, on the raw counts
# (DMP full scales: accel +/-2g -> 16384 LSB/g, gyro +/-2000 dps -> 16.4 LSB/dps).
STILL_GYRO_COUNTS = 60      # per-axis |gyro| below this (~3.7 deg/s) = not rotating
STILL_ACCEL_DELTA = 1200    # | |accel| - 1g | below this = not accelerating
STILL_MIN_SECS = 0.5        # stillness must hold this long before it's trusted

COLOR_REAL = "#2e7d32"      # green - value came from a fresh packet
COLOR_ASSUMED = "#c62828"   # red   - value was held over (missing/garbled)
COLOR_NODATA = "#9e9e9e"    # grey  - no packet received yet


def parse_line(text):
    """Parse one 'q0:..,ax:..,s0:..' telemetry line into a dict. q* fields are
    floats, everything else int. Raises on malformed input."""
    d = {}
    for pair in text.split(','):
        key, value = pair.split(':')
        d[key] = float(value) if key.startswith('q') else int(value)
    return d


class Link:
    """Base transport: shared parsed-sample state + read-side accessors. Both
    transports carry the same newline/packet ASCII protocol, so only the byte
    source differs. Subclasses implement start/stop/send_servos/description."""

    # non-telemetry replies the ESP32 sends over the same stream, kept per prefix
    CONTROL_PREFIXES = ("cal:", "wifi:", "id:")

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.last_seen = 0.0
        self.controls = {}          # prefix -> (full line, monotonic timestamp)

    def _ingest(self, text):
        try:
            d = parse_line(text)
        except Exception:
            for prefix in self.CONTROL_PREFIXES:
                if text.startswith(prefix):
                    with self.lock:
                        self.controls[prefix] = (text, time.monotonic())
                    break
            return
        with self.lock:
            self.latest = d
            self.last_seen = time.monotonic()

    def snapshot(self):
        with self.lock:
            return (self.latest.copy(), self.last_seen)

    def control(self, prefix):
        """Latest control reply starting with `prefix` as (line, timestamp),
        or (None, 0.0) if none arrived yet."""
        with self.lock:
            return self.controls.get(prefix, (None, 0.0))

    def is_connected(self, timeout=1.0):
        with self.lock:
            return (time.monotonic() - self.last_seen) <= timeout

    # --- overridden per transport ---
    def start(self):
        pass

    def stop(self):
        pass

    def send_servos(self, angles):
        return False

    def send_line(self, text):
        """Send one raw control line (e.g. 'cal', 'e0:0', 'wifi:off')."""
        return False

    def description(self):
        return "?"


class WifiLink(Link):
    """UDP telemetry in (port UDP_PORT) + commands out (CMD_PORT). Learns the
    ESP32 IP from incoming packets; the Connect tab also sets it via
    register_peer(), which sends the 'hello' that starts the stream."""

    def __init__(self, port=UDP_PORT):
        super().__init__()
        self.port = port
        self.peer_ip = None
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('', self.port))
        except OSError:
            self._running = False
            return
        sock.settimeout(0.5)                     # so stop() is honored promptly
        while self._running:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self._ingest(data.decode(errors="ignore"))
            with self.lock:
                self.peer_ip = addr[0]
        sock.close()

    def stop(self):
        self._running = False

    def register_peer(self, ip):
        """Remember the ESP32 IP and send 'hello' so the firmware learns our IP
        and starts streaming. Returns True if the datagram was sent."""
        with self.lock:
            self.peer_ip = ip
        try:
            self.tx_sock.sendto(b"hello", (ip, CMD_PORT))
            return True
        except OSError:
            return False

    def send_servos(self, angles):
        with self.lock:
            ip = self.peer_ip
        if not ip:
            return False
        msg = ",".join(f"s{i}:{int(a)}" for i, a in enumerate(angles))
        try:
            self.tx_sock.sendto(msg.encode(), (ip, CMD_PORT))
            return True
        except OSError:
            return False

    def description(self):
        with self.lock:
            ip = self.peer_ip
        return f"Wi-Fi {ip}" if ip else "Wi-Fi (no device)"


class BluetoothLink(Link):
    """Bluetooth Classic SPP over a paired COM port. Same ASCII protocol as
    WiFi, one newline-delimited line per sample. pyserial is imported lazily so
    the GUI still runs without it when only WiFi is used."""

    def __init__(self, com):
        super().__init__()
        self.com = com
        self.ser = None
        self._running = False

    def start(self):
        import serial                            # lazy: only needed for Bluetooth
        self.ser = serial.Serial(self.com, 115200, timeout=1)
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        import serial
        while self._running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError):
                break
            if raw:
                self._ingest(raw.decode(errors="ignore").strip())

    def stop(self):
        self._running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def send_line(self, text):
        if not self.ser:
            return False
        try:
            self.ser.write((text.rstrip() + "\n").encode())
            return True
        except Exception:
            return False

    def send_servos(self, angles):
        return self.send_line(",".join(f"s{i}:{int(a)}" for i, a in enumerate(angles)))

    def provision_video(self, ssid, password, ip):
        """Send Wi-Fi credentials + the laptop's IP so the ESP32 brings up Wi-Fi
        and streams video UDP back to us. Format: 'wifi:<ssid>|<pass>|<ip>'."""
        return self.send_line(f"wifi:{ssid}|{password}|{ip}")

    def description(self):
        return f"Bluetooth {self.com}"


class ConnectionManager:
    """Holds the active Link and proxies the read/write API the tabs use, so the
    Monitor/Visualizer/Servos tabs stay transport-agnostic. Activating a new
    link stops the previous one (freeing its socket / COM port)."""

    def __init__(self):
        self.active = None

    def set_active(self, link):
        old = self.active
        self.active = link
        if old is not None and old is not link:
            old.stop()

    def snapshot(self):
        return self.active.snapshot() if self.active else ({}, 0.0)

    def is_connected(self, timeout=1.0):
        return self.active.is_connected(timeout) if self.active else False

    def send_servos(self, angles):
        return self.active.send_servos(angles) if self.active else False

    def send_line(self, text):
        return self.active.send_line(text) if self.active else False

    def control(self, prefix):
        return self.active.control(prefix) if self.active else (None, 0.0)

    def description(self):
        return self.active.description() if self.active else "Not connected"

    def disconnect(self):
        """Explicitly drop the active link. For Bluetooth this closes the COM
        port, which tears the SPP connection down (the ESP32 sees its client
        vanish) - an active disconnect, not just ignored traffic."""
        if self.active:
            self.active.stop()
            self.active = None

    def close(self):
        if self.active:
            self.active.stop()


class StretchTabBar(QTabBar):
    """Tab bar whose tabs always split the full bar width into equal parts, so
    the selector row covers the window edge-to-edge with equal-width tabs
    (leftover pixels from the division go to the leftmost tabs)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setUsesScrollButtons(False)
        self.setElideMode(Qt.ElideRight)   # squeeze labels instead of scrolling

    def tabSizeHint(self, index):
        hint = super().tabSizeHint(index)
        count = self.count()
        if count > 0 and self.width() > 0:
            width, extra = divmod(self.width(), count)
            hint.setWidth(width + (1 if index < extra else 0))
        return hint

    def minimumTabSizeHint(self, index):
        # Qt's default minimum is derived from tabSizeHint, which here equals
        # width/count -- that pins the bar's minimum width to its CURRENT width,
        # so the window can never shrink and Windows frame rounding then grows
        # it a few px per layout pass, forever. Keep the minimum small instead;
        # labels elide when the window really is that narrow.
        hint = super().minimumTabSizeHint(index)
        hint.setWidth(48)
        return hint


class DeviceDiscovery(ServiceListener):
    """Live mDNS browser for the ESP32's _lifeos._udp service. Keeps a running
    map of {service name -> (ip, port)} that the Connect tab reads. Used only
    for discovery; once connected the link is plain UDP."""

    def __init__(self):
        self._lock = threading.Lock()
        self._devices = {}
        self.zc = Zeroconf()
        self.browser = ServiceBrowser(self.zc, SERVICE_TYPE, self)

    def _store(self, zc, type_, name):
        info = zc.get_service_info(type_, name, timeout=1000)
        if info:
            addrs = info.parsed_addresses()
            if addrs:
                with self._lock:
                    self._devices[name] = (addrs[0], info.port)

    # ServiceListener callbacks (run on zeroconf's own thread)
    def add_service(self, zc, type_, name):
        self._store(zc, type_, name)

    def update_service(self, zc, type_, name):
        self._store(zc, type_, name)

    def remove_service(self, zc, type_, name):
        with self._lock:
            self._devices.pop(name, None)

    def devices(self):
        with self._lock:
            return dict(self._devices)

    def close(self):
        try:
            self.browser.cancel()
        except Exception:
            pass
        self.zc.close()


class PortProber(QObject):
    """Runs probe_lifeos_port() off the GUI thread and reports the matched COM
    device via a signal (Qt queues it back onto the GUI thread safely)."""

    found = Signal(str)     # matched device, or "" if none

    def probe(self, skip=None):
        threading.Thread(target=self._run, args=(skip,), daemon=True).start()

    def _run(self, skip):
        dev = probe_lifeos_port(skip=skip)
        self.found.emit(dev or "")


class OrientationBridge(QObject):
    """Exposed to the three.js page over QWebChannel."""
    orientation = Signal(float, float, float, float)  # w, x, y, z
    zeroRequested = Signal()                           # "Zero / Level" pressed


class VisualizerPanel(QWidget):
    """A QWebEngineView hosting the three.js scene, fed the device quaternion.
    Embedded in the Monitor tab below the relative-state table."""

    def __init__(self, listener):
        super().__init__()
        self.listener = listener

        layout = QVBoxLayout(self)

        self.view = QWebEngineView()
        self.channel = QWebChannel()
        self.bridge = OrientationBridge()
        self.channel.registerObject("bridge", self.bridge)
        self.view.page().setWebChannel(self.channel)

        here = os.path.dirname(os.path.abspath(__file__))
        index = os.path.join(here, "web", "index.html")
        self.view.setUrl(QUrl.fromLocalFile(index))
        layout.addWidget(self.view)

        self.zero_btn = QPushButton("Zero / Level")
        self.zero_btn.clicked.connect(lambda: self.bridge.zeroRequested.emit())
        layout.addWidget(self.zero_btn)

        # Yaw de-drift: yaw has no absolute reference (no magnetometer), so any
        # residual gyro-z bias shows as a slow spin. While the raw counts say the
        # sensor is physically still, any yaw change IS drift: freeze it out and
        # learn the creep rate; while moving, keep subtracting the learned rate.
        self._yaw_off = 0.0         # degrees subtracted from the device yaw
        self._drift_rate = 0.0      # learned creep in deg/s (slow EMA)
        self._still_since = None    # when the current stillness began
        self._last_yaw = None
        self._last_t = None

        # Push the latest quaternion to the page at ~30 Hz, independent of the
        # Monitor tab's refresh so the 3D view stays smooth.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

    def _tick(self):
        values, _ = self.listener.snapshot()
        if values and all(k in values for k in QUAT):
            q = tuple(float(values[k]) for k in QUAT)
            w, x, y, z = self._dedrift(q, values)
            self.bridge.orientation.emit(w, x, y, z)

    def _is_still(self, values):
        try:
            gyro = [abs(int(values[k])) for k in ("gx", "gy", "gz")]
            acc = [int(values[k]) for k in ("ax", "ay", "az")]
        except (KeyError, ValueError, TypeError):
            return False
        amag = math.sqrt(sum(v * v for v in acc))
        return max(gyro) < STILL_GYRO_COUNTS and abs(amag - 16384) < STILL_ACCEL_DELTA

    def _dedrift(self, q, values):
        now = time.monotonic()
        yaw = quat_yaw(*q)
        dt = (now - self._last_t) if self._last_t is not None else 0.0
        dyaw = wrap180(yaw - self._last_yaw) if self._last_yaw is not None else 0.0
        self._last_t, self._last_yaw = now, yaw

        if self._is_still(values):
            if self._still_since is None:
                self._still_since = now
            elif now - self._still_since >= STILL_MIN_SECS:
                self._yaw_off += dyaw          # still => this yaw change is drift
                if dt > 0:
                    self._drift_rate += 0.02 * (dyaw / dt - self._drift_rate)
        else:
            self._still_since = None
            self._yaw_off += self._drift_rate * dt
        self._yaw_off = wrap180(self._yaw_off)

        # undo the accumulated drift about the world vertical
        return quat_mul(quat_about_z(-self._yaw_off), q)


class ServoTab(QWidget):
    """Control panel + visualizer for the ESP32 servos.

    Each servo has a slider/spinbox to pick a target angle and an 'Upload'
    button that sends it. The read-only dials follow the angle the ESP32 echoes
    back, so they show the device's real held position.

    Each servo also has an Input dropdown to MIMIC the hand: pick an orientation
    axis (Roll / Pitch / Yaw) and that servo continuously tracks that component
    of the Visualizer's orientation -- decomposing the 3D motion into one servo
    per axis. 'Manual' leaves the servo under slider control. Different servos
    can be linked to different axes independently."""

    def __init__(self, listener):
        super().__init__()
        self.listener = listener
        self.sliders = []
        self.spins = []
        self.dials = []
        self.dial_labels = []
        self.inputs = []            # per-servo QComboBox of MIMIC_INPUTS
        self.enables = []           # per-servo enable/disable toggle buttons
        self._last_sent = None      # last angles sent while mimicking (throttle)

        root = QVBoxLayout(self)

        self.status_label = QLabel("Waiting for the ESP32 to announce itself...")
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        hint = QLabel("Set a servo's Input to Roll/Pitch/Yaw to mimic that axis of "
                      "the hand; 'Manual' uses the slider.")
        hint.setWordWrap(True)
        hint.setAlignment(Qt.AlignCenter)
        root.addWidget(hint)

        for i in range(NUM_SERVOS):
            box = QGroupBox(f"Servo {i}  (GPIO {SERVO_GPIOS[i]})")
            row = QHBoxLayout(box)

            combo = QComboBox()
            combo.addItems(MIMIC_INPUTS)
            combo.currentTextChanged.connect(lambda _t, idx=i: self._input_changed(idx))

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)

            spin = QSpinBox()
            spin.setRange(0, 180)
            spin.setValue(90)
            spin.setSuffix(" deg")

            # keep slider <-> spinbox mirrored without recursing
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            # live-send when the user finishes dragging the slider (manual mode)
            slider.sliderReleased.connect(self.upload)

            # enable/disable: disabling detaches the servo on the ESP32 (no PWM
            # pulses, the horn goes limp) - an active off, not just "stop sending"
            en_btn = QPushButton("Enabled")
            en_btn.setCheckable(True)
            en_btn.setChecked(True)
            en_btn.toggled.connect(lambda on, idx=i: self._toggle_enable(idx, on))

            dial = QDial()
            dial.setRange(0, 180)
            dial.setValue(90)
            dial.setNotchesVisible(True)
            dial.setEnabled(False)          # display only
            dial.setFixedSize(90, 90)
            dlabel = QLabel("-")
            dlabel.setAlignment(Qt.AlignCenter)

            dial_col = QVBoxLayout()
            dial_col.addWidget(dial)
            dial_col.addWidget(dlabel)

            row.addWidget(QLabel("Input"))
            row.addWidget(combo)
            row.addWidget(slider, 1)
            row.addWidget(spin)
            row.addWidget(en_btn)
            row.addLayout(dial_col)
            root.addWidget(box)

            self.inputs.append(combo)
            self.sliders.append(slider)
            self.spins.append(spin)
            self.enables.append(en_btn)
            self.dials.append(dial)
            self.dial_labels.append(dlabel)

        self.upload_btn = QPushButton("Upload to ESP32")
        self.upload_btn.clicked.connect(self.upload)
        root.addWidget(self.upload_btn)

        self.sent_label = QLabel("Nothing sent yet.")
        self.sent_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.sent_label)

        root.addStretch(1)

        # refresh the dials from echoed telemetry at ~20 Hz
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)

    def upload(self):
        angles = [s.value() for s in self.sliders]
        if self.listener.send_servos(angles):
            self.sent_label.setText(
                "Sent " + ", ".join(f"S{i}={a} deg" for i, a in enumerate(angles)))
        else:
            self.sent_label.setText(
                "Cannot send - not connected. Use the Connect tab first.")

    def _toggle_enable(self, idx, on):
        self.enables[idx].setText("Enabled" if on else "Disabled")
        if self.listener.send_line(f"e{idx}:{1 if on else 0}"):
            self.sent_label.setText(
                f"Servo {idx} {'enabled' if on else 'disabled (detached, limp)'}.")
        else:
            self.sent_label.setText(
                "Cannot send - not connected. Use the Connect tab first.")

    def _input_changed(self, idx):
        """A servo in mimic mode is driven by orientation, so lock its manual
        controls; 'Manual' re-enables them."""
        mimic = self.inputs[idx].currentText() != "Manual"
        self.sliders[idx].setEnabled(not mimic)
        self.spins[idx].setEnabled(not mimic)

    def _refresh(self):
        if self.listener.is_connected():
            self.status_label.setText(f"Connected via {self.listener.description()}")
        else:
            self.status_label.setText("Not connected - use the Connect tab.")

        values, _ = self.listener.snapshot()

        # Mimic: drive any servo whose Input is an orientation axis from the live
        # quaternion, then push the whole set to the ESP32 (throttled to changes).
        axes = [c.currentText() for c in self.inputs]
        if any(a != "Manual" for a in axes) and all(k in values for k in QUAT):
            roll, pitch, yaw = quat_to_euler(
                values["q0"], values["q1"], values["q2"], values["q3"])
            for i, axis in enumerate(axes):
                ang = axis_to_servo(axis, roll, pitch, yaw)
                if ang is not None:
                    self.sliders[i].setValue(ang)   # setValue doesn't trigger a send
            angles = [s.value() for s in self.sliders]
            if angles != self._last_sent and self.listener.send_servos(angles):
                self._last_sent = angles

        # Dials always reflect the ESP32's echoed angles (true held position);
        # the echoed e0/e1 flags flag a detached servo (older firmware: assume on).
        for i, key in enumerate(SERVO_KEYS):
            if key in values:
                angle = int(values[key])
                self.dials[i].setValue(angle)
                enabled = int(values.get(f"e{i}", 1)) == 1
                self.dial_labels[i].setText(f"{angle} deg" if enabled else f"{angle} deg (off)")
            else:
                self.dial_labels[i].setText("-")


class ConnectTab(QWidget):
    """Set up the ESP32's two channels, in two sections:

    * Bluetooth (sensor & servos) -- pick the paired COM port ("Find lifeOs port"
      auto-detects it), Connect, optionally Save as default. This is the always-on
      control link the Monitor/Visualizer/Servos tabs read from.
    * Wi-Fi (video) -- enter the Wi-Fi SSID/password and confirm the laptop IP,
      then Connect: these are sent to the ESP32 OVER BLUETOOTH, and the ESP32
      brings up Wi-Fi and streams video UDP back to that IP. Sending the laptop's
      current IP each time sidesteps the changing-DHCP-address problem."""

    def __init__(self, manager):
        super().__init__()
        self.manager = manager
        self.config = load_connect_config()
        self._bt_rows = []             # row -> (com_device, description)
        self._detected_bt = None       # COM port confirmed to be lifeOs, if any
        self._auto_bt_attempted = False
        self._auto_video_attempted = False
        self.prober = PortProber()
        self.prober.found.connect(self._on_probe_found)

        root = QVBoxLayout(self)

        title = QLabel("Connect to your ESP32")
        f = title.font()
        f.setPointSize(16)
        title.setFont(f)
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        self.status_label = QLabel("Not connected.")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)

        root.addWidget(self._build_bt_group())
        root.addWidget(self._build_video_group())
        root.addStretch(1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(1000)
        self.refresh_bt()

    # --- Bluetooth (sensor & servos) ---
    def _build_bt_group(self):
        box = QGroupBox("Bluetooth  -  sensor & servos")
        v = QVBoxLayout(box)
        self.bt_list = QListWidget()
        self.bt_list.itemDoubleClicked.connect(lambda _: self.connect_bt())
        v.addWidget(self.bt_list)
        row = QHBoxLayout()
        self.bt_refresh_btn = QPushButton("Refresh ports")
        self.bt_refresh_btn.clicked.connect(self.refresh_bt)
        self.bt_find_btn = QPushButton("Find lifeOs port")
        self.bt_find_btn.clicked.connect(self.find_lifeos)
        self.bt_connect_btn = QPushButton("Connect")
        self.bt_connect_btn.clicked.connect(self.connect_bt)
        self.bt_disconnect_btn = QPushButton("Disconnect")
        self.bt_disconnect_btn.clicked.connect(self.disconnect_bt)
        self.bt_default_btn = QPushButton("Save as default")
        self.bt_default_btn.clicked.connect(self.save_bt_default)
        for b in (self.bt_refresh_btn, self.bt_find_btn, self.bt_connect_btn,
                  self.bt_disconnect_btn, self.bt_default_btn):
            row.addWidget(b)
        v.addLayout(row)
        v.addWidget(QLabel("Pair the ESP32 (\"lifeos\") in Windows Bluetooth first, then "
                           "\"Find lifeOs port\" to auto-detect which COM it is."))
        return box

    def _populate_bt(self):
        prev = None
        r = self.bt_list.currentRow()
        if 0 <= r < len(self._bt_rows):
            prev = self._bt_rows[r][0]
        self.bt_list.clear()
        self._bt_rows = []
        for dev, desc in list_serial_ports():
            label = f"{dev}    {desc}"
            if dev == self._detected_bt:
                label += "   <- lifeOs"
            self.bt_list.addItem(label)
            self._bt_rows.append((dev, desc))
        if not self._bt_rows:
            self.bt_list.addItem("(no COM ports - pair the ESP32 first, or install pyserial)")
        for i, (dev, _d) in enumerate(self._bt_rows):
            if dev == prev:
                self.bt_list.setCurrentRow(i)
                break

    def refresh_bt(self):
        self._populate_bt()

    def find_lifeos(self):
        """Probe every COM port for the lifeOs identity so the user doesn't have
        to guess which one it is. Skips a port we already hold open."""
        skip = set()
        if isinstance(self.manager.active, BluetoothLink):
            skip.add(self.manager.active.com)
        self.bt_find_btn.setEnabled(False)
        self.bt_find_btn.setText("Probing...")
        self.status_label.setText("Probing COM ports for lifeOs (opens each briefly)...")
        self.prober.probe(skip=skip)

    def _on_probe_found(self, dev):
        self.bt_find_btn.setEnabled(True)
        self.bt_find_btn.setText("Find lifeOs port")
        if dev:
            self._detected_bt = dev
            self._populate_bt()
            for i, (d, _desc) in enumerate(self._bt_rows):
                if d == dev:
                    self.bt_list.setCurrentRow(i)
                    break
            self.status_label.setText(f"Found lifeOs on {dev}. Press Connect.")
        else:
            self.status_label.setText(
                "No lifeOs device found on any COM port. Is it paired and powered on?")

    def connect_bt(self):
        r = self.bt_list.currentRow()
        if not (0 <= r < len(self._bt_rows)):
            self.status_label.setText("Select a COM port first (or Refresh ports).")
            return
        self._connect_bt(self._bt_rows[r][0])

    def _connect_bt(self, com):
        try:
            link = BluetoothLink(com)
            link.start()                        # opens the serial port; may raise
        except Exception as e:
            self.status_label.setText(f"Could not open {com}: {e}")
            return
        self.manager.set_active(link)
        self.status_label.setText(f"Opened {com} - waiting for sensor telemetry...")

    def disconnect_bt(self):
        """Active disconnect: closing the COM port tears the SPP link down, so
        the ESP32 sees its Bluetooth client vanish (hasClient() goes false)."""
        if not isinstance(self.manager.active, BluetoothLink):
            self.status_label.setText("No Bluetooth link to disconnect.")
            return
        com = self.manager.active.com
        self.manager.disconnect()
        self.status_label.setStyleSheet("")
        self.status_label.setText(
            f"Bluetooth disconnected ({com} closed). Wi-Fi video, if running, "
            f"continues until you stop it or re-provision.")

    def save_bt_default(self):
        r = self.bt_list.currentRow()
        if not (0 <= r < len(self._bt_rows)):
            self.status_label.setText("Select a COM port to save as default.")
            return
        self.config["bt_port"] = self._bt_rows[r][0]
        save_connect_config(self.config)
        self.status_label.setText(f"Saved {self._bt_rows[r][0]} as the default Bluetooth port.")

    # --- Wi-Fi (video) ---
    def _build_video_group(self):
        box = QGroupBox("Wi-Fi  -  video")
        form = QVBoxLayout(box)

        ssid_row = QHBoxLayout()
        ssid_row.addWidget(QLabel("SSID:"))
        self.ssid_edit = QLineEdit(self.config.get("video_ssid", ""))
        self.ssid_edit.setPlaceholderText("your Wi-Fi network name")
        ssid_row.addWidget(self.ssid_edit, 1)
        form.addLayout(ssid_row)

        pass_row = QHBoxLayout()
        pass_row.addWidget(QLabel("Password:"))
        self.pass_edit = QLineEdit(self.config.get("video_pass", ""))
        self.pass_edit.setEchoMode(QLineEdit.Password)
        pass_row.addWidget(self.pass_edit, 1)
        form.addLayout(pass_row)

        ip_row = QHBoxLayout()
        ip_row.addWidget(QLabel("Laptop IP:"))
        self.ip_edit = QLineEdit(get_local_ip())
        self.ip_edit.setToolTip("The ESP32 sends video UDP here (auto-detected; edit if wrong).")
        ip_row.addWidget(self.ip_edit, 1)
        form.addLayout(ip_row)

        row = QHBoxLayout()
        self.video_connect_btn = QPushButton("Connect (start video)")
        self.video_connect_btn.clicked.connect(self.connect_video)
        self.video_disconnect_btn = QPushButton("Disconnect (stop video)")
        self.video_disconnect_btn.clicked.connect(self.disconnect_video)
        self.video_default_btn = QPushButton("Save as default")
        self.video_default_btn.clicked.connect(self.save_video_default)
        row.addWidget(self.video_connect_btn)
        row.addWidget(self.video_disconnect_btn)
        row.addWidget(self.video_default_btn)
        form.addLayout(row)

        form.addWidget(QLabel(f"Credentials go to the ESP32 over Bluetooth; it then streams video "
                              f"UDP to Laptop IP:{VIDEO_PORT}. Verify with wifi_video_test.py."))
        return box

    def connect_video(self):
        if not isinstance(self.manager.active, BluetoothLink):
            self.status_label.setText(
                "Connect over Bluetooth first - Wi-Fi credentials are sent to the ESP32 over Bluetooth.")
            return
        ssid = self.ssid_edit.text().strip()
        password = self.pass_edit.text()
        ip = self.ip_edit.text().strip() or get_local_ip()
        if not ssid or not ip:
            self.status_label.setText("Enter the Wi-Fi SSID and confirm the laptop IP.")
            return
        if self.manager.active.provision_video(ssid, password, ip):
            self.status_label.setText(
                f"Sent Wi-Fi creds to ESP32 (SSID '{ssid}', video -> {ip}:{VIDEO_PORT}). "
                f"Run wifi_video_test.py to verify.")
        else:
            self.status_label.setText("Failed to send credentials over Bluetooth.")

    def disconnect_video(self):
        """Active disconnect: 'wifi:off' over Bluetooth makes the ESP32 stop the
        video stream and switch its Wi-Fi radio off (until re-provisioned),
        instead of us merely ignoring incoming packets."""
        if not isinstance(self.manager.active, BluetoothLink):
            self.status_label.setText(
                "Bluetooth link needed to stop video - 'wifi:off' is sent over Bluetooth.")
            return
        if self.manager.send_line("wifi:off"):
            self.status_label.setText(
                "Sent wifi:off - the ESP32 stops streaming and turns its Wi-Fi off.")
        else:
            self.status_label.setText("Failed to send wifi:off over Bluetooth.")

    def save_video_default(self):
        self.config["video_ssid"] = self.ssid_edit.text().strip()
        self.config["video_pass"] = self.pass_edit.text()
        save_connect_config(self.config)
        self.status_label.setText("Saved Wi-Fi video defaults.")

    # --- shared ---
    def _tick(self):
        if self.manager.is_connected():
            self.status_label.setText(f"Connected - {self.manager.description()}")
            self.status_label.setStyleSheet("color: #2e7d32;")

        # auto-connect the saved Bluetooth port once it appears
        if not self._auto_bt_attempted:
            com = self.config.get("bt_port")
            if com and any(dev == com for dev, _ in list_serial_ports()):
                self._auto_bt_attempted = True
                self._connect_bt(com)

        # once Bluetooth is up, auto-provision saved Wi-Fi video creds once
        if (not self._auto_video_attempted and self.config.get("video_ssid")
                and isinstance(self.manager.active, BluetoothLink)
                and self.manager.is_connected()):
            self._auto_video_attempted = True
            self.connect_video()


class HandModelTab(QWidget):
    """Video-input preview for the future hand-detection model. The dropdown
    lists every video input device on this PC (e.g. the laptop webcam); Apply
    streams the selected one into the preview box next to the selector. The
    ESP32's Wi-Fi UDP video will be added as another selectable source later."""

    def __init__(self):
        super().__init__()
        self._devices = []          # QCameraDevice per combo row
        self._camera = None
        self._session = None

        root = QVBoxLayout(self)

        if not HAVE_MULTIMEDIA:
            msg = QLabel("PySide6 QtMultimedia is not available - reinstall "
                         "PySide6 to use video input here.")
            msg.setWordWrap(True)
            msg.setAlignment(Qt.AlignCenter)
            root.addWidget(msg)
            root.addStretch(1)
            return

        row = QHBoxLayout()

        controls = QVBoxLayout()
        controls.addWidget(QLabel("Video input device:"))
        self.device_combo = QComboBox()
        controls.addWidget(self.device_combo)

        btn_row = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_devices)
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self.apply)
        btn_row.addWidget(self.refresh_btn)
        btn_row.addWidget(self.apply_btn)
        controls.addLayout(btn_row)

        self.status_label = QLabel("Pick a device and press Apply.")
        self.status_label.setWordWrap(True)
        controls.addWidget(self.status_label)
        controls.addStretch(1)
        row.addLayout(controls, 1)

        self.video = QVideoWidget()
        self.video.setFixedSize(320, 240)     # small preview beside the selector
        row.addWidget(self.video)

        root.addLayout(row)
        root.addStretch(1)

        self.refresh_devices()

    def refresh_devices(self):
        prev = self.device_combo.currentText()
        self.device_combo.clear()
        self._devices = list(QMediaDevices.videoInputs())
        for dev in self._devices:
            self.device_combo.addItem(dev.description())
        if not self._devices:
            self.device_combo.addItem("(no video input devices found)")
        idx = self.device_combo.findText(prev)
        if idx >= 0:
            self.device_combo.setCurrentIndex(idx)

    def apply(self):
        idx = self.device_combo.currentIndex()
        if not (0 <= idx < len(self._devices)):
            self.status_label.setText("No video input device selected.")
            return
        self.stop()
        dev = self._devices[idx]
        self._camera = QCamera(dev)
        self._camera.errorOccurred.connect(self._on_camera_error)
        self._session = QMediaCaptureSession(self)
        self._session.setCamera(self._camera)
        self._session.setVideoOutput(self.video)
        self._camera.start()
        self.status_label.setText(f"Streaming: {dev.description()}")

    def _on_camera_error(self, _error, message):
        self.status_label.setText(f"Camera error: {message}")

    def stop(self):
        """Release the camera (so other apps can use it / on window close)."""
        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass
            self._camera = None
        self._session = None


class StatusDot(QLabel):
    """A small colored box in the status bar. Its source color is fixed (so you
    can always tell which is which); a tick (live) or cross (no signal in the
    last second) shows the state."""

    def __init__(self, name, color):
        super().__init__()
        self.name = name
        self.color = color
        self.setFixedSize(30, 20)
        self.setAlignment(Qt.AlignCenter)
        font = self.font()
        font.setBold(True)
        self.setFont(font)
        self.set_live(False)

    def set_live(self, live):
        self.setText("✓" if live else "✗")   # tick / cross
        self.setStyleSheet(
            f"background-color: {self.color}; color: white; "
            f"border-radius: 3px; border: 1px solid #222;")
        self.setToolTip(f"{self.name}: {'live' if live else 'no signal (>1s)'}")


class MonitorWindow(QMainWindow):
    def __init__(self, listener):
        super().__init__()
        self.listener = listener
        self.setWindowTitle("lifeOs Monitor")

        # receiver-side zeroing: offsets are subtracted from every raw reading
        self.offsets = {name: 0 for name in CALIB_SENSORS}
        self.calib_accum = None          # running per-axis sum during a recalibration
        self.calib_left = 0              # fresh samples still needed to finish
        self._calib_last_seen = None     # last_seen of the most recent accumulated sample
        self._cal_waiting_device = False # True while the ESP32 runs its own bias cal
        self._cal_sent_at = 0.0          # when the 'cal' command was sent

        central_widget = QWidget()
        layout = QVBoxLayout()

        self.status_label = QLabel("Disconnected")
        font = self.status_label.font()
        font.setPointSize(16)
        self.status_label.setFont(font)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.clean = CleanInput()
        self.table = QTableWidget(len(SENSORS), 4)
        self.table.setHorizontalHeaderLabels(["Sensor", "Value", "Converted", "History"])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(36)   # room for the sparklines
        self.sparks = {}                 # sensor name -> its History-column Sparkline
        for i, name in enumerate(SENSORS):
            self.table.setItem(i, 0, QTableWidgetItem("temp" if name == "tp" else name))
            self.table.setItem(i, 1, QTableWidgetItem("-"))
            self.table.setItem(i, 2, QTableWidgetItem("-"))
            self.sparks[name] = sparkline_for(name)
            self.table.setCellWidget(i, 3, self.sparks[name])
        layout.addWidget(self.table)

        # --- Relative state: angle from the zero pose per axis, plus the die
        # temperature change since the stream started (self-heating indicator:
        # recalibrate once it flattens). The zero pose is captured when Reset /
        # Recalibrate locks in; before that, angles are relative to the DMP
        # startup pose. ---
        self._euler_zero = None
        self._temp_base = None           # first temperature (degC) seen this session
        self.angle_table = QTableWidget(4, 2)
        self.angle_table.setHorizontalHeaderLabels(["Metric", "Delta from zero"])
        self.angle_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.angle_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for i, metric in enumerate(("Roll", "Pitch", "Yaw", "Delta Temp")):
            self.angle_table.setItem(i, 0, QTableWidgetItem(metric))
            self.angle_table.setItem(i, 1, QTableWidgetItem("-"))
        self.angle_table.setFixedHeight(
            self.angle_table.rowCount() * self.angle_table.verticalHeader().defaultSectionSize()
            + self.angle_table.horizontalHeader().sizeHint().height()
            + 2 * self.angle_table.frameWidth())
        layout.addWidget(self.angle_table)

        # --- 3D orientation view, right below the relative-state table (takes
        # any spare vertical space; the tables above keep their natural size) ---
        self.visualizer = VisualizerPanel(self.listener)
        self.visualizer.setMinimumHeight(240)
        layout.addWidget(self.visualizer, 1)

        # --- Reset / recalibrate (zero the sensor) ---
        self.reset_btn = QPushButton("Reset / Recalibrate")
        self.reset_btn.clicked.connect(self.start_calibration)
        layout.addWidget(self.reset_btn)

        self.calib_status_label = QLabel("Press Reset to zero the sensor (hold it still).")
        self.calib_status_label.setWordWrap(True)
        self.calib_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.calib_status_label)

        central_widget.setLayout(layout)

        tabs = QTabWidget()
        tabs.setTabBar(StretchTabBar())
        # Document mode hands the tab bar the QTabWidget's full width (instead
        # of shrink-wrapping it to the tabs); StretchTabBar then splits that
        # width equally, so the selectors cover the window edge-to-edge.
        tabs.setDocumentMode(True)
        self.connect_tab = ConnectTab(self.listener)
        tabs.addTab(self.connect_tab, "Connect")
        tabs.addTab(central_widget, "Monitor")
        self.servos = ServoTab(self.listener)
        tabs.addTab(self.servos, "Servos")
        self.hand_model = HandModelTab()
        tabs.addTab(self.hand_model, "Hand Model")
        self.setCentralWidget(tabs)

        self._build_status_bar()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def _build_status_bar(self):
        """Bottom-left indicators, one colored box per source, tick/cross by
        whether we've had a live signal in the last second."""
        bar = self.statusBar()
        self.dot_bt = StatusDot("Bluetooth", "#1565c0")   # blue
        self.dot_wifi = StatusDot("Wi-Fi", "#2e7d32")     # green
        self.dot_s1 = StatusDot("Servo 1", "#c62828")     # red
        self.dot_s2 = StatusDot("Servo 2", "#c62828")     # red
        self.dot_mpu = StatusDot("MPU", "#f9a825")        # yellow
        for caption, dot in [("BT", self.dot_bt), ("WiFi", self.dot_wifi),
                             ("S1", self.dot_s1), ("S2", self.dot_s2), ("MPU", self.dot_mpu)]:
            bar.addWidget(QLabel(caption))
            bar.addWidget(dot)

    def _update_status_dots(self, values, connected):
        # Everything is reported over the (Bluetooth) telemetry stream, so all
        # dots are cross when it isn't fresh. `connected` == signal within 1s.
        is_bt = isinstance(getattr(self.listener, "active", None), BluetoothLink)
        self.dot_bt.set_live(connected and is_bt)
        self.dot_wifi.set_live(connected and int(values.get("wf", 0)) == 1)
        # a disabled (detached) servo counts as not live; older firmware without
        # the e0/e1 fields is treated as enabled
        self.dot_s1.set_live(connected and "s0" in values and int(values.get("e0", 1)) == 1)
        self.dot_s2.set_live(connected and "s1" in values and int(values.get("e1", 1)) == 1)
        # dmp defaults to 1 for older firmware that doesn't send the field
        self.dot_mpu.set_live(connected and "q0" in values and int(values.get("dmp", 1)) == 1)

    def start_calibration(self):
        """Recalibrate at the source first: ask the ESP32 to re-run its
        accel/gyro bias calibration ('cal'), then re-zero the displayed raw
        counts once it's done. Falls back to display-only zeroing when the
        link can't carry the command (e.g. legacy Wi-Fi sensor mode)."""
        if self.listener.is_connected() and self.listener.send_line("cal"):
            self._cal_waiting_device = True
            self._cal_sent_at = time.monotonic()
            self.calib_left = 0          # cancel any local averaging in progress
            self.calib_status_label.setText(
                "Recalibrating biases on the ESP32 - keep the sensor still "
                "(telemetry pauses ~2 s)...")
        else:
            self._begin_local_zero()

    def _begin_local_zero(self):
        """Begin averaging the next CALIB_SAMPLES fresh packets into a new zero baseline."""
        _, last_seen = self.listener.snapshot()
        self.calib_accum = {name: 0 for name in CALIB_SENSORS}
        self.calib_left = CALIB_SAMPLES
        self._calib_last_seen = last_seen  # only count packets newer than this
        self.calib_status_label.setText("Calibrating... hold the sensor still")

    def refresh(self):
        connected = self.listener.is_connected()
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("background-color: #2e7d32; color: white; padding: 8px;")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("background-color: #c62828; color: white; padding: 8px;")

        values, last_seen = self.listener.snapshot()
        self._update_status_dots(values, connected)   # from the raw snapshot (all fields)
        have_full = bool(values) and all(name in values for name in CALIB_SENSORS)
        quat = (tuple(float(values[k]) for k in QUAT)
                if all(k in values for k in QUAT) else None)

        # device-side calibration in flight: wait for the ESP32's cal reply,
        # then re-zero the display against the freshly calibrated stream
        if self._cal_waiting_device:
            reply, ts = self.listener.control("cal:")
            if reply == "cal:done" and ts >= self._cal_sent_at:
                self._cal_waiting_device = False
                self._begin_local_zero()
            elif reply and reply.startswith("cal:error") and ts >= self._cal_sent_at:
                self._cal_waiting_device = False
                self._begin_local_zero()
                self.calib_status_label.setText(
                    f"ESP32 refused ({reply}) - zeroing the display only.")
            elif time.monotonic() - self._cal_sent_at > DEVICE_CAL_TIMEOUT:
                self._cal_waiting_device = False
                self._begin_local_zero()
                self.calib_status_label.setText(
                    "No cal reply from the ESP32 - zeroing the display only.")

        # recalibration: accumulate only fresh, complete packets, then lock in offsets
        if self.calib_left > 0 and have_full and last_seen != self._calib_last_seen:
            self._calib_last_seen = last_seen
            for name in CALIB_SENSORS:
                self.calib_accum[name] += values[name]
            self.calib_left -= 1
            if self.calib_left == 0:
                self.offsets = {name: round(self.calib_accum[name] / CALIB_SAMPLES)
                                for name in CALIB_SENSORS}
                if quat:
                    self._euler_zero = quat_to_euler(*quat)   # zero pose for the angle table
                self.calib_status_label.setText("Calibrated. New zero set.")
            else:
                self.calib_status_label.setText(
                    f"Calibrating... {CALIB_SAMPLES - self.calib_left}/{CALIB_SAMPLES}")

        # subtract the zero baseline before anything downstream sees the values
        # (`values` is the snapshot's copy, so mutating it is safe; temperature
        # and the quaternion pass through un-zeroed)
        if have_full:
            for name in CALIB_SENSORS:
                values[name] -= self.offsets[name]

        sample = self.clean.update(values, last_seen)
        for i, name in enumerate(SENSORS):
            if name in values:
                self.table.item(i, 1).setText(str(values[name]))
            else:
                self.table.item(i, 1).setText("-")
            conv_item = self.table.item(i, 2)
            if sample.has_data and name in sample.raw:
                conv_item.setText(format_converted(name, sample.raw[name]))
                conv_item.setForeground(QColor(COLOR_ASSUMED if sample.assumed else COLOR_REAL))
            else:
                conv_item.setText("-")
                conv_item.setForeground(QColor(COLOR_NODATA))

        if sample.has_data:
            for name, spark in self.sparks.items():
                if name in sample.converted:
                    spark.add_value(sample.converted[name][0])

        self._update_angle_table(quat, values, connected)

    def _update_angle_table(self, quat, values, connected):
        """Show roll/pitch/yaw relative to the captured zero pose, and the die
        temperature change since the first reading (green while live, red when
        the value is stale, grey before any data)."""
        color = QColor(COLOR_REAL if connected else COLOR_ASSUMED)

        if quat is None:
            for i in range(3):
                item = self.angle_table.item(i, 1)
                item.setText("-")
                item.setForeground(QColor(COLOR_NODATA))
        else:
            current = quat_to_euler(*quat)
            zero = self._euler_zero or (0.0, 0.0, 0.0)
            for i, (cur, ref) in enumerate(zip(current, zero)):
                item = self.angle_table.item(i, 1)
                item.setText(f"{wrap180(cur - ref):+.1f} deg")
                item.setForeground(color)

        temp_item = self.angle_table.item(3, 1)
        if "tp" in values:
            temp_c = convert("tp", values["tp"])[0]
            if self._temp_base is None:
                self._temp_base = temp_c
            temp_item.setText(f"{temp_c - self._temp_base:+.2f} °C")
            temp_item.setForeground(color)
        else:
            temp_item.setText("-")
            temp_item.setForeground(QColor(COLOR_NODATA))

    def closeEvent(self, event):
        try:
            self.listener.close()    # stop the active link (socket / COM port)
        except Exception:
            pass
        try:
            self.hand_model.stop()   # release the webcam
        except Exception:
            pass
        super().closeEvent(event)


def main():
    # QtWebEngine wants shared GL contexts set before the QApplication exists.
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)
    manager = ConnectionManager()
    win = MonitorWindow(manager)
    win.resize(720, 860)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
