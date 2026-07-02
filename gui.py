import sys
import os
import json
import socket
import subprocess
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
from zeroconf import Zeroconf, ServiceBrowser, ServiceListener

from CleanInput import CleanInput, format_converted
from live_charts import ChartPanel

QUAT = ["q0", "q1", "q2", "q3"]

SENSORS = ["ax", "ay", "az", "gx", "gy", "gz"]

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

COLOR_REAL = "#2e7d32"      # green - value came from a fresh packet
COLOR_ASSUMED = "#c62828"   # red   - value was held over (missing/garbled)
COLOR_NODATA = "#9e9e9e"    # grey  - no packet received yet


def find_port_users(port):
    try:
        result = subprocess.run(
            ["netstat", "-ano", "-p", "UDP"],
            capture_output=True,
            text=True,
            creationflags=0x08000000  # CREATE_NO_WINDOW
        )
        lines = result.stdout.strip().split('\n')
        pids = set()

        for line in lines:
            parts = line.split()
            if len(parts) >= 4 and parts[0] == "UDP" and parts[1].endswith(f":{port}"):
                pid = parts[-1]
                pids.add(pid)

        users = []
        for pid in pids:
            try:
                tasklist_result = subprocess.run(
                    ["tasklist", "/FI", f"PID eq {pid}", "/NH", "/FO", "CSV"],
                    capture_output=True,
                    text=True,
                    creationflags=0x08000000  # CREATE_NO_WINDOW
                )
                tlines = tasklist_result.stdout.strip().split('\n')
                if tlines:
                    name = tlines[0].split(',')[0].strip().strip('"')
                    users.append((pid, name))
            except Exception:
                users.append((pid, "unknown"))

        return users

    except Exception:
        return []


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

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.last_seen = 0.0

    def _ingest(self, text):
        try:
            d = parse_line(text)
        except Exception:
            return
        with self.lock:
            self.latest = d
            self.last_seen = time.monotonic()

    def snapshot(self):
        with self.lock:
            return (self.latest.copy(), self.last_seen)

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

    def send_servos(self, angles):
        if not self.ser:
            return False
        msg = ",".join(f"s{i}:{int(a)}" for i, a in enumerate(angles)) + "\n"
        try:
            self.ser.write(msg.encode())
            return True
        except Exception:
            return False

    def provision_video(self, ssid, password, ip):
        """Send Wi-Fi credentials + the laptop's IP so the ESP32 brings up Wi-Fi
        and streams video UDP back to us. Format: 'wifi:<ssid>|<pass>|<ip>'."""
        if not self.ser:
            return False
        msg = f"wifi:{ssid}|{password}|{ip}\n"
        try:
            self.ser.write(msg.encode())
            return True
        except Exception:
            return False

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

    def description(self):
        return self.active.description() if self.active else "Not connected"

    def close(self):
        if self.active:
            self.active.stop()


class StretchTabBar(QTabBar):
    """Tab bar whose tabs divide the full widget width equally, so the tab
    selectors span the entire window instead of hugging the top-left."""

    def tabSizeHint(self, index):
        hint = super().tabSizeHint(index)
        count = self.count()
        if count > 0:
            hint.setWidth(max(hint.width(), self.width() // count))
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


class VisualizerTab(QWidget):
    """A QWebEngineView hosting the three.js scene, fed the device quaternion."""

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

        # Push the latest quaternion to the page at ~30 Hz, independent of the
        # Monitor tab's refresh so the 3D view stays smooth.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

    def _tick(self):
        values, _ = self.listener.snapshot()
        if values and all(k in values for k in QUAT):
            self.bridge.orientation.emit(
                float(values["q0"]), float(values["q1"]),
                float(values["q2"]), float(values["q3"]),
            )


class ServoTab(QWidget):
    """Control panel + visualizer for the ESP32 servos.

    Each servo has a slider/spinbox to pick a target angle and an 'Upload'
    button that sends it over the reverse UDP channel. The read-only dials do
    NOT follow the sliders directly -- they follow the angle the ESP32 echoes
    back in its telemetry, so the visualizer always reflects the device's real
    held position (even across a GUI restart or a dropped command)."""

    def __init__(self, listener):
        super().__init__()
        self.listener = listener
        self.sliders = []
        self.spins = []
        self.dials = []
        self.dial_labels = []

        root = QVBoxLayout(self)

        self.status_label = QLabel("Waiting for the ESP32 to announce itself...")
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        for i in range(NUM_SERVOS):
            box = QGroupBox(f"Servo {i}  (GPIO {SERVO_GPIOS[i]})")
            row = QHBoxLayout(box)

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
            # live-send when the user finishes dragging the slider
            slider.sliderReleased.connect(self.upload)

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

            row.addWidget(slider, 1)
            row.addWidget(spin)
            row.addLayout(dial_col)
            root.addWidget(box)

            self.sliders.append(slider)
            self.spins.append(spin)
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

    def _refresh(self):
        if self.listener.is_connected():
            self.status_label.setText(f"Connected via {self.listener.description()}")
        else:
            self.status_label.setText("Not connected - use the Connect tab.")

        values, _ = self.listener.snapshot()
        for i, key in enumerate(SERVO_KEYS):
            if key in values:
                angle = int(values[key])
                self.dials[i].setValue(angle)
                self.dial_labels[i].setText(f"{angle} deg")
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
        self.bt_default_btn = QPushButton("Save as default")
        self.bt_default_btn.clicked.connect(self.save_bt_default)
        for b in (self.bt_refresh_btn, self.bt_find_btn, self.bt_connect_btn, self.bt_default_btn):
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
        self.video_default_btn = QPushButton("Save as default")
        self.video_default_btn.clicked.connect(self.save_video_default)
        row.addWidget(self.video_connect_btn)
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


class MonitorWindow(QMainWindow):
    def __init__(self, listener):
        super().__init__()
        self.listener = listener
        self.setWindowTitle("lifeOs Monitor")

        # receiver-side zeroing: offsets are subtracted from every raw reading
        self.offsets = {name: 0 for name in SENSORS}
        self.calib_accum = None          # running per-axis sum during a recalibration
        self.calib_left = 0              # fresh samples still needed to finish
        self._calib_last_seen = None     # last_seen of the most recent accumulated sample

        central_widget = QWidget()
        layout = QVBoxLayout()

        self.status_label = QLabel("Disconnected")
        font = self.status_label.font()
        font.setPointSize(16)
        self.status_label.setFont(font)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.clean = CleanInput()
        self.table = QTableWidget(6, 3)
        self.table.setHorizontalHeaderLabels(["Sensor", "Value", "Converted"])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for i, name in enumerate(SENSORS):
            self.table.setItem(i, 0, QTableWidgetItem(name))
            self.table.setItem(i, 1, QTableWidgetItem("-"))
            self.table.setItem(i, 2, QTableWidgetItem("-"))
        layout.addWidget(self.table)

        # --- Reset / recalibrate (zero the sensor) ---
        self.reset_btn = QPushButton("Reset / Recalibrate")
        self.reset_btn.clicked.connect(self.start_calibration)
        layout.addWidget(self.reset_btn)

        self.calib_status_label = QLabel("Press Reset to zero the sensor (hold it still).")
        self.calib_status_label.setWordWrap(True)
        self.calib_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.calib_status_label)

        self.charts = ChartPanel()
        layout.addWidget(self.charts)

        # --- Port diagnostics (Wi-Fi mode) ---
        self.check_btn = QPushButton(f"Refresh / Check Port {UDP_PORT}")
        self.check_btn.clicked.connect(self.check_port)
        layout.addWidget(self.check_btn)

        self.port_result_label = QLabel("Click the button above to check who is using the port.")
        self.port_result_label.setWordWrap(True)
        layout.addWidget(self.port_result_label)

        kill_row = QHBoxLayout()
        self.kill_cmd_edit = QLineEdit()
        self.kill_cmd_edit.setReadOnly(True)
        self.kill_cmd_edit.setPlaceholderText("kill command will appear here when the port is busy")
        kill_row.addWidget(self.kill_cmd_edit)
        self.copy_btn = QPushButton("Copy")
        self.copy_btn.clicked.connect(self.copy_kill_cmd)
        kill_row.addWidget(self.copy_btn)
        layout.addLayout(kill_row)

        central_widget.setLayout(layout)

        tabs = QTabWidget()
        tabs.setTabBar(StretchTabBar())        # tabs span the full window width
        self.connect_tab = ConnectTab(self.listener)
        tabs.addTab(self.connect_tab, "Connect")
        tabs.addTab(central_widget, "Monitor")
        self.visualizer = VisualizerTab(self.listener)
        tabs.addTab(self.visualizer, "Visualizer")
        self.servos = ServoTab(self.listener)
        tabs.addTab(self.servos, "Servos")
        self.setCentralWidget(tabs)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def start_calibration(self):
        """Begin averaging the next CALIB_SAMPLES fresh packets into a new zero baseline."""
        _, last_seen = self.listener.snapshot()
        self.calib_accum = {name: 0 for name in SENSORS}
        self.calib_left = CALIB_SAMPLES
        self._calib_last_seen = last_seen  # only count packets newer than this
        self.calib_status_label.setText("Calibrating... hold the sensor still")

    def refresh(self):
        if self.listener.is_connected():
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("background-color: #2e7d32; color: white; padding: 8px;")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("background-color: #c62828; color: white; padding: 8px;")

        values, last_seen = self.listener.snapshot()
        have_full = bool(values) and all(name in values for name in SENSORS)

        # recalibration: accumulate only fresh, complete packets, then lock in offsets
        if self.calib_left > 0 and have_full and last_seen != self._calib_last_seen:
            self._calib_last_seen = last_seen
            for name in SENSORS:
                self.calib_accum[name] += values[name]
            self.calib_left -= 1
            if self.calib_left == 0:
                self.offsets = {name: round(self.calib_accum[name] / CALIB_SAMPLES)
                                for name in SENSORS}
                self.calib_status_label.setText("Calibrated. New zero set.")
            else:
                self.calib_status_label.setText(
                    f"Calibrating... {CALIB_SAMPLES - self.calib_left}/{CALIB_SAMPLES}")

        # subtract the zero baseline before anything downstream sees the values
        if have_full:
            values = {name: values[name] - self.offsets[name] for name in SENSORS}

        sample = self.clean.update(values, last_seen)
        for i, name in enumerate(SENSORS):
            if name in values:
                self.table.item(i, 1).setText(str(values[name]))
            else:
                self.table.item(i, 1).setText("-")
            conv_item = self.table.item(i, 2)
            if sample.has_data:
                conv_item.setText(format_converted(name, sample.raw[name]))
                conv_item.setForeground(QColor(COLOR_ASSUMED if sample.assumed else COLOR_REAL))
            else:
                conv_item.setText("-")
                conv_item.setForeground(QColor(COLOR_NODATA))

        if sample.has_data:
            self.charts.add_sample(sample.converted)

    def check_port(self):
        port = UDP_PORT
        users = find_port_users(port)
        me = os.getpid()

        if not users:
            self.port_result_label.setText(f"Port {port} is FREE.")
            self.kill_cmd_edit.setText("")
        else:
            summary = ", ".join(f"PID {pid} ({name})" for pid, name in users)
            if len(users) == 1 and int(users[0][0]) == me:
                self.port_result_label.setText(f"Port {port} is in use by THIS GUI (PID {me}) - expected.")
            else:
                self.port_result_label.setText(f"Port {port} in use by: {summary}")

            others = [pid for pid, _ in users if int(pid) != me]
            if others:
                self.kill_cmd_edit.setText("taskkill /F " + " ".join(f"/PID {pid}" for pid in others))
            else:
                self.kill_cmd_edit.setText("")

    def copy_kill_cmd(self):
        cmd = self.kill_cmd_edit.text()
        if cmd:
            QApplication.clipboard().setText(cmd)

    def closeEvent(self, event):
        try:
            self.listener.close()    # stop the active link (socket / COM port)
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
