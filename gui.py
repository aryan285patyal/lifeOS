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
UDP_PORT = 5005                       # telemetry port (WiFi mode); matches UDP_PORT in lifeOs.ino
CMD_PORT = 5006                       # must match CMD_PORT in lifeOs.ino

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


def list_serial_ports():
    """[(device, description)] of COM ports, or [] if pyserial isn't installed."""
    try:
        import serial.tools.list_ports as lp
    except Exception:
        return []
    return [(p.device, p.description) for p in lp.comports()]

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
    """Pick a transport (Bluetooth or Wi-Fi) and connect to the ESP32.

    Bluetooth: lists paired COM ports; Connect opens the port and streams over
    SPP -- network-independent, keeps the laptop's internet free.
    Wi-Fi: mDNS device list + a manual IP/hostname fallback; Connect sends a
    'hello' that starts the two-way UDP stream.
    Either way the chosen link is handed to the ConnectionManager, so the other
    tabs are unaffected. A remembered default auto-connects on launch."""

    def __init__(self, manager, discovery):
        super().__init__()
        self.manager = manager
        self.discovery = discovery
        self.config = load_connect_config()
        self._wifi_rows = []       # row -> (service_name, ip, port)
        self._bt_rows = []         # row -> (com_device, description)
        self._last_keys = None
        self._auto_attempted = False

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

        # transport selector -> switches the panel below
        trow = QHBoxLayout()
        trow.addWidget(QLabel("Transport:"))
        self.transport = QComboBox()
        self.transport.addItem("Bluetooth (SPP)", "bluetooth")   # index 0
        self.transport.addItem("Wi-Fi (UDP)", "wifi")            # index 1
        self.transport.currentIndexChanged.connect(self.stack_index_changed)
        trow.addWidget(self.transport, 1)
        root.addLayout(trow)

        self.stack = QStackedWidget()
        self.stack.addWidget(self._build_bt_page())    # index 0
        self.stack.addWidget(self._build_wifi_page())  # index 1
        root.addWidget(self.stack, 1)

        self.default_chk = QCheckBox("Make this my default connection")
        self.default_chk.setChecked(bool(self.config.get("default_transport")))
        root.addWidget(self.default_chk)

        # restore the last-used transport
        want = self.config.get("default_transport", "bluetooth")
        idx = 1 if want == "wifi" else 0
        self.transport.setCurrentIndex(idx)
        self.stack.setCurrentIndex(idx)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(1000)
        self.refresh_wifi()
        self.refresh_bt()

    def stack_index_changed(self, idx):
        self.stack.setCurrentIndex(idx)

    # --- page builders ---
    def _build_bt_page(self):
        page = QWidget()
        v = QVBoxLayout(page)
        v.addWidget(QLabel("Paired Bluetooth COM ports:"))
        self.bt_list = QListWidget()
        self.bt_list.itemDoubleClicked.connect(lambda _: self.connect_bt())
        v.addWidget(self.bt_list, 1)
        row = QHBoxLayout()
        self.bt_refresh_btn = QPushButton("Refresh ports")
        self.bt_refresh_btn.clicked.connect(self.refresh_bt)
        self.bt_connect_btn = QPushButton("Connect")
        self.bt_connect_btn.clicked.connect(self.connect_bt)
        row.addWidget(self.bt_refresh_btn)
        row.addWidget(self.bt_connect_btn)
        v.addLayout(row)
        v.addWidget(QLabel("Pair the ESP32 (\"lifeos\") in Windows Bluetooth settings first."))
        return page

    def _build_wifi_page(self):
        page = QWidget()
        v = QVBoxLayout(page)
        v.addWidget(QLabel("Discovered lifeOs devices (mDNS):"))
        self.device_list = QListWidget()
        self.device_list.itemDoubleClicked.connect(lambda _: self.connect_selected())
        v.addWidget(self.device_list, 1)
        row = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh devices")
        self.refresh_btn.clicked.connect(self.refresh_wifi)
        self.connect_btn = QPushButton("Connect to selected")
        self.connect_btn.clicked.connect(self.connect_selected)
        row.addWidget(self.refresh_btn)
        row.addWidget(self.connect_btn)
        v.addLayout(row)
        manual = QHBoxLayout()
        manual.addWidget(QLabel("Or connect directly:"))
        self.manual_edit = QLineEdit()
        self.manual_edit.setPlaceholderText("172.16.72.32  or  lifeos.local")
        self.manual_edit.returnPressed.connect(self.connect_manual)
        self.manual_btn = QPushButton("Connect")
        self.manual_btn.clicked.connect(self.connect_manual)
        manual.addWidget(self.manual_edit, 1)
        manual.addWidget(self.manual_btn)
        v.addLayout(manual)
        return page

    # --- Wi-Fi ---
    def _ensure_wifi_link(self):
        if not isinstance(self.manager.active, WifiLink):
            link = WifiLink()
            link.start()
            self.manager.set_active(link)   # stops any previous (e.g. Bluetooth)
        return self.manager.active

    def _populate_wifi(self, devices):
        prev = None
        r = self.device_list.currentRow()
        if 0 <= r < len(self._wifi_rows):
            prev = self._wifi_rows[r][0]
        self.device_list.clear()
        self._wifi_rows = []
        for name, (ip, port) in sorted(devices.items()):
            self.device_list.addItem(f"{name.split('.')[0]}    ({ip})")
            self._wifi_rows.append((name, ip, port))
        if not self._wifi_rows:
            self.device_list.addItem("(no lifeOs devices found - power on the ESP32, then Refresh)")
        for i, (name, _ip, _p) in enumerate(self._wifi_rows):
            if name == prev:
                self.device_list.setCurrentRow(i)
                break

    def refresh_wifi(self):
        self._populate_wifi(self.discovery.devices())

    def connect_selected(self):
        r = self.device_list.currentRow()
        if not (0 <= r < len(self._wifi_rows)):
            self.status_label.setText("Select a device from the list first.")
            return
        name, ip, _p = self._wifi_rows[r]
        self._connect_wifi(name, ip)

    def connect_manual(self):
        text = self.manual_edit.text().strip()
        if not text:
            self.status_label.setText("Enter the ESP32's IP (from the serial monitor) or lifeos.local.")
            return
        ip = text
        if not looks_like_ip(text):
            try:
                ip = socket.gethostbyname(text)   # resolve lifeos.local via the OS
            except OSError:
                self.status_label.setText(
                    f"Could not resolve '{text}'. Type the IP shown on the serial monitor instead.")
                return
        self._connect_wifi(text, ip)

    def _connect_wifi(self, name, ip):
        link = self._ensure_wifi_link()
        ok = link.register_peer(ip)
        label = name.split('.')[0]
        self.status_label.setText(
            f"Sent hello to {label} ({ip}) - waiting for telemetry..." if ok
            else f"Could not reach {label} ({ip}).")
        self._save_default("wifi", name)

    # --- Bluetooth ---
    def _populate_bt(self):
        prev = None
        r = self.bt_list.currentRow()
        if 0 <= r < len(self._bt_rows):
            prev = self._bt_rows[r][0]
        self.bt_list.clear()
        self._bt_rows = []
        for dev, desc in list_serial_ports():
            self.bt_list.addItem(f"{dev}    {desc}")
            self._bt_rows.append((dev, desc))
        if not self._bt_rows:
            self.bt_list.addItem("(no COM ports - pair the ESP32 first, or install pyserial)")
        for i, (dev, _d) in enumerate(self._bt_rows):
            if dev == prev:
                self.bt_list.setCurrentRow(i)
                break

    def refresh_bt(self):
        self._populate_bt()

    def connect_bt(self):
        r = self.bt_list.currentRow()
        if not (0 <= r < len(self._bt_rows)):
            self.status_label.setText("Select a COM port first (or Refresh ports).")
            return
        dev, _desc = self._bt_rows[r]
        self._connect_bt(dev)

    def _connect_bt(self, com):
        try:
            link = BluetoothLink(com)
            link.start()                        # opens the serial port; may raise
        except Exception as e:
            self.status_label.setText(f"Could not open {com}: {e}")
            return
        self.manager.set_active(link)           # stops any previous (e.g. Wi-Fi)
        self.status_label.setText(f"Opened {com} - waiting for telemetry...")
        self._save_default("bluetooth", com)

    # --- shared ---
    def _save_default(self, transport, target):
        if self.default_chk.isChecked():
            self.config["default_transport"] = transport
            self.config["wifi_target" if transport == "wifi" else "bt_port"] = target
        elif self.config.get("default_transport") == transport:
            self.config.pop("default_transport", None)
        save_connect_config(self.config)

    def _tick(self):
        devices = self.discovery.devices()
        keys = tuple(sorted(devices))
        if keys != self._last_keys:
            self._last_keys = keys
            self._populate_wifi(devices)

        if self.manager.is_connected():
            self.status_label.setText(f"Connected - {self.manager.description()}")
            self.status_label.setStyleSheet("color: #2e7d32;")

        self._auto_connect(devices)

    def _auto_connect(self, devices):
        if self._auto_attempted:
            return
        transport = self.config.get("default_transport")
        if transport == "bluetooth":
            com = self.config.get("bt_port")
            if com and any(dev == com for dev, _ in list_serial_ports()):
                self._auto_attempted = True
                self.default_chk.setChecked(True)
                self._connect_bt(com)
        elif transport == "wifi":
            target = self.config.get("wifi_target")
            if not target:
                return
            if target in devices:                        # a discovered mDNS device
                self._auto_attempted = True
                self.default_chk.setChecked(True)
                self._connect_wifi(target, devices[target][0])
            elif not target.endswith(SERVICE_TYPE):      # a saved manual IP / hostname
                ip = target if looks_like_ip(target) else None
                if ip is None:
                    try:
                        ip = socket.gethostbyname(target)
                    except OSError:
                        ip = None
                if ip:
                    self._auto_attempted = True
                    self.default_chk.setChecked(True)
                    self._connect_wifi(target, ip)


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

        self.discovery = DeviceDiscovery()

        tabs = QTabWidget()
        tabs.setTabBar(StretchTabBar())        # tabs span the full window width
        self.connect_tab = ConnectTab(self.listener, self.discovery)
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
        try:
            self.discovery.close()   # stop the zeroconf browser threads cleanly
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
