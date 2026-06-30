import sys
import os
import socket
import subprocess
import threading
import time

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QLineEdit, QPushButton, QTabWidget,
)
from PySide6.QtCore import QTimer, Qt, QUrl, QObject, Signal
from PySide6.QtGui import QColor
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel

from CleanInput import CleanInput, format_converted
from live_charts import ChartPanel

QUAT = ["q0", "q1", "q2", "q3"]

SENSORS = ["ax", "ay", "az", "gx", "gy", "gz"]

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


class Listener:
    def __init__(self, port=5005):
        self.lock = threading.Lock()
        self.latest = {}
        self.last_seen = 0.0
        self.port = port

    def start(self):
        thread = threading.Thread(target=self._run)
        thread.daemon = True
        thread.start()

    def _run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('', self.port))
            while True:
                data, addr = sock.recvfrom(1024)
                try:
                    parsed_data = {}
                    for pair in data.decode().split(','):
                        key, value = pair.split(':')
                        # quaternion fields (q0..q3) are floats; raw counts are ints
                        parsed_data[key] = float(value) if key.startswith('q') else int(value)
                    with self.lock:
                        self.latest = parsed_data
                        self.last_seen = time.monotonic()
                except Exception:
                    continue

    def snapshot(self):
        with self.lock:
            return (self.latest.copy(), self.last_seen)

    def is_connected(self, timeout=1.0):
        with self.lock:
            return (time.monotonic() - self.last_seen) <= timeout


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

        # --- Port diagnostics ---
        self.check_btn = QPushButton(f"Refresh / Check Port {listener.port}")
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
        tabs.addTab(central_widget, "Monitor")
        self.visualizer = VisualizerTab(self.listener)
        tabs.addTab(self.visualizer, "Visualizer")
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
        port = self.listener.port
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


def main():
    # QtWebEngine wants shared GL contexts set before the QApplication exists.
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)
    listener = Listener()
    listener.start()
    win = MonitorWindow(listener)
    win.resize(720, 860)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
