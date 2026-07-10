# smoke_gui.py - headless check that the Monitor tab tables are wired correctly.
import math
import sys
from PySide6.QtWidgets import QApplication
import gui


class FakeListener:
    port = 5005

    def __init__(self):
        # last_seen stays 1.0 across calls, so the 2nd refresh is "assumed".
        self.values = {"ax": 0, "ay": 0, "az": 16384,
                       "gx": 131, "gy": 0, "gz": 0, "tp": 340}

    def snapshot(self):
        return (dict(self.values), 1.0)

    def is_connected(self, timeout=1.0):
        return True


app = QApplication(sys.argv)
# Ignore this machine's saved calibration: its value_flips would negate the
# roll/pitch rows the assertions below check.
gui.load_calibration = lambda: {}
win = gui.MonitorWindow(FakeListener())

# Rows are interleaved per axis: ax, gx, ay, gy, az, gz, temp.
assert gui.SENSORS == ["ax", "gx", "ay", "gy", "az", "gz", "tp"]
assert win.table.rowCount() == 7
assert win.table.item(6, 0).text() == "temp"

# First refresh: fresh packet -> real -> green.
win.refresh()
assert win.table.columnCount() == 4
assert win.table.horizontalHeaderItem(2).text() == "Converted"
assert win.table.horizontalHeaderItem(3).text() == "History"
assert win.table.item(4, 2).text().startswith("1.0000 g")   # az -> 1 g
assert win.table.item(1, 2).text().startswith("1.00 °/s")   # gx -> 1 deg/s
assert win.table.item(6, 2).text() == "37.53 °C"            # tp 340 -> +1 degC
assert win.table.item(4, 2).foreground().color().name() == gui.COLOR_REAL
assert len(win.sparks["az"].values) == 1                    # sparklines got fed
assert len(win.sparks["tp"].values) == 1
# no quaternion in the packet -> the angle rows show no data; the Delta Temp
# row baselines on the first reading, so it starts at zero
assert win.angle_table.rowCount() == 4
assert win.angle_table.item(0, 1).text() == "-"
assert win.angle_table.item(3, 0).text() == "Delta Temp"
assert win.angle_table.item(3, 1).text() == "+0.00 °C"

# Second refresh: same last_seen (no new packet) -> assumed -> red.
win.refresh()
assert win.table.item(4, 2).foreground().color().name() == gui.COLOR_ASSUMED
assert len(win.sparks["az"].values) == 2                    # held samples still chart

# With a quaternion (pure 30 deg yaw), the angle table shows angles from zero;
# a warmer die shows as a temperature delta against the session baseline.
half = math.radians(30) / 2
win.listener.values.update(
    {"q0": math.cos(half), "q1": 0.0, "q2": 0.0, "q3": math.sin(half),
     "tp": 340 + 510})                                      # +1.5 degC vs start
win.refresh()
assert win.angle_table.item(2, 1).text() == "+30.0 deg"     # yaw row
assert win.angle_table.item(0, 1).text() == "+0.0 deg"      # roll row
assert win.angle_table.item(3, 1).text() == "+1.50 °C"      # delta temp row
print("GUI smoke OK")
