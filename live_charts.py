from collections import deque

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QColor, QPolygonF
from PySide6.QtCore import QPointF

WINDOW_SAMPLES = 100
AXIS_COLORS = {"x": "#e53935", "y": "#43a047", "z": "#1e88e5"}
TEMP_COLOR = "#f9a825"      # yellow, matching the MPU status dot

# Smallest y-axis span the autoscaler will zoom to, so tiny motions are visible
# without the axis collapsing onto sensor noise. (g / deg/s / degC.)
ACCEL_MIN_SPAN = 0.2
GYRO_MIN_SPAN = 5.0
TEMP_MIN_SPAN = 1.0
Y_PADDING = 0.15  # fraction of span added above/below the data


class Sparkline(QWidget):
    """A tiny autoscaled line chart for one signal, sized to live inside a
    table cell (the Monitor table's History column)."""

    def __init__(self, color, min_span, window=WINDOW_SAMPLES):
        super().__init__()
        self.values = deque(maxlen=window)
        self.pen = QPen(QColor(color), 1.5)
        self.min_span = min_span
        self.setMinimumSize(60, 24)

    def add_value(self, value):
        self.values.append(float(value))
        self.update()

    def paintEvent(self, _event):
        if len(self.values) < 2:
            return
        lo, hi = min(self.values), max(self.values)
        mid = (lo + hi) / 2.0
        half = (max(hi - lo, self.min_span) / 2.0) * (1.0 + Y_PADDING)
        top, span = mid + half, 2.0 * half

        pad = 2.0
        w = self.width() - 2 * pad
        h = self.height() - 2 * pad
        step = w / (self.values.maxlen - 1)
        points = QPolygonF([QPointF(pad + i * step, pad + (top - v) / span * h)
                            for i, v in enumerate(self.values)])
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(self.pen)
        painter.drawPolyline(points)
        painter.end()


def sparkline_for(sensor):
    """A Sparkline colored by axis (x red, y green, z blue, temp yellow) and
    scaled for the sensor family: accel in g, gyro in deg/s, temp in degC."""
    if sensor == "tp":
        return Sparkline(TEMP_COLOR, TEMP_MIN_SPAN)
    return Sparkline(AXIS_COLORS[sensor[1]],
                     ACCEL_MIN_SPAN if sensor.startswith("a") else GYRO_MIN_SPAN)


def _smoke():
    import sys
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    spark = sparkline_for("ax")
    assert spark.min_span == ACCEL_MIN_SPAN
    assert sparkline_for("gz").min_span == GYRO_MIN_SPAN
    assert sparkline_for("tp").min_span == TEMP_MIN_SPAN
    for i in range(WINDOW_SAMPLES + 50):
        spark.add_value(i % 7)
    assert len(spark.values) == WINDOW_SAMPLES
    spark.resize(120, 30)
    spark.grab()          # renders off-screen, exercising paintEvent
    print("Sparkline smoke OK")


if __name__ == "__main__":
    _smoke()
