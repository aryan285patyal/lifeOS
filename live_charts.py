from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtCore import Qt

WINDOW_SAMPLES = 100
ACCEL_AXES = ["ax", "ay", "az"]
GYRO_AXES = ["gx", "gy", "gz"]
AXIS_COLORS = {"x": "#e53935", "y": "#43a047", "z": "#1e88e5"}

class ChartPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.sample_index = 0
        self.series = {}     # name -> QLineSeries
        self._x_axes = []    # list of QValueAxis to scroll
        layout = QVBoxLayout()
        accel_view, accel_x = self._build_chart("Accelerometer (g)", ACCEL_AXES, (-2.1, 2.1))
        gyro_view, gyro_x = self._build_chart("Gyroscope (deg/s)", GYRO_AXES, (-260.0, 260.0))
        self._x_axes = [accel_x, gyro_x]
        layout.addWidget(accel_view)
        layout.addWidget(gyro_view)
        self.setLayout(layout)

    def _build_chart(self, title, names, y_range):
        chart = QChart()
        chart.setTitle(title)
        chart.legend().setVisible(True)
        chart.legend().setAlignment(Qt.AlignBottom)

        x_axis = QValueAxis()
        x_axis.setRange(0, WINDOW_SAMPLES)
        x_axis.setLabelFormat("%d")
        x_axis.setTitleText("samples")

        y_axis = QValueAxis()
        y_axis.setRange(*y_range)

        chart.addAxis(x_axis, Qt.AlignBottom)
        chart.addAxis(y_axis, Qt.AlignLeft)

        for name in names:
            series = QLineSeries()
            series.setName(name)
            pen = QPen(QColor(AXIS_COLORS[name[1]]), 2)
            series.setPen(pen)
            chart.addSeries(series)
            series.attachAxis(x_axis)
            series.attachAxis(y_axis)
            self.series[name] = series

        view = QChartView(chart)
        view.setRenderHint(QPainter.Antialiasing)
        view.setMinimumHeight(200)

        return (view, x_axis)

    def add_sample(self, converted):
        self.sample_index += 1
        x = self.sample_index
        for name, series in self.series.items():
            series.append(x, converted[name][0])
            if series.count() > WINDOW_SAMPLES:
                series.removePoints(0, series.count() - WINDOW_SAMPLES)
        left = max(0, x - WINDOW_SAMPLES)
        for ax in self._x_axes:
            ax.setRange(left, max(WINDOW_SAMPLES, x))

def _smoke():
    import sys
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    panel = ChartPanel()
    assert set(panel.series.keys()) == set(ACCEL_AXES + GYRO_AXES)
    conv = {n: (1.0, "u") for n in ACCEL_AXES + GYRO_AXES}
    for _ in range(WINDOW_SAMPLES + 50):
        panel.add_sample(conv)
    for n, s in panel.series.items():
        assert s.count() == WINDOW_SAMPLES, (n, s.count())
    print("ChartPanel smoke OK")

if __name__ == "__main__":
    _smoke()
