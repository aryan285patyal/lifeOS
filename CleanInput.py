"""
This module processes raw UDP sensor streams to produce a consistent, gap-free stream.
It handles snapshots of (values, last_seen), where values is a dictionary of raw sensor counts,
and last_seen is a monotonic timestamp that advances only when a fresh valid packet arrives.
Missing or garbled packets are handled by holding the last confirmed reading and flagging it as assumed.
"""

import time
from dataclasses import dataclass

SENSORS = ["ax", "ay", "az", "gx", "gy", "gz"]
ACCEL_LSB_PER_G = 16384.0  # MPU6050 default full-scale +/-2 g
GYRO_LSB_PER_DPS = 131.0   # MPU6050 default full-scale +/-250 deg/s


def convert(name, raw):
    """Converts raw sensor values to physical units."""
    if name in ("ax", "ay", "az"):
        return (raw / ACCEL_LSB_PER_G, "g")
    else:
        return (raw / GYRO_LSB_PER_DPS, "°/s")


def format_converted(name, raw):
    """Formats the converted sensor value with appropriate precision."""
    value, unit = convert(name, raw)
    decimals = 4 if unit == "g" else 2
    return f"{value:.{decimals}f} {unit}"


@dataclass
class CleanSample:
    """Data class representing a cleaned sensor sample."""
    t: float          # Timestamp
    has_data: bool    # Indicates if the sample contains real data
    assumed: bool     # Indicates if the sample is assumed (not fresh)
    raw: dict         # Raw sensor values
    converted: dict   # Converted sensor values


class CleanInput:
    """Class to process and clean UDP sensor streams."""

    def __init__(self):
        self._last_seen = None  # Last seen timestamp of a valid packet
        self._confirmed = None  # Last confirmed sensor readings

    def update(self, values, last_seen, now=None):
        """
        Updates the internal state with new sensor data and returns a CleanSample.

        :param values: Dictionary of raw sensor counts
        :param last_seen: Monotonic timestamp when the packet was received
        :param now: Current time (optional)
        :return: CleanSample instance
        """
        now = time.monotonic() if now is None else now
        is_real = bool(values) and all(k in values for k in SENSORS) and last_seen != self._last_seen
        if is_real:
            self._confirmed = {k: values[k] for k in SENSORS}
            self._last_seen = last_seen
        if self._confirmed is None:
            return CleanSample(now, False, True, None, None)
        converted = {k: convert(k, self._confirmed[k]) for k in SENSORS}
        return CleanSample(now, True, not is_real, dict(self._confirmed), converted)


def _smoke():
    """Runs a smoke test to verify the functionality of CleanInput."""
    ci = CleanInput()
    s = ci.update({}, 0.0)
    assert not s.has_data and s.assumed
    pkt = {"ax": 0, "ay": 0, "az": 16384, "gx": 131, "gy": 0, "gz": -262}
    s = ci.update(pkt, 1.2)
    assert s.has_data and not s.assumed
    assert abs(s.converted["az"][0] - 1.0) < 1e-9 and s.converted["az"][1] == "g"
    assert abs(s.converted["gx"][0] - 1.0) < 1e-9
    assert abs(s.converted["gz"][0] + 2.0) < 1e-9
    s = ci.update(pkt, 1.2)
    assert s.assumed and s.raw["az"] == 16384
    s = ci.update({}, 1.2)
    assert s.assumed and s.raw["az"] == 16384
    pkt2 = {"ax": 8192, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
    s = ci.update(pkt2, 1.3)
    assert not s.assumed and abs(s.converted["ax"][0] - 0.5) < 1e-9
    print("CleanInput smoke OK")


if __name__ == "__main__":
    _smoke()
