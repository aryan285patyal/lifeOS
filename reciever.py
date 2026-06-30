import socket
import msvcrt

PORT = 5005
CALIB_SAMPLES = 20  # samples averaged to build the resting baseline on reset
FIELDS = ("ax", "ay", "az", "gx", "gy", "gz")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', PORT))
s.settimeout(0.5)  # so we can poll the keyboard even when no packet arrives

offsets = {f: 0 for f in FIELDS}   # current zero point; subtracted from every reading
calib_accum = None                 # running sum while a recalibration is in progress
calib_left = 0                     # samples still needed to finish a recalibration


def parse(raw):
    """ 'q0:1.0,..,ax:1,ay:-2,...' -> dict (None if malformed).

    Quaternion fields (q0..q3) are floats; accel/gyro fields are ints. """
    values = {}
    for pair in raw.split(','):
        key, _, val = pair.partition(':')
        try:
            values[key] = float(val) if key.startswith('q') else int(val)
        except ValueError:
            return None
    if not all(f in values for f in FIELDS):
        return None
    return values


def start_calibration():
    global calib_accum, calib_left
    calib_accum = {f: 0 for f in FIELDS}
    calib_left = CALIB_SAMPLES
    print(f"\n[calibrating] hold the sensor still, averaging {CALIB_SAMPLES} samples...")


print(f"Listening on UDP {PORT}.  Press 'r' to recalibrate (zero) the sensor, Ctrl+C to quit.")

while True:
    # --- keyboard: 'r' triggers a recalibration ---
    if msvcrt.kbhit():
        key = msvcrt.getwch().lower()
        if key == 'r':
            start_calibration()

    # --- network: receive one packet ---
    try:
        data, addr = s.recvfrom(1024)
    except socket.timeout:
        continue

    values = parse(data.decode().strip())
    if values is None:
        continue

    # --- if mid-recalibration, accumulate and (when done) lock in the new offsets ---
    if calib_left > 0:
        for f in FIELDS:
            calib_accum[f] += values[f]
        calib_left -= 1
        if calib_left == 0:
            offsets = {f: round(calib_accum[f] / CALIB_SAMPLES) for f in FIELDS}
            print(f"[calibrated] new zero offsets: {offsets}\n")
        continue

    # --- normal operation: report calibrated (raw - offset) values ---
    cal = {f: values[f] - offsets[f] for f in FIELDS}
    print("ax:{ax} ay:{ay} az:{az}  gx:{gx} gy:{gy} gz:{gz}".format(**cal))
