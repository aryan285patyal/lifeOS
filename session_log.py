"""session_log.py -- one chronological log file per GUI run.

Every line the board sends (accepted telemetry, control replies, boot/driver
prints, and the lines `Link._ingest` *rejects*), every line the GUI sends back,
every button press, the visualizer's de-drift internals, the video stream's
health, and the console (stdout/stderr, tracebacks, Qt warnings) all land in

    log/<YYYY-MM-DD>/log-<YYYY-MM-DD>_<HH-MM-SS>.txt

The serial-monitor panel is a live view with a 2000-line ring buffer that hides
telemetry and stops draining while hidden, so it cannot answer "what was the
sensor doing 20 minutes ago?". This file can: it is written at the source, keeps
everything, and is the artifact to read when a value misbehaves.

Line format:  HH:MM:SS.mmm  TAG  VIA  payload   (the date is in the header)

The file is line-buffered, so a crash or a hard kill still leaves every line up
to that instant on disk. log() is a no-op until start() is called and never
raises - a logger must not be able to take the app down.
"""

import atexit
import collections
import datetime
import json
import os
import re
import subprocess
import sys
import threading
import time
import traceback

LOG_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")

# Tags in the order the footer summarises them.
TAGS = ("RX", "CTL", "RAW", "DROP", "TX", "PC", "STATE", "VID", "OUT", "ERR")
# Tags that count as "lines the board sent us" (for the drop-rate figure).
BOARD_TAGS = ("RX", "CTL", "RAW", "DROP")

_lock = threading.Lock()
_file = None
_path = None
_t0 = 0.0
_counts = collections.Counter()
_drop_reasons = collections.Counter()


def path():
    """Path of the log file for this run, or None if logging never started."""
    return _path


def redact(text):
    """Hide the password in a 'wifi:<ssid>|<password>|<ip>' provisioning line.
    Leaves every other line (including 'wifi:off') untouched."""
    if text.startswith("wifi:") and "|" in text:
        parts = text[5:].split("|")
        if len(parts) >= 2:
            parts[1] = "***"
            return "wifi:" + "|".join(parts)
    return text


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")     # color codes (Python colors tracebacks)


def _clean(s):
    """Control characters (a baud mismatch delivers binary) would corrupt the
    log's line structure, so map them to '?'; ANSI color codes just go."""
    return "".join(c if c >= " " else "?"
                   for c in _ANSI.sub("", s).replace("\t", "    "))


def log(tag, text, via=""):
    """Append one entry. Thread-safe; a no-op before start(); never raises."""
    if _file is None:
        return
    try:
        stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        lines = str(text).split("\n")
        with _lock:
            if _file is None:            # closed between the check and the lock
                return
            _counts[tag] += 1
            if tag == "DROP":
                _drop_reasons[lines[0].split(" ", 1)[0]] += 1
            for line in lines:           # a traceback stays one entry per line
                _file.write(f"{stamp}  {tag:<5}{via:<7}{_clean(line)}\n")
    except Exception:
        pass                             # logging must never break the caller


def start(meta=None):
    """Create today's folder, open this run's file, write the header. Returns
    the path (or None if the file could not be created)."""
    global _file, _path, _t0
    if _file is not None:
        return _path
    now = datetime.datetime.now()
    day = now.strftime("%Y-%m-%d")
    folder = os.path.join(LOG_ROOT, day)
    try:
        os.makedirs(folder, exist_ok=True)
    except OSError:
        return None
    base = f"log-{day}_{now.strftime('%H-%M-%S')}"   # ':' is illegal on Windows
    candidate = os.path.join(folder, base + ".txt")
    n = 1
    while os.path.exists(candidate):                 # two runs in the same second
        candidate = os.path.join(folder, f"{base}-{n}.txt")
        n += 1
    try:
        f = open(candidate, "a", buffering=1, encoding="utf-8", errors="replace")
    except OSError:
        return None
    with _lock:
        _file, _path, _t0 = f, candidate, time.monotonic()
        _counts.clear()
        _drop_reasons.clear()
    _write_header(now, meta or {})
    atexit.register(close)
    return _path


def close():
    """Write the summary footer and close the file. Safe to call twice."""
    global _file, _path
    with _lock:
        f, _file = _file, None
        counts, reasons, t0 = _counts.copy(), _drop_reasons.copy(), _t0
    if f is None:
        return
    try:
        elapsed = time.monotonic() - t0
        board = sum(counts[t] for t in BOARD_TAGS)
        dropped = counts["DROP"]
        why = ", ".join(f"{r} {n}" for r, n in reasons.most_common())
        f.write("#\n")
        f.write(f"# --- session ended {datetime.datetime.now():%H:%M:%S} after "
                f"{int(elapsed // 3600):d}h {int(elapsed // 60) % 60:02d}m "
                f"{int(elapsed) % 60:02d}s ---\n")
        f.write("# " + "  ".join(f"{t} {counts[t]}" for t in TAGS)
                + (f"   (drops: {why})" if why else "") + "\n")
        rate = (100.0 * dropped / board) if board else 0.0
        f.write(f"# board lines {board}, rejected {dropped} ({rate:.2f}%)\n")
        f.close()
    except Exception:
        pass


# --------------------------------------------------------------------------
# header
# --------------------------------------------------------------------------

def _git_version():
    """Short SHA of the checkout (+dirty), so a log can be tied to the code
    that produced it. Quiet-fails when git isn't on PATH."""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        run = lambda *a: subprocess.run(("git", "-C", here) + a, timeout=5,
                                        capture_output=True, text=True).stdout.strip()
        sha = run("rev-parse", "--short", "HEAD")
        return (sha or "unknown") + (" +dirty" if run("status", "--porcelain") else "")
    except Exception:
        return "unknown"


def _versions():
    py = "%d.%d.%d" % sys.version_info[:3]
    try:
        import PySide6
        return f"python {py}  |  PySide6 {PySide6.__version__}"
    except Exception:
        return f"python {py}"


def _write_header(now, meta):
    # any key that smells like a secret ('video_pass', 'password', ...)
    cfg = {k: ("***" if v and "pass" in k.lower() else v)
           for k, v in (meta.get("connect_config") or {}).items()}
    cal = meta.get("calibration") or {}
    rows = [
        ("started", f"{now:%Y-%m-%d %H:%M:%S}  (per-line stamps are "
                    f"HH:MM:SS.mmm, local time)"),
        ("file", _path),
        ("code", f"git {_git_version()}"),
        ("versions", _versions()),
        ("board", f"{meta.get('board', '?')}   serial {meta.get('serial_baud', '?')} baud"),
        ("laptop ip", str(meta.get("laptop_ip", "?"))),
        ("connect cfg", json.dumps(cfg, sort_keys=True)),
        ("calibration", f"accel_scale={json.dumps(cal.get('accel_scale', {}), sort_keys=True)}"
                        f"  value_flips={json.dumps(cal.get('value_flips', {}), sort_keys=True)}"),
    ]
    try:
        _file.write("# lifeOs session log\n")
        for key, value in rows:
            _file.write(f"# {key:<12}: {value}\n")
        _file.write("#\n")
        _file.write("# columns : HH:MM:SS.mmm  TAG  VIA  payload\n")
        _file.write("# RX    telemetry accepted    CTL  control reply       "
                    "RAW  other board output\n")
        _file.write("# DROP  line rejected (+why)  TX   sent to the board    "
                    "PC   gui event\n")
        _file.write("# STATE visualizer/de-drift   VID  video stream stats   "
                    "OUT/ERR console + tracebacks\n")
        _file.write("#\n")
    except Exception:
        pass


# --------------------------------------------------------------------------
# console capture
# --------------------------------------------------------------------------

class _Tee:
    """Mirrors a text stream into the log, one entry per complete line (print()
    writes the text and the '\\n' separately, so buffer until the newline)."""

    def __init__(self, stream, tag):
        self._stream = stream
        self._tag = tag
        self._buf = ""
        self._buf_lock = threading.Lock()   # not _lock: log() takes that one

    def write(self, s):
        if self._stream is not None:
            try:
                self._stream.write(s)
            except Exception:
                pass
        try:
            with self._buf_lock:
                self._buf += s
                lines = self._buf.split("\n")
                self._buf = lines.pop()
            for line in lines:
                if line.strip():
                    log(self._tag, line)
        except Exception:
            self._buf = ""
        return len(s)

    def flush(self):
        if self._stream is not None:
            try:
                self._stream.flush()
            except Exception:
                pass

    def isatty(self):
        return self._stream is not None and self._stream.isatty()

    def __getattr__(self, name):         # fileno(), encoding, ... from the original
        return getattr(self._stream, name)


_console_err = None                      # unwrapped stderr (writing to the tee'd
                                         # one would log the same text twice)
_qt_handler = None                       # Qt segfaults if the handler is GC'd


def _echo(text):
    """Put text on the real console, bypassing the tee."""
    if _console_err is None:
        return
    try:
        _console_err.write(text + "\n")
    except Exception:
        pass


def _hook_exception(exc_type, exc, tb):
    # Deliberately not chaining to the default hook: it prints to sys.stderr,
    # which is tee'd, so the traceback would be logged a second time - in the
    # interpreter's ANSI-colored form. Echo it ourselves instead.
    text = "".join(traceback.format_exception(exc_type, exc, tb)).rstrip()
    log("ERR", text)
    _echo(text)


def _hook_thread_exception(args):
    if args.exc_type is SystemExit:
        return
    text = (f"exception in thread {args.thread and args.thread.name}:\n"
            + "".join(traceback.format_exception(
                args.exc_type, args.exc_value, args.exc_traceback)).rstrip())
    log("ERR", text)
    _echo(text)


def install_console_capture():
    """Tee stdout/stderr into the log, and capture uncaught tracebacks (from
    any thread) plus Qt's own messages.

    Qt writes qWarning/qCritical to the C++ stderr, which replacing sys.stderr
    never sees - hence the message handler. QtWebEngine's Chromium logs go
    straight to the OS stderr file descriptor and are still not captured;
    catching those would need fd-level redirection."""
    global _console_err, _qt_handler

    _console_err = sys.stderr              # before wrapping
    sys.stdout = _Tee(sys.stdout, "OUT")   # may be None under pythonw.exe
    sys.stderr = _Tee(sys.stderr, "ERR")

    sys.excepthook = _hook_exception
    threading.excepthook = _hook_thread_exception

    try:
        from PySide6.QtCore import QtMsgType, qInstallMessageHandler
    except Exception:
        return
    loud = (QtMsgType.QtWarningMsg, QtMsgType.QtCriticalMsg, QtMsgType.QtFatalMsg)

    def handler(mode, _context, message):
        log("ERR" if mode in loud else "OUT", f"Qt: {message}")
        _echo(f"Qt: {message}")            # installing a handler silences the
                                           # default one, so keep printing

    _qt_handler = handler
    qInstallMessageHandler(_qt_handler)
