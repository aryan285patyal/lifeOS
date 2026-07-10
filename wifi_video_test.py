"""wifi_video_test.py -- verify the Wi-Fi video path end to end.

Once provisioned with your Wi-Fi SSID/password + this laptop's IP, the ESP32
streams video over UDP: real OV3660 JPEG frames as chunked "vf:" packets on
the S3-CAM, or synthetic sequence-numbered 1 KB "vid:" packets (WROOM-32, or
S3 camera-init failure). This script can do the provisioning over serial for
you, then binds the video port and measures frames/throughput/loss for
whichever stream arrives. --save dumps the last complete JPEG to a file.

Two ways to run:

  # 1) Provision over serial AND measure (fully standalone):
  python wifi_video_test.py --com COM7 --ssid MyWifi --password secret

  # 2) Just measure (you already provisioned via the GUI's Wi-Fi section):
  python wifi_video_test.py [--save frame.jpg]

Needs pyserial only for the --com provisioning path.
"""

import argparse
import socket
import time

VIDEO_PORT = 5010          # must match VIDEO_PORT in lifeOs.ino / gui.py


def get_local_ip():
    """This laptop's IP on the route to the internet (the address to receive on)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def provision(com, ssid, password, ip):
    """Send Wi-Fi creds to the ESP32 over Bluetooth and print its replies."""
    import serial
    print(f"Provisioning over {com}: ssid='{ssid}', video -> {ip}:{VIDEO_PORT}")
    with serial.Serial(com, 921600, timeout=1) as s:   # matches SERIAL_BAUD in gui.py
        s.write(f"wifi:{ssid}|{password}|{ip}\n".encode())
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            line = s.readline().decode(errors="ignore").strip()
            if line.startswith("wifi:"):
                print("  ESP32:", line)
                if line.startswith("wifi:connected") or line.startswith("wifi:error"):
                    return
    print("  (no 'wifi:connected' seen within 12s - continuing to listen anyway)")


def measure(seconds, save=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", VIDEO_PORT))
    sock.settimeout(1.0)
    print(f"\nListening for video on UDP {VIDEO_PORT} for {seconds}s ...")

    count = 0
    total_bytes = 0
    min_seq = None
    max_seq = None
    # vf: JPEG frame reassembly (same latest-wins protocol as gui.VideoReceiver)
    frames = 0
    frame_ids = set()
    last_jpeg = None
    cur_id, chunks, total = None, {}, 0
    start = time.monotonic()
    end = start + seconds

    while time.monotonic() < end:
        try:
            data, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        count += 1
        total_bytes += len(data)
        if data.startswith(b"vid:"):
            head = data[:24].decode(errors="ignore")
            try:
                seq = int(head[4:head.index(":", 4)])
                min_seq = seq if min_seq is None else min(min_seq, seq)
                max_seq = seq if max_seq is None else max(max_seq, seq)
            except ValueError:
                pass
        elif data.startswith(b"vf:"):
            try:  # vf:<frame>:<idx>/<count>:<payload>
                p1 = data.index(b":", 3)
                p2 = data.index(b":", p1 + 1)
                fid = int(data[3:p1])
                idx_s, cnt_s = data[p1 + 1:p2].split(b"/")
                idx, cnt = int(idx_s), int(cnt_s)
            except (ValueError, IndexError):
                continue
            frame_ids.add(fid)
            if cur_id is None or fid > cur_id:
                cur_id, chunks, total = fid, {}, cnt
            elif fid < cur_id:
                continue
            chunks[idx] = data[p2 + 1:]
            if total and len(chunks) == total:
                last_jpeg = b"".join(chunks[i] for i in range(total))
                frames += 1
                cur_id, chunks, total = None, {}, 0
    dur = time.monotonic() - start
    sock.close()

    print(f"\n--- results over {dur:.1f}s ---")
    print(f"packets received : {count}")
    if dur > 0:
        print(f"throughput       : {total_bytes / dur / 1024:.1f} KB/s "
              f"({count / dur:.0f} pkt/s, {total_bytes / dur * 8 / 1e6:.2f} Mbit/s)")
    if frame_ids:
        loss = 100 * (1 - frames / len(frame_ids)) if frame_ids else 0.0
        print(f"camera stream    : {frames} complete JPEG frames "
              f"({frames / dur:.1f} fps), {len(frame_ids)} frames seen "
              f"({loss:.1f}% incomplete)")
        if last_jpeg:
            print(f"last frame       : {len(last_jpeg)} bytes"
                  + (" (JPEG magic OK)" if last_jpeg[:2] == b"\xff\xd8" else
                     " (WARNING: not a JPEG?)"))
        if save and last_jpeg:
            with open(save, "wb") as f:
                f.write(last_jpeg)
            print(f"saved            : {save}")
    if min_seq is not None:
        expected = max_seq - min_seq + 1
        loss = 100 * (1 - count / expected) if expected > 0 else 0.0
        print(f"synthetic stream : seq {min_seq}..{max_seq} "
              f"(expected {expected} packets, loss {loss:.1f}%)")
    if count == 0:
        print("No packets. Check: ESP32 provisioned and Wi-Fi connected (serial monitor)? "
              f"Firewall allowing inbound UDP {VIDEO_PORT}? Laptop IP correct? "
              f"Laptop and ESP32 on the same network (see fixes.md)?")


def main():
    ap = argparse.ArgumentParser(description="Test the lifeOs Wi-Fi video path.")
    ap.add_argument("--com", help="Bluetooth COM port to provision over (optional if already provisioned via the GUI)")
    ap.add_argument("--ssid", help="Wi-Fi SSID (required with --com)")
    ap.add_argument("--password", default="", help="Wi-Fi password")
    ap.add_argument("--ip", default=None, help="laptop IP to receive on (default: auto-detect)")
    ap.add_argument("--seconds", type=int, default=10, help="how long to measure")
    ap.add_argument("--save", default=None,
                    help="write the last complete JPEG frame to this file")
    args = ap.parse_args()

    ip = args.ip or get_local_ip()
    print("laptop IP:", ip)
    if args.com:
        if not args.ssid:
            print("--ssid is required with --com")
            return
        provision(args.com, args.ssid, args.password, ip)
    measure(args.seconds, save=args.save)


if __name__ == "__main__":
    main()
