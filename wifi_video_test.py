"""wifi_video_test.py -- verify the Wi-Fi video path end to end.

The ESP32 streams synthetic "video" packets (1 KB each, sequence-numbered) over
UDP once it has been provisioned with your Wi-Fi SSID/password + this laptop's
IP. This script can do the provisioning over Bluetooth for you, then binds the
video port and measures throughput / packet loss.

Two ways to run:

  # 1) Provision over Bluetooth AND measure (fully standalone):
  python wifi_video_test.py --com COM7 --ssid MyWifi --password secret

  # 2) Just measure (you already provisioned via the GUI's Wi-Fi section):
  python wifi_video_test.py

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
    with serial.Serial(com, 115200, timeout=1) as s:
        s.write(f"wifi:{ssid}|{password}|{ip}\n".encode())
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            line = s.readline().decode(errors="ignore").strip()
            if line.startswith("wifi:"):
                print("  ESP32:", line)
                if line.startswith("wifi:connected") or line.startswith("wifi:error"):
                    return
    print("  (no 'wifi:connected' seen within 12s - continuing to listen anyway)")


def measure(seconds):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", VIDEO_PORT))
    sock.settimeout(1.0)
    print(f"\nListening for video on UDP {VIDEO_PORT} for {seconds}s ...")

    count = 0
    total_bytes = 0
    min_seq = None
    max_seq = None
    start = time.monotonic()
    end = start + seconds

    while time.monotonic() < end:
        try:
            data, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        count += 1
        total_bytes += len(data)
        head = data[:24].decode(errors="ignore")
        if head.startswith("vid:"):
            try:
                seq = int(head[4:head.index(":", 4)])
                min_seq = seq if min_seq is None else min(min_seq, seq)
                max_seq = seq if max_seq is None else max(max_seq, seq)
            except ValueError:
                pass
    dur = time.monotonic() - start
    sock.close()

    print(f"\n--- results over {dur:.1f}s ---")
    print(f"packets received : {count}")
    if dur > 0:
        print(f"throughput       : {total_bytes / dur / 1024:.1f} KB/s "
              f"({count / dur:.0f} pkt/s, {total_bytes / dur * 8 / 1e6:.2f} Mbit/s)")
    if min_seq is not None:
        expected = max_seq - min_seq + 1
        loss = 100 * (1 - count / expected) if expected > 0 else 0.0
        print(f"sequence range   : {min_seq}..{max_seq} (expected {expected} packets)")
        print(f"packet loss      : {loss:.1f}%")
    if count == 0:
        print("No packets. Check: ESP32 provisioned and Wi-Fi connected (serial monitor)? "
              f"Firewall allowing inbound UDP {VIDEO_PORT}? Laptop IP correct?")


def main():
    ap = argparse.ArgumentParser(description="Test the lifeOs Wi-Fi video path.")
    ap.add_argument("--com", help="Bluetooth COM port to provision over (optional if already provisioned via the GUI)")
    ap.add_argument("--ssid", help="Wi-Fi SSID (required with --com)")
    ap.add_argument("--password", default="", help="Wi-Fi password")
    ap.add_argument("--ip", default=None, help="laptop IP to receive on (default: auto-detect)")
    ap.add_argument("--seconds", type=int, default=10, help="how long to measure")
    args = ap.parse_args()

    ip = args.ip or get_local_ip()
    print("laptop IP:", ip)
    if args.com:
        if not args.ssid:
            print("--ssid is required with --com")
            return
        provision(args.com, args.ssid, args.password, ip)
    measure(args.seconds)


if __name__ == "__main__":
    main()
