"""bt_receiver.py -- prove the Bluetooth SPP link before wiring it into the GUI.

Flow to test:
  1. Flash lifeOs.ino with USE_BLUETOOTH 1.
  2. On Windows: Settings > Bluetooth > Add device, pair with "lifeos".
     Pairing creates one or more "Standard Serial over Bluetooth link (COMx)"
     ports. The OUTGOING one is what we open here.
  3. Run:  python bt_receiver.py            (auto-picks a Bluetooth COM port)
     or:    python bt_receiver.py COM7      (force a specific port)

Once connected it prints each telemetry line. Type two angles (e.g. "120 45")
and Enter to drive the servos -- that exercises the reverse direction. Ctrl+C
or "q" to quit.

Needs pyserial:  python -m pip install pyserial
"""

import sys
import threading

import serial
import serial.tools.list_ports as list_ports


def find_bt_port():
    """Return the first COM port that looks like a Bluetooth serial link."""
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if "bluetooth" in text:
            return p.device
    return None


def list_all_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("  (no COM ports found)")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")


def reader(ser):
    """Print telemetry lines as they arrive."""
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("\n[serial closed]")
            return
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line:
            print("RX:", line)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_bt_port()
    if not port:
        print("Could not auto-detect a Bluetooth COM port. Available ports:")
        list_all_ports()
        print("\nPass one explicitly:  python bt_receiver.py COM7")
        return

    print(f"Opening {port} ...")
    try:
        # Baud is irrelevant over Bluetooth SPP, but pyserial requires a value.
        ser = serial.Serial(port, 115200, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        print("Available ports:")
        list_all_ports()
        return

    print("Connected. Type 'S0 S1' angles (e.g. '120 45') to move servos, 'q' to quit.\n")
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() in ("q", "quit", "exit"):
                break
            parts = cmd.split()
            if len(parts) == 2 and all(p.lstrip("-").isdigit() for p in parts):
                a0 = max(0, min(180, int(parts[0])))
                a1 = max(0, min(180, int(parts[1])))
                msg = f"s0:{a0},s1:{a1}\n"
                ser.write(msg.encode())
                print("TX:", msg.strip())
            elif cmd.strip():
                print("Enter two numbers 0-180, e.g. '90 90'  (or 'q' to quit)")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\nClosed.")


if __name__ == "__main__":
    main()
