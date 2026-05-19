"""
=============================================================
  Arduino Serial Monitor Reader
  -- Reads everything from the serial port and saves to file
  -- Works exactly like the Arduino Serial Monitor, but saves
     all data automatically to a .txt and .csv file.
=============================================================

REQUIREMENTS:
    pip install pyserial

HOW TO USE:
    1. Close the Arduino IDE Serial Monitor (only one program
       can use the serial port at a time!)
    2. Find your port:
         Windows  →  COM3, COM4, COM5 ...  (check Device Manager)
         Linux    →  /dev/ttyUSB0  or  /dev/ttyACM0
         Mac      →  /dev/cu.usbmodem...
    3. Set PORT and BAUD_RATE below to match your sketch
    4. Run:  python serial_reader.py
    5. Press Ctrl+C to stop — files are saved automatically
=============================================================
"""

import serial
import serial.tools.list_ports
import datetime
import sys
import os

# ============================================================
#  SETTINGS — change these to match your setup
# ============================================================
PORT      = "COM8"     # Set to e.g. "COM3" or "/dev/ttyUSB0"
                        # If None, the script will auto-detect
BAUD_RATE = 115200        # Must match Serial.begin() in your .ino
TIMEOUT   = 2           # seconds

OUTPUT_DIR = "."        # Where to save the output files
# ============================================================


def list_ports():
    """Print all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found.")
    for p in ports:
        print(f"  {p.device:20s}  {p.description}")
    return ports


def auto_detect_port():
    """Try to find an Arduino automatically."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = p.description.lower()
        if "arduino" in desc or "usb serial" in desc or "ch340" in desc or "cp210" in desc:
            return p.device
    # If only one port exists, use it
    if len(ports) == 1:
        return ports[0].device
    return None


def main():
    global PORT

    print("=" * 60)
    print("  Arduino Serial Monitor Reader")
    print("=" * 60)

    # ── Port detection ──────────────────────────────────────
    if PORT is None:
        print("\nAvailable serial ports:")
        ports = list_ports()
        PORT = auto_detect_port()
        if PORT is None:
            if not ports:
                print("\n[ERROR] No ports found. Is the Arduino plugged in?")
                sys.exit(1)
            # Ask user to pick
            print("\nCould not auto-detect Arduino.")
            PORT = input("Enter port manually (e.g. COM3 or /dev/ttyUSB0): ").strip()
        else:
            print(f"\nAuto-detected port: {PORT}")

    # ── Output filenames ────────────────────────────────────
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    name_choice = input("\nChoose filename format:\n"
                         "  [1]--> Omega_RMin\n"
                         "  [2]--> Omega_RNom\n"
                         "  [3]--> Omega_RMax\n"
                         "Enter choice (1 or 2): ").strip()

    switch = {
        "1": f"Omega_RMin",
        "2": f"Omega_RNom",
        "3": f"Omega_RMax"
    }

    filename = switch.get(name_choice)
    if not filename:
        print("\n[ERROR] Invalid choice.")
        sys.exit(1)

    txt_file = os.path.join(OUTPUT_DIR, f"{filename}.txt")
    csv_file = os.path.join(OUTPUT_DIR, f"{filename}.csv")

    print(f"\nPort:      {PORT}")
    print(f"Baud rate: {BAUD_RATE}")
    print(f"Log file:  {txt_file}")
    print(f"CSV file:  {csv_file}  (only lines with commas)")
    print("\nPress Ctrl+C to stop.\n")
    print("-" * 60)

    # ── Open serial port ────────────────────────────────────
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"\n[ERROR] Could not open port {PORT}:\n  {e}")
        print("\nMake sure:")
        print("  1. The Arduino IDE Serial Monitor is CLOSED")
        print("  2. The port name is correct")
        sys.exit(1)

    # ── Read loop ───────────────────────────────────────────
    line_count  = 0
    csv_count   = 0

    try:
        with open(txt_file, "w", encoding="utf-8") as ftxt, \
             open(csv_file, "w", encoding="utf-8") as fcsv:

            # Write CSV header hint
            fcsv.write("# CSV lines extracted from serial output\n")
            fcsv.write("# Format from Arduino: Time_s,u_t,RPM\n")

            while True:
                raw = ser.readline()

                if not raw:
                    continue  # timeout, keep waiting

                try:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    line = repr(raw)

                # Print to terminal (like Serial Monitor)
                print(line)

                # Save every line to .txt
                ftxt.write(line + "\n")
                ftxt.flush()
                line_count += 1

                # Save CSV lines (contain commas and start with a digit)
                stripped = line.strip()
                if "," in stripped and stripped and stripped[0].isdigit():
                    fcsv.write(line + "\n")
                    fcsv.flush()
                    csv_count += 1

    except KeyboardInterrupt:
        print("\n" + "-" * 60)
        print("Stopped by user.")

    except serial.SerialException as e:
        print(f"\n[ERROR] Serial connection lost:\n  {e}")

    finally:
        ser.close()
        print(f"\nSaved {line_count} lines  →  {txt_file}")
        print(f"Saved {csv_count} CSV rows →  {csv_file}")
        print("Done.")


if __name__ == "__main__":
    main()