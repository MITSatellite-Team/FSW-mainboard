"""
Pull all files from the board's SD card (/sd) to a local directory.

Requires: pip install mpremote

Usage:
    python scripts/pull_sd.py                        # saves to ./sd_dump/
    python scripts/pull_sd.py -o /path/to/output     # saves to custom path
    python scripts/pull_sd.py -p /dev/tty.usbmodem1  # specify serial port

The board must be running (SD card mounted) and not in USB drive mode.
If the board is showing as a USB drive, eject it first so the serial port
becomes available.
"""

import argparse
import os
import subprocess
import sys


def find_board_port():
    """Auto-detect the board's serial port."""
    import serial.tools.list_ports

    candidates = []
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        if any(k in desc for k in ("circuitpython", "argus", "rp2040", "pico", "cdc")):
            candidates.append(port.device)
    if not candidates:
        # Fallback: grab any USB serial device
        for port in serial.tools.list_ports.comports():
            if "usb" in (port.hwid or "").lower():
                candidates.append(port.device)
    return candidates[0] if candidates else None


def list_sd_files(port):
    """Return a list of all file paths under /sd on the board."""
    script = (
        "import os\n"
        "def walk(path):\n"
        "    for name in os.listdir(path):\n"
        "        full = path + '/' + name\n"
        "        try:\n"
        "            os.listdir(full)\n"
        "            walk(full)\n"
        "        except OSError:\n"
        "            print(full)\n"
        "walk('/sd')\n"
    )
    result = subprocess.run(
        ["mpremote", "connect", port, "exec", script],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"Error listing SD files:\n{result.stderr}", file=sys.stderr)
        sys.exit(1)

    files = [line.strip() for line in result.stdout.splitlines() if line.strip().startswith("/sd/")]
    return files


def pull_file(port, remote_path, local_path):
    """Copy a single file from the board to local_path."""
    os.makedirs(os.path.dirname(local_path), exist_ok=True)
    result = subprocess.run(
        ["mpremote", "connect", port, "cp", f":{remote_path}", local_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"  FAILED: {result.stderr.strip()}", file=sys.stderr)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="Pull SD card contents from ARGUS board")
    parser.add_argument("-o", "--output", default="sd_dump", help="Local output directory (default: ./sd_dump)")
    parser.add_argument("-p", "--port", default=None, help="Serial port (auto-detected if omitted)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        try:
            port = find_board_port()
        except ImportError:
            pass

    if port is None:
        print("Could not auto-detect board port. Specify with -p /dev/tty.usbmodemXXXX")
        sys.exit(1)

    print(f"Connecting to board on {port}...")
    print("Listing files on /sd...")

    files = list_sd_files(port)
    if not files:
        print("No files found on /sd. Is the SD card mounted and ENABLE_SD_CARD = True?")
        sys.exit(0)

    print(f"Found {len(files)} file(s). Copying to '{args.output}/'...\n")

    success, failed = 0, 0
    for remote_path in files:
        # Strip leading /sd/ to build local relative path
        relative = remote_path[len("/sd/"):]
        local_path = os.path.join(args.output, relative)
        print(f"  {remote_path} -> {local_path}")
        if pull_file(port, remote_path, local_path):
            success += 1
        else:
            failed += 1

    print(f"\nDone. {success} copied, {failed} failed.")
    if success:
        print(f"Files saved to: {os.path.abspath(args.output)}/")


if __name__ == "__main__":
    main()
