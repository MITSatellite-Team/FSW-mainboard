import time

from hal.configuration import SATELLITE

print("Booting ARGUS...")
SATELLITE.boot_sequence()
print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")

if not SATELLITE.OPENLOG_AVAILABLE:
    print("ERROR: OpenLog not available.")
else:
    print("Waiting for OpenLog to finish startup...")
    SATELLITE.OPENLOG.wait_for_ready(timeout=3.0)
    SATELLITE.OPENLOG.write("Hello from Argus!\r\n")
    time.sleep(1)
    SATELLITE.OPENLOG.write("Second line.\r\n")
    print("Done. Safe to pull SD card.")
