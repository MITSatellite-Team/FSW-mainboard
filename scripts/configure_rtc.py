import time

from hal.configuration import SATELLITE

# Boot up SC
print("Booting ARGUS...")
SATELLITE.boot_sequence()
print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")

# Run `date +%s` in a terminal, then type the value below and hit enter
unix_timestamp = int(input("Enter current UTC unix timestamp: "))

SATELLITE.RTC.set_datetime(time.localtime(unix_timestamp))
print(f"RTC set to: {SATELLITE.RTC.datetime}")
print(f"lost_power cleared: {not SATELLITE.RTC.lost_power}")
