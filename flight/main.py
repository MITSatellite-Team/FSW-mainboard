# Import this first
import gc
import sys
import time
import board
import busio

from core import logger, setup_logger, state_manager
from core.satellite_config import main_config as CONFIG
from hal.configuration import SATELLITE

FIX_MODE_NAMES = {0: "NO_FIX", 1: "PREDICTION", 2: "2D", 3: "3D", 4: "DIFFERENTIAL"}
TMP117_ADDRESSES  = [0x48, 0x49, 0x4A, 0x4B]

# Memory stats
def print_memory_stats(call_gc=True):
    if call_gc:
        gc.collect()
    print(f"Memory stats after gc: {call_gc}")
    print(f"Total memory: {str(gc.mem_alloc() + gc.mem_free())} bytes")
    print(f"Memory free: {str(gc.mem_free())} bytes")
    print(f"Memory used: {int((gc.mem_alloc() / (gc.mem_alloc() + gc.mem_free())) * 100)}%")


for path in ["/hal", "/apps", "/core"]:
    if path not in sys.path:
        sys.path.append(path)

setup_logger(level=CONFIG.LOG_LEVEL)

print_memory_stats(call_gc=True)

print("Booting ARGUS...")
SATELLITE.boot_sequence()
print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")


print("Waiting 1 sec...")
time.sleep(1)


"""print("Running system diagnostics...")
errors = SATELLITE.run_system_diagnostics()
print("System diagnostics complete")
print("Errors:", errors)
"""

print_memory_stats(call_gc=False)
print_memory_stats(call_gc=True)

def collect_location(gps):
    print("Collecting Location...")

    if gps.update():
        fix_name = FIX_MODE_NAMES.get(gps.fix_mode, str(gps.fix_mode))

        if gps.has_fix():
            utc = gps.timestamp_utc

            print(
                "[FIX " + fix_name + "] " +
                str(utc["year"]) + "-" + str(utc["month"]) + "-" + str(utc["day"]) + " " +
                str(utc["hour"]) + ":" + str(utc["minute"]) + ":" + str(utc["second"]) + " UTC | " +
                "Lat " + str(round(gps.latitude, 5)) + " Lon " + str(round(gps.longitude, 5)) + " | " +
                "Alt " + str(round(gps.mean_sea_level_altitude, 1)) + " m MSL | " +
                "PDOP " + str(round(gps.pdop, 1))
            )

            return (gps.latitude, gps.longitude, gps.mean_sea_level_altitude)
        else:
            print("[" + fix_name + "] week=" + str(gps.week) + " tow=" + str(round(gps.tow, 1)) + " -- waiting for fix...")

    return None

def send_message(location, temperature, pressure, imu):
    # TODO: build packet, I just don't remember python byte manipulation :)

    payload = bytearray([])

    lattitude, longitude, mean_sea_level_altitude = (0, 0, 0)

    if not location is None:
        lattitude, longitude, mean_sea_level_altitude = location

    payload.extend(lattitude.to_bytes(32, 'little'))
    payload.extend(longitude.to_bytes(32, 'little'))
    payload.extend(mean_sea_level_altitude.to_bytes(32, 'little'))

    if temperature is None:
        temperature = 0

    payload.extend(temperature.to_bytes(32, 'little'))

    if pressure is None:
        pressure = 0

    payload.extend(temperature.to_bytes(32, 'little'))

    # TODO: IMU

    print(f"Sending message... {payload}")
    SATELLITE.RADIO.send(payload)

def _read_tmp117(i2c, address):
    buf = bytearray(2)

    try:
        i2c.writeto_then_readfrom(address, bytes([0x00]), buf)
    except Exception as e:
        print("  Read error at {}: {}".format(hex(address), e))

        return None

    raw = (buf[0] << 8) | buf[1]
    if raw & 0x8000:
        raw -= 0x10000

    return raw * 0.0078125

def collect_temperature(i2c1):
    counter = 0
    while not i2c1.try_lock():
        counter += 1

        if counter > 300: # WE LOVE NASA PROGRAMMING PRACTICES
            return None

    all_addresses = i2c1.scan()
    tmp117s = [a for a in all_addresses if a in TMP117_ADDRESSES]

    if not tmp117s:
        print("No TMP117 found. Addresses on bus:", [hex(a) for a in all_addresses])
    else:
        for addr in tmp117s:
            temp_c = _read_tmp117(i2c1, addr)

            if temp_c is not None:
                return temp_c

    i2c1.unlock()

    return None

send_message((0, 0, 0), 0, 0, None)

try:
    print("Initializing GPS...")
    SATELLITE.GPS.obj._board = "PX1120S"
    gps = SATELLITE.GPS.obj

    print("Initializing I2C")
    i2c1 = busio.I2C(board.SCL1, board.SDA1)

    print("Beginning Flight...")
    while True:

        # TODO: IMU data
        imu = None

        # TODO: temperature data
        temperature = collect_temperature(i2c1)

        # TODO: Collect pressure data
        pressure = None

        location = collect_location(gps)

        send_message(location, temperature, pressure, imu)

        time.sleep(5)

except Exception as e:
    logger.critical("ERROR:", e)
    # TODO Log the error
