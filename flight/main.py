# Import this first
import gc
import sys
import time
import board
import busio
import struct

from core import logger, setup_logger, state_manager
from core.logging import StreamHandler
from core.satellite_config import main_config as CONFIG
from hal.configuration import SATELLITE
from hal.argus_v4 import ArgusV4Interfaces
from hal.drivers.ms8607 import MS8607

FIX_MODE_NAMES = {0: "NO_FIX", 1: "PREDICTION", 2: "2D", 3: "3D", 4: "DIFFERENTIAL"}
TMP117_ADDRESSES  =  [0x48, 0x49, 0x4A, 0x4B]

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

_openlog_stream = None
if SATELLITE.OPENLOG_AVAILABLE:
    try:
        print("Waiting for OpenLog to finish startup...")
        SATELLITE.OPENLOG.wait_for_ready(timeout=3.0)
        openlog_handler = StreamHandler(SATELLITE.OPENLOG)
        logger.addHandler(openlog_handler)
        _openlog_stream = SATELLITE.OPENLOG
        _openlog_stream.write("time_s,gps_valid,gps_fix,gps_utc,lat,lon,alt_m,temp_0_f,temp_1_f,temp_2_f,temp_3_f,pressure_hpa,humidity_pct,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x_uT,mag_y_uT,mag_z_uT\r\n")
        print("OpenLog ready — CSV header written.")
    except Exception:
        _openlog_stream = None
        print("OpenLog setup failed — logging to serial only.")
else:
    print("OpenLog not available, logging to serial only.")


def log_csv_row(t, location, temperatures, pressure, humidity, imu_accel, imu_gyro, imu_mag):
    if _openlog_stream is None:
        return
    gps_valid = location.get("gps_valid", 0)
    gps_fix = location.get("fix_mode", 0)
    gps_utc = location.get("gps_utc", "")
    lat = location.get("lat", "")
    lon = location.get("lon", "")
    alt = location.get("alt", "")
    temps = (temperatures or []) + ["", "", "", ""]
    t0, t1, t2, t3 = temps[0], temps[1], temps[2], temps[3]
    pres = "" if pressure is None else pressure
    hum = "" if humidity is None else humidity
    if imu_accel and imu_gyro:
        ax, ay, az = imu_accel
        gx, gy, gz = imu_gyro
    else:
        ax = ay = az = gx = gy = gz = ""
    if imu_mag:
        mx, my, mz = imu_mag
    else:
        mx = my = mz = ""
    row = f"{t},{gps_valid},{gps_fix},{gps_utc},{lat},{lon},{alt},{t0},{t1},{t2},{t3},{pres},{hum},{ax},{ay},{az},{gx},{gy},{gz},{mx},{my},{mz}\r\n"
    try:
        _openlog_stream.write(row)
    except Exception:
        pass


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
    result = {"gps_valid": 0, "fix_mode": 0, "gps_utc": "", "lat": 0.0, "lon": 0.0, "alt": 0.0}

    if gps.update():
        result["gps_valid"] = 1
        result["fix_mode"] = gps.fix_mode
        fix_name = FIX_MODE_NAMES.get(gps.fix_mode, str(gps.fix_mode))

        if gps.has_fix():
            utc = gps.timestamp_utc
            gps_utc = f"{utc['year']}-{utc['month']:02d}-{utc['day']:02d} {utc['hour']:02d}:{utc['minute']:02d}:{utc['second']:02d}"

            print(
                "[FIX " + fix_name + "] " + gps_utc + " UTC | " +
                "Lat " + str(round(gps.latitude, 5)) + " Lon " + str(round(gps.longitude, 5)) + " | " +
                "Alt " + str(round(gps.mean_sea_level_altitude, 1)) + " m MSL | " +
                "PDOP " + str(round(gps.pdop, 1))
            )

            result["gps_utc"] = gps_utc
            result["lat"] = gps.latitude
            result["lon"] = gps.longitude
            result["alt"] = gps.mean_sea_level_altitude
        else:
            print("[" + fix_name + "] week=" + str(gps.week) + " tow=" + str(round(gps.tow, 1)) + " -- waiting for fix...")

    return result

_PACKET_FMT = '<2sIBBfffBffffBffBfffffffff2s'

def send_message(location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag):
    gps_valid = location.get("gps_valid", 0)
    gps_fix = location.get("fix_mode", 0)
    lat = location.get("lat", 0.0)
    lon = location.get("lon", 0.0)
    alt = location.get("alt", 0.0)

    temps = (temperature or []) + [0.0, 0.0, 0.0, 0.0]
    temp_valid = 0
    if temperature:
        for i in range(min(len(temperature), 4)):
            temp_valid |= (1 << i)
    t0, t1, t2, t3 = float(temps[0]), float(temps[1]), float(temps[2]), float(temps[3])

    baro_valid = 0 if (pressure is None or humidity is None) else 1
    pressure = float(pressure or 0.0)
    humidity = float(humidity or 0.0)

    imu_valid = 0 if (imu_accel is None or imu_gyro is None or imu_mag is None) else 1
    if imu_valid:
        ax, ay, az = imu_accel
        gx, gy, gz = imu_gyro
        mx, my, mz = imu_mag
    else:
        ax = ay = az = gx = gy = gz = mx = my = mz = 0.0

    payload = struct.pack(
        _PACKET_FMT,
        b'ST',
        int(time.monotonic()),
        gps_valid, gps_fix, lat, lon, alt,
        temp_valid, t0, t1, t2, t3,
        baro_valid, pressure, humidity,
        imu_valid, ax, ay, az, gx, gy, gz, mx, my, mz,
        b'RD',
    )

    print(f"Sending {len(payload)}B | GPS {gps_valid}/{gps_fix} ({lat},{lon},{alt}) | Temps {temperature} | Baro {pressure}hPa {humidity}% | IMU {imu_valid}")
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
        if counter > 300:
            return None

    all_addresses = i2c1.scan()
    tmp117s = [a for a in all_addresses if a in TMP117_ADDRESSES]
    # print(all_addresses)
    result = []

    if not tmp117s:
        print("No TMP117 found. Addresses on bus:", [hex(a) for a in all_addresses])
    else:
        for addr in tmp117s:
            temp_c = _read_tmp117(i2c1, addr)
            
            if temp_c is not None:
                temp_f = temp_c * 9 / 5 + 32
                result.append(temp_f)

    i2c1.unlock()
    if(len(result) == 0):
        return None
    else:
        return result
    # return (len(result) == 0 ) ? None: result 
# def collect_pressure(i2c1):
    

send_message({"gps_valid": 0, "fix_mode": 0, "lat": 0.0, "lon": 0.0, "alt": 0.0}, None, None, None, None, None, None)

try:
    print("Initializing GPS...")
    SATELLITE.GPS.obj._board = "PX1120S"
    gps = SATELLITE.GPS.obj

    print("Initializing I2C")
    i2c1 = ArgusV4Interfaces.I2C1

    print("Beginning Flight...")
    ms8607 = MS8607(i2c1)
    
    while True:

        # TODO: IMU data
        imu = SATELLITE.IMU
        if SATELLITE.IMU_AVAILABLE:
            imu_accel = imu.accel()
            imu_gyro = imu.gyro()
            imu_mag = imu.mag()
        else:
            imu_accel = None
            imu_gyro = None
            imu_mag = None

        # TODO: temperature data
        temperature = collect_temperature(i2c1)

        # TODO: Collect pressure data
        pressure = ms8607.pressure
        humidity = ms8607.relative_humidity

        location = collect_location(gps)

        log_csv_row(time.monotonic(), location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag)

        send_message(location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag)

        time.sleep(5)

except Exception as e:
    logger.critical("ERROR:", e)
    # TODO Log the error
