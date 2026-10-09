import gc
import sys
import time
import board
import busio
import struct
import digitalio
import bitbangio
import microcontroller.pin as GPIO

from hal.drivers.arducam_mega import ArducamMega, FMT_YUV, MODE_96X96

csPin2 = digitalio.DigitalInOut(GPIO.GPIO13)
csPin2.switch_to_output(value=True)

loraPin = digitalio.DigitalInOut(GPIO.GPIO33)
loraPin.switch_to_output(value=True)

# spi = busio.SPI(GPIO.GPIO6, MOSI=GPIO.GPIO7, MISO=GPIO.GPIO4)
spi = bitbangio.SPI(GPIO.GPIO6, MOSI=GPIO.GPIO7, MISO=GPIO.GPIO4)
cam1 = ArducamMega(spi, GPIO.GPIO8)
cam1.boot()
cam1.configure(MODE_96X96, FMT_YUV)

frame = bytearray(96 * 96 * 2)
while True:
    length = cam1.take_picture()

    if length >= len(frame):
        cam1.read_fifo(frame)
        ArducamMega.render(frame)

from core import logger, setup_logger, state_manager
from core.logging import StreamHandler
from core.satellite_config import main_config as CONFIG
from core.time_processor import TimeProcessor as TPM
from hal.configuration import SATELLITE
from hal.argus_v4 import ArgusV4Interfaces
from hal.drivers.ms8607 import MS8607

FIX_MODE_NAMES = {0: "NO_FIX", 1: "PREDICTION", 2: "2D", 3: "3D", 4: "DIFFERENTIAL"}
TMP117_ADDRESSES  =  [0x48, 0x49, 0x4A, 0x4B]

for path in ["/hal", "/apps", "/core"]:
    if path not in sys.path:
        sys.path.append(path)

print("Booting ARGUS...")

SATELLITE.boot_sequence()
# cam1.boot()

print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")

# setup_logger(level=CONFIG.LOG_LEVEL)

# print("Waiting 1 sec...")
# time.sleep(1)

# def collect_location(gps):
#     print("Collecting Location...")
#     result = {"gps_valid": 0, "fix_mode": 0, "gps_utc": "", "lat": 0.0, "lon": 0.0, "alt": 0.0}

#     if gps.update():
#         result["gps_valid"] = 1
#         result["fix_mode"] = gps.fix_mode
#         fix_name = FIX_MODE_NAMES.get(gps.fix_mode, str(gps.fix_mode))

#         if not SATELLITE.RTC_AVAILABLE:
#             TPM.set_time_reference(gps.unix_time)

#         if gps.has_fix():
#             utc = gps.timestamp_utc
#             gps_utc = f"{utc['year']}-{utc['month']:02d}-{utc['day']:02d} {utc['hour']:02d}:{utc['minute']:02d}:{utc['second']:02d}"

#             print(
#                 "[FIX " + fix_name + "] " + gps_utc + " UTC | " +
#                 "Lat " + str(round(gps.latitude, 5)) + " Lon " + str(round(gps.longitude, 5)) + " | " +
#                 "Alt " + str(round(gps.mean_sea_level_altitude, 1)) + " m MSL | " +
#                 "PDOP " + str(round(gps.pdop, 1))
#             )

#             if gps.has_3d_fix():
#                 TPM.set_time(gps.unix_time)

#             result["gps_utc"] = gps_utc
#             result["lat"] = gps.latitude
#             result["lon"] = gps.longitude
#             result["alt"] = gps.mean_sea_level_altitude
#         else:
#             print("[" + fix_name + "] week=" + str(gps.week) + " tow=" + str(round(gps.tow, 1)) + " -- waiting for fix...")

#     return result

# _PACKET_FMT = '<2sIBBfffBffffBffBfffffffff2s'

# def send_message(location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag):
#     gps_valid = location.get("gps_valid", 0)
#     gps_fix = location.get("fix_mode", 0)
#     lat = location.get("lat", 0.0)
#     lon = location.get("lon", 0.0)
#     alt = location.get("alt", 0.0)

#     temps = (temperature or []) + [0.0, 0.0, 0.0, 0.0]
#     temp_valid = 0
#     if temperature:
#         for i in range(min(len(temperature), 4)):
#             temp_valid |= (1 << i)
#     t0, t1, t2, t3 = float(temps[0]), float(temps[1]), float(temps[2]), float(temps[3])

#     baro_valid = 0 if (pressure is None or humidity is None) else 1
#     pressure = float(pressure or 0.0)
#     humidity = float(humidity or 0.0)

#     imu_valid = 0 if (imu_accel is None or imu_gyro is None or imu_mag is None) else 1
#     if imu_valid:
#         ax, ay, az = imu_accel
#         gx, gy, gz = imu_gyro
#         mx, my, mz = imu_mag
#     else:
#         ax = ay = az = gx = gy = gz = mx = my = mz = 0.0

#     payload = struct.pack(
#         _PACKET_FMT,
#         b'ST',
#         TPM.time(),
#         gps_valid, gps_fix, lat, lon, alt,
#         temp_valid, t0, t1, t2, t3,
#         baro_valid, pressure, humidity,
#         imu_valid, ax, ay, az, gx, gy, gz, mx, my, mz,
#         b'RD',
#     )

#     print(f"Sending {len(payload)}B | t={TPM.time()} | GPS {gps_valid}/{gps_fix} ({lat},{lon},{alt}) | Temps {temperature} | Baro {pressure}hPa {humidity}% | IMU {imu_valid}")
#     SATELLITE.RADIO.send(payload)

# def _read_tmp117(i2c, address):
#     buf = bytearray(2)

#     try:
#         i2c.writeto_then_readfrom(address, bytes([0x00]), buf)
#     except Exception as e:
#         print("  Read error at {}: {}".format(hex(address), e))

#         return None

#     raw = (buf[0] << 8) | buf[1]
#     if raw & 0x8000:
#         raw -= 0x10000

#     return raw * 0.0078125

# def collect_temperature(i2c1):
#     counter = 0
#     while not i2c1.try_lock():
#         counter += 1
#         if counter > 300:
#             return None

#     all_addresses = i2c1.scan()
#     tmp117s = [a for a in all_addresses if a in TMP117_ADDRESSES]
#     # print(all_addresses)
#     result = []

#     if not tmp117s:
#         print("No TMP117 found. Addresses on bus:", [hex(a) for a in all_addresses])
#     else:
#         for addr in tmp117s:
#             temp_c = _read_tmp117(i2c1, addr)
            
#             if temp_c is not None:
#                 temp_f = temp_c * 9 / 5 + 32
#                 result.append(temp_f)

#     i2c1.unlock()
#     if(len(result) == 0):
#         return None
#     else:
#         return result

# send_message({"gps_valid": 0, "fix_mode": 0, "lat": 0.0, "lon": 0.0, "alt": 0.0}, None, None, None, None, None, None)

# try:
#     print("Initializing GPS...")
#     SATELLITE.GPS.obj._board = "PX1120S"
#     gps = SATELLITE.GPS.obj

#     print("Initializing I2C")
#     i2c1 = ArgusV4Interfaces.I2C1

#     print("Beginning Flight...")
#     if TPM.time() < 1735689600:  # RTC is uninitialized (before 2025), seed with compile-time approximation
#         TPM.set_time(1777669052)  # will be corrected on first 3D GPS fix
#     ms8607 = MS8607(i2c1)
    
#     while True:

#         # TODO: IMU data
#         imu = SATELLITE.IMU
#         if SATELLITE.IMU_AVAILABLE:
#             imu_accel = imu.accel()
#             imu_gyro = imu.gyro()
#             imu_mag = imu.mag()
#         else:
#             imu_accel = None
#             imu_gyro = None
#             imu_mag = None

#         # TODO: temperature data
#         temperature = collect_temperature(i2c1)

#         # TODO: Collect pressure data
#         pressure = ms8607.pressure
#         humidity = ms8607.relative_humidity

#         location = collect_location(gps)

#         send_message(location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag)

#         time.sleep(15)

# except Exception as e:
#     logger.critical("ERROR:", e)
#     # TODO Log the error
