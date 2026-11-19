import gc
import sys
import time
import board
import busio
import struct
import digitalio
import bitbangio
import os
import microcontroller.pin as GPIO
import binascii

from hal.drivers.gpio_camera import get_image
from core import logger, setup_logger, state_manager
from core.logging import StreamHandler
from core.satellite_config import main_config as CONFIG
from core.time_processor import TimeProcessor as TPM
from hal.configuration import SATELLITE
from hal.argus_v4 import ArgusV4Interfaces
from hal.drivers.ms8607 import MS8607
from hal.drivers.i2c_sdcard import QwiicOpenLog

FIX_MODE_NAMES = {0: "NO_FIX", 1: "PREDICTION", 2: "2D", 3: "3D", 4: "DIFFERENTIAL"}
TMP117_ADDRESSES  =  [0x4A, 0x4B, 0x49]


IMG_W, IMG_H = 160, 120
FRAME = bytearray(IMG_W * IMG_H)

OK, TX_TIMEOUT = 0, -5

for path in ["/hal", "/apps", "/core"]:
    if path not in sys.path:
        sys.path.append(path)

print("Booting ARGUS...")

SATELLITE.boot_sequence()

print("ARGUS booted.")
print(f"Boot Errors: {SATELLITE.ERRORS}")

setup_logger(level=CONFIG.LOG_LEVEL)

print("Waiting 1 sec...")
time.sleep(1)

def collect_location(gps):
    print("Collecting Location...")
    result = {"gps_valid": 0, "fix_mode": 0, "gps_utc": "", "lat": 0.0, "lon": 0.0, "alt": 0.0}

    if gps.update():
        result["gps_valid"] = 1
        result["fix_mode"] = gps.fix_mode
        fix_name = FIX_MODE_NAMES.get(gps.fix_mode, str(gps.fix_mode))

        if not SATELLITE.RTC_AVAILABLE:
            TPM.set_time_reference(gps.unix_time)

        if gps.has_fix():
            utc = gps.timestamp_utc
            gps_utc = f"{utc['year']}-{utc['month']:02d}-{utc['day']:02d} {utc['hour']:02d}:{utc['minute']:02d}:{utc['second']:02d}"

            print(
                "[FIX " + fix_name + "] " + gps_utc + " UTC | " +
                "Lat " + str(round(gps.latitude, 5)) + " Lon " + str(round(gps.longitude, 5)) + " | " +
                "Alt " + str(round(gps.mean_sea_level_altitude, 1)) + " m MSL | " +
                "PDOP " + str(round(gps.pdop, 1))
            )

            if gps.has_3d_fix():
                TPM.set_time(gps.unix_time)

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
        TPM.time(),
        gps_valid, gps_fix, lat, lon, alt,
        temp_valid, t0, t1, t2, t3,
        baro_valid, pressure, humidity,
        imu_valid, ax, ay, az, gx, gy, gz, mx, my, mz,
        b'RD',
    )

    print(f"Sending {len(payload)}B | t={TPM.time()} | GPS {gps_valid}/{gps_fix} ({lat},{lon},{alt}) | Temps {temperature} | Baro {pressure}hPa {humidity}% | IMU {imu_valid}")

    try:
        sd_card.append("flight.txt")
        sd_card.write(binascii.b2a_base64(payload) + '\n')
        sd_card.sync()
    except:
        print("Failed to save packet")
    
    SATELLITE.RADIO.send(payload)

def read_tmp117(address):
    buf = bytearray(2)

    try:
        ArgusV4Interfaces.I2C1.writeto_then_readfrom(address, bytes([0x00]), buf)
    except Exception as e:
        print("  Read error at {}: {}".format(hex(address), e))

        return None

    raw = (buf[0] << 8) | buf[1]
    if raw & 0x8000:
        raw -= 0x10000

    return raw * 0.0078125

def collect_temperature():
    counter = 0
    while not ArgusV4Interfaces.I2C1.try_lock():
        counter += 1
        if counter > 300:
            return None

    all_addresses = ArgusV4Interfaces.I2C1.scan()
    print(all_addresses)
    tmp117s = [a for a in all_addresses if a in TMP117_ADDRESSES]

    result = []

    if not tmp117s:
        print("No TMP117 found. Addresses on bus:", [hex(a) for a in all_addresses])
    else:
        for addr in tmp117s:
            temp_c = read_tmp117(addr)
            
            if temp_c is not None:
                temp_f = temp_c * 9 / 5 + 32
                result.append(temp_f)

    ArgusV4Interfaces.I2C1.unlock()

    if(len(result) == 0):
        return None
    else:
        return result

def rearm_rx():
    SATELLITE.RADIO.tx_en.value = False
    SATELLITE.RADIO.rx_en.value = True
    SATELLITE.RADIO.startReceive()

def send(payload):
    try:
        n, state = SATELLITE.RADIO.send(payload)
    except RuntimeError as e:
        print("tx error:", e)
        state = -1
    if state != OK:
        rearm_rx()
    return state

def poll():
    if not SATELLITE.RADIO.RX_available():
        return None
    try:
        data, state = SATELLITE.RADIO.recv()
    except RuntimeError as e:
        print("bad packet:", e)
        return None
    return data if state == OK else None

def rid():
    b = bytearray(os.urandom(4))
    return "".join("%02x" % x for x in b)

SATELLITE.RADIO.startReceive()

last_handled_packet_id = -1
img_size = 0

sd_card = None
try:
    sd_card = QwiicOpenLog(ArgusV4Interfaces.I2C1)
    print("fw", sd_card.version(), sd_card.status())
except:
    print("SD card failed to initialize")

gps = None
try:
    SATELLITE.GPS.obj._board = "PX1120S"
    gps = SATELLITE.GPS.obj
except:
    print("GPS card failed to initialize")

ms8607 = None
try:
    ms8607 = MS8607(ArgusV4Interfaces.I2C1)
except:
    print("Barometer failed to initialize")

last_tx = 0
while True:
    try:
        if time.monotonic() - last_tx > 15:
            temperatures = collect_temperature()
            print(f"Collected temperatures: {temperatures}")

            if SATELLITE.IMU_AVAILABLE:
                imu_accel = SATELLITE.IMU.accel()
                imu_gyro = SATELLITE.IMU.gyro()
                imu_mag = SATELLITE.IMU.mag()

                print(f"Collected IMU a: {imu_accel} g: {imu_gyro} m: {imu_mag}")
            else:
                imu_accel = None
                imu_gyro = None
                imu_mag = None

            location = None
            if not gps is None:
                location = collect_location(gps)
                print(f"Collected location {location}")

            pressure = None
            humidity = None

            if not ms8607 is None:
                pressure = ms8607.pressure
                humidity = ms8607.relative_humidity

                print(f"Collected pressure and humidity {pressure} {humidity}")

            send_message(location, temperatures, pressure, humidity, imu_accel, imu_gyro, imu_mag)

            last_tx = time.monotonic()
    except:
        print("Failed to send update")

    packet = poll()

    try:
        if packet is not None:
            print("Packet:", packet, SATELLITE.RADIO.rssi())

            if len(packet) >= 4 and packet[:2] == b'CM':
                command = packet[2]
                id = packet[3]

                print("Command:", command, id)
                
                if last_handled_packet_id != id:
                    if command == 0:
                        print("Taking image...")
    
                        size = get_image(FRAME)
                        
                        if size is None:
                            img_size = 0
                        else:
                            img_size = size
                        
                        print(size)

                        try:
                            sd_card.append(rid() + ".b64")
                            sd_card.write(binascii.b2a_base64(FRAME[0:size]))
                            sd_card.sync()
                        except:
                            print("Failed to save image")

                if command == 0:
                    print("Took image!")
                    print(id)
                    print(img_size)
                    send(struct.pack(
                        '<2sII',
                        b'FL',
                        id,
                        img_size
                    ))
                elif command == 1:
                    print(packet[4], packet[5], (packet[4] << 8) + packet[5], len(FRAME) - 1)
                    read_index = min(len(FRAME) - 1, (packet[4] << 8) + packet[5])

                    print("Sending image data from image", read_index)
                    
                    send(struct.pack(
                        '<2sI200s',
                        b'FL',
                        id,
                        FRAME[read_index:min(len(FRAME), read_index + 200)]
                    ))

                last_handled_packet_id = id
            else:
                print("Invalid packet:", packet[:2])
    except:
        print("Failed to handle command")

    time.sleep(0.01)