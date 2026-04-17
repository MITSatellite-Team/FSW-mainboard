import time

import board
import neopixel
import supervisor
from hal.configuration import SATELLITE

supervisor.runtime.autoreload = False

FIX_MODE_NAMES = {0: "NO_FIX", 1: "PREDICTION", 2: "2D", 3: "3D", 4: "DIFFERENTIAL"}

print("Booting ARGUS...")
SATELLITE.boot_sequence()
print("Boot complete. Errors: " + str(SATELLITE.ERRORS))

if not SATELLITE.GPS_AVAILABLE:
    print("ERROR: GPS not available after boot. Halting.")
    while True:
        pass

# Override board type for ground testing with PX1120S module.
# The driver defaults to S1216F8-GL (flight module, 81-byte AN0030 payload).
# PX1120S sends 59-byte AN0037 payloads — change this to "S1216F8-GL" for flight hardware.
# Must set on .obj (the raw GPS driver) because SATELLITE.GPS returns an objectWrapper;
# setting attributes on the wrapper does not forward to the wrapped instance.
SATELLITE.GPS.obj._board = "PX1120S"
print("GPS board type set to: " + str(SATELLITE.GPS.obj._board))
print("GPS online. Waiting for fix...")

if SATELLITE.NEOPIXEL_AVAILABLE:
    SATELLITE.NEOPIXEL.deinit()
led = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.2, pixel_order=neopixel.GRB)
led[0] = (255, 255, 0)  # yellow = searching

# Cache the raw driver to avoid objectWrapper allocating a new closure on every method call.
gps = SATELLITE.GPS.obj

sd_path = None
try:
    date = SATELLITE.RTC.datetime
    ts = str(date.tm_year) + "-" + str(date.tm_mon) + "-" + str(date.tm_mday) + "_" + str(date.tm_hour) + "-" + str(date.tm_min)
    sd_path = "/sd/gps_collect_" + ts + ".csv"
    header = "t_mono_ms,unix_time,fix_mode,week,tow,lat_deg,lon_deg,alt_ellipsoid_m,alt_msl_m,ecef_x,ecef_y,ecef_z,ecef_vx,ecef_vy,ecef_vz,gdop,pdop,hdop,vdop,tdop\n"
    with open(sd_path, "w") as f:
        f.write(header)
    print("SD logging to: " + sd_path)
except Exception as e:
    print("SD unavailable, logging to serial only. (" + str(e) + ")")
    sd_path = None

buf = []
last_flush = time.monotonic()
FLUSH_INTERVAL = 30

total = 0
fixes = 0


def flush():
    global last_flush
    if sd_path and buf:
        try:
            with open(sd_path, "a") as f:
                for row in buf:
                    f.write(row)
            print("  [" + str(len(buf)) + " rows flushed to SD]")
        except Exception as e:
            print("  [SD flush error: " + str(e) + "]")
    buf.clear()
    last_flush = time.monotonic()


while True:
    if gps.update():
        fix_name = FIX_MODE_NAMES.get(gps.fix_mode, str(gps.fix_mode))

        if gps.has_fix():
            led[0] = (0, 255, 0)  # green = fix
            fixes += 1
            t_ms = int(time.monotonic() * 1000)

            row = (
                str(t_ms) + "," +
                str(int(gps.unix_time)) + "," +
                str(gps.fix_mode) + "," +
                str(gps.week) + "," +
                str(round(gps.tow, 3)) + "," +
                str(round(gps.latitude, 6)) + "," +
                str(round(gps.longitude, 6)) + "," +
                str(round(gps.ellipsoid_altitude, 2)) + "," +
                str(round(gps.mean_sea_level_altitude, 2)) + "," +
                str(round(gps.ecef_x, 2)) + "," +
                str(round(gps.ecef_y, 2)) + "," +
                str(round(gps.ecef_z, 2)) + "," +
                str(round(gps.ecef_vx, 4)) + "," +
                str(round(gps.ecef_vy, 4)) + "," +
                str(round(gps.ecef_vz, 4)) + "," +
                str(round(gps.gdop, 2)) + "," +
                str(round(gps.pdop, 2)) + "," +
                str(round(gps.hdop, 2)) + "," +
                str(round(gps.vdop, 2)) + "," +
                str(round(gps.tdop, 2)) + "\n"
            )
            buf.append(row)

            utc = gps.timestamp_utc
            print(
                "[FIX " + fix_name + "] " +
                str(utc["year"]) + "-" + str(utc["month"]) + "-" + str(utc["day"]) + " " +
                str(utc["hour"]) + ":" + str(utc["minute"]) + ":" + str(utc["second"]) + " UTC | " +
                "Lat " + str(round(gps.latitude, 5)) + " Lon " + str(round(gps.longitude, 5)) + " | " +
                "Alt " + str(round(gps.mean_sea_level_altitude, 1)) + " m MSL | " +
                "PDOP " + str(round(gps.pdop, 1))
            )
        else:
            led[0] = (255, 255, 0)  # yellow = no fix yet
            print("[" + fix_name + "] week=" + str(gps.week) + " tow=" + str(round(gps.tow, 1)) + " -- waiting for fix...")

        total += 1

    else:
        pass

    if time.monotonic() - last_flush >= FLUSH_INTERVAL:
        flush()
        print("  Stats: " + str(fixes) + "/" + str(total) + " updates had a fix")

    time.sleep(0.5)
