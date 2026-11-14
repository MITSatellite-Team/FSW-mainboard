"""Argus5 GPS test (SkyTraq PX1120S on UART1, 1PPS on GPIO21).

Checks, in order:
  1. Power: GPS rail on (green LED D13), 3V3_PERIPH current via INA226 U9
  2. UART: bytes arrive, baud found, NMEA checksums valid
  3. Sky: satellites in view per constellation and their signal strength (SNR)
  4. Fix: fix type, satellites used, HDOP, position, time to first fix
  5. 1PPS: one pulse per second once a 3D fix is held

Use a PASSIVE antenna on J12 (no bias voltage is supplied for an active one),
with a clear view of the sky.
"""
import time
import board
import busio
import digitalio
from adafruit_bus_device.i2c_device import I2CDevice

# --- Power --------------------------------------------------------------------
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)  # feeds GPS load switch U14
time.sleep(0.01)
gps_en = digitalio.DigitalInOut(board.GPS_EN)
gps_en.switch_to_output(value=True)
t_start = time.monotonic()
print("GPS power enabled (green LED D13 should be on)")

try:
    ina = I2CDevice(board.I2C(), 0x41)  # 3V3_PERIPH rail, includes the GPS
except (OSError, ValueError, RuntimeError):
    ina = None


def periph_ma():
    b = bytearray(2)
    with ina as d:
        d.write_then_readinto(b"\x01", b)
    raw = (b[0] << 8) | b[1]
    if raw & 0x8000:
        raw -= 0x10000
    return raw * 2.5e-6 / 0.010 * 1000


# --- 1PPS counter -------------------------------------------------------------
try:
    import countio
    pps = countio.Counter(board.GPS_1PPS, edge=countio.Edge.RISE)
except (ImportError, ValueError, RuntimeError, NotImplementedError) as e:
    pps = None
    print("1PPS counter unavailable:", e)


# --- UART and NMEA ------------------------------------------------------------
def checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def find_baud():
    for baud in (115200, 9600, 38400, 57600, 4800):
        u = busio.UART(board.TX1, board.RX1, baudrate=baud, timeout=0,
                       receiver_buffer_size=2048)
        buf, got, good = b"", 0, 0
        end = time.monotonic() + 2.5
        while time.monotonic() < end and good < 3:
            data = u.read(256)
            if data:
                got += len(data)
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        if checksum_ok(line.decode().strip()):
                            good += 1
                    except UnicodeError:
                        pass
        print("  %6d baud: %4d bytes, %d valid NMEA" % (baud, got, good))
        if good >= 3:
            return u, baud
        u.deinit()
    return None, None


print("\n[2] Looking for NMEA on UART1 (TX1/RX1)...")
time.sleep(0.5)  # receiver boot
uart, baud = find_baud()
if not uart:
    print("\nNo valid NMEA at any baud rate.")
    print("No bytes at all: check D13 (power), GPS_nRST high, R122 to U15 TXD.")
    print("Bytes but no valid NMEA: noise or wrong UART settings.")
    raise SystemExit
print("  -> receiver found at %d baud" % baud)

CONSTELLATIONS = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
                  "GB": "BeiDou", "BD": "BeiDou", "GQ": "QZSS"}
sats = {}          # (talker, prn) -> (snr or None, last seen)
fix = {}           # latest GGA/RMC fields
stats = {"ok": 0, "bad": 0}
ttff = None


def to_deg(value, hemi):
    if not value:
        return None
    dot = value.find(".")
    deg = float(value[:dot - 2]) + float(value[dot - 2:]) / 60
    return -deg if hemi in ("S", "W") else deg


def handle(line):
    global ttff
    if not checksum_ok(line):
        stats["bad"] += 1
        return
    stats["ok"] += 1
    f = line[1:line.index("*")].split(",")
    talker, kind = f[0][:2], f[0][2:]
    if kind == "GGA" and len(f) > 9:
        fix.update(utc=f[1], lat=to_deg(f[2], f[3]), lon=to_deg(f[4], f[5]),
                   quality=int(f[6] or 0), used=int(f[7] or 0),
                   hdop=f[8], alt=f[9])
        if fix["quality"] > 0 and ttff is None:
            ttff = time.monotonic() - t_start
    elif kind == "RMC" and len(f) > 9:
        fix.update(status=f[2], date=f[9])
    elif kind == "GSA" and len(f) > 2:
        fix.update(mode=f[2])  # 1 = no fix, 2 = 2D, 3 = 3D
    elif kind == "GSV":
        now = time.monotonic()
        for i in range(4, len(f) - 3, 4):
            if f[i]:
                snr = int(f[i + 3]) if f[i + 3] else None
                sats[(talker, f[i])] = (snr, now)


# --- Main loop ----------------------------------------------------------------
print("\n[3-5] Live status (once per second). Outdoors, a cold start fix can")
print("take 30 s to a few minutes; with the backup cell charged, seconds.\n")
buf = b""
last_print = time.monotonic()
last_pps = pps.count if pps else 0

while True:
    data = uart.read(512)
    if data:
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                handle(line.decode().strip())
            except (UnicodeError, ValueError, IndexError):
                stats["bad"] += 1

    now = time.monotonic()
    if now - last_print < 1.0:
        continue
    last_print = now

    for k in [k for k, (s, t) in sats.items() if now - t > 5]:
        del sats[k]
    view, tracked = {}, []
    for (talker, prn), (snr, _) in sats.items():
        name = CONSTELLATIONS.get(talker, talker)
        view[name] = view.get(name, 0) + 1
        if snr:
            tracked.append(snr)
    tracked.sort(reverse=True)

    mode = {"1": "no fix", "2": "2D", "3": "3D"}.get(fix.get("mode", "1"), "no fix")
    pps_new = 0
    if pps:
        pps_new, last_pps = pps.count - last_pps, pps.count

    print("t=%4.0fs  NMEA ok/bad %d/%d  UTC %s %s" % (
        now - t_start, stats["ok"], stats["bad"],
        fix.get("utc", "-")[:6] or "-", fix.get("date", "") or ""))
    print("  in view: %s" % (", ".join("%s %d" % kv for kv in sorted(view.items())) or "none"))
    print("  SNR (dB-Hz, best first): %s" % (tracked[:8] or "-"))
    line = "  fix: %s, %d used, HDOP %s" % (mode, fix.get("used", 0), fix.get("hdop", "-") or "-")
    if fix.get("lat") is not None:
        line += ", %.6f, %.6f, %s m" % (fix["lat"], fix["lon"], fix.get("alt", "?"))
    print(line)
    extra = "  1PPS pulses this second: %s" % (pps_new if pps else "n/a")
    if ttff is not None:
        extra += "   TTFF %.0f s" % ttff
    if ina:
        extra += "   3V3_PERIPH %.1f mA" % periph_ma()
    print(extra + "\n")
