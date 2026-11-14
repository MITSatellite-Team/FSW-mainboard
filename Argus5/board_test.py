"""Argus5 full board self-test (CircuitPython 9.2.9).

Save as code.py. Runs every subsystem check in order and prints a summary.
Each result is PASS, WARN (works but something looks off or needs the
environment, e.g. sky view), FAIL, or SKIP.

Order (cheap and non-destructive first):
   1. System info            7. RTC (RV-3032)            13. GPS (PX1120S)
   2. PSRAM                  8. Magnetometer (MMC5983MA) 14. Watchdog (optional,
   3. SPI flash              9. IMU (ASM330LHHG1)            resets the board)
   4. Power rails + INA226s 10. SD card
   5. I2C0 scan             11. LoRa radio, receive
   6. NeoPixel              12. LoRa radio, transmit (optional)

Not covered: backplane BUS_IO pins (need the mating board) and the buttons.
"""
import gc
import os
import struct
import time

import board
import busio
import digitalio
import microcontroller
import supervisor
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_bus_device.spi_device import SPIDevice

# =============================================================================
# Settings
# =============================================================================
PSRAM_TEST_MB = 2          # PSRAM pattern test size
SD_BAUD = 4_000_000        # conservative for the 1k series resistors on SPI
LORA_FREQ_HZ = 433_000_000 # multiple of 125 kHz
GPS_FIX_TIMEOUT = 90       # seconds to wait for a fix (needs sky view)

RUN_TX_TEST = False        # transmits! antenna/load on J11 + licensed operator
CALLSIGN = "N0CALL"        # required for RUN_TX_TEST
TX_DRIVE_DBM = -9          # SX1268 drive for the TX test (lowest = -9)

RUN_WATCHDOG_TEST = False  # resets the board at the end; result on next boot

# =============================================================================
# Results
# =============================================================================
results = []


def record(name, status, detail=""):
    results.append((name, status, detail))
    print("  [%s] %s%s" % (status, name, (" - " + detail) if detail else ""))


def section(n, title):
    print("\n=== %d. %s %s" % (n, title, "=" * max(0, 56 - len(title))))


def run(n, title, fn):
    section(n, title)
    try:
        fn()
    except Exception as e:  # keep going so one bad part doesn't hide the rest
        record(title, "FAIL", "%s: %s" % (type(e).__name__, e))
    gc.collect()


def summary():
    print("\n" + "=" * 64)
    print("SUMMARY")
    print("=" * 64)
    counts = {}
    for name, status, detail in results:
        counts[status] = counts.get(status, 0) + 1
        print("  %-4s  %-30s %s" % (status, name, detail))
    print("-" * 64)
    print("  " + "  ".join("%s %d" % (s, counts.get(s, 0))
                           for s in ("PASS", "WARN", "FAIL", "SKIP")))
    print("=" * 64)


def out(pin):
    p = digitalio.DigitalInOut(pin)
    p.switch_to_output(value=False)
    return p


# =============================================================================
# Watchdog result from a previous run (checked first, before anything else)
# =============================================================================
WDT_MAGIC = b"WDT1"
nvm = microcontroller.nvm
if supervisor.runtime.run_reason == supervisor.RunReason.STARTUP and nvm[0:4] == WDT_MAGIC:
    last = struct.unpack(">H", nvm[4:6])[0] / 10
    nvm[0:6] = b"\x00" * 6
    reason = microcontroller.cpu.reset_reason
    print("\n=== Watchdog test result (from the previous run) ===")
    if reason == microcontroller.ResetReason.RESET_PIN:
        print("  [PASS] Watchdog reset the MCU via RUN, ~%.1f s after the last kick" % last)
    else:
        print("  [FAIL] Test was armed but reset reason is %s" % reason)
    print("  The other results were printed before the reset.")
    print("  Save the file or press Ctrl-D to run the full test again.")
    while True:
        time.sleep(1)

# =============================================================================
# Pins (everything off at start; enables have 100k pull-downs anyway)
# =============================================================================
periph_en = out(board.PERIPH_PWR_EN)
lora_en = out(board.LORA_EN)
mag_en = out(board.MAG_EN)
imu_en = out(board.IMU_EN)
rtc_en = out(board.RTC_EN)
gps_en = out(board.GPS_EN)
sd_en = out(board.SD_EN)
wdt_en = out(board.WDT_EN)    # watchdog stays disconnected from RUN
wdt_wdi = out(board.WDT_WDI)

t_suite = time.monotonic()
print("\nArgus5 board self-test starting")


# =============================================================================
# Drivers
# =============================================================================
class INA226:
    def __init__(self, i2c, address, shunt_ohms):
        self.address = address
        self._dev = I2CDevice(i2c, address)
        self._shunt = shunt_ohms
        self._buf = bytearray(3)
        self.mfr = self._read(0xFE)
        self.die = self._read(0xFF)
        self._write(0x00, 0x4527)  # continuous, 1.1 ms conversions, 16x avg

    def _read(self, reg):
        self._buf[0] = reg
        with self._dev as d:
            d.write_then_readinto(self._buf, self._buf, out_end=1, in_end=2)
        return (self._buf[0] << 8) | self._buf[1]

    def _write(self, reg, value):
        self._buf[0], self._buf[1], self._buf[2] = reg, value >> 8, value & 0xFF
        with self._dev as d:
            d.write(self._buf)

    @property
    def voltage(self):
        return self._read(0x02) * 0.00125

    @property
    def current_ma(self):
        raw = self._read(0x01)
        if raw & 0x8000:
            raw -= 0x10000
        return raw * 2.5e-6 / self._shunt * 1000

    def average_ma(self, seconds=0.3):
        time.sleep(0.05)
        total, n, end = 0.0, 0, time.monotonic() + seconds
        while time.monotonic() < end:
            total, n = total + self.current_ma, n + 1
            time.sleep(0.04)
        return total / n


def _bcd(b):
    return (b >> 4) * 10 + (b & 0x0F)


class RV3032:
    def __init__(self, i2c):
        self._dev = I2CDevice(i2c, 0x51)

    def read(self, reg, n):
        b = bytearray(n)
        with self._dev as d:
            d.write_then_readinto(bytes((reg,)), b)
        return b

    def seconds_of_day(self):
        b = self.read(0x01, 3)
        return _bcd(b[2] & 0x3F) * 3600 + _bcd(b[1] & 0x7F) * 60 + _bcd(b[0] & 0x7F)

    def datetime_str(self):
        b = self.read(0x01, 7)
        return "%04d-%02d-%02d %02d:%02d:%02d" % (
            2000 + _bcd(b[6]), _bcd(b[5] & 0x1F), _bcd(b[4] & 0x3F),
            _bcd(b[2] & 0x3F), _bcd(b[1] & 0x7F), _bcd(b[0] & 0x7F))

    def temperature(self):
        def raw():
            b = self.read(0x0E, 2)
            msb = b[1] - 256 if b[1] & 0x80 else b[1]
            return (msb << 4) | (b[0] >> 4)
        prev, cur = None, raw()
        while cur != prev:
            prev, cur = cur, raw()
        return cur / 16


class MMC5983MA:
    def __init__(self, spi, cs_pin):
        self._dev = SPIDevice(spi, digitalio.DigitalInOut(cs_pin),
                              baudrate=1_000_000, polarity=0, phase=0)
        self._buf = bytearray(7)
        self._w(0x0A, 0x80)  # software reset
        time.sleep(0.015)
        self.product_id = self._r(0x2F, 1)[0]

    def _w(self, reg, value):
        with self._dev as spi:
            spi.write(bytes((reg, value)))

    def _r(self, reg, n):
        buf = memoryview(self._buf)[:n]
        with self._dev as spi:
            spi.write(bytes((0x80 | reg,)))
            spi.readinto(buf)
        return buf

    def _wait(self, bit):
        end = time.monotonic() + 0.05
        while not self._r(0x08, 1)[0] & bit:
            if time.monotonic() > end:
                raise RuntimeError("measurement timeout")

    def _raw(self):
        self._w(0x09, 0x01)
        time.sleep(0.009)
        self._wait(0x01)
        b = self._r(0x00, 7)
        return ((b[0] << 10) | (b[1] << 2) | ((b[6] >> 6) & 3),
                (b[2] << 10) | (b[3] << 2) | ((b[6] >> 4) & 3),
                (b[4] << 10) | (b[5] << 2) | ((b[6] >> 2) & 3))

    def magnetic(self):
        self._w(0x09, 0x08)  # SET
        time.sleep(0.001)
        s = self._raw()
        self._w(0x09, 0x10)  # RESET
        time.sleep(0.001)
        r = self._raw()
        return tuple((s[i] - r[i]) / 2 / 163.84 for i in range(3))

    def temperature(self):
        self._w(0x09, 0x02)
        time.sleep(0.002)
        self._wait(0x02)
        return -75 + 0.8 * self._r(0x07, 1)[0]


class ASM330LHHG1:
    def __init__(self, spi, cs_pin):
        self._dev = SPIDevice(spi, digitalio.DigitalInOut(cs_pin),
                              baudrate=1_000_000, polarity=1, phase=1)
        self._buf = bytearray(14)
        self.who_am_i = self._r(0x0F, 1)[0]
        if self.who_am_i != 0x6B:
            return
        self._w(0x12, 0x01)   # software reset
        time.sleep(0.01)
        self._w(0x12, 0x44)   # BDU + auto-increment
        self._w(0x13, 0x04)   # I2C off (shared SPI bus)
        self._w(0x10, 0x48)   # accel 104 Hz, +/-4 g
        self._w(0x11, 0x44)   # gyro 104 Hz, +/-500 dps
        time.sleep(0.1)

    def _w(self, reg, value):
        with self._dev as spi:
            spi.write(bytes((reg, value)))

    def _r(self, reg, n):
        buf = memoryview(self._buf)[:n]
        with self._dev as spi:
            spi.write(bytes((0x80 | reg,)))
            spi.readinto(buf)
        return buf

    def read(self):
        t, gx, gy, gz, ax, ay, az = struct.unpack("<7h", self._r(0x20, 14))
        k = 0.122e-3 * 9.80665
        return (ax * k, ay * k, az * k), (gx * 0.0175, gy * 0.0175, gz * 0.0175), t / 256 + 25


# =============================================================================
# Shared buses and sensors (created lazily by the tests)
# =============================================================================
state = {"i2c": None, "spi": None, "ina": {}}


def i2c():
    if state["i2c"] is None:
        state["i2c"] = board.I2C()
    return state["i2c"]


def spi():
    if state["spi"] is None:
        state["spi"] = board.SPI()
    return state["spi"]


def periph_ma():
    ina = state["ina"].get("3V3_PERIPH")
    return ina.average_ma() if ina else None


def delta_str(before):
    after = periph_ma()
    if before is None or after is None:
        return "", after
    return "+%.1f mA on 3V3_PERIPH" % (after - before), after


# =============================================================================
# 1. System info
# =============================================================================
def test_info():
    u = os.uname()
    cpu = microcontroller.cpu
    print("  CircuitPython %s, board %s" % (u.release, board.board_id))
    print("  CPU %.0f MHz, %.1f C, reset reason %s" % (
        cpu.frequency / 1e6, cpu.temperature, cpu.reset_reason))
    t = cpu.temperature
    record("MCU temperature", "PASS" if -20 < t < 85 else "WARN", "%.1f C" % t)


# =============================================================================
# 2. PSRAM
# =============================================================================
def test_psram():
    gc.collect()
    heap = gc.mem_free() + gc.mem_alloc()
    if heap < 1024 * 1024:
        record("PSRAM detected", "FAIL", "heap %.2f MB (internal SRAM only)" % (heap / 1048576))
        return
    record("PSRAM detected", "PASS", "heap %.2f MB" % (heap / 1048576))
    block = 4096
    size = min(PSRAM_TEST_MB * 1048576, gc.mem_free() - 512 * 1024) // block * block
    buf = bytearray(size)
    mv = memoryview(buf)
    n = size // block
    base = bytearray(os.urandom(block))
    check = bytearray(block)
    errors = 0
    gc.disable()  # a GC would scan the whole random buffer over QSPI
    try:
        t0 = time.monotonic()
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            mv[i * block:(i + 1) * block] = base
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            check[:] = mv[i * block:(i + 1) * block]
            if check != base:
                errors += 1
        dt = time.monotonic() - t0
    finally:
        gc.enable()
    del mv, buf
    record("PSRAM pattern test", "PASS" if errors == 0 else "FAIL",
           "%.1f MB, %d bad blocks, %.1f s" % (size / 1048576, errors, dt))


# =============================================================================
# 3. SPI flash
# =============================================================================
def test_flash():
    st = os.statvfs("/")
    total, free = st[0] * st[2] / 1048576, st[0] * st[3] / 1048576
    record("CIRCUITPY filesystem", "PASS" if total > 2.5 else "WARN",
           "%.2f MB total, %.2f MB free" % (total, free))
    original = nvm[0:len(nvm)]
    test = bytes(os.urandom(len(nvm)))
    nvm[0:len(nvm)] = test
    ok = nvm[0:len(nvm)] == test
    nvm[0:len(nvm)] = original
    ok = ok and nvm[0:len(nvm)] == original
    record("Flash erase/write (NVM)", "PASS" if ok else "FAIL", "%d bytes, restored" % len(nvm))
    try:
        data = os.urandom(4096) * 16
        with open("/_selftest.bin", "wb") as f:
            f.write(data)
        with open("/_selftest.bin", "rb") as f:
            ok = f.read() == data
        os.remove("/_selftest.bin")
        record("Flash filesystem write", "PASS" if ok else "FAIL", "64 KB round trip")
    except OSError as e:
        if e.errno == 30:
            record("Flash filesystem write", "SKIP", "CIRCUITPY read-only while USB-mounted")
        else:
            raise


# =============================================================================
# 4. Power rails and current sensors
# =============================================================================
RAILS = (("3V3_MAIN", 0x40, 3.0, 3.6), ("3V3_PERIPH", 0x41, 3.0, 3.6), ("LORA", 0x44, None, None))


def test_power():
    periph_en.value = True
    time.sleep(0.05)
    record("PERIPH_PWR_EN", "PASS", "+3V3_P enabled")
    for name, addr, vmin, vmax in RAILS:
        try:
            ina = INA226(i2c(), addr, 0.010)
        except (OSError, ValueError) as e:
            record("INA226 %s (0x%02X)" % (name, addr), "FAIL", "not responding: %s" % e)
            continue
        ok_id = ina.mfr == 0x5449 and ina.die == 0x2260
        record("INA226 %s (0x%02X)" % (name, addr), "PASS" if ok_id else "FAIL",
               "IDs 0x%04X/0x%04X" % (ina.mfr, ina.die))
        if ok_id:
            state["ina"][name] = ina
    time.sleep(0.1)
    for name, addr, vmin, vmax in RAILS:
        ina = state["ina"].get(name)
        if not ina:
            continue
        v, i = ina.voltage, ina.average_ma()
        if vmin is None:  # LoRa rail: should be OFF until LORA_EN
            record("Rail %s (LORA_EN off)" % name, "PASS" if v < 0.5 else "WARN",
                   "%.3f V, %.1f mA (expect ~0 V)" % (v, i))
        else:
            record("Rail %s" % name, "PASS" if vmin <= v <= vmax else "FAIL",
                   "%.3f V, %.1f mA" % (v, i))


# =============================================================================
# 5. I2C0 scan
# =============================================================================
def test_i2c_scan():
    bus = i2c()
    while not bus.try_lock():
        pass
    try:
        found = bus.scan()
    finally:
        bus.unlock()
    expected = {0x40, 0x41, 0x44}
    missing = expected - set(found)
    extra = [a for a in found if a not in expected | {0x51}]
    detail = "found " + ", ".join("0x%02X" % a for a in found)
    if extra:
        detail += " (backplane/unknown: %s)" % ", ".join("0x%02X" % a for a in extra)
    record("I2C0 scan", "FAIL" if missing else "PASS",
           detail + ((" MISSING " + ", ".join("0x%02X" % a for a in missing)) if missing else ""))


# =============================================================================
# 6. NeoPixel
# =============================================================================
def test_neopixel():
    import neopixel
    if "3V3_PERIPH" not in state["ina"]:
        record("NeoPixel", "SKIP", "needs the 3V3_PERIPH current sensor")
        return
    pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.5, auto_write=True)
    try:
        pixel[0] = (0, 0, 0)
        base = periph_ma()
        deltas = []
        for name, rgb in (("R", (255, 0, 0)), ("G", (0, 255, 0)),
                          ("B", (0, 0, 255)), ("W", (255, 255, 255))):
            pixel[0] = rgb
            deltas.append((name, periph_ma() - base))
        pixel[0] = (0, 0, 0)
    finally:
        pixel.deinit()
    detail = ", ".join("%s +%.1f mA" % d for d in deltas)
    weak = [n for n, d in deltas if d < 2.0]
    record("NeoPixel R/G/B/W current", "FAIL" if weak else "PASS",
           detail + ((" (no draw on %s)" % "".join(weak)) if weak else ""))


# =============================================================================
# 7. RTC
# =============================================================================
def test_rtc():
    before = periph_ma()
    rtc_en.value = True
    rv, t0 = None, time.monotonic()
    while rv is None and time.monotonic() - t0 < 1.0:
        try:
            rv = RV3032(i2c())
            rv.read(0x0D, 1)
        except (OSError, ValueError):
            rv = None
            time.sleep(0.01)
    if rv is None:
        record("RTC (0x51)", "FAIL", "no response within 1 s of RTC_EN")
        return
    d, _ = delta_str(before)
    record("RTC (0x51)", "PASS", "responded after %.0f ms %s" % (
        (time.monotonic() - t0) * 1000, d))
    status = rv.read(0x0D, 1)[0]
    if status & 0x03:
        record("RTC time valid", "WARN", "not set since power loss (PORF/VLF)")
    else:
        record("RTC time valid", "PASS", rv.datetime_str())
    s1 = rv.seconds_of_day()
    time.sleep(1.2)
    step = (rv.seconds_of_day() - s1) % 86400
    record("RTC ticking", "PASS" if step in (1, 2) else "FAIL", "+%d s in 1.2 s" % step)
    t = rv.temperature()
    record("RTC temperature", "PASS" if -20 < t < 70 else "WARN", "%.2f C" % t)
    bsm = (rv.read(0xC0, 1)[0] >> 4) & 3
    record("RTC backup switchover", "PASS" if bsm in (1, 2) else "WARN",
           ("OFF", "direct", "level", "OFF")[bsm] + ("" if bsm in (1, 2) else
           " - coin cell unused, set via RTC script"))


# =============================================================================
# 8. Magnetometer
# =============================================================================
def test_mag():
    before = periph_ma()
    mag_en.value = True
    time.sleep(0.02)
    mag = MMC5983MA(spi(), board.MAG_nCS)
    d, _ = delta_str(before)
    if mag.product_id != 0x30:
        record("Magnetometer ID", "FAIL", "0x%02X, expected 0x30" % mag.product_id)
        return
    record("Magnetometer ID", "PASS", "0x30 %s" % d)
    x, y, z = mag.magnetic()
    b = (x * x + y * y + z * z) ** 0.5
    record("Magnetometer field", "PASS" if 15 < b < 100 else "WARN",
           "X %.1f Y %.1f Z %.1f, |B| %.1f uT (Earth ~25-65)" % (x, y, z, b))
    t = mag.temperature()
    record("Magnetometer temperature", "PASS" if -20 < t < 70 else "WARN", "%.1f C" % t)


# =============================================================================
# 9. IMU
# =============================================================================
def test_imu():
    before = periph_ma()
    imu_en.value = True
    time.sleep(0.05)
    imu = ASM330LHHG1(spi(), board.IMU_nCS)
    d, _ = delta_str(before)
    if imu.who_am_i != 0x6B:
        record("IMU WHO_AM_I", "FAIL", "0x%02X, expected 0x6B" % imu.who_am_i)
        return
    record("IMU WHO_AM_I", "PASS", "0x6B %s" % d)
    (ax, ay, az), (gx, gy, gz), t = imu.read()
    a = (ax * ax + ay * ay + az * az) ** 0.5
    record("IMU accelerometer", "PASS" if 8.3 < a < 11.3 else "WARN",
           "%.2f %.2f %.2f, |a| %.2f m/s^2 (still: ~9.81)" % (ax, ay, az, a))
    g = max(abs(gx), abs(gy), abs(gz))
    record("IMU gyroscope", "PASS" if g < 10 else "WARN",
           "%.2f %.2f %.2f dps (still: near 0)" % (gx, gy, gz))
    record("IMU temperature", "PASS" if -20 < t < 70 else "WARN", "%.1f C" % t)


# =============================================================================
# 10. SD card
# =============================================================================
def test_sd():
    import sdcardio
    import storage
    sd_en.value = True
    time.sleep(0.05)
    try:
        sd = sdcardio.SDCard(spi(), board.SD_CS, baudrate=SD_BAUD)
    except OSError as e:
        record("SD card", "FAIL", "%s (card inserted?)" % e)
        sd_en.value = False
        return
    try:
        record("SD card detected", "PASS", "%.2f GB" % (sd.count() * 512 / 1e9))
        storage.mount(storage.VfsFat(sd), "/sd")
        try:
            data = os.urandom(4096) * 64  # 256 KB
            t0 = time.monotonic()
            with open("/sd/_selftest.bin", "wb") as f:
                f.write(data)
            t1 = time.monotonic()
            with open("/sd/_selftest.bin", "rb") as f:
                ok = f.read() == data
            t2 = time.monotonic()
            os.remove("/sd/_selftest.bin")
            record("SD card read/write", "PASS" if ok else "FAIL",
                   "256 KB, write %.0f KB/s, read %.0f KB/s" % (256 / (t1 - t0), 256 / (t2 - t1)))
        finally:
            storage.umount("/sd")
    finally:
        sd.deinit()
        sd_en.value = False


# =============================================================================
# 11-12. LoRa radio
# =============================================================================
lora = {}


def lora_setup():
    busy = digitalio.DigitalInOut(board.LORA_BUSY)
    busy.direction = digitalio.Direction.INPUT
    dio1 = digitalio.DigitalInOut(board.LORA_INT)
    dio1.direction = digitalio.Direction.INPUT
    lora.update(busy=busy, dio1=dio1, rx=out(board.LORA_RX_EN), tx=out(board.LORA_TX_EN),
                nrst=out(board.LORA_nRST),
                dev=SPIDevice(spi(), digitalio.DigitalInOut(board.LORA_nCS),
                              baudrate=1_000_000, polarity=0, phase=0))


def lwait(timeout=0.1):
    end = time.monotonic() + timeout
    while lora["busy"].value:
        if time.monotonic() > end:
            raise RuntimeError("LoRa BUSY stuck high")


def lcmd(data, nread=0):
    lwait()
    o = bytearray(data) + bytearray(nread)
    r = bytearray(len(o))
    with lora["dev"] as s:
        s.write_readinto(o, r)
    return r


def lmode():
    return {2: "STBY_RC", 3: "STBY_XOSC", 4: "FS", 5: "RX", 6: "TX"}.get(
        (lcmd((0xC0,), 1)[1] >> 4) & 7, "?")


def lerrors():
    r = lcmd((0x17,), 3)
    return (r[2] << 8) | r[3]


def lrf_off():
    try:
        lcmd((0x80, 0x01))
    finally:
        lora["tx"].value = False
        lora["rx"].value = False


def test_radio_rx():
    lora_en.value = True
    time.sleep(0.05)
    ina = state["ina"].get("LORA")
    if ina:
        v = ina.voltage
        record("LoRa rail (LORA_EN on)", "PASS" if 4.8 <= v <= 5.5 else "FAIL",
               "%.3f V (needs >=5.0 V for full TX power)" % v)
    lora_setup()
    lora["nrst"].value = False
    time.sleep(0.002)
    lora["nrst"].value = True
    lwait(0.1)
    record("LoRa reset/BUSY", "PASS", "mode %s" % lmode())
    sync = bytes(lcmd((0x1D, 0x07, 0x40), 3)[4:])
    lcmd((0x0D, 0x07, 0x40, 0x5A, 0xA5))
    back = bytes(lcmd((0x1D, 0x07, 0x40), 3)[4:])
    lcmd((0x0D, 0x07, 0x40) + tuple(sync))
    record("LoRa SPI", "PASS" if sync == b"\x14\x24" and back == b"\x5a\xa5" else "FAIL",
           "sync word %s, write-back %s" % (sync.hex(), back.hex()))
    lcmd((0x97, 0x02, 0x00, 0x01, 0x40))  # DIO3 -> TCXO 1.8 V
    lcmd((0x07, 0x00, 0x00))
    lcmd((0x89, 0x7F))
    time.sleep(0.01)
    lwait(0.1)
    err = lerrors()
    lcmd((0x80, 0x01))
    mode = lmode()
    record("LoRa TCXO + calibration", "PASS" if err == 0 and mode == "STBY_XOSC" else "FAIL",
           "errors 0x%04X, mode %s" % (err, mode))
    lcmd((0x8A, 0x01))
    f = (LORA_FREQ_HZ // 125_000) * 131_072
    lcmd((0x86, (f >> 24) & 0xFF, (f >> 16) & 0xFF, (f >> 8) & 0xFF, f & 0xFF))
    lcmd((0x98, 0x6B, 0x6F))
    time.sleep(0.01)
    lwait(0.1)
    lora["rx"].value = True
    lcmd((0x82, 0xFF, 0xFF, 0xFF))
    time.sleep(0.05)
    readings = []
    for _ in range(20):
        readings.append(-lcmd((0x15,), 2)[2] / 2)
        time.sleep(0.05)
    avg = sum(readings) / len(readings)
    rx_ma = ina.average_ma() if ina else None
    lrf_off()
    record("LoRa receive (noise floor)", "PASS" if -135 < avg < -70 else "WARN",
           "%.1f dBm avg @ %.3f MHz%s" % (avg, LORA_FREQ_HZ / 1e6,
                                          ", %.1f mA in RX" % rx_ma if rx_ma else ""))
    lora["ok"] = True


def test_radio_tx():
    # if not RUN_TX_TEST:
    #     record("LoRa transmit", "SKIP", "RUN_TX_TEST = False")
    #     return
    # if CALLSIGN == "N0CALL":
    #     record("LoRa transmit", "SKIP", "set CALLSIGN first")
    #     return
    # if not lora.get("ok"):
    #     record("LoRa transmit", "SKIP", "receive test failed")
    #     return
    ina = state["ina"].get("LORA")
    drive = max(-9, min(TX_DRIVE_DBM, 22))
    lcmd((0x95, 0x04, 0x07, 0x00, 0x01))  # PA config
    lcmd((0x8E, drive & 0xFF, 0x04))
    idle = ina.average_ma() if ina else 0
    try:
        lora["tx"].value = True
        lcmd((0xD1,))  # CW carrier
        time.sleep(0.05)
        vmin, imax, end = 99.0, 0.0, time.monotonic() + 0.5
        while time.monotonic() < end and ina:
            vmin, imax = min(vmin, ina.voltage), max(imax, ina.current_ma)
    finally:
        lrf_off()
    err = lerrors()
    record("LoRa TX carrier", "PASS" if err == 0 and imax > idle + 20 else "FAIL",
           "drive %+d dBm, peak %.0f mA (idle %.0f), min %.3f V, errors 0x%04X" % (
               drive, imax, idle, vmin, err))
    payload = ("%s Argus5 self-test" % CALLSIGN).encode()
    lcmd((0x8B, 0x07, 0x04, 0x01, 0x00))
    lcmd((0x8F, 0x00, 0x00))
    lcmd((0x08, 0x02, 0x01, 0x02, 0x01, 0, 0, 0, 0))
    lcmd((0x8C, 0x00, 0x08, 0x00, len(payload), 0x01, 0x00))
    lcmd((0x0E, 0x00) + tuple(payload))
    lcmd((0x02, 0xFF, 0xFF))
    try:
        lora["tx"].value = True
        t0 = time.monotonic()
        lcmd((0x83, 0x00, 0xFA, 0x00))
        while not lora["dio1"].value and time.monotonic() - t0 < 2.0:
            pass
        dt = (time.monotonic() - t0) * 1000
        r = lcmd((0x12,), 3)
    finally:
        lrf_off()
    irq = (r[2] << 8) | r[3]
    lcmd((0x02, 0xFF, 0xFF))
    record("LoRa TX packet", "PASS" if irq & 1 else "FAIL",
           "%s, %.0f ms on air, '%s'" % ("TxDone" if irq & 1 else "IRQ 0x%04X" % irq,
                                         dt, payload.decode()))


# =============================================================================
# 13. GPS
# =============================================================================
def nmea_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    c = 0
    for ch in body:
        c ^= ord(ch)
    try:
        return c == int(cs[:2], 16)
    except ValueError:
        return False


def test_gps():
    before = periph_ma()
    gps_en.value = True
    t_on = time.monotonic()
    time.sleep(0.5)
    d, _ = delta_str(before)
    uart = busio.UART(board.TX1, board.RX1, baudrate=115200, timeout=0,
                      receiver_buffer_size=2048)
    try:
        import countio
        pps = countio.Counter(board.GPS_1PPS, edge=countio.Edge.RISE)
    except Exception:
        pps = None
    try:
        buf, good, bad = b"", 0, 0
        sats, fix_t, used, hdop = {}, None, 0, "-"
        pps_at_fix = 0
        last_note = time.monotonic()
        end = time.monotonic() + GPS_FIX_TIMEOUT
        while time.monotonic() < end:
            data = uart.read(512)
            if data:
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    try:
                        line = raw.decode().strip()
                    except UnicodeError:
                        bad += 1
                        continue
                    if not nmea_ok(line):
                        bad += 1
                        continue
                    good += 1
                    f = line[1:line.index("*")].split(",")
                    kind = f[0][2:]
                    if kind == "GSV":
                        for i in range(4, len(f) - 3, 4):
                            if f[i] and f[i + 3]:
                                sats[(f[0][:2], f[i])] = int(f[i + 3])
                    elif kind == "GGA" and len(f) > 8 and (f[6] or "0") != "0":
                        used, hdop = int(f[7] or 0), f[8]
                        if fix_t is None:
                            fix_t = time.monotonic()
                            pps_at_fix = pps.count if pps else 0
            if fix_t and time.monotonic() - fix_t > 4:
                break
            if good == 0 and time.monotonic() - t_on > 5:
                break
            if time.monotonic() - last_note > 10:
                last_note = time.monotonic()
                print("    ... %ds: %d NMEA, %d sats with signal, %s" % (
                    time.monotonic() - t_on, good, len(sats), "fix" if fix_t else "no fix yet"))
        record("GPS power/UART", "PASS" if good else "FAIL",
               "%d valid / %d bad NMEA @115200 %s" % (good, bad, d))
        if not good:
            return
        snrs = sorted(sats.values(), reverse=True)
        record("GPS satellites", "PASS" if snrs and snrs[0] >= 25 else "WARN",
               "%d with signal, best SNR %s dB-Hz" % (len(snrs), snrs[:4] or "-")
               + ("" if snrs else " (sky view?)"))
        if fix_t is None:
            record("GPS fix", "WARN", "none in %d s (needs sky view; cold start can take minutes)"
                   % GPS_FIX_TIMEOUT)
            record("GPS 1PPS", "SKIP", "only output with a fix")
            return
        record("GPS fix", "PASS", "TTFF %.0f s, %d sats used, HDOP %s" % (fix_t - t_on, used, hdop))
        if pps:
            n = pps.count - pps_at_fix
            record("GPS 1PPS", "PASS" if 2 <= n <= 6 else "FAIL",
                   "%d pulses in ~%.0f s after fix" % (n, time.monotonic() - fix_t))
        else:
            record("GPS 1PPS", "SKIP", "counter unavailable")
    finally:
        uart.deinit()
        if pps:
            pps.deinit()


# =============================================================================
# 14. Watchdog (destructive, runs after the summary)
# =============================================================================
def run_watchdog_test():
    print("\n=== 14. Watchdog (board will reset) " + "=" * 27)
    print("  Eject CIRCUITPY first. Result is printed on the next boot.")

    def kick():
        wdt_wdi.value = True
        wdt_wdi.value = False

    kick()
    time.sleep(0.1)
    kick()
    wdt_en.value = True
    print("  Kicking for 12 s (must NOT reset)...")
    for _ in range(12):
        kick()
        time.sleep(1)
    print("  Kicked phase OK. Stopping kicks; expect reset in ~3.2-4.8 s")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15:
        e = time.monotonic() - t0
        nvm[0:6] = WDT_MAGIC + struct.pack(">H", int(e * 10))
        print("  %.1f s" % e)
        time.sleep(0.5)
    wdt_en.value = False
    nvm[0:6] = b"\x00" * 6
    print("  [FAIL] Watchdog - no reset after 15 s without kicks")


# =============================================================================
# Run
# =============================================================================
TESTS = (
    ("System info", test_info),
    # ("PSRAM", test_psram),
    # ("SPI flash", test_flash),
    # ("Power rails + current sensors", test_power),
    # ("I2C0 scan", test_i2c_scan),
    # ("NeoPixel", test_neopixel),
    # ("RTC", test_rtc),
    # ("Magnetometer", test_mag),
    # ("IMU", test_imu),
    # ("SD card", test_sd),
    ("LoRa receive", test_radio_rx),
    ("LoRa transmit", test_radio_tx),
    ("GPS", test_gps),
)
for n, (title, fn) in enumerate(TESTS, 1):
    run(n, title, fn)

if not RUN_WATCHDOG_TEST:
    record("Watchdog", "SKIP", "RUN_WATCHDOG_TEST = False (resets the board)")
print("\nSuite time %.0f s" % (time.monotonic() - t_suite))
summary()

if RUN_WATCHDOG_TEST:
    run_watchdog_test()
