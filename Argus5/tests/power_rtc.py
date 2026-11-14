import time
import board
import digitalio
import rtc
from adafruit_bus_device.i2c_device import I2CDevice

# --- Settings ---------------------------------------------------------------
# To set the RTC: fill in (year, month, day, hour, minute, second), save,
# then set it back to None and save again so it isn't reset on every run.
SET_TIME = None  # e.g. (2026, 10, 10, 14, 30, 0)

# One-time EEPROM change so the coin cell (BT2) keeps the clock running when
# the RTC rail is off. Uses level-switching mode with trickle charging OFF,
# which is right for a non-rechargeable lithium coin cell.
CONFIGURE_BACKUP_SWITCHOVER = False

# --- Power rails ------------------------------------------------------------
# The INA226 sensors and the RTC's load switch are fed from +3V3_P,
# so this must come first.
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)
time.sleep(0.01)

lora_en = digitalio.DigitalInOut(board.LORA_EN)
lora_en.switch_to_output(value=True)

rtc_en = digitalio.DigitalInOut(board.RTC_EN)
rtc_en.switch_to_output(value=True)
time.sleep(0.05)  # load switch rise + RV-3032 power-up

print("Peripheral, LoRa and RTC power enabled")


# --- INA226 -----------------------------------------------------------------
class INA226:
    _CONFIG = 0x00
    _SHUNT_V = 0x01  # signed, 2.5 uV/LSB
    _BUS_V = 0x02    # 1.25 mV/LSB
    _MFR_ID = 0xFE   # reads 0x5449 ("TI")

    def __init__(self, i2c, address, shunt_ohms):
        self._dev = I2CDevice(i2c, address)
        self._shunt = shunt_ohms
        self._buf = bytearray(3)
        if self._read(self._MFR_ID) != 0x5449:
            raise RuntimeError("not an INA226 at 0x%02X" % address)
        # Continuous shunt + bus, 1.1 ms conversions, 16-sample averaging
        self._write(self._CONFIG, 0x4527)

    def _read(self, reg):
        self._buf[0] = reg
        with self._dev as d:
            d.write_then_readinto(self._buf, self._buf, out_end=1, in_end=2)
        return (self._buf[0] << 8) | self._buf[1]

    def _write(self, reg, value):
        self._buf[0] = reg
        self._buf[1] = value >> 8
        self._buf[2] = value & 0xFF
        with self._dev as d:
            d.write(self._buf)

    @property
    def voltage(self):
        """Bus (load-side) voltage in volts."""
        return self._read(self._BUS_V) * 0.00125

    @property
    def current(self):
        """Current through the shunt in amps."""
        raw = self._read(self._SHUNT_V)
        if raw & 0x8000:
            raw -= 0x10000
        return raw * 2.5e-6 / self._shunt


# --- RV-3032-C7 -------------------------------------------------------------
def _bcd2bin(b):
    return (b >> 4) * 10 + (b & 0x0F)


def _bin2bcd(n):
    return ((n // 10) << 4) | (n % 10)


class RV3032:
    ADDRESS = 0x51
    _SEC = 0x01       # 0x01-0x07: sec, min, hour, weekday, day, month, year
    _STATUS = 0x0D
    _TLSB = 0x0E
    _CTRL1 = 0x10
    _EE_CMD = 0x3F
    _PMU = 0xC0

    _PORF = 0x02      # power-on reset: time is invalid
    _VLF = 0x01       # voltage low: time may be invalid
    _EERD = 0x04      # CTRL1: disable automatic EEPROM refresh
    _EEBUSY = 0x04    # TLSB: EEPROM busy

    def __init__(self, i2c):
        self._dev = I2CDevice(i2c, self.ADDRESS)
        self._buf = bytearray(8)

    def _read(self, reg, n):
        buf = memoryview(self._buf)[:n]
        with self._dev as d:
            d.write_then_readinto(bytes((reg,)), buf)
        return buf

    def _write(self, reg, data):
        with self._dev as d:
            d.write(bytes((reg,)) + bytes(data))

    @property
    def status(self):
        return self._read(self._STATUS, 1)[0]

    @property
    def time_valid(self):
        return not self.status & (self._PORF | self._VLF)

    @property
    def datetime(self):
        b = self._read(self._SEC, 7)
        return time.struct_time((
            2000 + _bcd2bin(b[6]),
            _bcd2bin(b[5] & 0x1F),
            _bcd2bin(b[4] & 0x3F),
            _bcd2bin(b[2] & 0x3F),
            _bcd2bin(b[1] & 0x7F),
            _bcd2bin(b[0] & 0x7F),
            b[3] & 0x07, -1, -1))

    @datetime.setter
    def datetime(self, t):
        self._write(self._SEC, (
            _bin2bcd(t.tm_sec), _bin2bcd(t.tm_min), _bin2bcd(t.tm_hour),
            t.tm_wday, _bin2bcd(t.tm_mday), _bin2bcd(t.tm_mon),
            _bin2bcd(t.tm_year - 2000)))
        # Clear PORF and VLF so the time is marked valid
        self._write(self._STATUS, (self.status & ~(self._PORF | self._VLF) & 0xFF,))

    @property
    def temperature(self):
        """Die temperature in C, 1/16 C resolution."""
        def raw():
            b = self._read(self._TLSB, 2)
            msb = b[1] - 256 if b[1] & 0x80 else b[1]
            return (msb << 4) | (b[0] >> 4)
        prev, cur = None, raw()
        while cur != prev:  # TLSB/TMSB aren't latched; read until stable
            prev, cur = cur, raw()
        return cur / 16

    @property
    def backup_mode(self):
        """(BSM, TCM) from the PMU register. BSM 0=off, 1=direct, 2=level."""
        pmu = self._read(self._PMU, 1)[0]
        return (pmu >> 4) & 0x03, pmu & 0x03

    def _wait_eeprom(self):
        deadline = time.monotonic() + 0.2
        while self._read(self._TLSB, 1)[0] & self._EEBUSY:
            if time.monotonic() > deadline:
                raise RuntimeError("EEPROM busy timeout")

    def enable_backup_switchover(self):
        """Level-switching backup, trickle charger off. Written to EEPROM."""
        ctrl1 = self._read(self._CTRL1, 1)[0]
        self._write(self._CTRL1, (ctrl1 | self._EERD,))
        try:
            self._wait_eeprom()
            pmu = self._read(self._PMU, 1)[0]
            pmu = (pmu & ~0x33 & 0xFF) | (2 << 4)  # BSM=LSM, TCM=off
            self._write(self._PMU, (pmu,))
            self._write(self._EE_CMD, (0x11,))      # update EEPROM from RAM
            time.sleep(0.05)
            self._wait_eeprom()
        finally:
            self._write(self._CTRL1, (ctrl1 & ~self._EERD & 0xFF,))


# --- Set up sensors ---------------------------------------------------------
# name, I2C address, shunt resistance (ohms), schematic ref
RAILS = (
    ("3V3_MAIN",   0x40, 0.010),  # U7,  R29
    ("3V3_PERIPH", 0x41, 0.010),  # U9,  R51
    ("LORA",       0x44, 0.010),  # U12, R86
)

i2c = board.I2C()
sensors = []
for name, addr, shunt in RAILS:
    try:
        sensors.append((name, INA226(i2c, addr, shunt)))
    except (OSError, ValueError, RuntimeError) as e:
        print("%s (0x%02X) not found: %s" % (name, addr, e))

# The RV-3032 copies its configuration from EEPROM for ~66 ms (tPREFR)
# after power-up, so retry for up to 1 s instead of probing once.
rv = None
deadline = time.monotonic() + 1.0
while rv is None:
    try:
        rv = RV3032(i2c)
        rv.status  # probe
    except (OSError, ValueError) as e:
        rv = None
        if time.monotonic() > deadline:
            print("RTC not found at 0x%02X: %s" % (RV3032.ADDRESS, e))
            break
        time.sleep(0.02)
if rv is None:
    while not i2c.try_lock():
        pass
    print("I2C devices present:", [hex(a) for a in i2c.scan()])
    i2c.unlock()

if rv:
    if CONFIGURE_BACKUP_SWITCHOVER and rv.backup_mode != (2, 0):
        rv.enable_backup_switchover()
        print("RTC: backup switchover enabled (written to EEPROM)")

    bsm, tcm = rv.backup_mode
    print("RTC backup switchover:", ("OFF", "direct", "level", "level")[bsm],
          "| trickle charger:", "off" if tcm == 0 else "ON (mode %d)" % tcm)
    if bsm == 0:
        print("  WARNING: coin cell is not used; the clock stops whenever the")
        print("  RTC rail is off. Set CONFIGURE_BACKUP_SWITCHOVER = True once.")
    if tcm != 0:
        print("  WARNING: trickle charging a non-rechargeable coin cell is unsafe.")

    if SET_TIME:
        t = time.struct_time(SET_TIME + (0, -1, -1))
        t = time.localtime(time.mktime(t))  # fills in the weekday
        rv.datetime = t
        print("RTC set to %04d-%02d-%02d %02d:%02d:%02d" % SET_TIME)

    if rv.time_valid:
        rtc.RTC().datetime = rv.datetime  # sync CircuitPython's clock
    else:
        print("RTC time is not valid yet (power was lost). Set it with SET_TIME.")

time.sleep(0.05)  # let the first averaged INA226 conversion finish

WEEKDAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")

# --- Main loop --------------------------------------------------------------
while True:
    for name, ina in sensors:
        v = ina.voltage
        i = ina.current
        print("%-11s %6.3f V  %8.2f mA  %8.2f mW" % (name, v, i * 1000, v * i * 1000))
    if rv:
        try:
            t = rv.datetime
            print("RTC  %s %04d-%02d-%02d %02d:%02d:%02d  %s  (%.2f C)" % (
                WEEKDAYS[t.tm_wday % 7], t.tm_year, t.tm_mon, t.tm_mday,
                t.tm_hour, t.tm_min, t.tm_sec,
                "valid" if rv.time_valid else "NOT SET", rv.temperature))
        except OSError as e:
            print("RTC  read failed:", e)
    print()
    time.sleep(1)
