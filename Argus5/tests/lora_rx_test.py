"""Argus5 LoRa radio test (E22P-433M30S / SX1268) - RECEIVE ONLY.

Nothing in this script transmits. It checks, in order:
  1. Power: LoRa supply comes up, current/voltage via INA226 U12
  2. Control: reset + BUSY handshake
  3. SPI: status byte, register read/write-back
  4. TCXO: DIO3 powers the 32 MHz TCXO; calibration must report no errors
  5. RX path: continuous receive at FREQ_HZ, printing the noise floor (RSSI)
"""
import time
import board
import digitalio
from adafruit_bus_device.spi_device import SPIDevice
from adafruit_bus_device.i2c_device import I2CDevice

FREQ_HZ = 433_000_000  # must be a multiple of 125 kHz
TCXO_VOLTAGE = 0x02    # 1.8 V (code 0x00=1.6 V ... 0x07=3.3 V)

# --- Power --------------------------------------------------------------------
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)   # powers INA226 U12 for current readings
time.sleep(0.01)
lora_en = digitalio.DigitalInOut(board.LORA_EN)
lora_en.switch_to_output(value=True)     # buck U10 -> LDO U11 -> module
time.sleep(0.05)
print("LoRa power enabled (green LED D11 should be on)")

# RF switch: never have both high. Both low = isolated.
rx_en = digitalio.DigitalInOut(board.LORA_RX_EN)
rx_en.switch_to_output(value=False)
tx_en = digitalio.DigitalInOut(board.LORA_TX_EN)
tx_en.switch_to_output(value=False)

busy = digitalio.DigitalInOut(board.LORA_BUSY)
busy.direction = digitalio.Direction.INPUT
nrst = digitalio.DigitalInOut(board.LORA_nRST)
nrst.switch_to_output(value=True)


def wait_busy(timeout=0.1):
    deadline = time.monotonic() + timeout
    while busy.value:
        if time.monotonic() > deadline:
            raise RuntimeError("BUSY stuck high")


# --- INA226 U12 (LoRa rail) -----------------------------------------------------
try:
    ina = I2CDevice(board.I2C(), 0x44)
except (OSError, ValueError, RuntimeError):
    ina = None
    print("INA226 0x44 not found - no current readings")


def lora_power():
    def reg(r):
        b = bytearray(2)
        with ina as d:
            d.write_then_readinto(bytes((r,)), b)
        return (b[0] << 8) | b[1]
    shunt = reg(0x01)
    if shunt & 0x8000:
        shunt -= 0x10000
    return reg(0x02) * 0.00125, shunt * 2.5e-6 / 0.010 * 1000  # V, mA


# --- SX126x -------------------------------------------------------------------
MODES = {2: "STBY_RC", 3: "STBY_XOSC", 4: "FS", 5: "RX", 6: "TX"}
ERRORS = ("RC64K_CALIB", "RC13M_CALIB", "PLL_CALIB", "ADC_CALIB", "IMG_CALIB",
          "XOSC_START", "PLL_LOCK", "", "PA_RAMP")

spi_dev = SPIDevice(board.SPI(), digitalio.DigitalInOut(board.LORA_nCS),
                    baudrate=1_000_000, polarity=0, phase=0)


def cmd(data, nread=0):
    wait_busy()
    out = bytearray(data) + bytearray(nread)
    res = bytearray(len(out))
    with spi_dev as spi:
        spi.write_readinto(out, res)
    return res


def status():
    s = cmd((0xC0,), 1)[1]
    return MODES.get((s >> 4) & 7, "?%d" % ((s >> 4) & 7))


def read_reg(addr, n):
    return cmd((0x1D, addr >> 8, addr & 0xFF), 1 + n)[4:]


def write_reg(addr, data):
    cmd((0x0D, addr >> 8, addr & 0xFF) + tuple(data))


def device_errors():
    r = cmd((0x17,), 3)
    bits = (r[2] << 8) | r[3]
    return bits, [ERRORS[i] for i in range(len(ERRORS)) if bits & (1 << i) and ERRORS[i]]


ok = True

# 2. Reset and BUSY handshake
print("\n[2] Reset")
nrst.value = False
time.sleep(0.002)
nrst.value = True
try:
    wait_busy(0.1)
    print("  BUSY released after reset: OK")
except RuntimeError as e:
    print("  FAIL:", e, "- check supply (D11), NRST and BUSY wiring")
    raise SystemExit

# 3. SPI
print("\n[3] SPI")
print("  Mode after reset:", status(), "(expect STBY_RC)")
sync = bytes(read_reg(0x0740, 2))
print("  LoRa sync word register: %s (expect 1424)" % sync.hex())
ok &= sync == b"\x14\x24"
write_reg(0x0740, (0x5A, 0xA5))
back = bytes(read_reg(0x0740, 2))
write_reg(0x0740, sync)
print("  Write/read-back: %s" % ("OK" if back == b"\x5a\xa5" else "FAIL " + back.hex()))
ok &= back == b"\x5a\xa5"

# 4. TCXO and calibration
print("\n[4] TCXO + calibration")
cmd((0x97, TCXO_VOLTAGE, 0x00, 0x01, 0x40))  # DIO3 -> TCXO, 5 ms startup
cmd((0x07, 0x00, 0x00))                       # clear errors from power-up
cmd((0x89, 0x7F))                             # calibrate all blocks
time.sleep(0.01)
wait_busy(0.1)
bits, names = device_errors()
print("  Device errors: 0x%04X %s" % (bits, names or "none"))
ok &= bits == 0
cmd((0x80, 0x01))  # standby on the TCXO
mode = status()
print("  Mode:", mode, "(expect STBY_XOSC = TCXO running)")
ok &= mode == "STBY_XOSC"

# 5. Receive and measure noise floor
print("\n[5] RX at %.3f MHz" % (FREQ_HZ / 1e6))
cmd((0x8A, 0x01))                               # LoRa packet type
f = (FREQ_HZ // 125_000) * 131_072              # FREQ * 2^25 / 32 MHz
cmd((0x86, (f >> 24) & 0xFF, (f >> 16) & 0xFF, (f >> 8) & 0xFF, f & 0xFF))
cmd((0x98, 0x6B, 0x6F))                         # image calibration 430-440 MHz
time.sleep(0.01)
wait_busy(0.1)
tx_en.value = False
rx_en.value = True                              # RF switch -> receive (LNA)
cmd((0x82, 0xFF, 0xFF, 0xFF))                   # continuous RX
time.sleep(0.01)
mode = status()
print("  Mode:", mode, "(expect RX)")
ok &= mode == "RX"
bits, names = device_errors()
if bits:
    print("  Device errors after RX start: 0x%04X %s" % (bits, names))
    ok = False

print("\nSetup %s. Printing noise floor; key a 433 MHz transmitter nearby" % (
    "PASSED" if ok else "had FAILURES"))
print("to see RSSI jump. Ctrl-C to stop.\n")

while True:
    rssi = -cmd((0x15,), 2)[2] / 2
    line = "RSSI %7.1f dBm" % rssi
    if ina:
        v, ma = lora_power()
        line += "   LoRa rail %.3f V  %6.1f mA" % (v, ma)
    print(line)
    time.sleep(0.5)
