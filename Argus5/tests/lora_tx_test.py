"""Argus5 LoRa TRANSMIT test (E22P-433M30S / SX1268).

Requires: antenna (VSWR < 1.5) or 50-ohm load on J11, licensed operator,
supply able to deliver ~650 mA at 5 V on the LoRa rail.

Part A: short CW carriers at increasing SX1268 drive levels, measuring the
        LoRa rail current and voltage (INA226 U12) during each.
Part B: a few LoRa packets carrying CALLSIGN, timed via the DIO1 TxDone IRQ.

The SX1268 drive level is the input to the module's external PA, not the
antenna output. Drivers for E22 M30S modules use +22 dBm drive for ~30 dBm
out; Ebyte does not publish a drive limit for the E22P, so start below that.
"""
import time
import board
import digitalio
from adafruit_bus_device.spi_device import SPIDevice
from adafruit_bus_device.i2c_device import I2CDevice

# --- Settings -----------------------------------------------------------------
CALLSIGN = "N0CALL"          # your callsign; sent in every packet
FREQ_HZ = 433_000_000        # multiple of 125 kHz
DRIVE_STEPS = (-9, -3, 3, 9, 14)   # SX1268 drive levels for the CW ramp, dBm
MAX_DRIVE = 14               # hard cap; SX1268 accepts up to +22
CW_SECONDS = 1.0             # length of each carrier
PACKET_DRIVE = 14            # drive level for Part B
PACKETS = 5                  # number of packets in Part B
PACKET_INTERVAL = 2.0        # seconds between packets
TCXO_VOLTAGE = 0x02          # 1.8 V, per Ebyte E22 M30S spec

if CALLSIGN == "N0CALL":
    raise SystemExit("Set CALLSIGN before transmitting.")
MAX_DRIVE = min(MAX_DRIVE, 22)

# --- Power and pins -------------------------------------------------------------
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)
time.sleep(0.01)
lora_en = digitalio.DigitalInOut(board.LORA_EN)
lora_en.switch_to_output(value=True)
time.sleep(0.05)

rx_en = digitalio.DigitalInOut(board.LORA_RX_EN)
rx_en.switch_to_output(value=False)
tx_en = digitalio.DigitalInOut(board.LORA_TX_EN)
tx_en.switch_to_output(value=False)
busy = digitalio.DigitalInOut(board.LORA_BUSY)
busy.direction = digitalio.Direction.INPUT
dio1 = digitalio.DigitalInOut(board.LORA_INT)
dio1.direction = digitalio.Direction.INPUT
nrst = digitalio.DigitalInOut(board.LORA_nRST)
nrst.switch_to_output(value=True)

ina = I2CDevice(board.I2C(), 0x44)


def ina_reg(r):
    b = bytearray(2)
    with ina as d:
        d.write_then_readinto(bytes((r,)), b)
    return (b[0] << 8) | b[1]


def rail():
    """(volts, mA) on the LoRa rail."""
    s = ina_reg(0x01)
    if s & 0x8000:
        s -= 0x10000
    return ina_reg(0x02) * 0.00125, s * 2.5e-6 / 0.010 * 1000


# --- SX126x -------------------------------------------------------------------
spi_dev = SPIDevice(board.SPI(), digitalio.DigitalInOut(board.LORA_nCS),
                    baudrate=1_000_000, polarity=0, phase=0)
ERRORS = ("RC64K_CALIB", "RC13M_CALIB", "PLL_CALIB", "ADC_CALIB", "IMG_CALIB",
          "XOSC_START", "PLL_LOCK", "", "PA_RAMP")


def wait_busy(timeout=0.1):
    deadline = time.monotonic() + timeout
    while busy.value:
        if time.monotonic() > deadline:
            raise RuntimeError("BUSY stuck high")


def cmd(data, nread=0):
    wait_busy()
    out = bytearray(data) + bytearray(nread)
    res = bytearray(len(out))
    with spi_dev as spi:
        spi.write_readinto(out, res)
    return res


def errors():
    r = cmd((0x17,), 3)
    bits = (r[2] << 8) | r[3]
    return [ERRORS[i] for i in range(len(ERRORS)) if bits & (1 << i) and ERRORS[i]]


def rf_off():
    """Standby on TCXO and RF switch isolated. Safe to call any time."""
    try:
        cmd((0x80, 0x01))
    finally:
        tx_en.value = False
        rx_en.value = False


def set_drive(dbm):
    dbm = max(-9, min(dbm, MAX_DRIVE))
    cmd((0x8E, dbm & 0xFF, 0x04))  # SetTxParams, 200 us ramp
    return dbm


# --- Init ---------------------------------------------------------------------
nrst.value = False
time.sleep(0.002)
nrst.value = True
wait_busy(0.1)
cmd((0x97, TCXO_VOLTAGE, 0x00, 0x01, 0x40))       # DIO3 -> TCXO, 5 ms
cmd((0x07, 0x00, 0x00))                           # clear power-up errors
cmd((0x89, 0x7F))                                 # calibrate all
time.sleep(0.01)
wait_busy(0.1)
cmd((0x80, 0x01))                                 # STBY_XOSC
cmd((0x8A, 0x01))                                 # LoRa
f = (FREQ_HZ // 125_000) * 131_072
cmd((0x86, (f >> 24) & 0xFF, (f >> 16) & 0xFF, (f >> 8) & 0xFF, f & 0xFF))
cmd((0x98, 0x6B, 0x6F))                           # image cal 430-440 MHz
time.sleep(0.01)
wait_busy(0.1)
cmd((0x95, 0x04, 0x07, 0x00, 0x01))               # PA config for up to +22 dBm
errs = errors()
if errs:
    raise SystemExit("Init errors %s - run lora_test.py first" % errs)

v0, i0 = rail()
print("Init OK. Idle LoRa rail: %.3f V, %.1f mA" % (v0, i0))
print("Transmitting on %.3f MHz as %s\n" % (FREQ_HZ / 1e6, CALLSIGN))

try:
    # --- Part A: CW ramp ---------------------------------------------------------
    print("Part A: CW carrier, %.1f s per step" % CW_SECONDS)
    print("  drive    rail V (min)   current mA (avg / max)")
    for step in DRIVE_STEPS:
        if step > MAX_DRIVE:
            break
        d = set_drive(step)
        rx_en.value = False
        tx_en.value = True
        cmd((0xD1,))                              # SetTxContinuousWave
        time.sleep(0.05)                          # past the PA ramp
        vmin, imax, isum, n = 99.0, 0.0, 0.0, 0
        end = time.monotonic() + CW_SECONDS
        while time.monotonic() < end:
            v, i = rail()
            vmin, imax, isum, n = min(vmin, v), max(imax, i), isum + i, n + 1
        rf_off()
        errs = errors()
        print("  %+3d dBm   %6.3f        %6.1f / %6.1f %s" % (
            d, vmin, isum / n, imax, ("  ERRORS " + str(errs)) if errs else ""))
        if errs or vmin < 4.8:
            print("  Stopping ramp: %s" % ("device errors" if errs else "rail sagged below 4.8 V"))
            break
        time.sleep(1.0)                           # let the PA cool between steps

    # --- Part B: LoRa packets ----------------------------------------------------
    print("\nPart B: %d LoRa packets, SF7 / BW125 / CR4-5, sync word 0x1424" % PACKETS)
    d = set_drive(PACKET_DRIVE)
    cmd((0x8B, 0x07, 0x04, 0x01, 0x00))           # SF7, BW125, CR4/5, LDRO off
    cmd((0x8F, 0x00, 0x00))                       # buffer base addresses
    cmd((0x08, 0x02, 0x01, 0x02, 0x01, 0, 0, 0, 0))  # TxDone|Timeout -> DIO1
    for n in range(1, PACKETS + 1):
        payload = ("%s Argus5 test %d" % (CALLSIGN, n)).encode()
        cmd((0x8C, 0x00, 0x08, 0x00, len(payload), 0x01, 0x00))  # pkt params
        cmd((0x0E, 0x00) + tuple(payload))        # write buffer
        cmd((0x02, 0xFF, 0xFF))                   # clear IRQs
        rx_en.value = False
        tx_en.value = True
        t0 = time.monotonic()
        cmd((0x83, 0x00, 0xFA, 0x00))             # SetTx, 1 s timeout
        vmin, imax = 99.0, 0.0
        while not dio1.value and time.monotonic() - t0 < 2.0:
            v, i = rail()
            vmin, imax = min(vmin, v), max(imax, i)
        dt = (time.monotonic() - t0) * 1000
        r = cmd((0x12,), 3)
        irq = (r[2] << 8) | r[3]
        rf_off()
        cmd((0x02, 0xFF, 0xFF))
        result = "TxDone" if irq & 0x0001 else ("TIMEOUT" if irq & 0x0200 else "no IRQ")
        print("  #%d %-7s %5.0f ms on air   drive %+d dBm   min %.3f V   peak %.1f mA   '%s'" % (
            n, result, dt, d, vmin, imax, payload.decode()))
        time.sleep(PACKET_INTERVAL)
finally:
    rf_off()
    print("\nRF off (standby, switch isolated). Done.")

while True:
    time.sleep(1)
