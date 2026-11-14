"""
Argus V4 radio TX test.

Powers the SX1268, configures LoRa over raw SPI (no driver), and sends
packets in the flight telemetry format. Copy to CIRCUITPY as main.py.

Connect an antenna or a 50-ohm load before running.
"""
import binascii
import struct
import time

import board
import busio
import digitalio

# ---------------- Radio settings: must match the receiver ----------------
FREQ_HZ = 434000000      # SX1268 range: 410-810 MHz
BW_KHZ = 125             # 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500
SF = 9                   # 5-12
CR_DEN = 7               # coding rate 4/5..4/8 -> 5..8
PREAMBLE = 8
SYNC_WORD = 0x12         # 0x12 private, 0x34 public
CRC_ON = True
TX_POWER_DBM = 14        # SX1268 output (-9..22); any external PA adds gain on top
# Oscillator settings tried in order until calibration passes:
# a TCXO voltage on DIO3, or None for a plain crystal.
# 1.6/1.7/1.8 V and crystal all failed with a 5 ms delay. This run tries 1.8 V
# with a 3 s delay: DIO3 stays on long enough to measure with a multimeter,
# and it also tests whether the TCXO is just slow to start.
# Check the schematic before adding 2.2-3.3 V.
OSC_CANDIDATES = (1.8,)
TCXO_DELAY_US = 3000000
USE_DCDC = True         # same as the flight driver; set False (LDO) if TX fails

N_PACKETS = 5
INTERVAL_S = 2

_PACKET_FMT = "<2sIBBfffBffffBffBfffffffff2s"


# ---------------- Pins and power ----------------
def _out(pin, value):
    p = digitalio.DigitalInOut(pin)
    p.switch_to_output(value=value)
    return p


def _in(pin, pull=None):
    p = digitalio.DigitalInOut(pin)
    p.switch_to_input(pull=pull)
    return p


lora_en = _out(board.LORA_EN, True)
periph_en = _out(board.PERIPH_PWR_EN, True)
sd_cs = _out(board.SD_CS, True)
lora_flt = _in(board.LORA_FLT, digitalio.Pull.UP)
time.sleep(0.05)

cs = _out(board.LORA_nCS, True)
rst = _out(board.LORA_nRST, True)
tx_en = _out(board.LORA_TX_EN, False)
rx_en = _out(board.LORA_RX_EN, False)
busy = _in(board.LORA_BUSY)
dio1 = _in(board.LORA_INT)
spi = busio.SPI(board.CLK0, MOSI=board.MOSI0, MISO=board.MISO0)


# ---------------- Low-level SPI ----------------
def wait_busy(timeout_ms=10000):
    t0 = time.monotonic_ns()
    while busy.value:
        if time.monotonic_ns() - t0 > timeout_ms * 1000000:
            raise RuntimeError("BUSY stuck high")


def xfer(data):
    wait_busy()
    out = bytes(data)
    buf = bytearray(len(out))
    while not spi.try_lock():
        pass
    try:
        spi.configure(baudrate=1000000, phase=0, polarity=0)
        cs.value = False
        spi.write_readinto(out, buf)
    finally:
        cs.value = True
        spi.unlock()
    return buf


_CMD_STATUS = {3: "timeout", 4: "invalid", 5: "failed to execute"}


def cmd(name, *data):
    """Send a command, then GetStatus and warn if the chip rejected it."""
    resp = xfer(data)
    status = xfer((0xC0, 0x00))[1]
    code = (status >> 1) & 0x07
    if code in _CMD_STATUS:
        print("  ! %s -> status 0x%02X (%s)" % (name, status, _CMD_STATUS[code]))
    return resp


def read_reg(addr, n):
    return xfer([0x1D, addr >> 8, addr & 0xFF, 0x00] + [0x00] * n)[4:]


def write_reg(addr, values):
    cmd("WriteRegister 0x%04X" % addr, 0x0D, addr >> 8, addr & 0xFF, *values)


_ERR_BITS = (
    (0x001, "RC64K_CALIB"), (0x002, "RC13M_CALIB"), (0x004, "PLL_CALIB"),
    (0x008, "ADC_CALIB"), (0x010, "IMG_CALIB"), (0x020, "XOSC_START"),
    (0x040, "PLL_LOCK"), (0x100, "PA_RAMP"),
)


def device_errors(label):
    r = xfer((0x17, 0x00, 0x00, 0x00))
    err = (r[2] << 8) | r[3]
    names = [name for bit, name in _ERR_BITS if err & bit]
    print("%s: device errors 0x%04X %s" % (label, err, names))
    return err


# ---------------- Radio setup ----------------
_BW_CODES = {7.8: 0x00, 10.4: 0x08, 15.6: 0x01, 20.8: 0x09, 31.25: 0x02,
             41.7: 0x0A, 62.5: 0x03, 125.0: 0x04, 250.0: 0x05, 500.0: 0x06}
_TCXO_CODES = {1.6: 0x00, 1.7: 0x01, 1.8: 0x02, 2.2: 0x03,
               2.4: 0x04, 2.7: 0x05, 3.0: 0x06, 3.3: 0x07}
_LDRO = 1 if (1 << SF) / BW_KHZ >= 16 else 0


def _image_cal(freq_hz):
    mhz = freq_hz // 1000000
    if mhz > 900:
        return (0xE1, 0xE9)
    if mhz > 850:
        return (0xD7, 0xDB)
    if mhz > 770:
        return (0xC1, 0xC5)
    if mhz > 460:
        return (0x75, 0x81)
    return (0x6B, 0x6F)


def reset_radio():
    rst.value = False
    time.sleep(0.002)
    rst.value = True
    time.sleep(0.01)
    wait_busy()
    xfer((0x80, 0x00))  # first command after reset is sometimes rejected
    cmd("SetStandby", 0x80, 0x00)


def start_oscillator(tcxo_voltage):
    """Reset, enable the TCXO (or not), run a full calibration. Returns device error bits."""
    reset_radio()
    if tcxo_voltage:
        delay = TCXO_DELAY_US * 64 // 1000  # 15.625 us steps
        cmd("SetDIO3AsTCXOCtrl", 0x97, _TCXO_CODES[tcxo_voltage],
            (delay >> 16) & 0xFF, (delay >> 8) & 0xFF, delay & 0xFF)
    cmd("ClearDeviceErrors", 0x07, 0x00, 0x00)
    if tcxo_voltage:
        print("  Calibrating: DIO3 should read %.1f V for ~%.1f s. Measure it now." % (
            tcxo_voltage, TCXO_DELAY_US / 1000000))
    t0 = time.monotonic_ns()
    cmd("Calibrate", 0x89, 0x7F)
    print("  Calibrate took %d ms" % ((time.monotonic_ns() - t0) // 1000000))
    time.sleep(0.01)
    label = ("TCXO %.1f V" % tcxo_voltage) if tcxo_voltage else "Crystal (no TCXO)"
    return device_errors("  " + label)


def radio_setup():
    reset_radio()
    chip = bytes(read_reg(0x0320, 16)).split(b"\x00")[0]
    print("Chip:", chip)
    if not chip.startswith(b"SX126"):
        raise RuntimeError("radio not answering")
    device_errors("After reset")

    print("Trying oscillator settings:")
    for v in OSC_CANDIDATES:
        if start_oscillator(v) == 0:
            break
    else:
        raise RuntimeError("Oscillator never started with any setting tried. "
                           "Check the TCXO part number and how it is powered in the schematic.")
    print("Oscillator OK with", ("TCXO %.1f V" % v) if v else "crystal")

    cmd("SetRegulatorMode", 0x96, 0x01 if USE_DCDC else 0x00)
    cmd("SetDIO2AsRfSwitchCtrl", 0x9D, 0x01)
    cmd("SetPacketType", 0x8A, 0x01)  # LoRa
    cmd("CalibrateImage", 0x98, *_image_cal(FREQ_HZ))
    frf = FREQ_HZ * (1 << 25) // 32000000
    cmd("SetRfFrequency", 0x86,
        (frf >> 24) & 0xFF, (frf >> 16) & 0xFF, (frf >> 8) & 0xFF, frf & 0xFF)
    cmd("SetPaConfig", 0x95, 0x04, 0x07, 0x00, 0x01)  # high-power PA, same as driver
    cmd("SetTxParams", 0x8E, TX_POWER_DBM & 0xFF, 0x04)  # 200 us ramp
    v = read_reg(0x08D8, 1)[0]
    write_reg(0x08D8, [v | 0x1E])  # datasheet 15.2: PA clamping fix
    cmd("SetBufferBaseAddress", 0x8F, 0x00, 0x00)
    cmd("SetModulationParams", 0x8B, SF, _BW_CODES[float(BW_KHZ)], CR_DEN - 4, _LDRO)
    write_reg(0x0740, [(SYNC_WORD & 0xF0) | 0x04, ((SYNC_WORD & 0x0F) << 4) | 0x04])
    v = read_reg(0x0736, 1)[0]
    write_reg(0x0736, [v | 0x04])  # datasheet 15.4: bit 2 = 1 for standard IQ
    cmd("SetDioIrqParams", 0x08, 0x02, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00)
    device_errors("After setup")


def time_on_air_ms(n):
    tsym = (1 << SF) / BW_KHZ
    num = 8 * n - 4 * SF + 28 + (16 if CRC_ON else 0)
    den = 4 * (SF - 2 * _LDRO)
    nsym = 8 + max(-(-num // den) * CR_DEN, 0)
    return (PREAMBLE + 4.25 + nsym) * tsym


# ---------------- Transmit ----------------
def lora_send(payload):
    n = len(payload)
    timeout_ms = int(time_on_air_ms(n) * 2) + 200

    cmd("SetStandby", 0x80, 0x00)
    cmd("SetPacketParams", 0x8C, PREAMBLE >> 8, PREAMBLE & 0xFF,
        0x00, n, 0x01 if CRC_ON else 0x00, 0x00)  # explicit header, standard IQ
    v = read_reg(0x0889, 1)[0]
    write_reg(0x0889, [(v & 0xFB) if BW_KHZ == 500 else (v | 0x04)])  # datasheet 15.1
    cmd("WriteBuffer", 0x0E, 0x00, *payload)
    cmd("ClearIrqStatus", 0x02, 0x03, 0xFF)

    rx_en.value = False
    tx_en.value = True
    t0 = time.monotonic_ns()
    cmd("SetTx", 0x83, 0x00, 0x00, 0x00)

    done = False
    while (time.monotonic_ns() - t0) // 1000000 < timeout_ms:
        r = xfer((0x12, 0x00, 0x00, 0x00))  # GetIrqStatus
        if r[3] & 0x01:  # TxDone
            done = True
            break
        time.sleep(0.002)
    elapsed_ms = (time.monotonic_ns() - t0) // 1000000
    dio1_high = dio1.value

    tx_en.value = False
    cmd("ClearIrqStatus", 0x02, 0x03, 0xFF)
    if not done:
        cmd("SetStandby", 0x80, 0x00)
    return done, elapsed_ms, dio1_high


# ---------------- Packet (same layout as send_message) ----------------
def build_payload(location, temperature, pressure, humidity, imu_accel, imu_gyro, imu_mag):
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

    return struct.pack(
        _PACKET_FMT,
        b"ST",
        int(time.time()),  # stands in for TPM.time()
        gps_valid, gps_fix, lat, lon, alt,
        temp_valid, t0, t1, t2, t3,
        baro_valid, pressure, humidity,
        imu_valid, ax, ay, az, gx, gy, gz, mx, my, mz,
        b"RD",
    )


# ---------------- Main ----------------
print("LORA_FLT:", lora_flt.value)
radio_setup()

size = struct.calcsize(_PACKET_FMT)
print("Config: %d Hz, SF%d, BW%s kHz, CR4/%d, sync 0x%02X, %d dBm" % (
    FREQ_HZ, SF, BW_KHZ, CR_DEN, SYNC_WORD, TX_POWER_DBM))
print("Payload %d bytes, ~%d ms on air" % (size, time_on_air_ms(size)))

for i in range(N_PACKETS):
    payload = build_payload(
        {"gps_valid": 1, "fix_mode": 3, "lat": 42.3601, "lon": -71.0942, "alt": 10.0 + i},
        [20.0 + i, 21.0, 22.0],
        1013.25, 45.0,
        (0.0, 0.0, 9.81), (0.01, 0.02, 0.03), (20.0, -5.0, 40.0),
    )
    done, ms, dio1_high = lora_send(payload)
    print("Packet %d: %s in %d ms | DIO1 high at end: %s" % (
        i, "sent" if done else "TX TIMEOUT", ms, dio1_high))
    if i == 0:
        print("  bytes:", binascii.hexlify(payload).decode())
    if not done:
        device_errors("  After TX timeout")
    time.sleep(INTERVAL_S)

cmd("SetStandby", 0x80, 0x00)
print("Done")