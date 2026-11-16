# code.py - Arducam Mega 5MP on an Adafruit Feather RP2040 (CircuitPython)
#
# Wiring (Mega -> Feather):
#   VCC  -> 3V          GND  -> GND
#   SCK  -> SCK  (GPIO18)
#   MOSI -> MO   (GPIO19)
#   MISO -> MI   (GPIO20)
#   CS   -> GPIO7 (the pin labeled "5" on the Feather)
#
# Prints the camera's info (sensor ID, firmware date, FPGA version), then
# grabs one 160x120 frame and prints it to the serial console as ASCII art.
# Open the serial console at least 80 characters wide.

import time
import board
import digitalio
import microcontroller

# ---- Registers (from Arducam's Arducam_Mega SDK) ----
REG_FIFO_CTRL = 0x04
REG_SENSOR_RESET = 0x07
REG_FORMAT = 0x20
REG_RESOLUTION = 0x21
REG_SENSOR_ID = 0x40
REG_YEAR = 0x41
REG_MONTH = 0x42
REG_DAY = 0x43
REG_SENSOR_STATE = 0x44  # bits 1:0 = sensor I2C state, bit 2 = capture done
REG_FIFO_SIZE1 = 0x45
REG_FIFO_SIZE2 = 0x46
REG_FIFO_SIZE3 = 0x47
REG_FPGA_VERSION = 0x49
CMD_BURST_READ = 0x3C

FIFO_CLEAR = 0x01
FIFO_START = 0x02
SENSOR_RESET = 0x40
STATE_IDLE = 0x02
CAPTURE_DONE = 0x04

FORMAT_YUV = 0x03  # YUYV 4:2:2 -> every even byte is a brightness (Y) value
RES_QQVGA = 0x02  # 160x120

SENSOR_NAMES = {
    0x81: "5MP",
    0x82: "3MP",
    0x83: "5MP (rev 2)",
    0x84: "3MP (rev 2)",
    0x85: "5MP",
    0x86: "3MP",
    0x87: "2MP",
}

SPI_BAUD = 4000000  # Mega supports up to 8 MHz; 4 MHz is safer on jumper wires

# ---- ASCII art settings ----
IMG_W, IMG_H = 160, 120
COLS, ROWS = 80, 30  # each character covers a 2x4 pixel block
RAMP = " .:-=+*#%@"  # dark -> bright (for a dark-background terminal)
INVERT = False  # set True if your terminal has a light background
if INVERT:
    RAMP = "@%#*+=-:. "

# ---- SPI setup ----
spi = board.SPI()
cs = digitalio.DigitalInOut(microcontroller.pin.GPIO7)
cs.switch_to_output(value=True)

donePin = digitalio.DigitalInOut(microcontroller.pin.GPIO12)
donePin.switch_to_output(value=True)

while not spi.try_lock():
    pass
spi.configure(baudrate=SPI_BAUD, phase=0, polarity=0)

_tx = bytearray(3)
_rx = bytearray(3)


def write_reg(addr, value):
    cs.value = False
    spi.write(bytes((addr | 0x80, value)))  # bit 7 set = write
    cs.value = True
    time.sleep(0.001)


def read_reg(addr):
    # Send address, then the Mega clocks out one dummy byte before the value.
    _tx[0] = addr & 0x7F
    cs.value = False
    spi.write_readinto(_tx, _rx)
    cs.value = True
    return _rx[2]


def wait_idle(timeout=2.0):
    deadline = time.monotonic() + timeout
    while (read_reg(REG_SENSOR_STATE) & 0x03) != STATE_IDLE:
        if time.monotonic() > deadline:
            raise RuntimeError("Camera never went idle - check wiring, CS pin and 3V power")
        time.sleep(0.002)


def capture():
    """Trigger one capture and return the number of bytes waiting in the FIFO."""
    write_reg(REG_FIFO_CTRL, FIFO_CLEAR)
    write_reg(REG_FIFO_CTRL, FIFO_START)
    deadline = time.monotonic() + 5
    while not read_reg(REG_SENSOR_STATE) & CAPTURE_DONE:
        if time.monotonic() > deadline:
            raise RuntimeError("Capture timed out")
        time.sleep(0.01)
    return (
        read_reg(REG_FIFO_SIZE1)
        | (read_reg(REG_FIFO_SIZE2) << 8)
        | (read_reg(REG_FIFO_SIZE3) << 16)
    )


# ---- Read camera info ----
time.sleep(0.2)  # module needs ~100 ms to boot after power-up
read_reg(REG_SENSOR_ID)  # throwaway read; the first transfer after power-up can be junk

write_reg(REG_SENSOR_RESET, SENSOR_RESET)
wait_idle()

sensor_id = read_reg(REG_SENSOR_ID)
wait_idle()
year = 2000 + (read_reg(REG_YEAR) & 0x3F)
month = read_reg(REG_MONTH) & 0x0F
day = read_reg(REG_DAY) & 0x1F
fpga = read_reg(REG_FPGA_VERSION)

print("---- Arducam Mega info ----")
print("Sensor ID:     0x{:02X} ({})".format(sensor_id, SENSOR_NAMES.get(sensor_id, "unknown")))
print("Firmware date: {:04d}-{:02d}-{:02d}".format(year, month, day))
print("FPGA version:  0x{:02X}".format(fpga))

if sensor_id not in SENSOR_NAMES:
    raise RuntimeError("Unexpected sensor ID - check wiring")

# ---- Grab a raw 160x120 YUV frame ----
write_reg(REG_FORMAT, FORMAT_YUV)
wait_idle()
write_reg(REG_RESOLUTION, RES_QQVGA)  # bit 7 = 0 -> still-capture mode
wait_idle()
time.sleep(1.0)  # let auto-exposure settle after the mode change

length = capture()
frame_bytes = IMG_W * IMG_H * 2
print("Captured {} bytes (expected {})".format(length, frame_bytes))
if length < frame_bytes:
    raise RuntimeError("Frame is smaller than expected")

# Stream the FIFO one pixel row at a time and average each 2x4 block of
# brightness values. Burst read = command byte, one dummy byte, then data.
row = bytearray(IMG_W * 2)
block_h = IMG_H // ROWS
blocks = []
sums = [0] * COLS
cs.value = False
spi.write(bytes((CMD_BURST_READ,)))
spi.readinto(row, end=1)  # dummy byte
for y in range(IMG_H):
    spi.readinto(row)
    # YUYV: Y0 U Y1 V -> the two pixels of each column sit at bytes 4x and 4x+2
    for x in range(COLS):
        i = x * 4
        sums[x] += row[i] + row[i + 2]
    if y % block_h == block_h - 1:
        blocks.append(sums)
        sums = [0] * COLS
cs.value = True

# ---- Stretch contrast and print ----
lo = min(min(r) for r in blocks)
hi = max(max(r) for r in blocks)
span = max(hi - lo, 1)
steps = len(RAMP) - 1

print()
for r in blocks:
    print("".join(RAMP[(v - lo) * steps // span] for v in r))
print()

spi.unlock()

donePin.value = False

while True:
    pass
