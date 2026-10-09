# Arducam Mega -> ASCII art (MicroPython, Adafruit Feather RP2040)
# Wiring: SCK=GPIO18, MO=GPIO19, MI=GPIO20, CS=GPIO7 (silkscreen "5"), 3V, GND
# Talks to the camera's registers directly, mirroring the Arducam_Mega Arduino library.

from machine import Pin, SPI
import time

# ---- Settings ----
IMG_W, IMG_H = 96, 96
COLS, ROWS = 48, 24                 # characters are ~2x taller than wide
BX, BY = IMG_W // COLS, IMG_H // ROWS
Y_OFFSET = 0                        # 0 = YUYV byte order, 1 = UYVY. Flip if you see only noise.
INVERT = False                      # True for light-background terminals
RAMP = " .:-=+*#%@"                 # dark -> bright

# ---- Camera registers (from Arducam_Mega ArducamCamera.c / .h) ----
REG_FIFO            = 0x04          # ARDUCHIP_FIFO
FIFO_CLEAR          = 0x01          # FIFO_CLEAR_ID_MASK
FIFO_START          = 0x02          # FIFO_START_MASK
REG_SENSOR_RESET    = 0x07
SENSOR_RESET_ENABLE = 0x40
REG_FORMAT          = 0x20          # CAM_REG_FORMAT
REG_RESOLUTION      = 0x21          # CAM_REG_CAPTURE_RESOLUTION
REG_SENSOR_ID       = 0x40
REG_STATE           = 0x44          # sensor state, also ARDUCHIP_TRIG
CAP_DONE            = 0x04          # CAP_DONE_MASK
REG_FIFO_SIZE       = (0x45, 0x46, 0x47)
BURST_READ          = 0x3C          # BURST_FIFO_READ
FMT_YUV             = 0x03          # CAM_IMAGE_PIX_FMT_YUV
MODE_96X96          = 0x0A          # CAM_IMAGE_MODE_96X96

# ---- SPI setup ----
spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(20))
cs = Pin(7, Pin.OUT, value=1)
_tx = bytearray(3)
_rx = bytearray(3)

def read_reg(addr):
    _tx[0] = addr & 0x7F            # read command
    cs(0)
    spi.write_readinto(_tx, _rx)    # address, dummy, value
    cs(1)
    return _rx[2]

def write_reg(addr, val):
    cs(0)
    spi.write(bytes((addr | 0x80, val)))
    cs(1)

def wait_idle(timeout_ms=1000):
    # Same check as the library's waitI2cIdle(), but with a timeout instead of hanging
    t0 = time.ticks_ms()
    while (read_reg(REG_STATE) & 0x03) != 0x02:
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            print("not idle")
            # raise OSError("camera never reported idle")
        time.sleep_ms(2)

def cam_begin():
    write_reg(REG_SENSOR_RESET, SENSOR_RESET_ENABLE)
    wait_idle()
    cid = read_reg(REG_SENSOR_ID)
    wait_idle()
    return cid

def cam_configure(mode, fmt):
    write_reg(REG_FORMAT, fmt)
    wait_idle()
    write_reg(REG_RESOLUTION, mode)
    wait_idle()

def take_picture(timeout_ms=3000):
    write_reg(REG_FIFO, FIFO_CLEAR)
    write_reg(REG_FIFO, FIFO_START)
    t0 = time.ticks_ms()
    while not (read_reg(REG_STATE) & CAP_DONE):
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            raise OSError("capture never finished")
    b0, b1, b2 = (read_reg(r) for r in REG_FIFO_SIZE)
    return ((b2 << 16) | (b1 << 8) | b0) & 0xFFFFFF

def read_fifo(buf):
    # One burst read: command byte, one dummy byte, then the image data
    cs(0)
    spi.write(bytes((BURST_READ, 0x00)))
    spi.readinto(buf, 0x00)
    cs(1)

# ---- Image -> ASCII ----
frame = bytearray(IMG_W * IMG_H * 2)        # YUV = 2 bytes per pixel
sums = [0] * (ROWS * COLS)
PIX = BX * BY
LEVELS = len(RAMP) - 1

def render():
    for i in range(len(sums)):
        sums[i] = 0
    row_bytes = IMG_W * 2
    for y in range(IMG_H):
        i = y * row_bytes + Y_OFFSET        # first brightness byte in this row
        base = (y // BY) * COLS
        for x in range(IMG_W):
            sums[base + x // BX] += frame[i]
            i += 2                          # skip the color byte

    avgs = [s // PIX for s in sums]
    lo, hi = min(avgs), max(avgs)
    rng = (hi - lo) or 1                            # stretch contrast

    for r in range(ROWS):
        line = []
        for c in range(COLS):
            v = (avgs[r * COLS + c] - lo) * LEVELS // rng
            if INVERT:
                v = LEVELS - v
            line.append(RAMP[v])
        print("".join(line))
    print("-" * COLS)

# ---- Main ----
time.sleep_ms(1000)                         # let the camera power up
print("Starting...")

write_reg(REG_SENSOR_RESET, SENSOR_RESET_ENABLE)

# while True:
#     wait_idle()

cid = cam_begin()
print("Sensor ID: 0x%02X" % cid)
cam_configure(MODE_96X96, FMT_YUV)
time.sleep_ms(500)                          # let the sensor settle
take_picture()                              # discard first frame
print("Began!")

while True:
    try:
        length = take_picture()
    except OSError as e:
        print("FAIL:", e)
        time.sleep_ms(1000)
        continue
    if length < len(frame):
        print("FAIL: got %d bytes, expected %d" % (length, len(frame)))
        time.sleep_ms(1000)
        continue
    read_fifo(frame)
    render()
    time.sleep_ms(500)