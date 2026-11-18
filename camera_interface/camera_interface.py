import time
import board
import digitalio
import microcontroller
import gc
from digitalio import Pull

REG_FIFO_CTRL = 0x04
REG_SENSOR_RESET = 0x07
REG_FORMAT = 0x20
REG_RESOLUTION = 0x21
REG_COLOR_EFFECT = 0x27
REG_IMAGE_QUALITY = 0x2A
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

FORMAT_JPEG = 0x01
FX_NONE = 0x00       # normal color

# Sensor IDs (register 0x40), from Arducam's SDK v3 ArducamCamera.h
SENSOR_NAMES = {
    0x81: "5MP v1", 0x82: "3MP v1",
    0x83: "5MP v2", 0x84: "3MP v2",
    0x85: "5MP (new)", 0x86: "3MP (new)", 0x87: "2MP (new)",
}
FIRST_NEW_SENSOR = 0x85   # 0x85 and up use the new resolution codes

# Resolution register (0x21) codes as {(width, height): code}, from Arducam's
# SDK v3 (CAM_IMAGE_MODE in ArducamCamera.h, legacyMode() in ArducamCamera.c).
RES_CODES_NEW = {         # sensor ID 0x85 and up
    (96, 96): 0x00,
    (128, 128): 0x01,
    (160, 120): 0x02,
    (320, 240): 0x03,
    (320, 320): 0x04,
    (640, 480): 0x05,
    (800, 600): 0x06,
    (1024, 768): 0x07,
    (1280, 720): 0x08,
    (1280, 1024): 0x09,
    (1600, 1200): 0x0A,
    (1920, 1080): 0x0B,
    (2048, 1536): 0x0C,
    (2592, 1944): 0x0D,
}
RES_CODES_LEGACY = {      # sensor ID 0x81-0x84
    # 160x120 (0x00), 800x600 (0x03) and 1280x960 (0x05) don't work on the old 5MP
    (96, 96): 0x0A,
    (128, 128): 0x0B,
    (320, 240): 0x01,
    (320, 320): 0x0C,
    (640, 480): 0x02,
    (1280, 720): 0x04,
    (1600, 1200): 0x06,
    (1920, 1080): 0x07,
    (2048, 1536): 0x08,   # 3MP max
    (2592, 1944): 0x09,   # 5MP max
}

# JPEG quality (the camera only offers three levels)
QUALITY_HIGH = 0x00
QUALITY_DEFAULT = 0x01
QUALITY_LOW = 0x02
QUALITY_NAMES = {QUALITY_HIGH: "high", QUALITY_DEFAULT: "default", QUALITY_LOW: "low"}

# ---- size budget ----
BUDGET = 20_000      # max bytes per image

# Settings to try, best-looking first, as ((width, height), quality). JPEG size
# depends on the scene, so the code steps down this list when an image is over
# budget and steps back up when there's plenty of room. Sizes are looked up in
# the code table for whichever camera is attached. To pin one setting, leave a
# single entry. (The new sensors also offer 800x600 and 1024x768, which sit
# between 640x480 and 1280x720 if you want finer steps.)
SETTINGS = (
    ((1280, 720), QUALITY_LOW),      # 16:9, so the framing differs from 4:3
    ((640, 480),  QUALITY_DEFAULT),
    ((640, 480),  QUALITY_LOW),
    ((320, 240),  QUALITY_DEFAULT),
    ((320, 240),  QUALITY_LOW),
)
START_SETTING = ((640, 480), QUALITY_LOW)
STEP_UP_BELOW = 0.4      # step up when an image uses < 40% of the budget...
STEP_UP_COOLDOWN = 5     # ...but not within 5 captures of having to step down
SETTLE_S = 0.5           # pause after changing settings

SPI_BAUD = 4000000  # Mega supports up to 8 MHz; 4 MHz is safer on jumper wires

TIMEOUT_NS = 500_000_000

spi = board.SPI()
cs = digitalio.DigitalInOut(microcontroller.pin.GPIO7)
cs.switch_to_output(value=True)

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

def setup():
    """Reset the camera and return its sensor ID."""
    spi.configure(baudrate=SPI_BAUD, phase=0, polarity=0)

    # wait for power up
    time.sleep(0.2)

    # first read is sometimes invalid
    read_reg(REG_SENSOR_ID)

    write_reg(REG_SENSOR_RESET, SENSOR_RESET)
    wait_idle()

    sensor_id = read_reg(REG_SENSOR_ID)
    if sensor_id not in SENSOR_NAMES:
        raise RuntimeError("Unknown sensor ID 0x%02X - check wiring" % sensor_id)
    print("Camera sensor ID 0x%02X (%s), FPGA version 0x%02X" % (
        sensor_id, SENSOR_NAMES[sensor_id], read_reg(REG_FPGA_VERSION)))
    return sensor_id

def build_ladder(codes):
    """Turn SETTINGS into (name, code, quality) entries using this camera's
    resolution codes. Returns the ladder and the level to start at."""
    ladder = []
    start = None
    for (w, h), quality in SETTINGS:
        code = codes.get((w, h))
        if code is None:
            print("This camera has no %dx%d mode - skipping that setting" % (w, h))
            continue
        if ((w, h), quality) == START_SETTING:
            start = len(ladder)
        ladder.append(("%dx%d %s" % (w, h, QUALITY_NAMES[quality]), code, quality))
    if not ladder:
        raise RuntimeError("None of the sizes in SETTINGS work on this camera")
    if start is None:
        start = len(ladder) - 1       # smallest; it steps up if there's room
    return tuple(ladder), start

def apply_level(level):
    name, res, quality = LADDER[level]
    print("Camera setting:", name)
    write_reg(REG_RESOLUTION, res)  # bit 7 = 0 -> still-capture mode
    wait_idle()
    write_reg(REG_IMAGE_QUALITY, quality)
    wait_idle()
    time.sleep(SETTLE_S)

def configure(level):
    # Format first: the SDK rewrites the resolution after a format change on
    # the new sensors, so the resolution must come after it.
    write_reg(REG_FORMAT, FORMAT_JPEG)
    wait_idle()
    write_reg(REG_COLOR_EFFECT, FX_NONE)
    wait_idle()
    apply_level(level)

    # let auto-exposure settle after the mode change
    time.sleep(1.0)

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

JPEG = bytearray(BUDGET)              # JPEG buffer, allocated once
_burst = bytes((CMD_BURST_READ,))
_dummy = bytearray(1)

def get_jpeg():
    """Capture one JPEG. Returns a zero-copy memoryview of the image bytes,
    or None if the image is over budget (it is never read in that case)."""
    length = capture()

    if length > BUDGET:
        print("Over budget:", length, "bytes")
        return None
    if length < 4:
        raise RuntimeError("Bad JPEG length: %d" % length)

    cs.value = False
    spi.write(_burst)
    spi.readinto(_dummy)              # Mega sends one dummy byte first
    spi.readinto(JPEG, end=length)    # whole FIFO in one go
    cs.value = True

    if JPEG[0] != 0xFF or JPEG[1] != 0xD8:
        raise RuntimeError("Missing JPEG start marker (got %02X %02X)" % (JPEG[0], JPEG[1]))

    # The FIFO length can include padding after the image: trim to the last FF D9.
    end = length
    while end > 4 and not (JPEG[end - 2] == 0xFF and JPEG[end - 1] == 0xD9):
        end -= 1
    if end <= 4:
        raise RuntimeError("Missing JPEG end marker")

    return memoryview(JPEG)[:end]

def jpeg_size(buf, n):
    """Return (width, height) from a JPEG's frame header, or None.
    Used to print each image's real size as a check on the settings."""
    i = 2                             # skip FF D8
    while i + 9 <= n:
        if buf[i] != 0xFF:
            return None
        marker = buf[i + 1]
        if marker == 0xFF:            # padding byte
            i += 1
            continue
        if marker in (0xC0, 0xC1, 0xC2):   # SOF: FF Cx, len(2), bits, H(2), W(2)
            return ((buf[i + 7] << 8) | buf[i + 8], (buf[i + 5] << 8) | buf[i + 6])
        if marker == 0xDA:            # start of image data, no size found
            return None
        i += 2 + ((buf[i + 2] << 8) | buf[i + 3])
    return None

cooldown = 0

def take_jpeg():
    """Capture at the current setting, stepping down and retaking until it fits."""
    global level, cooldown
    while True:
        jpeg = get_jpeg()
        if jpeg is not None:
            return jpeg
        if level == len(LADDER) - 1:
            raise RuntimeError("Even the smallest setting is over budget")
        level += 1
        cooldown = STEP_UP_COOLDOWN
        apply_level(level)

def maybe_step_up(size):
    """Called between requests: move to a better setting if there's room."""
    global level, cooldown
    if cooldown:
        cooldown -= 1
    elif level > 0 and size < BUDGET * STEP_UP_BELOW:
        level -= 1
        apply_level(level)

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

class Line:
    def __init__(self, pin):
        self.io = digitalio.DigitalInOut(pin)
        self.release()
    def low(self):
        self.io.switch_to_output(value=False)
    def release(self):
        self.io.switch_to_input(pull=Pull.UP)

L = (Line(microcontroller.pin.GPIO12), Line(microcontroller.pin.GPIO11))

def _reset():
    L[0].release(); L[1].release()

def _wait(line, level, timeout_ns=TIMEOUT_NS):
    io = line.io
    deadline = time.monotonic_ns() + timeout_ns
    while io.value != level:
        if time.monotonic_ns() > deadline:
            _reset()
            raise TimeoutError

def send_bit(b):
    me, other = L[b], L[1 - b]
    me.low()               # 1. assert bit
    _wait(other, False)    # 2. receiver ACKs on the other line
    me.release()           # 3. release
    _wait(other, True)     # 4. back to idle

def recv_bit(timeout_ns=TIMEOUT_NS):
    """Only used to receive the 1-bit capture request from receiver.py."""
    a, c = L[0].io, L[1].io
    deadline = None if timeout_ns is None else time.monotonic_ns() + timeout_ns
    while True:
        if not a.value: b = 0; break
        if not c.value: b = 1; break
        if deadline and time.monotonic_ns() > deadline:
            _reset(); raise TimeoutError
    ack = L[1 - b]
    ack.low()              # 2. ACK
    _wait(L[b], True)      # 3. wait for sender release
    ack.release()          # 4. idle
    return b

def send_byte(x):
    for i in range(7, -1, -1):
        send_bit((x >> i) & 1)

def send_bytes(buf):
    for i, x in enumerate(buf):
        if i % 1024 == 0:   # printing every byte slows the link a lot
            print("Sending byte", i)
        send_byte(x)

# ---- framing: 4-byte big-endian length, then the JPEG bytes (see receiver.py) ----

def send_jpeg(jpeg):
    send_bytes(len(jpeg).to_bytes(4, "big"))
    send_bytes(jpeg)

while not spi.try_lock():
    pass

sensor_id = setup()

if sensor_id >= FIRST_NEW_SENSOR:
    LADDER, level = build_ladder(RES_CODES_NEW)
else:
    LADDER, level = build_ladder(RES_CODES_LEGACY)

configure(level)

led.value = True

while True:
    while True:
        try:
            print("Waiting for bit...")
            if recv_bit():
                break
        except:
            pass

    print("Taking image...")
    led.value = False

    try:
        jpeg = take_jpeg()
        wh = jpeg_size(jpeg, len(jpeg))
        print("JPEG is", len(jpeg), "bytes,", "%dx%d" % wh if wh else "size unknown",
              "(setting:", LADDER[level][0] + ")")
    except RuntimeError as e:
        print("Capture failed:", e)
        jpeg = b""   # length 0 tells the receiver there is no image this time

    print("Sending image...")
    t0 = time.monotonic()

    try:
        send_jpeg(jpeg)
        dt = time.monotonic() - t0
        print("Sent image in %.1f s (%d bytes/s)" % (dt, len(jpeg) / dt if dt else 0))
    except:
        print("Send failed")

    led.value = True

    size = len(jpeg)
    jpeg = None
    gc.collect()

    if size:
        maybe_step_up(size)