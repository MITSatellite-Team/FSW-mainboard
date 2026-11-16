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
IMG_W, IMG_H = 160, 120

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
    spi.configure(baudrate=SPI_BAUD, phase=0, polarity=0)

    # wait for power up
    time.sleep(0.2)

    # first read is sometimes invalid
    read_reg(REG_SENSOR_ID)

    write_reg(REG_SENSOR_RESET, SENSOR_RESET)
    wait_idle()

def configure():
    write_reg(REG_FORMAT, FORMAT_YUV)
    wait_idle()
    write_reg(REG_RESOLUTION, RES_QQVGA)  # bit 7 = 0 -> still-capture mode
    wait_idle()

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

FRAME = bytearray(IMG_W * IMG_H)      # grayscale (Y) frame, allocated once
_row = bytearray(IMG_W * 2)           # one YUYV row
_burst = bytes((CMD_BURST_READ,))
_dummy = bytearray(1)

def get_frame():
    length = capture()

    if length < IMG_W * IMG_H * 2:
        raise RuntimeError("Frame is smaller than expected")

    cs.value = False
    
    spi.write(_burst)
    spi.readinto(_dummy)              # Mega sends one dummy byte first

    i = 0
    for _ in range(IMG_H):
        spi.readinto(_row)
        for x in range(0, IMG_W * 2, 2):   # keep only Y bytes
            FRAME[i] = _row[x]
            i += 1

    cs.value = True

    return FRAME

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

def recv_byte(first_timeout_ns=TIMEOUT_NS):
    x = recv_bit(first_timeout_ns)
    for _ in range(7):
        x = (x << 1) | recv_bit()
    return x

def send_bytes(buf):
    i = 0

    for x in buf:
        print("Sending byte", i)
        i += 1

        send_byte(x)

def recv_bytes(n):
    out = bytearray(n)
    for i in range(n):
        out[i] = recv_byte()
    return out

STEP = 2
RAMP = " .:-=+*#%@"

def print_frame():
    n = len(RAMP)

    for y in range(0, IMG_H, STEP):
        base = y * IMG_W
        row = []

        for x in range(0, IMG_W, STEP):
            v = FRAME[base + x]
            row.append(RAMP[v * n // 256])

        print("".join(row))

    print()

while not spi.try_lock():
    pass

setup()

configure()

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

    frame = get_frame()

    print("Sending image...")

    try:
        send_bytes(FRAME)
    except:
        pass

    print("Sent image...")
    led.value = True

    gc.collect()

    print_frame()

spi.unlock()