import digitalio
import time
import microcontroller.pin as GPIO

from adafruit_bus_device.spi_device import SPIDevice

REG_STATE = 0x44
REG_SENSOR_RESET = 0x07
REG_SENSOR_ID = 0x40
REG_FORMAT = 0x20
REG_RESOLUTION = 0x21
REG_FIFO = 0x04  

SENSOR_RESET_ENABLE = 0x40
FIFO_CLEAR = 0x01
FIFO_START = 0x02
CAP_DONE = 0x04
REG_FIFO_SIZE = (0x45, 0x46, 0x47)
BURST_READ = 0x3C

FMT_YUV = 0x03
MODE_96X96 = 0x0A

IMG_W, IMG_H = 96, 96
DW, DH = 48, 48
SX, SY = IMG_W // DW, IMG_H // DH
Y_OFFSET = 0
INVERT = False
HALF = (" ", "\u2584", "\u2580", "\u2588")
_px = [0] * (DW * DH)

class ArducamMega:
    def __init__(self, spi, cs):
        self.cam = SPIDevice(spi, digitalio.DigitalInOut(cs), baudrate=1_000_000, polarity=0, phase=0)

    def write_register(self, register, value):
        with self.cam as bus:
            bus.write(bytes([register | 0x80, value]))

    def read_register(self, register):
        buf = bytearray(2)
        with self.cam as bus:
            bus.write(bytes([register & 0x7F]))
            bus.readinto(buf)
        return buf[1]

    def wait_idle(self, timeout_ms=1000):
        t0 = time.monotonic_ns() // 1_000_000

        while (self.read_register(REG_STATE) & 0x03) != 0x02:
            if (time.monotonic_ns() // 1_000_000) - t0 > timeout_ms:
                raise ValueError("Failed to reach idle!")

            time.sleep(0.002)

    def boot(self):
        time.sleep(1.0)
        
        self.write_register(REG_SENSOR_RESET, SENSOR_RESET_ENABLE)

        self.wait_idle()

        sensor_id = self.read_register(REG_SENSOR_ID)

        print(f"Sensor id: {sensor_id}")

        self.wait_idle()

        self.write_register(0x0A, 0x78)

        self.wait_idle()

    def configure(self, mode, fmt):
        self.write_register(REG_FORMAT, fmt)
        
        self.wait_idle()
        
        self.write_register(REG_RESOLUTION, mode)

        self.wait_idle()

    def take_picture(self, timeout_ms=3000):
        self.write_register(REG_FIFO, FIFO_CLEAR)
        self.write_register(REG_FIFO, FIFO_START)

        t0 = time.monotonic_ns() // 1_000_000
        
        while not (self.read_register(REG_STATE) & CAP_DONE):
            if (time.monotonic_ns() // 1_000_000) - t0 > timeout_ms:
                raise OSError("capture never finished")
        
        b0, b1, b2 = (self.read_register(r) for r in REG_FIFO_SIZE)
        
        return ((b2 << 16) | (b1 << 8) | b0) & 0xFFFFFF

    def read_fifo(self, buf, length=None):
        n = len(buf) if length is None else min(length, len(buf))

        with self.cam as bus:
            bus.write(bytes([BURST_READ, 0x00]))
            bus.readinto(memoryview(buf)[:n], write_value=0x00)
        
        return n

    def render(frame):
        # 1. Downsample Y channel to DW x DH
        row_bytes = IMG_W * 2
        area = SX * SY
        for dy in range(DH):
            for dx in range(DW):
                s = 0
                for yy in range(dy * SY, dy * SY + SY):
                    i = yy * row_bytes + dx * SX * 2 + Y_OFFSET
                    for _ in range(SX):
                        s += frame[i]
                        i += 2
                _px[dy * DW + dx] = s // area

        # 2. Stretch contrast to 0..255
        lo, hi = min(_px), max(_px)
        rng = (hi - lo) or 1
        for i in range(len(_px)):
            v = (_px[i] - lo) * 255 // rng
            _px[i] = 255 - v if INVERT else v

        # 3. Floyd-Steinberg (integer, errors pushed right and down)
        for y in range(DH):
            last_row = y == DH - 1
            for x in range(DW):
                i = y * DW + x
                old = _px[i]
                new = 255 if old >= 128 else 0
                _px[i] = new
                err = old - new
                if x + 1 < DW:
                    _px[i + 1] += err * 7 // 16
                if not last_row:
                    j = i + DW
                    if x > 0:
                        _px[j - 1] += err * 3 // 16
                    _px[j] += err * 5 // 16
                    if x + 1 < DW:
                        _px[j + 1] += err // 16

        # 4. Print two pixel rows per text line
        for y in range(0, DH, 2):
            top = y * DW
            bot = top + DW
            print("".join(
                HALF[((_px[top + x] > 0) << 1) | (_px[bot + x] > 0)]
                for x in range(DW)
            ))
        print("-" * DW)