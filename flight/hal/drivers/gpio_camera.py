import time
import board
import digitalio
import microcontroller
from digitalio import Pull

IMG_W, IMG_H = 160, 120
FRAME = bytearray(IMG_W * IMG_H)

TIMEOUT_NS = 500_000_000

class Line:
    def __init__(self, pin):
        self.io = digitalio.DigitalInOut(pin)
        self.release()
    def low(self):
        self.io.switch_to_output(value=False)
    def release(self):
        self.io.switch_to_input(pull=Pull.UP)

L = (Line(microcontroller.pin.GPIO8), Line(microcontroller.pin.GPIO9))

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
    for x in buf:
        send_byte(x)

def recv_bytes(n):
    out = bytearray(n)
    for i in range(n):
        out[i] = recv_byte()
    return out

def recv_bytes_no_alloc(n, buff):
    for i in range(n):
        print("Receiving byte", i)
        buff[i] = recv_byte()

def get_image(buff):
    try:
        print("Sending bit!")

        send_bit(True)

        print("Receiving bytes!")
        recv_bytes_no_alloc(len(buff), buff)
        print("Received all bytes!")
    except:
        pass