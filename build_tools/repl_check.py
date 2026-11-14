"""
Show exactly what a CircuitPython board sends back over serial, step by step.
usage: python3 repl_check.py /dev/ttyUSB0 [baud]
"""
import sys
import time

import serial

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200


def listen(s, seconds):
    end, data = time.monotonic() + seconds, b""
    while time.monotonic() < end:
        data += s.read(s.in_waiting or 1)
    return data


def step(s, label, send, seconds):
    if send:
        s.write(send)
        s.flush()
    got = listen(s, seconds)
    print(f"--- {label}: received {len(got)} bytes")
    print(repr(got))
    return got


s = serial.Serial(port, baud, timeout=0.05)
print(f"Opened {port} at {baud} baud (DTR={s.dtr}, RTS={s.rts})")
step(s, "idle for 2 s (anything main.py prints)", None, 2)
step(s, "sent Enter", b"\r", 1)
step(s, "sent Ctrl-C twice", b"\x03\x03", 1.5)
step(s, "sent Enter again", b"\r", 1)
got = step(s, "sent Ctrl-A (enter raw REPL)", b"\x01", 1.5)
if b"raw REPL; CTRL-B to exit" in got:
    step(s, "sent print(1+1) + Ctrl-D", b"print(1+1)\x04", 1.5)
    print("\nRESULT: raw REPL works on this port.")
else:
    print("\nRESULT: no raw REPL banner. Paste this whole output back.")
step(s, "sent Ctrl-B (back to normal REPL)", b"\x02", 1)
s.close()