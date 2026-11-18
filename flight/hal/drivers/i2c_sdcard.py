import time
import board
import busio

class QwiicOpenLog:
    REG_STATUS      = 0x01
    REG_FW_MAJOR    = 0x02
    REG_FW_MINOR    = 0x03
    REG_CREATE_FILE = 0x06
    REG_MKDIR       = 0x07
    REG_CD          = 0x08
    REG_OPEN_FILE   = 0x0B   # open a file and append to it
    REG_WRITE_FILE  = 0x0C   # data written here goes into the open file
    REG_RM          = 0x0F
    REG_SYNC        = 0x11   # flush buffered data to the card

    def __init__(self, i2c, address=0x2A):
        self.i2c = i2c
        self.addr = address

    def _cmd(self, reg, data=b""):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, bytes([reg]) + data)
        finally:
            self.i2c.unlock()

    def _read(self, reg, n=1):
        buf = bytearray(n)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, bytes([reg]))
            self.i2c.readfrom_into(self.addr, buf)
        finally:
            self.i2c.unlock()
        return buf

    def version(self):
        return "%d.%d" % (self._read(self.REG_FW_MAJOR)[0], self._read(self.REG_FW_MINOR)[0])

    def status(self):
        s = self._read(self.REG_STATUS)[0]
        return {"sd_ok": bool(s & 0x01), "last_cmd_ok": bool(s & 0x02),
                "last_cmd_known": bool(s & 0x04), "file_open": bool(s & 0x08),
                "in_root": bool(s & 0x10)}

    def append(self, name):
        self._cmd(self.REG_OPEN_FILE, name.encode())

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        for i in range(0, len(data), 31):          # at most 31 data bytes per transaction
            self._cmd(self.REG_WRITE_FILE, data[i:i + 31])

    def sync(self):
        self._cmd(self.REG_SYNC)