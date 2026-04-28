import time


class QwiicOpenLog:
    """Stream-compatible driver for the SparkFun Qwiic OpenLog (I2C, 0x2A).

    The OpenLog firmware starts I2C before it finishes SD init and file open,
    so call wait_for_ready() before the first write to avoid silent data loss.

    Register protocol (SparkFun Qwiic OpenLog firmware):
      0x0C = WRITE_FILE — prefix byte before data (max 31 data bytes/transaction)
      0x11 = SYNC_FILE  — standalone byte to flush buffer to SD card
    """

    def __init__(self, i2c, address=0x2A):
        self._i2c = i2c
        self._address = address

    def wait_for_ready(self, timeout=3.0):
        """Sleep until the OpenLog has had time to initialize the SD card and open its log file."""
        time.sleep(timeout)

    def write(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        timeout = 1000
        while not self._i2c.try_lock():
            timeout -= 1
            if timeout <= 0:
                return 0
        try:
            view = memoryview(data)
            offset = 0
            while offset < len(view):
                end = offset + 31
                chunk = bytes([0x0C]) + bytes(view[offset:end])
                self._i2c.writeto(self._address, chunk)
                offset = end
            self._i2c.writeto(self._address, bytes([0x11]))
        finally:
            self._i2c.unlock()
        return len(data)

    def flush(self):
        timeout = 1000
        while not self._i2c.try_lock():
            timeout -= 1
            if timeout <= 0:
                return
        try:
            self._i2c.writeto(self._address, bytes([0x11]))
        finally:
            self._i2c.unlock()

    @property
    def device_errors(self):
        return []

    def deinit(self):
        pass
