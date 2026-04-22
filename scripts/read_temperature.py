# SPDX-FileCopyrightText: 2017 Limor Fried for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython I2C - TMP117 Temperature Sensor Read on I2C1"""
import time

import board
import busio
import digitalio


if hasattr(board, "PERIPH_PWR_EN"):
    _pwr = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
    _pwr.deinit()

    PERIPH_PWR_EN = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
    PERIPH_PWR_EN.direction = digitalio.Direction.OUTPUT
    PERIPH_PWR_EN.value = True
    time.sleep(2)

if hasattr(board, "LORA_EN"):
    RADIO_EN = digitalio.DigitalInOut(board.LORA_EN)
    RADIO_EN.direction = digitalio.Direction.OUTPUT
    RADIO_EN.value = True

if hasattr(board, "COIL_EN"):
    COIL_EN = digitalio.DigitalInOut(board.COIL_EN)
    COIL_EN.direction = digitalio.Direction.OUTPUT
    COIL_EN.value = True


# TMP117 constants
TMP117_RESOLUTION = 0.0078125  # degrees C per LSB
TMP117_TEMP_REG   = 0x00
TMP117_ADDRESSES  = [0x48, 0x49, 0x4A, 0x4B]  # Possible addresses (ADD0 pin)


def read_tmp117(i2c, address):
    """Read temperature from a TMP117 at the given address. Returns Celsius or None."""
    buf = bytearray(2)
    try:
        i2c.writeto_then_readfrom(address, bytes([TMP117_TEMP_REG]), buf)
    except Exception as e:
        print("  Read error at {}: {}".format(hex(address), e))
        return None

    # Combine 2 bytes into a signed 16-bit integer
    raw = (buf[0] << 8) | buf[1]
    if raw & 0x8000:        # Handle negative (two's complement)
        raw -= 0x10000

    return raw * TMP117_RESOLUTION


# Initialize I2C1 only
try:
    i2c1 = busio.I2C(board.SCL1, board.SDA1)
    print("I2C1 initialized successfully.")
except Exception as e:
    i2c1 = None
    print("I2C1 not found:", e)


if i2c1:
    print("-" * 40)
    print("TMP117 READ LOOP on I2C1")
    print("-" * 40)
    while True:
        while not i2c1.try_lock():
            pass

        # Scan and filter to only TMP117 addresses
        all_addresses = i2c1.scan()
        tmp117s = [a for a in all_addresses if a in TMP117_ADDRESSES]

        if not tmp117s:
            print("No TMP117 found. Addresses on bus:", [hex(a) for a in all_addresses])
        else:
            for addr in tmp117s:
                temp_c = read_tmp117(i2c1, addr)
                if temp_c is not None:
                    temp_f = temp_c * 9 / 5 + 32
                    print("TMP117 @ {}: {:.4f} °C  /  {:.4f} °F".format(
                        hex(addr), temp_c, temp_f))

        i2c1.unlock()
        time.sleep(1)
else:
    print("No valid I2C1 bus found.")