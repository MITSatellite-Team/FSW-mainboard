"""Argus5 RTC diagnostic: when does the RV-3032 (0x51) appear on I2C?"""
import time
import board
import digitalio

periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)
time.sleep(0.05)

i2c = board.I2C()


def scan():
    while not i2c.try_lock():
        pass
    try:
        return [hex(a) for a in i2c.scan()]
    finally:
        i2c.unlock()


print("Before RTC_EN:", scan())

rtc_en = digitalio.DigitalInOut(board.RTC_EN)
rtc_en.switch_to_output(value=True)
t0 = time.monotonic()
print("RTC_EN high - green LED D16 should now be on")

seen = None
while time.monotonic() - t0 < 2.0:
    found = scan()
    t = (time.monotonic() - t0) * 1000
    if "0x51" in found:
        seen = t
        print("%6.0f ms: 0x51 present  %s" % (t, found))
        break
    print("%6.0f ms: no 0x51       %s" % (t, found))
    time.sleep(0.02)

if seen is None:
    print("\n0x51 never appeared within 2 s.")
    print("Check: D16 lit? +3V3_RTC at C76 ~3.3 V? U17 ON pin high?")
    print("Then U18 soldering, and SDA/SCL on U18 pins 2/8 with a scope.")
else:
    print("\nRV-3032 responds %.0f ms after RTC_EN." % seen)

while True:
    time.sleep(1)
