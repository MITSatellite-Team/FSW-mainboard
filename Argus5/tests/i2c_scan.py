import time
import board
import digitalio

# On-board I2C0 devices sit behind protection circuits powered from their own
# rails, so they only answer when those rails are on.
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)   # INA226s (and RTC load switch input)
rtc_en = digitalio.DigitalInOut(board.RTC_EN)
rtc_en.switch_to_output(value=True)      # RV-3032
time.sleep(0.15)                         # RTC needs ~91 ms after power-up

KNOWN = {
    0x40: "INA226 U7  - 3V3_MAIN current",
    0x41: "INA226 U9  - 3V3_PERIPH current",
    0x44: "INA226 U12 - LoRa current",
    0x51: "RV-3032 U18 - RTC",
}

i2c = board.I2C()  # I2C0: SDA GPIO32, SCL GPIO33
while not i2c.try_lock():
    pass
try:
    found = i2c.scan()
finally:
    i2c.unlock()

print("I2C0 scan: %d device(s)" % len(found))
for addr in found:
    print("  0x%02X  %s" % (addr, KNOWN.get(addr, "unknown (backplane device?)")))
for addr, name in KNOWN.items():
    if addr not in found:
        print("  0x%02X  MISSING: %s" % (addr, name))

while True:
    time.sleep(1)
