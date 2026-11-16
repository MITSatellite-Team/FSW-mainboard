import time, busio, digitalio
import microcontroller.pin as GPIO

# Deselect everything else you know of on SPI0
for p in (GPIO.GPIO13, GPIO.GPIO33):
    d = digitalio.DigitalInOut(p); d.switch_to_output(value=True)

cs = digitalio.DigitalInOut(GPIO.GPIO8); cs.switch_to_output(value=True)

# 1) Raw MISO level with the bus idle and a pull-up enabled
miso = digitalio.DigitalInOut(GPIO.GPIO4)
miso.switch_to_input(pull=digitalio.Pull.UP)
print("MISO idle, CS high:", miso.value)   # expect True
cs.value = False; time.sleep(0.001)
print("MISO idle, CS low :", miso.value)
cs.value = True
miso.deinit()

# 2) Register reads at a slow clock
spi = busio.SPI(GPIO.GPIO6, MOSI=GPIO.GPIO7, MISO=GPIO.GPIO4)
while not spi.try_lock(): pass
spi.configure(baudrate=250_000, polarity=0, phase=0)

def rd(a):
    tx = bytes([a & 0x7F, 0, 0]); rx = bytearray(3)
    cs.value = False; spi.write_readinto(tx, rx); cs.value = True
    return rx

for _ in range(5):
    print("0x40 ->", [hex(b) for b in rd(0x40)], " 0x44 ->", [hex(b) for b in rd(0x44)])
    time.sleep(0.2)