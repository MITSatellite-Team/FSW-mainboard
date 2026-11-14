import time, board, busio, digitalio

# Turn on radio power
lora_en = digitalio.DigitalInOut(board.LORA_EN); lora_en.switch_to_output(value=True)
periph = digitalio.DigitalInOut(board.PERIPH_PWR_EN); periph.switch_to_output(value=True)
lora_flt = digitalio.DigitalInOut(board.LORA_FLT)
lora_flt.switch_to_input(pull=digitalio.Pull.UP)
time.sleep(0.05)
print("LORA_FLT:", lora_flt.value)

spi  = busio.SPI(board.CLK0, MOSI=board.MOSI0, MISO=board.MISO0)
cs   = digitalio.DigitalInOut(board.LORA_nCS);  cs.switch_to_output(value=True)
rst  = digitalio.DigitalInOut(board.LORA_nRST); rst.switch_to_output(value=True)
busy = digitalio.DigitalInOut(board.LORA_BUSY); busy.switch_to_input()

cs2 = digitalio.DigitalInOut(board.SD_CS); cs2.switch_to_output(value=True)

def xfer(out):
    buf = bytearray(len(out))
    while not spi.try_lock():
        pass
    spi.configure(baudrate=1000000, phase=0, polarity=0)
    cs.value = False
    spi.write_readinto(bytes(out), buf)
    cs.value = True
    spi.unlock()
    return buf

rst.value = False; time.sleep(0.002); rst.value = True
t0 = time.monotonic_ns()
while busy.value and time.monotonic_ns() - t0 < 1000000000:
    pass
print("BUSY went low after %d us" % ((time.monotonic_ns() - t0) // 1000))

print("GetStatus:", [hex(b) for b in xfer([0xC0, 0x00])])
r = xfer([0x1D, 0x03, 0x20, 0x00] + [0x00] * 16)
print("status 0x%02X, version %r" % (r[3], bytes(r[4:])))