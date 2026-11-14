"""Argus5 memory self-test: PSRAM, SPI flash and SD card.

Copy to CIRCUITPY as code.py (or import it from the REPL).
"""
import gc
import os
import struct
import time

import board
import digitalio
import microcontroller
import sdcardio
import storage

BLOCK = 4096
results = {}


def mb(n):
    return n / (1024 * 1024)


def pattern_check(write_block, read_block, nblocks):
    """Write nblocks of random data, each stamped with its index, then verify.

    All blocks are written before any are read, so broken or shorted address
    lines show up as one block overwriting another.
    Returns (errors, write_seconds, read_seconds).
    """
    # No allocations inside the loops: a garbage collection here would have
    # to scan the whole test buffer, which is very slow on PSRAM.
    base = bytearray(os.urandom(BLOCK))
    check = bytearray(BLOCK)
    step = max(1, nblocks // 4)
    t0 = time.monotonic()
    for i in range(nblocks):
        struct.pack_into(">I", base, 0, i)
        write_block(i, base)
        if i % step == step - 1:
            print("    written %d%%" % ((i + 1) * 100 // nblocks))
    t1 = time.monotonic()
    errors = 0
    for i in range(nblocks):
        struct.pack_into(">I", base, 0, i)
        read_block(i, check)
        if check != base:
            errors += 1
            if errors <= 5:
                print("    mismatch in block %d" % i)
        if i % step == step - 1:
            print("    verified %d%%" % ((i + 1) * 100 // nblocks))
    t2 = time.monotonic()
    return errors, t1 - t0, t2 - t1


# --- Power ------------------------------------------------------------------
periph_en = digitalio.DigitalInOut(board.PERIPH_PWR_EN)
periph_en.switch_to_output(value=True)
sd_en = digitalio.DigitalInOut(board.SD_EN)
sd_en.switch_to_output(value=True)
time.sleep(0.05)


# --- PSRAM ------------------------------------------------------------------
print("\n== PSRAM ==")
gc.collect()
heap = gc.mem_free() + gc.mem_alloc()
print("  Heap size: %.2f MB" % mb(heap))
if heap < 1024 * 1024:
    print("  PSRAM not detected (heap is internal SRAM only)")
    results["PSRAM"] = False
else:
    size = min(4 * 1024 * 1024, (gc.mem_free() - 512 * 1024)) // BLOCK * BLOCK
    buf = bytearray(size)
    mv = memoryview(buf)
    n = size // BLOCK
    print("  Testing %.2f MB..." % mb(size))

    def ps_write(i, data):
        mv[i * BLOCK:(i + 1) * BLOCK] = data

    def ps_read(i, out):
        out[:] = mv[i * BLOCK:(i + 1) * BLOCK]

    # With the 4 MB random buffer live, every collection has to scan all of
    # it over QSPI. Disabling auto-collect makes the heap grow instead.
    gc.disable()
    try:
        err, tw, tr = pattern_check(ps_write, ps_read, n)
    finally:
        gc.enable()
    print("  Write %.2f MB/s, verify %.2f MB/s" % (mb(size) / tw, mb(size) / tr))
    results["PSRAM"] = err == 0
    del mv, buf
    gc.collect()


# --- SPI flash --------------------------------------------------------------
print("\n== SPI flash ==")
st = os.statvfs("/")
print("  CIRCUITPY: %.2f MB total, %.2f MB free" % (mb(st[0] * st[2]), mb(st[0] * st[3])))

# microcontroller.nvm lives in flash and is writable even while USB has
# CIRCUITPY mounted, so it gives a real erase/program/read check.
nvm = microcontroller.nvm
original = nvm[0:len(nvm)]
test = bytes(os.urandom(len(nvm)))
nvm[0:len(nvm)] = test
nvm_ok = nvm[0:len(nvm)] == test
nvm[0:len(nvm)] = original
nvm_ok = nvm_ok and nvm[0:len(nvm)] == original
print("  NVM write/read/restore (%d bytes): %s" % (len(nvm), "OK" if nvm_ok else "FAILED"))

fs_ok = True
try:
    n = 64  # 256 KB
    base = bytearray(os.urandom(BLOCK))
    # Write, then reopen to read back from flash rather than a buffer.
    with open("/_flashtest.bin", "wb") as f:
        t0 = time.monotonic()
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            f.write(base)
        tw = time.monotonic() - t0
    with open("/_flashtest.bin", "rb") as f:
        t0 = time.monotonic()
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            if f.read(BLOCK) != base:
                fs_ok = False
        tr = time.monotonic() - t0
    os.remove("/_flashtest.bin")
    print("  Filesystem 256 KB round trip: %s (write %.0f KB/s, read %.0f KB/s)"
          % ("OK" if fs_ok else "FAILED", 256 / tw, 256 / tr))
except OSError as e:
    if e.errno == 30:  # EROFS
        print("  Filesystem write skipped: CIRCUITPY is read-only while USB is connected")
    else:
        print("  Filesystem write failed:", e)
        fs_ok = False
results["Flash"] = nvm_ok and fs_ok


# --- SD card ----------------------------------------------------------------
print("\n== SD card ==")
sd_ok = False
try:
    spi = board.SPI()
    sd = sdcardio.SDCard(spi, board.SD_CS, baudrate=4_000_000)
    print("  Card size: %.2f GB" % (sd.count() * 512 / 1e9))
    vfs = storage.VfsFat(sd)
    storage.mount(vfs, "/sd")
    st = os.statvfs("/sd")
    print("  Free: %.2f GB" % (st[0] * st[3] / 1e9))

    n = 256  # 1 MB
    path = "/sd/_sdtest.bin"
    base = bytearray(os.urandom(BLOCK))
    t0 = time.monotonic()
    with open(path, "wb") as f:
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            f.write(base)
    tw = time.monotonic() - t0
    sd_ok = True
    t0 = time.monotonic()
    with open(path, "rb") as f:
        for i in range(n):
            struct.pack_into(">I", base, 0, i)
            if f.read(BLOCK) != base:
                sd_ok = False
    tr = time.monotonic() - t0
    os.remove(path)
    print("  1 MB round trip: %s (write %.0f KB/s, read %.0f KB/s)"
          % ("OK" if sd_ok else "FAILED", 1024 / tw, 1024 / tr))
    storage.umount("/sd")
    sd.deinit()
except OSError as e:
    print("  SD error:", e, "(card inserted?)")
results["SD card"] = sd_ok


# --- Summary ----------------------------------------------------------------
print("\n== Summary ==")
for name, ok in results.items():
    print("  %-8s %s" % (name, "PASS" if ok else "FAIL"))
