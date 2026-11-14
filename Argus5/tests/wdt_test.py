"""Argus5 external watchdog test (TPS3435 U4 -> Q1 -> RUN).

Save as code.py. The test runs when the file is saved or on Ctrl-D, and
reports the result after the board resets itself.

  Part 1: WDT_EN on, WDI kicked every second for 15 s -> must NOT reset.
  Part 2: kicking stops -> watchdog must reset the RP2350 via RUN after
          about 4 s (100 nF on CWD, x8 scaling from SET0 high).

Expected timing from the TPS3435-Q1 datasheet: 0.495 s x 8 = ~4.0 s,
+/-20% plus capacitor tolerance, so roughly 3.2 s to 4.8 s.
"""
import struct
import time

import board
import digitalio
import microcontroller
import supervisor

MAGIC = b"WDT1"
nvm = microcontroller.nvm


def save_state(elapsed):
    nvm[0:6] = MAGIC + struct.pack(">H", int(elapsed * 10))


def load_state():
    if nvm[0:4] != MAGIC:
        return None
    return struct.unpack(">H", nvm[4:6])[0] / 10


def clear_state():
    nvm[0:6] = b"\x00" * 6


reset_reason = microcontroller.cpu.reset_reason
run_reason = supervisor.runtime.run_reason
print("\nReset reason:", reset_reason)
print("Run reason:  ", run_reason)

# --- After a hardware reset: report, never re-run (avoids reset loops) -------
if run_reason == supervisor.RunReason.STARTUP:
    last = load_state()
    if last is not None:
        clear_state()
        if reset_reason == microcontroller.ResetReason.RESET_PIN:
            print("\nPASS: watchdog reset the MCU through RUN")
            print("Last time logged before reset: %.1f s after the final kick" % last)
            print("(actual timeout is up to 0.5 s later; expect ~3.2-4.8 s)")
        else:
            print("\nFAIL?: test was armed but reset reason is", reset_reason)
            print("(a power cycle during the test also causes this)")
    else:
        print("\nWatchdog test is idle after a hardware reset.")
        print("Save this file or press Ctrl-D in the REPL to run it.")
    while True:
        time.sleep(1)

# --- Run the test -----------------------------------------------------------
print("\n*** This test resets the board. Eject CIRCUITPY first if files are open. ***")

wdi = digitalio.DigitalInOut(board.WDT_WDI)
wdi.switch_to_output(value=False)


def kick():
    wdi.value = True
    wdi.value = False  # Python toggle takes several us; TPS3435 needs >= 500 ns


# The TPS3435 runs whenever +3V3_M is up and may be mid-way through
# asserting WDO from earlier unkicked time. Get it into a clean monitoring
# frame before connecting WDO to RUN.
kick()
time.sleep(0.1)  # longer than the ~50 ms WDO pulse (10 nF on CRST)
kick()

wdt_en = digitalio.DigitalInOut(board.WDT_EN)
wdt_en.switch_to_output(value=True)
print("WDT_EN high: WDO now connected to RUN")

print("\nPart 1: kicking every 1 s for 15 s (board must stay up)")
for s in range(15):
    kick()
    time.sleep(1)
    print("  %2d s alive" % (s + 1))
print("Part 1 PASS: no reset while kicked")

print("\nPart 2: stopped kicking. Expect a reset in ~3.2-4.8 s...")
save_state(0)
t0 = time.monotonic()
while True:
    elapsed = time.monotonic() - t0
    if elapsed > 15:
        break
    save_state(elapsed)
    print("  %.1f s" % elapsed)
    time.sleep(0.5)

# Still running: the reset never came.
wdt_en.value = False
clear_state()
print("\nFAIL: no reset after 15 s without kicks.")
print("Check: Q1 gate drive (WDT_EN), R12 fitted, WDO activity on a scope,")
print("and that C23 (CWD) is 100 nF.")
