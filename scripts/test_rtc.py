import time

from hal.configuration import SATELLITE

PASS = "[PASS]"
FAIL = "[FAIL]"
WARN = "[WARN]"

passed = 0
failed = 0


def ok(msg):
    global passed
    passed += 1
    print(f"{PASS} {msg}")


def fail(msg):
    global failed
    failed += 1
    print(f"{FAIL} {msg}")


def section(title):
    print()
    print(f"── {title} ──")


# Boot

print("Booting ARGUS...")
SATELLITE.boot_sequence()
print(f"Boot Errors: {SATELLITE.ERRORS}")

# Test 1: RTC available after boot

section("TEST 1 – RTC available")
if SATELLITE.RTC_AVAILABLE:
    ok("RTC booted successfully")
else:
    fail("RTC not available after boot — all remaining tests skipped")
    print(f"\nRESULT: FAIL ({passed}/{passed + failed})")
    raise SystemExit

rtc = SATELLITE.RTC

# Test 2: Oscillator not disabled

section("TEST 2 – Oscillator enabled")
try:
    if rtc.disable_oscillator:
        fail("Oscillator is DISABLED (EOSC bit set) — clock is not running")
    else:
        ok("Oscillator enabled (EOSC bit clear)")
except Exception as e:
    fail(f"Could not read disable_oscillator: {e}")

# Test 3: Lost-power flag

section("TEST 3 – Lost-power (OSF) flag")
try:
    if rtc.lost_power:
        fail("lost_power flag SET — RTC lost power or time was never set")
    else:
        ok("lost_power flag clear — RTC maintained power")
except Exception as e:
    fail(f"Could not read lost_power: {e}")

# Test 4: Write a known time and read back

section("TEST 4 – Write / read-back")
TEST_TIME = time.struct_time((2025, 1, 1, 12, 0, 0, 2, 1, -1))
TEST_UNIX = 1735732800  # 2025-01-01 12:00:00 UTC

try:
    rtc.set_datetime(TEST_TIME)
    readback = rtc.datetime
    delta = abs(time.mktime(readback) - TEST_UNIX)
    print(f"  Written : {TEST_UNIX}")
    print(f"  Read    : {time.mktime(readback)}  delta={delta}s")
    if delta <= 2:
        ok(f"Read-back matches (delta {delta}s)")
    else:
        fail(f"Read-back differs by {delta}s")
except Exception as e:
    fail(f"Write/read raised: {e}")

# Test 5: Clock ticking

section("TEST 5 – Clock ticking (3 s)")
WAIT = 3
try:
    t0 = time.mktime(rtc.datetime)
    time.sleep(WAIT)
    t1 = time.mktime(rtc.datetime)
    advance = t1 - t0
    print(f"  Wall time: {WAIT}s  RTC advanced: {advance}s")
    if 1 <= advance <= WAIT + 2:
        ok(f"Clock is ticking ({advance}s in {WAIT}s)")
    elif advance == 0:
        fail("Clock did NOT advance — oscillator stopped?")
    else:
        fail(f"Unexpected advance: {advance}s")
except Exception as e:
    fail(f"Tick test raised: {e}")

# Test 6: Onboard temperature (DS3231 has an internal temp sensor used for oscillator compensation)

section("TEST 6 – DS3231 temperature sensor")
try:
    temp = rtc.temperature
    print(f"  DS3231 temp: {temp:.2f} °C")
    if -40.0 <= temp <= 85.0:
        ok(f"Temperature in rated range ({temp:.2f} °C)")
    else:
        fail(f"Temperature out of range: {temp:.2f} °C")
except Exception as e:
    fail(f"temperature read raised: {e}")

# Summary

section("Summary")
total = passed + failed
print(f"  Passed: {passed}  Failed: {failed}")
print()
if failed == 0:
    print(f"RESULT: PASS ({passed}/{total})")
else:
    print(f"RESULT: FAIL ({passed}/{total} passed, {failed} failed)")

print()
print("NOTE: DS3231 device_errors always returns [] — lost_power and battery_low")
print("are never surfaced through the normal HAL health check (unlike PCF8523).")
