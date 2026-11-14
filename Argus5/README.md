# Argus firmware bundle: CircuitPython 9.2.9

## Contents

| Path | What it is |
|---|---|
| `Argus5/argus5-circuitpython-9.2.9-en_US.uf2` | Firmware for Argus5 (RP2350B + 8 MB PSRAM) |
| `Argus5/board/` | Board definition: `ports/raspberrypi/boards/argus5/` |
| `Argus3/argus3-circuitpython-9.2.9-en_US.uf2` | Firmware for Argus3 |
| `Argus3/board/` | Board definition: `ports/raspberrypi/boards/argus3/` |
| `board_test.py` | Full Argus5 self-test (save as `code.py`) |
| `tests/` | Single-subsystem scripts for deeper debugging |

## Build details

- CircuitPython tag `9.2.9` (commit `67d8d36`), translation `en_US`
- Toolchain: `arm-none-eabi-gcc` 13.2.1 (Ubuntu `gcc-arm-none-eabi`)
- Frozen libraries: BusDevice, Register, SD, NeoPixel
- Argus5 firmware: 852,064 B (81.6% of the 1020 KB firmware region)

SHA-256:
```
d4f775cb5747654979cd2753b382e6f2d48b8622128bb49c604d05f5fd93ffa5  Argus5/argus5-circuitpython-9.2.9-en_US.uf2
494c50d9b5582d4ee43f37cc0816515558eaed6b1a1cf9176cfe3952b85bc075  Argus3/argus3-circuitpython-9.2.9-en_US.uf2
```

## Flashing

Hold BOOTSEL while plugging in or resetting the board, then copy the `.uf2` onto
the drive that appears. The board reboots as the `ARGUS` drive.

## Rebuilding

```sh
git clone --depth 1 --branch 9.2.9 https://github.com/adafruit/circuitpython.git
cd circuitpython
pip install -r requirements-dev.txt
cp -r <bundle>/Argus5/board ports/raspberrypi/boards/argus5
cd ports/raspberrypi
make fetch-port-submodules
make BOARD=argus5 TRANSLATION=en_US
# -> build-argus5/firmware.uf2
```

## Argus5 hardware notes (from bring-up)

- **USB VID/PID:** both boards use `0x2E8A:0x10A3` (Raspberry Pi VID). Request
  a separate PID from Raspberry Pi before distributing or upstreaming.
- **Power enables:** `PERIPH_PWR_EN` feeds +3V3_P, which supplies the INA226s,
  the NeoPixel, and the input of every sub-rail switch (MAG, IMU, RTC, GPS, SD).
  Each sub-rail also needs its own enable. All enables have 100k pull-downs,
  so everything turns off when `code.py` exits or the board resets.
- **PSRAM:** APS6404L on GPIO0 (XIP_CS1). During large buffer tests, disable
  the GC (`gc.disable()`); otherwise each collection scans the buffer over QSPI.
- **INA226 addresses:** 0x40 3V3_MAIN, 0x41 3V3_PERIPH, 0x44 LoRa rail;
  10 mOhm shunts.
- **RTC (RV-3032, 0x51):** answers about 91 ms after `RTC_EN` (EEPROM refresh),
  so poll instead of using a fixed delay. Backup switchover is OFF from the
  factory; enable it once (see `tests/power_rtc.py`) so the BT2 coin cell keeps time.
- **IMU / magnetometer:** on shared SPI0; IMU uses SPI mode 3, magnetometer
  mode 0. Disable the IMU's I2C interface (CTRL4_C) on the shared bus.
- **SD card:** 4 MHz default; 1k series resistors on CLK/MOSI.
- **LoRa (E22P-433M30S, SX1268):** TCXO 1.8 V via DIO3; RXEN/TXEN active-high
  and MCU-driven (never both high). Rail is about 5.1 V (needs >= 5.0 V for full
  power). The SX1268 setting is the drive into the external PA: start low;
  E22 M30S drivers use +22 dBm for about 30 dBm out.
- **GPS (PX1120S):** UART1 at 115200 baud; 1PPS on GPIO21 only with a 3D fix.
  J12 has no antenna bias, so use a passive antenna. BT1 (MS621FE) is
  rechargeable from the GPS rail.
- **Watchdog (TPS3435):** always running; reaches RUN only while `WDT_EN` is
  high. Timeout about 4.0 s (100 nF, x8); kick about once per second.

## board_test.py

Save as `code.py`. Defaults are safe: nothing transmits and nothing resets.
Opt in at the top of the file:

- `RUN_TX_TEST = True` and set `CALLSIGN`: short low-drive carrier plus one packet.
- `RUN_WATCHDOG_TEST = True`: resets the board after the summary; the result
  prints on the next boot.

The GPS stage waits up to `GPS_FIX_TIMEOUT` (90 s) for a fix. Indoors this
shows WARN, not FAIL.

Not covered: backplane BUS_IO pins (need the mating board) and the buttons.
