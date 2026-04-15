"""
Radio transmit interval test.

Boots the satellite HAL and repeatedly transmits a simple counter message
("TX: 0", "TX: 1", ...) at a short interval so you can verify the radio is
downlinking without waiting the normal 30-second heartbeat period.

Usage:
    Copy this script to the board and run it directly, e.g. via mpremote:
        mpremote run scripts/radio_tx_interval_test.py

Adjust TX_INTERVAL_S below before copying if you want a different interval.
"""

import time

from hal.configuration import SATELLITE

# --- Configuration ---
TX_INTERVAL_S = 1  # seconds between transmissions (change to 2 if preferred)
# ---------------------


def main():
    print("Booting ARGUS-1...")
    SATELLITE.boot_sequence()
    print(f"Boot complete. Errors: {SATELLITE.ERRORS}")

    print(f"Starting radio TX test — transmitting every {TX_INTERVAL_S}s. Ctrl-C to stop.")
    tx_count = 0

    while True:
        try:
            msg = f"TX: {tx_count}"
            packet = bytearray(msg.encode("utf-8"))

            print(f"Sending: {msg}")
            SATELLITE.RADIO.send(packet)
            tx_count += 1

            # Also check for any incoming packets while we wait
            time.sleep(TX_INTERVAL_S)

            if SATELLITE.RADIO.RX_available():
                rx, _ = SATELLITE.RADIO.recv(len=0, timeout_en=True, timeout_ms=1000)
                print(f"  RX: {rx}")

        except KeyboardInterrupt:
            print(f"\nDone. Transmitted {tx_count} packet(s).")
            break


main()
