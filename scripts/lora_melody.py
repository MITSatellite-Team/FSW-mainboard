"""
LoRa Melody Demo - Twinkle Twinkle Little Star

Transmits a continuous carrier wave and hops between frequencies
corresponding to musical notes. On an SDR in USB mode, the offset
from your tuned center frequency is heard as an audio tone.

How to listen:
  1. Open SDR software (SDR#, GQRX, SDRangel, etc.)
  2. Tune to CENTER_FREQ_MHZ (default 915.0 MHz)
  3. Set demodulation mode to USB (Upper Sideband)
  4. Zoom in so you can see ~1 kHz of bandwidth
  5. Run this script — you should see a dot jumping around on
     the waterfall and hear Twinkle Twinkle Little Star

On the REPL:
    exec(open("lora_melody.py").read())
"""

import time
from hal.configuration import SATELLITE

# ── Config ───────────────────────────────────────────────────────────
CENTER_FREQ_MHZ = 435.0   # Tune your SDR to exactly this in USB mode
BPM             = 108     # Tempo
GAP_S           = 0.04    # Brief silence between notes (articulation)
# ─────────────────────────────────────────────────────────────────────

# Note frequencies in Hz (audio offset above center carrier)
# In USB mode: signal at (center + N) MHz sounds like N Hz
# Higher octaves = larger Hz gaps between notes = easier to distinguish
#   C4 octave: notes ~30 Hz apart  (muddy)
#   C6 octave: notes ~120 Hz apart (clear)
#   C7 octave: notes ~240 Hz apart (very clear)
NOTES = {
    'C4': 261.63,  'D4': 293.66,  'E4': 329.63,  'F4': 349.23,
    'G4': 392.00,  'A4': 440.00,  'B4': 493.88,
    'C5': 523.25,  'D5': 587.33,  'E5': 659.25,  'F5': 698.46,
    'G5': 783.99,  'A5': 880.00,  'B5': 987.77,
    'C6': 1046.50, 'D6': 1174.66, 'E6': 1318.51, 'F6': 1396.91,
    'G6': 1567.98, 'A6': 1760.00, 'B6': 1975.53,
    'C7': 2093.00, 'D7': 2349.32, 'E7': 2637.02, 'F7': 2793.83,
    'G7': 3135.96, 'A7': 3520.00, 'B7': 3951.07,
    'C8': 4186.01,
    'R':  None,
}

BEAT = 60.0 / BPM  # quarter note duration in seconds

# Twinkle Twinkle Little Star
# Verse 1: C6 (~1000-1760 Hz audio) — notes ~120 Hz apart, very clear
# Verse 2: C7 (~2000-3520 Hz audio) — notes ~240 Hz apart, even clearer
MELODY = [
    # Verse 1 — C6 octave
    ('C6', 1), ('C6', 1), ('G6', 1), ('G6', 1),
    ('A6', 1), ('A6', 1), ('G6', 2), ('R', 0.5),

    ('F6', 1), ('F6', 1), ('E6', 1), ('E6', 1),
    ('D6', 1), ('D6', 1), ('C6', 2), ('R', 0.5),

    ('G6', 1), ('G6', 1), ('F6', 1), ('F6', 1),
    ('E6', 1), ('E6', 1), ('D6', 2), ('R', 0.5),

    ('G6', 1), ('G6', 1), ('F6', 1), ('F6', 1),
    ('E6', 1), ('E6', 1), ('D6', 2), ('R', 0.5),

    ('C6', 1), ('C6', 1), ('G6', 1), ('G6', 1),
    ('A6', 1), ('A6', 1), ('G6', 2), ('R', 0.5),

    ('F6', 1), ('F6', 1), ('E6', 1), ('E6', 1),
    ('D6', 1), ('D6', 1), ('C6', 2), ('R', 1),

    # Verse 2 — C7 octave (notes 240 Hz apart — should sound very distinct)
    ('C7', 1), ('C7', 1), ('G7', 1), ('G7', 1),
    ('A7', 1), ('A7', 1), ('G7', 2), ('R', 0.5),

    ('F7', 1), ('F7', 1), ('E7', 1), ('E7', 1),
    ('D7', 1), ('D7', 1), ('C7', 2), ('R', 0.5),

    ('G7', 1), ('G7', 1), ('F7', 1), ('F7', 1),
    ('E7', 1), ('E7', 1), ('D7', 2), ('R', 0.5),

    ('G7', 1), ('G7', 1), ('F7', 1), ('F7', 1),
    ('E7', 1), ('E7', 1), ('D7', 2), ('R', 0.5),

    ('C7', 1), ('C7', 1), ('G7', 1), ('G7', 1),
    ('A7', 1), ('A7', 1), ('G7', 2), ('R', 0.5),

    ('F7', 1), ('F7', 1), ('E7', 1), ('E7', 1),
    ('D7', 1), ('D7', 1), ('C7', 2),
]

# Two-octave sweep C6→C8 and back — stays in the audible range where notes are distinct
TUNE_UP = [
    ('C6', 0.3), ('D6', 0.3), ('E6', 0.3), ('F6', 0.3), ('G6', 0.3), ('A6', 0.3), ('B6', 0.3),
    ('C7', 0.3), ('D7', 0.3), ('E7', 0.3), ('F7', 0.3), ('G7', 0.3), ('A7', 0.3), ('B7', 0.3),
    ('C8', 0.6),
    ('B7', 0.3), ('A7', 0.3), ('G7', 0.3), ('F7', 0.3), ('E7', 0.3), ('D7', 0.3), ('C7', 0.3),
    ('B6', 0.3), ('A6', 0.3), ('G6', 0.3), ('F6', 0.3), ('E6', 0.3), ('D6', 0.3), ('C6', 0.6),
    ('R', 0.5),
]


TUNING_NOTE = 'A6'  # 1760 Hz — clear, mid-range, easy to find on the waterfall


def play(radio, note_hz, duration):
    """Transmit CW at center+note_hz for duration seconds, then silence for GAP_S."""
    radio.standby()
    if note_hz is not None:
        radio.setFrequencyRaw(CENTER_FREQ_MHZ + note_hz / 1_000_000)
        radio.transmitDirect()
        time.sleep(max(0.0, duration - GAP_S))
        radio.standby()
    time.sleep(GAP_S)


def play_sequence(radio, sequence):
    for note_name, beats in sequence:
        play(radio, NOTES[note_name], beats * BEAT)


def tuning_tone(radio):
    """Hold a single tone until Ctrl-C, then proceed to the melody."""
    note_hz = NOTES[TUNING_NOTE]
    freq_mhz = CENTER_FREQ_MHZ + note_hz / 1_000_000
    radio.setFrequencyRaw(freq_mhz)
    radio.transmitDirect()
    print(f"Tuning tone: {TUNING_NOTE} ({note_hz:.0f} Hz above center)")
    print(f"You should see a carrier {note_hz:.0f} Hz above {CENTER_FREQ_MHZ} MHz on the waterfall.")
    print("Press Ctrl-C to stop tuning and play the melody.\n")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        radio.standby()
        print("Tuning done.\n")


def main():
    print("Booting...")
    SATELLITE.boot_sequence()
    print(f"Boot complete. Errors: {SATELLITE.ERRORS}")

    radio = SATELLITE.RADIO
    radio.tx_en.value = True
    radio.rx_en.value = False

    print(f"\nTune your SDR to {CENTER_FREQ_MHZ} MHz (434.9997 MHz recommended) in USB mode.\n")

    tuning_tone(radio)

    print(">> Scale sweep")
    play_sequence(radio, TUNE_UP)
    time.sleep(0.5)

    print(">> Twinkle Twinkle Little Star")
    play_sequence(radio, MELODY)

    radio.standby()
    radio.tx_en.value = False
    radio.rx_en.value = True
    print("Done!")


main()
