# LEONIRD Flight Software — MIT Satellite Team

Flight software for the **LEONIRD** mission by the [MIT Satellite Team](https://mitsatellite.org). This firmware is being tested and validated on a **High Altitude Balloon (HAB) flight** ahead of the primary mission.

> This repository is a fork of the [CMU Argus 2 flight software](https://github.com/cmu-argus-2/FSW-mainboard), which provided the initial hardware platform and software architecture. We gratefully acknowledge their work as the foundation for this project.

## Hardware

Target hardware: **Argus v4.2** mainboard.

## Architecture

See [High-Level Architecture](docs/architecture.md)

## Installation

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
git submodule init
git submodule update
sh install.sh
```

Alternatively, if you use Nix, a dev shell is provided:
```bash
nix develop
```

## Build and Execution

### With mainboard

Build and deploy to the connected board:
```bash
./run.sh
```

This compiles flight software to `.mpy` files and transfers them to the board. Supported on Linux, macOS, Windows, and RPi.

For a flight build with a specific Argus ID:
```bash
python3 build_tools/build.py --flight --argus-id <id>
```

### Without mainboard

To run the emulator:
```bash
source .venv/bin/activate
./run.sh emulate
```

To run the simulator:
```bash
source .venv/bin/activate
./run.sh simulate
```

See `sil/README.md` for advanced simulator configuration.

## Common Problems

### Board Stuck in Read-Only Mode

Access REPL and run:
```python
import storage
storage.erase_filesystem()
```

### Reflashing the board

With physical button access, use the on-board bootloader buttons — see [firmware guide](firmware/README.md).

Without button access, from REPL:
```python
import microcontroller
microcontroller.on_next_reset(microcontroller.RunMode.UF2)
microcontroller.reset()
```
**Do not put this in `main.py` — the board will boot-loop into the bootloader.**

### Nuking the board

If the board is stuck in bootloader or behaving unexpectedly, flash the appropriate nuke firmware from the `firmware/` folder. See [firmware guide](firmware/README.md).

### Can't deploy to board

If you see `Error: Destination folder does not exist. Is the board connected?`:
1. Confirm the board is powered (if via USB, check VSYS/GND or inhibitor jumpers).
2. Confirm the board drive is named `ARGUS` (new boards default to `CIRCUITPY`).
3. Check the USB connector on the mainboard.

### Data Handler errors

If the data structure has changed, wipe the SD card using the provided script.

### Mainboard brown-out / random restarts

Radio cannot run without adequate power supply. If on USB power, disable some peripherals. Burn wires can also cause brown-outs — lower their drive strength.
