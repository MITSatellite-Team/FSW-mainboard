"""
Build script for Argus
This script is only used for compiling .py files to .mpy files

The CircuitPython version and board ID decide which mpy-cross and drivers are used.
They are found in this order:
  1. --cpy-version / --board-id flags
  2. the serial REPL, when --port or ARGUS_PORT is given
  3. boot_out.txt on a mounted ARGUS drive
  4. the serial REPL on the only USB serial adapter attached
  5. defaults (CircuitPython 8, "Argus"), with a warning
"""

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys

try:
    import yaml

    _HAS_YAML = True
except Exception:
    yaml = None
    _HAS_YAML = False

system = platform.system()
machine = platform.machine().lower()

DEFAULT_CPY_VERSION = 8
DEFAULT_BOARD_ID = "Argus"


def get_board_path():
    if system == "Windows":
        BOARD_PATH = "D:\\"
    elif system == "Linux":
        username = os.getlogin()
        BOARD_PATH = f"/media/{username}/ARGUS"
    elif system == "Darwin":
        BOARD_PATH = "/Volumes/ARGUS"
    elif platform.node() == "raspberrypi":
        BOARD_PATH = "/mnt/mainboard"
    return BOARD_PATH


def get_circuitpython_version(BOARD_PATH):
    # First line looks like "Adafruit CircuitPython 10.3.1 on 2026-11-10; Argus4 with rp2350b"
    with open(os.path.join(BOARD_PATH, "boot_out.txt")) as boot:
        first_line = boot.readline()
    version = first_line.split(";")[0].split(" ")[-3]
    return int(version.split(".")[0])


def get_board_id(BOARD_PATH):
    with open(os.path.join(BOARD_PATH, "boot_out.txt")) as boot:
        lines = boot.readlines()
        if len(lines) > 1:
            second_line = lines[1].strip()
            board_id = second_line.split(":")[1].strip()
            return board_id
    # Default to compiling for Argus
    return DEFAULT_BOARD_ID


def detect_board_serial(port=None, baud=None):
    """Read (CircuitPython major version, board ID) over the serial REPL.
    Stops whatever is running and leaves the board at the REPL."""
    from board_serial import Board

    with Board(port, baud) as board:
        return board.cpy_version(), board.board_id(), f"serial REPL on {board.port}"


def resolve_board(args):
    cpy_version, board_id, source = args.cpy_version, args.board_id, "command line"

    if cpy_version is None or board_id is None:
        found = None
        port = args.port or os.environ.get("ARGUS_PORT")
        if port:
            try:
                found = detect_board_serial(port, args.baud)
            except Exception as e:
                print(f"Error: could not read the board over serial ({e})")
                print("Fix the connection, or pass --cpy-version and --board-id.")
                sys.exit(1)
        else:
            try:
                board_path = get_board_path()
            except Exception:
                board_path = None
            if board_path and os.path.exists(board_path):
                found = (
                    get_circuitpython_version(board_path),
                    get_board_id(board_path),
                    f"boot_out.txt on {board_path}",
                )
            else:
                try:
                    found = detect_board_serial(None, args.baud)
                except Exception as e:
                    print(f"No board detected ({e})")

        if found:
            if cpy_version is None:
                cpy_version = found[0]
            if board_id is None:
                board_id = found[1]
            source = found[2]

    if cpy_version is None:
        cpy_version = DEFAULT_CPY_VERSION
        source = "defaults"
        print("#" * 64)
        print(f"WARNING: board not detected; compiling for CircuitPython {cpy_version}.")
        print("If the board runs a different version the .mpy files won't load.")
        print("Pass --port or --cpy-version to be sure.")
        print("#" * 64)
    if board_id is None:
        board_id = DEFAULT_BOARD_ID

    return cpy_version, board_id, source


def get_mpy_cross_path(cpy_version, override=None):
    if override:
        return os.path.abspath(override)

    if system == "Windows":
        path = shutil.which("mpy-cross")
        if path:
            print(f"mpy-cross found at {path}")
            return path
        print("mpy-cross not found, please install with 'pip install mpy_cross'")
        sys.exit(-1)

    if platform.node() == "raspberrypi":
        name = "mpy-cross-rpi"
    elif system == "Darwin":
        name = "mpy-cross-macos"
    elif machine in ("aarch64", "arm64"):
        # Jetson Orin Nano goes here
        name = "mpy-cross-linux-aarch64"
    else:
        name = "mpy-cross"

    # CPY 8 binaries have no suffix; later versions are e.g. mpy-cross-cpy9, mpy-cross-macos-cpy10
    if cpy_version >= 9:
        name += f"-cpy{cpy_version}"
    return f"{os.getcwd()}/build_tools/{name}"


def get_commit_hash():
    try:
        commit_hash = os.popen("git rev-parse HEAD").read().strip()
        return commit_hash
    except Exception as e:
        print(f"Error getting commit hash: {e}")
        return None


def get_branch_name():
    try:
        branch_name = os.popen("git rev-parse --abbrev-ref HEAD").read().strip()
        return branch_name
    except Exception as e:
        print(f"Error getting branch name: {e}")
        return None


def check_clean_git():
    try:
        status = os.popen("git status --porcelain").read().strip()
        return status == ""
    except Exception as e:
        print(f"Error checking git status: {e}")
        return None


def parse_argus_id(value):
    try:
        return int(value, 0)
    except ValueError:
        raise argparse.ArgumentTypeError("Argus ID must be an integer (decimal or hex).")


def check_directory_location(source_folder, mpy_cross_path, cpy_version):
    if not os.path.exists(mpy_cross_path):
        raise FileNotFoundError(
            f"mpy-cross not found at {mpy_cross_path}. Download the CircuitPython {cpy_version}.x "
            "mpy-cross for this platform from "
            "https://adafruit-circuit-python.s3.amazonaws.com/index.html?prefix=bin/mpy-cross/ "
            "and save it under that name (chmod +x), or pass --mpy-cross."
        )

    if not os.path.exists(f"{source_folder}"):
        raise FileNotFoundError(f"Source folder {source_folder} not found")


def format_value(value):
    """
    Format a Python value as a string, using double quotes for strings.

    Args:
        value: The value to format

    Returns:
        String representation of the value
    """
    if isinstance(value, str):
        return (
            '"'
            + value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t")
            + '"'
        )
    else:
        result = repr(value)
        if len(result) >= 2 and result[0] == "'" and result[-1] == "'":
            return '"' + result[1:-1].replace('"', '\\"') + '"'
        return result


def coerce_hex(value):
    """
    Convert hex string literals (e.g., "0x1F") to integers; otherwise return the value unchanged.
    """
    if isinstance(value, str) and re.fullmatch(r"-?0[xX][0-9a-fA-F]+", value):
        try:
            return int(value, 16)
        except ValueError:
            pass
    return value


def generate_satellite_config(source_folder, use_flight_config=False, argus_id=None):
    """
    Generate satellite_config.py from ground.yaml or flight.yaml.

    Args:
        source_folder: Path to the flight source folder
        use_flight_config: If True, use flight.yaml; otherwise use ground.yaml
        argus_id: Argus spacecraft ID to inject when building the flight configuration
    """
    if not _HAS_YAML:
        raise ImportError("PyYAML is not installed; cannot generate satellite_config.py")

    config_file = "flight.yaml" if use_flight_config else "ground.yaml"
    yaml_path = os.path.join(source_folder, "configuration", config_file)

    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Configuration file {yaml_path} not found; cannot generate satellite_config.py")

    try:
        with open(yaml_path, "r") as yf:
            config_data = yaml.safe_load(yf)

        if config_data is None:
            print(f"Configuration file {yaml_path} is empty; skipping generation of satellite_config.py")
            return

        if use_flight_config:
            if argus_id is None:
                raise ValueError("Argus ID is required when compiling with the flight configuration.")
            comms_config = config_data.setdefault("comms", {})
            comms_config["ARGUS_ID"] = {"value": argus_id, "_const": True, "_hex": True}

        output_lines = [
            "# Auto-generated from " + config_file,
            "# Do not edit - changes will be overwritten by the build system.",
            "",
            "from micropython import const",
            "",
            "",
        ]

        for category_name, category_data in config_data.items():
            if category_data is None:
                continue

            class_name = category_name + "_config"
            output_lines.append(f"class {class_name}:")

            if not category_data or not isinstance(category_data, dict):
                output_lines.append("    pass")
                for _ in range(2):
                    output_lines.append("")
                continue

            for key, value in category_data.items():
                if isinstance(value, dict) and "value" in value:
                    actual_value = coerce_hex(value["value"])
                    use_const = value.get("_const", False)
                    use_hex = value.get("_hex", False)

                    if use_const and isinstance(actual_value, int):
                        if use_hex:
                            python_value = f"const(0x{actual_value:X})"
                        else:
                            python_value = f"const({actual_value})"
                    else:
                        python_value = format_value(actual_value)
                else:
                    # Simple value without metadata
                    python_value = format_value(coerce_hex(value))

                output_lines.append(f"    {key} = {python_value}")

            for _ in range(2):
                output_lines.append("")

        output_lines.pop()  # Remove last extra newline

        core_folder = os.path.join(source_folder, "core")
        if not os.path.exists(core_folder):
            os.makedirs(core_folder, exist_ok=True)

        config_py_path = os.path.join(core_folder, "satellite_config.py")
        with open(config_py_path, "w") as py_file:
            py_file.write("\n".join(output_lines))

        print(f"Generated {config_py_path} from {config_file}")

    except Exception as e:
        print(f"Failed to generate satellite_config.py from {yaml_path}: {e}")


def create_build(source_folder, flight_build, board_id, mpy_cross_path, git_branch, git_commit, git_clean):
    build_folder = "build/"
    if os.path.exists(build_folder):
        shutil.rmtree(build_folder)

    build_folder = os.path.join(build_folder, "lib/")

    os.makedirs(build_folder)

    for root, _, files in os.walk(source_folder):
        # only include drivers or drivers_PYC_V05
        if board_id.lower().startswith("argus") and os.path.relpath(root, source_folder).startswith("hal/drivers_PYC_V05"):
            print(f"Skipping {os.path.relpath(root, source_folder)}")
            continue
        elif board_id.lower().startswith("pycubed") and os.path.relpath(root, source_folder).startswith("hal/drivers"):
            print(f"Skipping {os.path.relpath(root, source_folder)}")
            continue
        if "splat_web" in root or "splat/examples" in root or "splat/docs" in root:  # ignore unecessary
            print(f"Skipping {os.path.relpath(root, source_folder)}")
            continue

        for file in files:
            if file.endswith(".py") or file.endswith(".mpy"):
                source_path = os.path.join(root, file)

                build_path = os.path.join(build_folder, os.path.relpath(source_path, source_folder))

                os.makedirs(os.path.dirname(build_path), exist_ok=True)
                shutil.copy2(source_path, build_path)
                print(f"Copied {source_path} to {build_path}")

                current_dir = os.getcwd()

                # Change directory to the build path folder
                os.chdir(os.path.dirname(build_path))

                if file == "main.py":
                    # rename main.py to main_module.py
                    os.rename("main.py", "main_module.py")
                    file_name = "main_module.py"
                else:
                    # Extract file name
                    file_name = os.path.basename(file)

                if file_name.endswith(".py"):
                    result = subprocess.run([mpy_cross_path, file_name, "-O3"])
                    if result.returncode != 0:
                        os.chdir(current_dir)
                        print(f"Error: mpy-cross failed on {source_path}; build stopped.")
                        sys.exit(1)

                    # Delete file python file once it has been compiled
                    os.remove(file_name)

                os.chdir(current_dir)

    # Create main.py file with single import statement "import main_module"
    build_folder = os.path.join(build_folder, "..")
    with open(os.path.join(build_folder, "main.py"), "w") as f:
        f.write('print("")\n')
        f.write('print("################################")\n')
        f.write(f'print("Build Config: {"FLIGHT" if flight_build else "GROUND"}")\n')
        if git_branch:
            f.write(f'print("Branch: {git_branch}")\n')
        if git_commit:
            if not git_clean:
                f.write('print("Warning: Uncommitted changes detected!")\n')
                f.write(f'print("DIRTY Commit: {git_commit}")\n')
            else:
                f.write(f'print("CLEAN Commit: {git_commit}")\n')
        f.write('print("################################")\n')
        f.write("import main_module\n")

    # Create SD folder
    os.makedirs(os.path.join(build_folder, "sd/"), exist_ok=True)


if __name__ == "__main__":
    # Parses command line arguments.
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-s",
        "--source_folder",
        type=str,
        default="flight",
        help="Source folder path",
        required=False,
    )
    parser.add_argument(
        "--flight",
        action="store_true",
        help="Use flight.yaml configuration instead of ground.yaml",
    )
    parser.add_argument(
        "--argus-id",
        type=parse_argus_id,
        help="Argus spacecraft ID (required when using --flight)",
    )
    parser.add_argument("-p", "--port", help="Serial port of the board's REPL (or set ARGUS_PORT)")
    parser.add_argument("-b", "--baud", type=int, help="Serial baud rate (default 115200, or ARGUS_BAUD)")
    parser.add_argument("--cpy-version", type=int, help="CircuitPython major version; skips board detection")
    parser.add_argument("--board-id", help="Board ID, e.g. argus4; skips board detection")
    parser.add_argument("--mpy-cross", help="Path to the mpy-cross binary to use")
    args = parser.parse_args()

    source_folder = args.source_folder

    flight_build = args.flight
    argus_id = args.argus_id

    if flight_build and argus_id is None:
        parser.error("--argus-id is required when compiling with the flight configuration.")

    if flight_build and argus_id not in (0, 1):
        parser.error("--argus-id should be 0 or 1")

    cpy_version, board_id, board_source = resolve_board(args)
    mpy_cross_path = get_mpy_cross_path(cpy_version, args.mpy_cross)
    check_directory_location(source_folder, mpy_cross_path, cpy_version)

    generate_satellite_config(source_folder, use_flight_config=flight_build, argus_id=argus_id)

    git_branch = get_branch_name()
    git_commit = get_commit_hash()
    git_clean = check_clean_git()

    if git_branch:
        print(f"Branch: {git_branch}")
    if git_commit:
        if not git_clean:
            print("Warning: Uncommitted changes detected!")
            print(f"DIRTY Commit: {git_commit}")
        else:
            print(f"CLEAN Commit: {git_commit}")

    print(f"CircuitPython version: {cpy_version} (from {board_source})")
    print(f"Board ID: {board_id}")
    print(f"mpy-cross: {mpy_cross_path}")
    if flight_build:
        print(f"Argus ID: {argus_id}")

    create_build(source_folder, flight_build, board_id, mpy_cross_path, git_branch, git_commit, git_clean)