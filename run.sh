# Detect the operating system
OS=$(uname -s)
export ARGUS_ROOT=$(pwd)

# Extract optional Argus ID argument and rebuild the positional args list
ARGUS_ID_ARG=""
REMAINING_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --argus-id)
            if [[ -n "${2-}" ]]; then
                ARGUS_ID_ARG="--argus-id $2"
                shift 2
                continue
            else
                echo "Error: --argus-id requires a value."
                exit 1
            fi
            ;;
        --argus-id=*)
            ARGUS_ID_ARG="--argus-id ${1#*=}"
            shift
            continue
            ;;
        *)
            REMAINING_ARGS+=("$1")
            shift
            ;;
    esac
done
set -- "${REMAINING_ARGS[@]}"

# Support positional Argus ID for flight builds: "./run.sh flight 0"
if [[ -z "$ARGUS_ID_ARG" && "$1" == "flight" && -n "${2-}" ]]; then
    if [[ "$2" =~ ^[0-9]+$ ]]; then
        ARGUS_ID_ARG="--argus-id $2"
        # Drop the numeric ID from the positional arguments
        set -- "$1" "${@:3}"
    fi
fi

# Set the correct mpy-cross executable based on the OS
if [ "$OS" == "Linux" ]; then
    MPY_EXEC="mpy-cross"
elif [ "$OS" == "Darwin" ]; then
    MPY_EXEC="mpy-cross-macos"
elif [[ "$OS" == *_NT* ]]; then
    MPY_EXEC="mpy-cross"
else
    echo "Unsupported OS: $OS"
    exit 1
fi

# Make the mpy-cross executables (every CircuitPython version) runnable
if [[ ! "$OS" == *_NT* ]]; then
    for f in build_tools/$MPY_EXEC*; do
        [ -f "$f" ] && chmod +x "$f" && echo "$(basename "$f") is now executable"
    done
fi

# Detect the appropriate Python command
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo "Python is not installed or not found in PATH."
    exit 1
fi

echo "Using Python command: $PYTHON_CMD"

# Find the board's serial REPL once (skipped if ARGUS_PORT is already set)
find_board_port() {
    if [[ -z "${ARGUS_PORT-}" ]]; then
        ARGUS_PORT=$($PYTHON_CMD build_tools/board_serial.py) || exit 1
    fi
    export ARGUS_PORT
    echo "Using serial port: $ARGUS_PORT"
}

export ARGUS_SIMULATION_FLAG=0

# Check for flight flag
FLIGHT_FLAG=""
if [[ " $@ " =~ " flight " ]]; then
    FLIGHT_FLAG="--flight"
    echo "Using flight configuration (flight.yaml)"
else
    echo "Using ground configuration (ground.yaml)"
fi

if [[ -n "$FLIGHT_FLAG" && -z "$ARGUS_ID_ARG" ]]; then
    echo "Error: --argus-id is required when building with the flight configuration."
    exit 1
fi

BUILD_ARGS=($FLIGHT_FLAG)
if [[ -n "$ARGUS_ID_ARG" ]]; then
    BUILD_ARGS+=($ARGUS_ID_ARG)
fi

if [[ -z $1 ]]; then
    find_board_port
    $PYTHON_CMD build_tools/build.py "${BUILD_ARGS[@]}" || exit 1
    $PYTHON_CMD build_tools/move_to_board.py
elif [ "$1" == "emulate" ]; then
    $PYTHON_CMD build_tools/build-emulator.py $FLIGHT_FLAG
    cd build/ && $PYTHON_CMD main.py
    cd -
elif [ "$1" == "emulate-profile" ]; then
    $PYTHON_CMD build_tools/build-emulator.py $FLIGHT_FLAG
    cd build/ && mprof run --python main.py
    mprof plot -o output.png
    cd -
elif [ "$1" == "simulate" ]; then
    export ARGUS_SIMULATION_FLAG=1
    export SIM_REAL_SPEEDUP=275
    echo "ARGUS_SIMULATION_FLAG set to 1 for simulation mode."
    if [[ -z $2 ]]; then
        echo "Running simulation with random trial number."
    elif [[ -z $3 ]]; then
        export ARGUS_SIMULATION_TRIAL=$2
        echo "Running simulation with trial number $2"
    elif [[ -z $4 ]]; then
        export ARGUS_SIMULATION_TRIAL=$2
        export ARGUS_SIMULATION_DATE=$3
        echo "Running simulation with trial number $2 at date $3"
    else
        export ARGUS_SIMULATION_TRIAL=$2
        export ARGUS_SIMULATION_DATE=$3
        export ARGUS_SIMULATION_SET_NAME=$4
        echo "Running simulation from set $4 with trial number $2 at date $3"
    fi
    $PYTHON_CMD build_tools/build-emulator.py $FLIGHT_FLAG
    cd build/ && rm -rf sd && $PYTHON_CMD main.py
    cd -
elif [ "$1" == "flight" ]; then
    # If --flight is the only argument, build with flight config
    find_board_port
    $PYTHON_CMD build_tools/build.py "${BUILD_ARGS[@]}" || exit 1
    $PYTHON_CMD build_tools/move_to_board.py
else
    # Any other argument is the board's serial port, e.g. ./run.sh /dev/ttyUSB0
    export ARGUS_PORT="$1"
    find_board_port
    $PYTHON_CMD build_tools/build.py "${BUILD_ARGS[@]}" || exit 1
    $PYTHON_CMD build_tools/move_to_board.py
fi