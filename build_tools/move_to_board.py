"""
Move files from build folder to mainboard over the serial REPL (UART or USB serial).

Only files that are missing or different on the board (by size + CRC32) are sent.
Each upload goes to a temp file, is verified on the board, then replaces the old copy.

Requires: pip install pyserial
"""

import argparse
import os
import sys
import time
import zlib

from board_serial import Board, BoardError

# Never overwritten by a deploy: boot.py keeps the filesystem writable over serial.
PROTECTED = {"/boot.py", "/boot_out.txt", "/settings.toml"}
INFO_BATCH = 40


def to_remote(relative_path):
    return "/" + relative_path.replace(os.sep, "/")


def remote_dirname(path):
    return path.rsplit("/", 1)[0] or "/"


def batched_info(board, items):
    results = []
    for i in range(0, len(items), INFO_BATCH):
        results.extend(board.info(items[i : i + INFO_BATCH]))
    return results


def copy_folder(board, source_folder, cpy_version, show_identical_files=True):
    dirs, files = [], []
    for root, dir_names, file_names in os.walk(source_folder):
        dir_names.sort()
        for d in dir_names:
            dirs.append(to_remote(os.path.relpath(os.path.join(root, d), source_folder)))
        for f in sorted(file_names):
            local = os.path.join(root, f)
            remote = to_remote(os.path.relpath(local, source_folder))
            if remote in PROTECTED:
                print(f"Skipping {remote} (protected)")
                continue
            files.append((local, remote))

    # Directories: all of them on CPY 9+, only parents of files on CPY 8 (matches the old behaviour)
    wanted_dirs = set(remote_dirname(r) for _, r in files) - {"/"}
    if cpy_version >= 9:
        wanted_dirs |= set(dirs)
    wanted_dirs = sorted(wanted_dirs)
    for d, state in zip(wanted_dirs, batched_info(board, [(d, -1) for d in wanted_dirs])):
        if state is None:
            board.mkdirs(d)
            print(f"Created directory {d}")
        elif state != "d":
            print(f"Error: {d} exists on the board as a file; remove it and run again.")
            sys.exit(1)

    contents = {remote: open(local, "rb").read() for local, remote in files}
    states = batched_info(board, [(r, len(contents[r])) for _, r in files])

    sent_files = sent_bytes = 0
    start = time.monotonic()
    for (local, remote), state in zip(files, states):
        data = contents[remote]
        if state == "d":
            print(f"Error: {remote} is a directory on the board; skipped.")
            continue
        if state is not None and state == (len(data), zlib.crc32(data) & 0xFFFFFFFF):
            if show_identical_files:
                print(f"File {local} already exists and is identical.")
            continue

        retries = board.put(data, remote)
        note = f" ({retries} retr{'y' if retries == 1 else 'ies'})" if retries else ""
        verb = "Copied" if state is None else "Overwrote"
        print(f"{verb} {local} to {remote} [{len(data)} B]{note}")
        sent_files += 1
        sent_bytes += len(data)

    elapsed = time.monotonic() - start
    print(f"Sent {sent_files} file(s), {sent_bytes} bytes in {elapsed:.1f} s")

    # Attempt to remove the SD folder if in CPY 8
    if cpy_version == 8 and board.info([("/sd", -1)])[0] == "d":
        try:
            board.rmdir("/sd")
            print("Removed /sd")
        except BoardError as e:
            print(f"Could not remove /sd: {e}")

    # code.py would run instead of main.py
    if board.info([("/code.py", 0)])[0] is not None:
        try:
            board.remove("/code.py")
            print("Removed /code.py")
        except BoardError as e:
            print(f"Could not remove /code.py: {e}")


if __name__ == "__main__":
    # Parses command line arguments.
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-s",
        "--source_folder",
        type=str,
        default="build",
        help="Source folder path",
        required=False,
    )
    parser.add_argument("-p", "--port", help="Serial port of the board's REPL (or set ARGUS_PORT)")
    parser.add_argument("-b", "--baud", type=int, help="Serial baud rate (default 115200, or ARGUS_BAUD)")
    parser.add_argument("--chunk", type=int, default=1024, help="Bytes sent per REPL command (default 1024)")
    parser.add_argument("-q", "--quiet", action="store_true", help="Don't list files that are already identical")
    parser.add_argument("--no-reload", action="store_true", help="Leave the board at the REPL instead of restarting it")
    args = parser.parse_args()

    source_folder = args.source_folder
    if not os.path.isdir(source_folder):
        print(f"Error: Source folder '{source_folder}' does not exist. Run build.py first.")
        sys.exit(1)

    try:
        board = Board(args.port, args.baud, chunk=args.chunk)
    except BoardError as e:
        print(f"Error: {e}")
        sys.exit(1)

    print("SOURCE FOLDER: ", source_folder)
    print(f"DESTINATION: board on {board.port} @ {board.baud} baud")

    try:
        board.enter_raw()
        cpy_version = board.cpy_version()
        print(f"CircuitPython version: {cpy_version}")
        if board.is_readonly():
            print(
                "Error: the board's filesystem is read-only to CircuitPython.\n"
                "At the REPL run: import storage; storage.remount('/', readonly=False)\n"
                "and keep it that way across resets with a boot.py that calls storage.disable_usb_drive()\n"
                "and storage.remount('/', readonly=False)."
            )
            board.close()
            sys.exit(1)

        copy_folder(board, source_folder, cpy_version, show_identical_files=not args.quiet)
    except BoardError as e:
        print(f"Error talking to the board: {e}")
        board.close()
        sys.exit(1)

    board.close(reload=not args.no_reload)
    print("Board left at the REPL." if args.no_reload else "Board restarted (soft reboot).")