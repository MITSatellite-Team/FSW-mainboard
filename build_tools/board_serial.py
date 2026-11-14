"""
Minimal client for a CircuitPython board's serial REPL (UART console or USB serial).

Uses the raw REPL (Ctrl-A) so commands run without echo and tracebacks come back
separately from normal output. Raw-paste mode (flow controlled) is used when the
firmware supports it; otherwise commands are sent in paced blocks. Every uploaded
file is CRC32-checked on the board before it replaces the old copy.

Requires: pip install pyserial
"""

import ast
import base64
import os
import struct
import time
import zlib

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None

DEFAULT_BAUD = 115200
RAW_BANNER = b"raw REPL; CTRL-B to exit\r\n>"
# The board answers protocol handshakes immediately; a longer wait means a byte was lost.
QUICK = 1.5

# Helpers defined on the board once per session. Only uses modules CircuitPython ships.
_BOARD_HELPERS = r'''
import os, binascii
try:
    binascii.crc32
    def _crcu(c, b):
        return binascii.crc32(b, c)
except AttributeError:
    _T = []
    for _i in range(256):
        _c = _i
        for _j in range(8):
            _c = (_c >> 1) ^ 0xEDB88320 if _c & 1 else _c >> 1
        _T.append(_c)
    def _crcu(c, b):
        c ^= 0xFFFFFFFF
        for x in b:
            c = _T[(c ^ x) & 0xFF] ^ (c >> 8)
        return c ^ 0xFFFFFFFF
def _crc(p):
    c = 0
    with open(p, "rb") as f:
        while True:
            b = f.read(1024)
            if not b:
                break
            c = _crcu(c, b)
    return c & 0xFFFFFFFF
def _st(p):
    try:
        s = os.stat(p)
    except OSError:
        return None
    return ((s[0] & 0x4000) != 0, s[6])
def _info(items):
    r = []
    for p, n in items:
        s = _st(p)
        if s is None:
            r.append(None)
        elif s[0]:
            r.append("d")
        elif s[1] != n:
            r.append((s[1], None))
        else:
            r.append((s[1], _crc(p)))
    print(repr(r))
def _sync():
    # Write cached flash sectors now, while the host is waiting for this command to finish.
    # Left for later, the flush runs in the background with interrupts off and drops UART bytes.
    if _f:
        _f.flush()
    try:
        os.sync()
    except AttributeError:
        pass
def _mkdirs(p):
    cur = ""
    for part in p.strip("/").split("/"):
        cur += "/" + part
        s = _st(cur)
        if s is None:
            os.mkdir(cur)
        elif not s[0]:
            raise OSError("not a directory: " + cur)
    _sync()
_f = None
def _open(p):
    global _f
    _f = open(p, "wb")
def _w(b, off, crc):
    d = binascii.a2b_base64(b)
    if _crcu(off, d) & 0xFFFFFFFF != crc:
        raise ValueError("chunk crc mismatch")
    if _f is None:
        raise RuntimeError("no file open")
    _f.seek(off)
    _f.write(d)
    _sync()
def _rm(p, is_dir=False):
    if _st(p) is not None:
        (os.rmdir if is_dir else os.remove)(p)
    _sync()
def _close():
    global _f
    if _f:
        _f.close()
    _f = None
    _sync()
def _finish(tmp, dst, n, crc):
    s = _st(tmp)
    if s is None or s[1] != n or _crc(tmp) != crc:
        try:
            os.remove(tmp)
        except OSError:
            pass
        _sync()
        print(False)
        return
    if _st(dst) is not None:
        os.remove(dst)
    os.rename(tmp, dst)
    _sync()
    print(True)
'''


def _helper_blocks():
    """Split the helpers into top-level blocks so each one is small to (re)send."""
    blocks, cur = [], []
    for line in _BOARD_HELPERS.strip("\n").split("\n"):
        if cur and line[:1] not in (" ", "") and not line.startswith("except"):
            blocks.append("\n".join(cur))
            cur = []
        cur.append(line)
    blocks.append("\n".join(cur))
    return blocks


def _verified(code):
    """Wrap code so the board runs it only if it arrived intact. A dropped byte can turn
    code into different code that still parses (e.g. a changed name), so plain commands
    aren't safe over a lossy link."""
    raw = code.encode()
    return (
        "import binascii\n"
        f"_s=binascii.a2b_base64({base64.b64encode(raw)!r})\n"
        f"if len(_s)!={len(raw)} or sum(_s)!={sum(raw)} or "
        f"(hasattr(binascii,'crc32') and binascii.crc32(_s)!={zlib.crc32(raw)}):\n"
        " raise ValueError('command arrived damaged')\n"
        "exec(_s.decode())\n"
    )


class BoardError(Exception):
    """Communication problem with the board."""


class RemoteError(BoardError):
    """Code on the board raised an exception (the message is the traceback)."""


def port_users(device):
    """Linux only, best effort: other processes that have this serial port open.
    Two programs on one port split the incoming data, so the REPL replies get lost."""
    users = []
    if not os.path.isdir("/proc"):
        return users
    target = os.path.realpath(device)
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    with open(f"/proc/{pid}/comm") as f:
                        users.append(f"{f.read().strip()} (pid {pid})")
                    break
        except OSError:
            continue
    return users


def _probe(device, baud):
    """(True, "") if a CircuitPython REPL answers on this port, else (False, reason)."""
    try:
        board = Board(device, baud)
    except BoardError as e:
        return False, str(e)
    try:
        board.enter_raw(attempts=2)
        return True, ""
    except BoardError as e:
        return False, str(e)
    finally:
        board.close()


def find_port(baud=DEFAULT_BAUD, log=print):
    """Return the serial port with a CircuitPython REPL on it, or raise with a list of choices."""
    if serial is None:
        raise BoardError("pyserial is not installed: pip install pyserial")
    ports = [p for p in list_ports.comports() if p.vid is not None]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise BoardError(
            "No USB serial adapter found. Pass --port (or set ARGUS_PORT), "
            "e.g. /dev/ttyUSB0, /dev/serial0 or COM3."
        )
    log(f"Several serial ports found; looking for the CircuitPython REPL at {baud} baud...")
    hits = []
    for p in ports:
        users = port_users(p.device)
        if users:
            log(f"  {p.device} is open in {', '.join(users)}; close it to use that port")
        else:
            ok, reason = _probe(p.device, baud)
            if ok:
                hits.append(p)
            else:
                log(f"  {p.device}: {reason}")
    if len(hits) == 1:
        log(f"Found the board on {hits[0].device}. Set ARGUS_PORT={hits[0].device} to skip this check.")
        return hits[0].device
    listing = "\n".join(f"  {p.device}  {p.description}" for p in ports)
    reason = "none of them answered" if not hits else "more than one answered"
    raise BoardError(f"Several serial ports found and {reason}; choose one with --port (or ARGUS_PORT):\n{listing}")


class Board:
    def __init__(self, port=None, baud=None, chunk=512, timeout=10, raw_paste=False):
        if serial is None:
            raise BoardError("pyserial is not installed: pip install pyserial")
        self.baud = int(baud or os.environ.get("ARGUS_BAUD") or DEFAULT_BAUD)
        self.port = port or os.environ.get("ARGUS_PORT") or find_port(self.baud)
        self.chunk = chunk
        self.timeout = timeout
        self._rx = bytearray()
        self._raw = False
        # Raw-paste adds flow control, but over a lossy UART a lost byte stalls it until a
        # timeout; the plain raw REPL reports the damaged command at once, so it recovers faster.
        self._raw_paste = raw_paste
        self._helpers = False
        users = port_users(self.port)
        if users:
            raise BoardError(
                f"{self.port} is already open in {', '.join(users)}. "
                "Close that serial terminal first; two programs on one port lose each other's data."
            )
        try:
            self.s = serial.Serial(self.port, self.baud, timeout=0.05)
        except serial.SerialException as e:
            hint = ""
            if e.errno == 13 or "Permission denied" in str(e):
                hint = (
                    " Your user isn't allowed to open serial ports; add it to the group that owns "
                    f"the port (see `ls -l {self.port}`, usually dialout), then log out and back in."
                )
            raise BoardError(f"Could not open {self.port}: {e}.{hint}") from e

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---------------------------------------------------------------- low level
    def _fill(self):
        data = self.s.read(self.s.in_waiting or 1)
        if data:
            self._rx.extend(data)

    def _read_until(self, token, timeout=None):
        end = time.monotonic() + (timeout or self.timeout)
        while True:
            i = self._rx.find(token)
            if i >= 0:
                out = bytes(self._rx[: i + len(token)])
                del self._rx[: i + len(token)]
                return out
            if time.monotonic() > end:
                tail = bytes(self._rx[-200:])
                raise BoardError(f"Timed out waiting for {token!r} from the board; last output: {tail!r}")
            self._fill()

    def _read_exact(self, n, timeout=None):
        end = time.monotonic() + (timeout or self.timeout)
        while len(self._rx) < n:
            if time.monotonic() > end:
                raise BoardError(f"Timed out waiting for {n} byte(s) from the board")
            self._fill()
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def _write_paced(self, data):
        for i in range(0, len(data), 256):
            self.s.write(data[i : i + 256])
            self.s.flush()
            time.sleep(0.01)

    # ---------------------------------------------------------------- raw REPL
    def enter_raw(self, attempts=5):
        """Interrupt running code and switch to the raw REPL."""
        heard = bytearray()
        for _ in range(attempts):
            self.s.write(b"\r\x03\x03")
            time.sleep(0.2)
            heard += self.s.read(self.s.in_waiting)
            self._rx.clear()
            self.s.write(b"\x01")
            try:
                self._read_until(RAW_BANNER, timeout=2)
                self._raw = True
                return
            except BoardError:
                heard += self._rx
                continue
        self.s.write(b"\x02")  # in case only the banner was lost, don't leave it in raw mode
        raise BoardError(self._no_repl_message(bytes(heard)))

    def _no_repl_message(self, heard):
        msg = f"Could not reach the raw REPL on {self.port} at {self.baud} baud. "
        if not heard:
            return msg + (
                "Nothing came back at all: check that this is the port wired to the board's console "
                "UART (TX to RX, RX to TX, common ground) and that no serial terminal has it open."
            )
        tail = heard[-300:]
        readable = sum(32 <= b < 127 or b in (9, 10, 13) for b in tail) / len(tail)
        if readable < 0.8:
            return msg + "Got unreadable data back, so the baud rate is probably wrong (use -b or ARGUS_BAUD)."
        return msg + (
            "Last output from the board:\n" + tail.decode(errors="replace") + "\n"
            "If main.py is still running, it may be catching Ctrl-C (a bare 'except:' does)."
        )

    def exit_raw(self):
        if self._raw:
            self.s.write(b"\x02")
            try:
                self._read_until(b">>> ", timeout=2)
            except BoardError:
                pass
            self._raw = False

    def _raw_paste_write(self, data, timeout):
        """Send a command with raw-paste flow control. Returns False if unsupported."""
        self.s.write(b"\x05A\x01")
        resp = self._read_exact(2, QUICK)
        if resp == b"R\x01":
            window = struct.unpack("<H", self._read_exact(2, QUICK))[0]
            remain, i = window, 0
            while i < len(data):
                while remain == 0 or self.s.in_waiting or self._rx:
                    # If bytes were lost the board never asks for more.
                    c = self._read_exact(1, QUICK)
                    if c == b"\x01":
                        remain += window
                    elif c == b"\x04":  # board ended early; output/error follows
                        self.s.write(b"\x04")
                        return True
                    else:
                        raise BoardError(f"Unexpected byte during raw paste: {c!r}")
                n = min(remain, len(data) - i)
                self.s.write(data[i : i + n])
                remain -= n
                i += n
            self.s.write(b"\x04")
            self._read_until(b"\x04", QUICK)
            return True
        self._raw_paste = False
        if resp != b"R\x00":
            # Firmware without raw paste treats the Ctrl-A as a raw REPL reset.
            self._read_until(RAW_BANNER[2:], QUICK)
        return False

    def exec(self, code, timeout=None):
        """Run code on the board and return its printed output."""
        timeout = timeout or self.timeout
        if not self._raw:
            self.enter_raw()
        data = code.encode()
        if not (self._raw_paste and self._raw_paste_write(data, timeout)):
            self._write_paced(data)
            self.s.write(b"\x04")
            self._read_until(b"OK", QUICK)
        out = self._read_until(b"\x04", timeout)[:-1]
        err = self._read_until(b"\x04", timeout)[:-1]
        self._read_until(b">", timeout)
        if err:
            raise RemoteError(err.decode(errors="replace").strip())
        return out.decode(errors="replace")

    def eval(self, expr, timeout=None):
        return ast.literal_eval(self.exec(f"print(repr({expr}))", timeout).strip())

    def call(self, code, timeout=None, tries=8):
        """Run a command that is safe to repeat: checked on arrival, retried after transmission errors."""
        return self._retrying(lambda: self.exec(_verified(code), timeout), tries)[0]

    def _ensure_helpers(self):
        if not self._helpers:
            for block in _helper_blocks():
                self.call(block)
            self._helpers = True

    def _helpers_present(self):
        try:
            return self.exec("print('_finish' in globals())").strip() == "True"
        except BoardError:
            return False

    # ---------------------------------------------------------------- queries
    def cpy_version(self):
        return int(self.call("import sys\nprint(sys.implementation.version[0])").strip())

    def board_id(self):
        return ast.literal_eval(self.call("import board\nprint(repr(board.board_id))").strip())

    def is_readonly(self):
        return self.call("import storage\nprint(storage.getmount('/').readonly)").strip() == "True"

    def info(self, items, timeout=120):
        """For each (path, expected_size): None if missing, "d" if a directory,
        (size, None) if the size differs, else (size, crc32)."""
        self._ensure_helpers()
        return ast.literal_eval(self.call(f"_info({list(items)!r})", timeout).strip())

    # ---------------------------------------------------------------- changes
    def mkdirs(self, path):
        self._ensure_helpers()
        self.call(f"_mkdirs({path!r})")

    def remove(self, path):
        self._ensure_helpers()
        self.call(f"_rm({path!r})")

    def rmdir(self, path):
        self._ensure_helpers()
        self.call(f"_rm({path!r}, True)")

    def _recover(self):
        """Get back to a clean raw REPL prompt after a garbled or lost command."""
        self._raw = False
        time.sleep(0.2)
        self.s.reset_input_buffer()
        self._rx.clear()
        self.enter_raw()

    def _retrying(self, fn, tries):
        """Run fn(); after a transmission error, recover the REPL and run it again.
        fn must be safe to repeat. Returns (result, retries used)."""
        last = None
        for n in range(tries + 1):
            try:
                return fn(), n
            except RemoteError as e:
                msg = str(e)
                if "NameError" in msg and self._helpers and not self._helpers_present():
                    # The board restarted and lost the helpers; reload them and try again.
                    self._helpers = False
                    self._ensure_helpers()
                if "no file open" in msg:
                    raise BoardError("file was closed during upload") from e
                if "OSError" in msg:  # e.g. disk full: retrying won't help
                    raise
                last = e  # garbled command: SyntaxError, bad base64, chunk CRC mismatch
            except BoardError as e:
                last = e
                self._recover()
        raise BoardError(f"gave up after {tries + 1} tries ({last})")

    def put(self, data, remote, retries=2, chunk_retries=8):
        """Write bytes to a file on the board via a temp file. Each chunk is CRC-checked on
        arrival and re-sent if damaged; the whole file is checked again before it replaces
        the old copy. Returns the number of retries needed."""
        self._ensure_helpers()
        crc = zlib.crc32(data) & 0xFFFFFFFF
        tmp = remote + ".tmp"
        retried, last = 0, None

        def send_chunk(off):
            piece = data[off : off + self.chunk]
            piece_crc = zlib.crc32(piece, off & 0xFFFFFFFF) & 0xFFFFFFFF  # also catches a garbled offset
            self.exec(f"_w({base64.b64encode(piece)!r},{off},{piece_crc})", timeout=15)

        for _ in range(retries + 1):
            try:
                _, n = self._retrying(lambda: self.exec(_verified(f"_open({tmp!r})")), chunk_retries)
                retried += n
                # Chunks carry their own CRC (covering data and offset), so they're sent as is.
                for off in range(0, len(data), self.chunk):
                    _, n = self._retrying(lambda off=off: send_chunk(off), chunk_retries)
                    retried += n
                _, n = self._retrying(lambda: self.exec(_verified("_close()")), chunk_retries)
                retried += n
                ok, n = self._retrying(
                    lambda: self.exec(
                        _verified(f"_finish({tmp!r},{remote!r},{len(data)},{crc})"), timeout=60
                    ).strip(),
                    chunk_retries,
                )
                retried += n
            except RemoteError:
                raise
            except BoardError as e:
                last = e
                retried += 1
                self._recover()
                continue
            if ok == "True":
                return retried
            last = BoardError("whole-file CRC mismatch")
            retried += 1
        raise BoardError(f"{remote}: upload failed after {retries + 1} attempts ({last})")

    # ---------------------------------------------------------------- teardown
    def soft_reboot(self):
        """Leave the raw REPL and press Ctrl-D so main.py runs again."""
        if not self._raw:
            self.enter_raw()
        self.exit_raw()
        self.s.write(b"\x04")
        self.s.flush()

    def close(self, reload=False):
        try:
            if reload:
                self.soft_reboot()
            else:
                self.exit_raw()
        finally:
            self.s.close()


if __name__ == "__main__":
    # Print the board's serial port (used by run.sh to detect it once per run).
    import sys

    try:
        print(find_port(int(os.environ.get("ARGUS_BAUD") or DEFAULT_BAUD), log=lambda m: print(m, file=sys.stderr)))
    except BoardError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)