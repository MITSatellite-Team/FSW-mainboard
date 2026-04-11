import argparse
import math
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:
    serial = None


LOG_PATTERN = re.compile(r"^\[(?P<timestamp>[^\]]+)\]\[(?P<level>[^\]]+)\] \[(?P<task_id>\d+)\]\[(?P<tag>[^\]]+)\] (?P<message>.*)$")
NUMBER_PATTERN = re.compile(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?")

MODE_NAMES = {
    0: "TUMBLING",
    1: "STABLE",
    2: "SUN_POINTED",
    3: "ACS_OFF",
}


@dataclass
class Sample3:
    t: float
    x: float
    y: float
    z: float


class OrientationFilter:
    def __init__(self, alpha: float = 0.97, yaw_alpha: float = 0.985):
        self.alpha = alpha
        self.yaw_alpha = yaw_alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_update_time = None

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _blend_angle(self, current: float, measured: float, weight: float) -> float:
        delta = self._wrap_angle(measured - current)
        return self._wrap_angle(current + weight * delta)

    def update(
        self,
        t_now: float,
        gyro: Optional[np.ndarray],
        accel: Optional[np.ndarray],
        mag: Optional[np.ndarray],
    ) -> None:
        if gyro is not None and self._last_update_time is not None:
            dt = max(0.0, t_now - self._last_update_time)
            self.roll += float(gyro[0]) * dt
            self.pitch += float(gyro[1]) * dt
            self.yaw += float(gyro[2]) * dt
            self.roll = self._wrap_angle(self.roll)
            self.pitch = self._wrap_angle(self.pitch)
            self.yaw = self._wrap_angle(self.yaw)

        if gyro is not None:
            self._last_update_time = t_now

        if accel is not None:
            accel_norm = np.linalg.norm(accel)
            if accel_norm > 1e-6:
                ax, ay, az = accel / accel_norm
                accel_roll = math.atan2(ay, az)
                accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                self.roll = self.alpha * self.roll + (1.0 - self.alpha) * accel_roll
                self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * accel_pitch

        if accel is not None and mag is not None:
            accel_norm = np.linalg.norm(accel)
            mag_norm = np.linalg.norm(mag)
            if accel_norm > 1e-6 and mag_norm > 1e-12:
                mx, my, mz = mag / mag_norm
                cr = math.cos(self.roll)
                sr = math.sin(self.roll)
                cp = math.cos(self.pitch)
                sp = math.sin(self.pitch)
                xh = mx * cp + my * sr * sp + mz * cr * sp
                yh = my * cr - mz * sr
                yaw_mag = math.atan2(-yh, xh)
                self.yaw = self._blend_angle(self.yaw, yaw_mag, 1.0 - self.yaw_alpha)

    def rotation_matrix(self) -> np.ndarray:
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return rz @ ry @ rx

    def euler_deg(self) -> tuple[float, float, float]:
        return (
            math.degrees(self.roll),
            math.degrees(self.pitch),
            math.degrees(self.yaw),
        )


class TelemetryState:
    def __init__(self, history_seconds: float):
        max_points = max(500, int(history_seconds * 20))
        self.history_seconds = history_seconds
        self.gyro: Deque[Sample3] = deque(maxlen=max_points)
        self.accel: Deque[Sample3] = deque(maxlen=max_points)
        self.mag: Deque[Sample3] = deque(maxlen=max_points)
        self.orientation = OrientationFilter()
        self.latest_gyro = None
        self.latest_accel = None
        self.latest_mag = None
        self.latest_adcs_mode = None
        self.latest_global_state = None
        self.latest_gyro_status = None
        self.latest_mag_status = None
        self.last_line = ""
        self.t0 = None

    def _relative_time(self) -> float:
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    @staticmethod
    def _parse_vector(message: str, prefix: str) -> Optional[np.ndarray]:
        if not message.startswith(prefix):
            return None
        values = NUMBER_PATTERN.findall(message[len(prefix) :])
        if len(values) < 3:
            return None
        return np.array([float(values[0]), float(values[1]), float(values[2])], dtype=float)

    def ingest_line(self, line: str) -> None:
        self.last_line = line.rstrip()
        match = LOG_PATTERN.match(self.last_line)
        if not match:
            return

        message = match.group("message")
        t_rel = self._relative_time()

        gyro = self._parse_vector(message, "Gyro Ang Vel :")
        if gyro is not None:
            self.latest_gyro = gyro
            self.gyro.append(Sample3(t_rel, gyro[0], gyro[1], gyro[2]))
            self.orientation.update(t_rel, gyro=gyro, accel=self.latest_accel, mag=self.latest_mag)
            return

        accel = self._parse_vector(message, "Accel :")
        if accel is not None:
            self.latest_accel = accel
            self.accel.append(Sample3(t_rel, accel[0], accel[1], accel[2]))
            self.orientation.update(t_rel, gyro=None, accel=accel, mag=self.latest_mag)
            return

        mag = self._parse_vector(message, "Mag Field :")
        if mag is not None:
            self.latest_mag = mag
            self.mag.append(Sample3(t_rel, mag[0], mag[1], mag[2]))
            self.orientation.update(t_rel, gyro=None, accel=self.latest_accel, mag=mag)
            return

        if message.startswith("ADCS Mode :"):
            mode_match = re.search(r"ADCS Mode : (\d+)", message)
            if mode_match:
                self.latest_adcs_mode = int(mode_match.group(1))
            return

        if message.startswith("GLOBAL STATE:"):
            state_match = re.search(r"GLOBAL STATE: ([A-Z_]+)", message)
            if state_match:
                self.latest_global_state = state_match.group(1)
            return

        if message.startswith("Gyro Status :"):
            status_match = re.search(r"Gyro Status : (\d+)", message)
            if status_match:
                self.latest_gyro_status = int(status_match.group(1))
            return

        if message.startswith("Mag Status :"):
            status_match = re.search(r"Mag Status : (\d+)", message)
            if status_match:
                self.latest_mag_status = int(status_match.group(1))


class LineReader(threading.Thread):
    def __init__(
        self,
        telemetry: TelemetryState,
        stop_event: threading.Event,
        port: Optional[str],
        baud: int,
        replay_file: Optional[Path],
        save_log: Optional[Path],
    ):
        super().__init__(daemon=True)
        self.telemetry = telemetry
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.replay_file = replay_file
        self.save_log = save_log
        self.error_queue: "queue.Queue[str]" = queue.Queue()

    def _write_log(self, line: str) -> None:
        if self.save_log is None:
            return
        self.save_log.parent.mkdir(parents=True, exist_ok=True)
        with self.save_log.open("a", encoding="utf-8") as handle:
            handle.write(line)

    def _run_replay(self) -> None:
        previous_ts = None
        with self.replay_file.open("r", encoding="utf-8") as handle:
            for line in handle:
                if self.stop_event.is_set():
                    return
                match = LOG_PATTERN.match(line.rstrip())
                if match:
                    ts = match.group("timestamp")
                    if previous_ts is not None and ts != previous_ts:
                        time.sleep(0.05)
                    previous_ts = ts
                self.telemetry.ingest_line(line)

    def _run_serial(self) -> None:
        if serial is None:
            self.error_queue.put("pyserial is not installed. Install it with `pip install pyserial`.")
            return

        try:
            with serial.Serial(self.port, self.baud, timeout=0.25) as ser:
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="replace")
                    except Exception:
                        continue
                    self._write_log(line)
                    self.telemetry.ingest_line(line)
        except Exception as exc:
            self.error_queue.put(f"Serial reader failed: {exc}")

    def run(self) -> None:
        if self.replay_file is not None:
            self._run_replay()
        else:
            self._run_serial()


_SENSOR_YLIMS = {
    "gyro": (-0.5, 0.5),      # rad/s
    "accel": (-12.0, 12.0),   # m/s²
    "mag": (-1e-4, 1e-4),     # T
}


class LivePlotter:
    def __init__(self, telemetry: TelemetryState, reader: LineReader, history_seconds: float, source_label: str):
        self.telemetry = telemetry
        self.reader = reader
        self.history_seconds = history_seconds
        self.source_label = source_label
        self._current_xlim = (0.0, history_seconds)

        self.fig = plt.figure(figsize=(15, 9), constrained_layout=True)
        grid = self.fig.add_gridspec(3, 2, width_ratios=[2.1, 1.2])
        self.ax_gyro = self.fig.add_subplot(grid[0, 0])
        self.ax_accel = self.fig.add_subplot(grid[1, 0], sharex=self.ax_gyro)
        self.ax_mag = self.fig.add_subplot(grid[2, 0], sharex=self.ax_gyro)
        self.ax_board = self.fig.add_subplot(grid[:2, 1], projection="3d")
        self.ax_text = self.fig.add_subplot(grid[2, 1])
        self.ax_text.axis("off")

        self.gyro_lines = self._make_axis(self.ax_gyro, "Gyro [rad/s]", _SENSOR_YLIMS["gyro"])
        self.accel_lines = self._make_axis(self.ax_accel, "Accel [m/s²]", _SENSOR_YLIMS["accel"])
        self.mag_lines = self._make_axis(self.ax_mag, "Mag [T]", _SENSOR_YLIMS["mag"])
        self.ax_mag.set_xlabel("Time [s]")
        self.text_artist = self.ax_text.text(0.02, 0.98, "", va="top", ha="left", family="monospace", fontsize=9,
                                             transform=self.ax_text.transAxes)
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.anim = animation.FuncAnimation(self.fig, self._update, interval=50, cache_frame_data=False)

    @staticmethod
    def _make_axis(ax, ylabel: str, ylim: tuple):
        colors = ("tab:red", "tab:green", "tab:blue")
        labels = ("X", "Y", "Z")
        lines = []
        for color, label in zip(colors, labels):
            (line,) = ax.plot([], [], color=color, label=label, linewidth=1.5)
            lines.append(line)
        ax.set_ylabel(ylabel)
        ax.set_ylim(*ylim)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper left", fontsize=8)
        return lines

    @staticmethod
    def _board_vertices() -> np.ndarray:
        return np.array(
            [
                [-0.6, -0.4, -0.08],
                [0.6, -0.4, -0.08],
                [0.6, 0.4, -0.08],
                [-0.6, 0.4, -0.08],
                [-0.6, -0.4, 0.08],
                [0.6, -0.4, 0.08],
                [0.6, 0.4, 0.08],
                [-0.6, 0.4, 0.08],
            ],
            dtype=float,
        )

    def _draw_board(self) -> None:
        self.ax_board.cla()
        rotation = self.telemetry.orientation.rotation_matrix()
        vertices = (rotation @ self._board_vertices().T).T
        faces = [
            [vertices[idx] for idx in [0, 1, 2, 3]],
            [vertices[idx] for idx in [4, 5, 6, 7]],
            [vertices[idx] for idx in [0, 1, 5, 4]],
            [vertices[idx] for idx in [2, 3, 7, 6]],
            [vertices[idx] for idx in [1, 2, 6, 5]],
            [vertices[idx] for idx in [0, 3, 7, 4]],
        ]
        board = Poly3DCollection(faces, facecolors="#6ea8fe", edgecolors="#1f1f1f", linewidths=1.0, alpha=0.85)
        self.ax_board.add_collection3d(board)

        origin = np.zeros(3)
        axes = rotation @ np.eye(3)
        colors = ("tab:red", "tab:green", "tab:blue")
        labels = ("X", "Y", "Z")
        for vec, color, label in zip(axes.T, colors, labels):
            self.ax_board.quiver(*origin, *vec, color=color, linewidth=2.0, arrow_length_ratio=0.15)
            self.ax_board.text(*(vec * 1.1), label, color=color)

        self.ax_board.set_title("Board Attitude Estimate")
        self.ax_board.set_xlim(-1.2, 1.2)
        self.ax_board.set_ylim(-1.2, 1.2)
        self.ax_board.set_zlim(-1.2, 1.2)
        self.ax_board.set_box_aspect((1.4, 1.0, 0.6))
        self.ax_board.set_xlabel("X")
        self.ax_board.set_ylabel("Y")
        self.ax_board.set_zlabel("Z")

    @staticmethod
    def _series_to_arrays(series: Deque[Sample3], history_seconds: float):
        if not series:
            return np.array([]), np.empty((0, 3))
        latest_t = series[-1].t
        cutoff = latest_t - history_seconds
        filtered = [s for s in series if s.t >= cutoff]
        times = np.array([s.t for s in filtered])
        values = np.array([[s.x, s.y, s.z] for s in filtered])
        return times, values

    @staticmethod
    def _update_lines(lines, times: np.ndarray, values: np.ndarray) -> None:
        if times.size == 0:
            return
        for idx, line in enumerate(lines):
            line.set_data(times, values[:, idx])

    def _update_text(self) -> None:
        roll, pitch, yaw = self.telemetry.orientation.euler_deg()
        adcs_mode = MODE_NAMES.get(self.telemetry.latest_adcs_mode, str(self.telemetry.latest_adcs_mode))

        def fmt_vec(vec: Optional[np.ndarray]) -> str:
            if vec is None:
                return "n/a"
            return f"[{vec[0]: .3f}, {vec[1]: .3f}, {vec[2]: .3f}]"

        status_lines = [
            f"Source      : {self.source_label}",
            f"Global State: {self.telemetry.latest_global_state or 'n/a'}",
            f"ADCS Mode   : {adcs_mode}",
            f"Gyro Status : {self.telemetry.latest_gyro_status if self.telemetry.latest_gyro_status is not None else 'n/a'}",
            f"Mag Status  : {self.telemetry.latest_mag_status if self.telemetry.latest_mag_status is not None else 'n/a'}",
            "",
            f"Roll  [deg] : {roll: .1f}",
            f"Pitch [deg] : {pitch: .1f}",
            f"Yaw   [deg] : {yaw: .1f}",
            "",
            f"Gyro        : {fmt_vec(self.telemetry.latest_gyro)}",
            f"Accel       : {fmt_vec(self.telemetry.latest_accel)}",
            f"Mag         : {fmt_vec(self.telemetry.latest_mag)}",
        ]

        try:
            status_lines.extend(["", self.reader.error_queue.get_nowait()])
        except queue.Empty:
            pass

        self.text_artist.set_text("\n".join(status_lines))

    def _update(self, _frame):
        gyro_t, gyro_v = self._series_to_arrays(self.telemetry.gyro, self.history_seconds)
        accel_t, accel_v = self._series_to_arrays(self.telemetry.accel, self.history_seconds)
        mag_t, mag_v = self._series_to_arrays(self.telemetry.mag, self.history_seconds)

        self._update_lines(self.gyro_lines, gyro_t, gyro_v)
        self._update_lines(self.accel_lines, accel_t, accel_v)
        self._update_lines(self.mag_lines, mag_t, mag_v)

        latest_t = gyro_t[-1] if gyro_t.size else 0.0
        self.ax_gyro.set_xlim(max(0.0, latest_t - self.history_seconds), max(self.history_seconds, latest_t))

        self._draw_board()
        self._update_text()
        return [*self.gyro_lines, *self.accel_lines, *self.mag_lines, self.text_artist]

    def _on_close(self, _event) -> None:
        self.reader.stop_event.set()

    def show(self) -> None:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser(description="Live serial plotter for FSW ADCS telemetry.")
    parser.add_argument("--port", help="Serial port to open, for example /dev/tty.usbmodemXXXX.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--history", type=float, default=30.0, help="Seconds of history to keep on the plots.")
    parser.add_argument("--save-log", type=Path, help="Optional path to save the raw serial log stream.")
    parser.add_argument("--replay-file", type=Path, help="Replay a previously captured log file instead of opening serial.")
    args = parser.parse_args()

    if args.port is None and args.replay_file is None:
        parser.error("either --port or --replay-file is required")
    return args


def main():
    args = parse_args()
    telemetry = TelemetryState(history_seconds=args.history)
    stop_event = threading.Event()
    source_label = str(args.replay_file) if args.replay_file is not None else f"{args.port} @ {args.baud}"
    reader = LineReader(
        telemetry=telemetry,
        stop_event=stop_event,
        port=args.port,
        baud=args.baud,
        replay_file=args.replay_file,
        save_log=args.save_log,
    )
    reader.start()
    plotter = LivePlotter(telemetry=telemetry, reader=reader, history_seconds=args.history, source_label=source_label)
    plotter.show()
    stop_event.set()
    reader.join(timeout=1.0)


if __name__ == "__main__":
    main()
