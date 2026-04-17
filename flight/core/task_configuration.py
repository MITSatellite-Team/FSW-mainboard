from core.states import TASK
from core.satellite_config import feature_flags_config as FEATURES
from tasks.adcs import Task as adcs
from tasks.command import Task as command
from tasks.comms import Task as comms
from tasks.eps import Task as eps
from tasks.gps import Task as gps
from tasks.hal_monitor import Task as hal_monitor
from tasks.obdh import Task as obdh
from tasks.watchdog import Task as watchdog

TASK_CONFIG = {
    TASK.COMMAND: {"Task": command, "Frequency": 2, "Priority": 2},
    TASK.WATCHDOG: {"Task": watchdog, "Frequency": 1, "Priority": 1},
    TASK.EPS: {"Task": eps, "Frequency": 5, "Priority": 2},
    TASK.OBDH: {"Task": obdh, "Frequency": 0.5, "Priority": 2},
    TASK.COMMS: {"Task": comms, "Frequency": 1, "Priority": 2, "ScheduleLater": True},
    TASK.ADCS: {"Task": adcs, "Frequency": 5, "Priority": 1},
    TASK.GPS: {"Task": gps, "Frequency": 2, "Priority": 3, "ScheduleLater": True},  # GPS <= 1 Hz
    # Watchdog needs to have priority over HAL monitor to ensure it is serviced
    # HAL monitor can take too long on boot and cause watchdog resets
    TASK.HAL_MONITOR: {"Task": hal_monitor, "Frequency": 5, "Priority": 2},
}

if FEATURES.ENABLE_PAYLOAD:
    from tasks.payload import Task as payload

    TASK_CONFIG[TASK.PAYLOAD] = {"Task": payload, "Frequency": 5, "Priority": 3, "ScheduleLater": True}
