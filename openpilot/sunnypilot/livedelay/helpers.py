"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import threading

from openpilot.common.params import Params
from openpilot.common.realtime import drop_realtime
from openpilot.common.swaglog import cloudlog


def get_lat_delay(params: Params, stock_lat_delay: float) -> float:
# live learning on: use what lagd publishes.
# off: use the fixed steerActuatorDelay + software delay sum that LagdToggle caches.

  if params.get_bool("LagdToggle"):
    return stock_lat_delay

  return float(params.get("LagdValueCache", return_default=True))


class LateralDelayCache:
  """Keep filesystem reads off the model loop while retaining live setting updates."""

  def __init__(self, steer_actuator_delay: float, refresh_interval: float = 1.0, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.steer_actuator_delay = steer_actuator_delay
    self.refresh_interval = refresh_interval
    self._state = self._read_state()
    self._stop = threading.Event()
    self._thread = threading.Thread(target=self._refresh_loop, daemon=True)
    self._thread.start()

  def _read_state(self) -> tuple[bool, float]:
    live_learning = self.params.get_bool("LagdToggle")
    fixed_delay = 0.0 if live_learning else self.steer_actuator_delay + float(self.params.get("LagdToggleDelay", return_default=True))
    return live_learning, fixed_delay

  def _refresh_loop(self) -> None:
    drop_realtime()
    while not self._stop.wait(self.refresh_interval):
      try:
        self._state = self._read_state()
      except Exception:
        cloudlog.exception("failed to refresh lateral delay params")

  def get(self, stock_lat_delay: float) -> float:
    live_learning, fixed_delay = self._state
    return stock_lat_delay if live_learning else fixed_delay

  def close(self) -> None:
    self._stop.set()
    self._thread.join(timeout=1)
