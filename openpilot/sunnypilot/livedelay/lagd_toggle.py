"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.cereal import log

from opendbc.car import structs
from openpilot.common.params import Params


CACHE_IMMEDIATE_DELTA = 0.005  # Persist changes of at least 5 ms immediately.
CACHE_PERIODIC_INTERVAL = 60.0


class LagdToggle:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    self.lag = 0.0

    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = self.params.get("LagdToggleDelay", return_default=True)
    self._cached_lag = float(self.params.get("LagdValueCache", return_default=True))
    self._last_cache_write = time.monotonic()
    self._first_update = True

  def read_params(self) -> None:
    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = self.params.get("LagdToggleDelay", return_default=True)

  def update(self, lag_msg: log.LateralDelay) -> None:
    previous_settings = (self.lagd_toggle, self.software_delay)
    self.read_params()

    if not self.lagd_toggle:
      self.lag = self.CP.steerActuatorDelay + self.software_delay
    else:
      self.lag = lag_msg.lateralDelay.lateralDelay

    now = time.monotonic()
    delta = abs(self.lag - self._cached_lag)
    settings_changed = previous_settings != (self.lagd_toggle, self.software_delay)
    write_on_change = delta > 0 and (self._first_update or settings_changed or delta >= CACHE_IMMEDIATE_DELTA)
    # Refresh once a minute even when unchanged, so a failed async write is retried.
    write_periodically = now - self._last_cache_write >= CACHE_PERIODIC_INTERVAL
    if write_on_change or write_periodically:
      self.params.put("LagdValueCache", self.lag)
      self._cached_lag = self.lag
      self._last_cache_write = now
    self._first_update = False
