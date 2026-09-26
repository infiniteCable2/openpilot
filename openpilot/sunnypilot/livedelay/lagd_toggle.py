"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.cereal import log

from opendbc.car import structs
from openpilot.common.params import Params


class LagdToggle:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    self.lag = 0.0
    self.last_cached_lag = self.params.get("LagdValueCache")
    self.last_cache_write_time = time.monotonic()

    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = self.params.get("LagdToggleDelay", return_default=True)

  def read_params(self) -> None:
    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = self.params.get("LagdToggleDelay", return_default=True)

  def update(self, lag_msg: log.LateralDelay) -> None:
    self.read_params()

    if not self.lagd_toggle:
      steer_actuator_delay = self.CP.steerActuatorDelay
      delay = self.software_delay
      self.lag = (steer_actuator_delay + delay)
      self.cache_lag_if_changed()
      return

    lateral_delay = lag_msg.lateralDelay.lateralDelay
    self.lag = lateral_delay
    self.cache_lag_if_changed()

  def cache_lag_if_changed(self) -> None:
    now = time.monotonic()
    if self.lag != self.last_cached_lag or now - self.last_cache_write_time >= 60.0:
      self.params.put("LagdValueCache", self.lag)
      self.last_cached_lag = self.lag
      self.last_cache_write_time = now
