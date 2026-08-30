import numpy as np
from collections.abc import Sequence

Gain = int | float | tuple[Sequence[float], Sequence[float]] | list[list[float]]

class PIDController:
  def __init__(self, k_p: Gain, k_i: Gain, k_d: Gain = 0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = ([0], [k_p]) if isinstance(k_p, (int, float)) else k_p
    self._k_i = ([0], [k_i]) if isinstance(k_i, (int, float)) else k_i
    self._k_d = ([0], [k_d]) if isinstance(k_d, (int, float)) else k_d

    self.set_limits(pos_limit, neg_limit)

    self.i_dt = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = self.k_p * float(error)
    self.d = self.k_d * error_rate
    self.f = feedforward

    if not freeze_integrator:
      i = self.i + self.k_i * self.i_dt * error

      # Don't allow windup if already clipping
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = np.clip(i, i_lowerbound, i_upperbound)

    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control


class MultiplicativeUnwindPID:
  DEFAULT_UNWIND_TIME = 0.1  # seconds

  def __init__(self, k_p: Gain, k_i: Gain, k_f=0., k_d: Gain = 0., pos_limit=1e308, neg_limit=-1e308,
               rate=100, min_cmd=1e-10, unwind_time=DEFAULT_UNWIND_TIME):
    self._k_p = ([0], [k_p]) if isinstance(k_p, (int, float)) else k_p
    self._k_i = ([0], [k_i]) if isinstance(k_i, (int, float)) else k_i
    self._k_d = ([0], [k_d]) if isinstance(k_d, (int, float)) else k_d
    self.k_f = float(k_f)
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit
    self.rate = float(rate)
    self.i_dt = 1.0 / rate
    self.min_cmd = abs(min_cmd)
    self.speed = 0.0
    self.set_unwind_time(unwind_time)
    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0
    self.unwind_time_prev: float | None = None
    self.i_unwind_factor = 1.0

  def set_unwind_time(self, unwind_time: float) -> None:
    if unwind_time <= 0.0:
      raise ValueError("unwind_time must be greater than zero")
    self.unwind_time = float(unwind_time)

  def _calc_unwind_factor(self):
    # Recalculate when unwinding starts or the requested duration changes.
    if self.unwind_time == self.unwind_time_prev:
      return
    if abs(self.i) <= self.min_cmd:
      self.i_unwind_factor = 0.0
      return
    steps = max(int(self.unwind_time * self.rate), 1)
    factor = (self.min_cmd / abs(self.i)) ** (1.0 / steps)
    self.i_unwind_factor = min(factor, 1.0)

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self._calc_unwind_factor()
      self.i *= self.i_unwind_factor
      if abs(self.i) < self.min_cmd:
        self.i = 0.0
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_dt

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = np.clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f

    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    self.unwind_time_prev = self.unwind_time if override else None
    return self.control
