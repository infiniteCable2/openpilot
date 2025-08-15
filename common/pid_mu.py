from openpilot.common.pid import PIDController
import numpy as np

class MultiplicativeUnwindPID(PIDController):
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    super().__init__(k_p, k_i, k_f=k_f, k_d=k_d, pos_limit=pos_limit, neg_limit=neg_limit, rate=rate)
      
    self.i_unwind_rate = 0.3 / rate
    
  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self.i *= (1.0 - self.i_unwind_rate)
      if abs(self.i) < 1e-10:
        self.i = 0.0
    else:
      if not freeze_integrator:
        i = self.i + error * self.k_i * self.i_rate

        # Don't allow windup if already clipping
        test_control = self.p + i + self.d + self.f
        i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
        i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
        self.i = np.clip(i, i_lowerbound, i_upperbound)

    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control