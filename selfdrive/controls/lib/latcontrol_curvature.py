import math
import numpy as np

from cereal import log
from openpilot.common.pid import MultiplicativeUnwindPID
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE

LAT_ACCEL_SATURATION_THRESHOLD = 0.2  # m/s^2


class LatControlCurvature(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.enable_pid = False
    self.curvature_correction = 0.0
    if CP.lateralTuning.which() == 'pid':
      ct = CP.lateralTuning.pid
      self.pid = MultiplicativeUnwindPID((ct.kpBP, ct.kpV), (ct.kiBP, ct.kiV),
                                         k_f=ct.kf,
                                         pos_limit=MAX_CURVATURE, neg_limit=-MAX_CURVATURE,
                                         rate=1 / dt, min_cmd=1e-6, ki_red_time=0.1)
      self.kf = ct.kf
    else:
      self.pid = None
      self.kf = 1.
      
  def set_pid_enabled(self, enabled: bool) -> None:
    self.enable_pid = enabled

  def set_curvature_correction(self, correction: float) -> None:
    self.curvature_correction = correction

  def reset(self):
    super().reset()
    if self.pid is not None:
      self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()
    roll_compensation = -VM.roll_compensation(params.roll, CS.vEgo)
    actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    error = desired_curvature - actual_curvature
    feedforward = self.kf * desired_curvature

    if not active:
      output_curvature = 0.0
      curvature_log.active = False
      if self.pid is not None:
        self.pid.reset()
    elif self.pid is None or not self.enable_pid:
      if self.pid is not None:
        self.pid.reset()
      output_curvature = feedforward
      curvature_log.active = True
    else:
      freeze_integrator = steer_limited_by_safety or CS.vEgo < 5
      output_curvature = self.pid.update(error, speed=CS.vEgo, feedforward=feedforward,
                                         override=CS.steeringPressed,
                                         freeze_integrator=freeze_integrator)
      curvature_log.p = float(self.pid.p)
      curvature_log.i = float(self.pid.i)
      curvature_log.f = float(self.pid.f)
      curvature_log.active = True

    curvature_log.error = float(error)
    curvature_log.actualCurvature = float(actual_curvature)
    curvature_log.desiredCurvature = float(desired_curvature)
    output_curvature = output_curvature - roll_compensation + self.curvature_correction
    curvature_log.output = float(output_curvature)
    lat_accel_error = error * CS.vEgo ** 2
    curvature_log.saturated = bool(self._check_saturation(abs(lat_accel_error) > LAT_ACCEL_SATURATION_THRESHOLD, CS,
                                                          False, curvature_limited))
    return 0.0, 0.0, float(output_curvature), curvature_log
