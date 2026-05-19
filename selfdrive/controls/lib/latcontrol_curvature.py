import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import MultiplicativeUnwindPID
LAT_CURVATURE_SATURATION_ACCEL = 0.2


class LatControlCurvature(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    ct = CP.lateralTuning.curvature
    self.pid = MultiplicativeUnwindPID((ct.kpBP, ct.kpV),
                                       (ct.kiBP, ct.kiV),
                                       k_f=ct.kf,
                                       pos_limit=self.curvature_max, neg_limit=-self.curvature_max)
    self.useCarSteerCurvature = ct.useCarSteerCurvature
    self.curvature_correction = 0.0
    self.enable_pid = False

  def set_curvature_correction(self, correction: float) -> None:
    self.curvature_correction = correction

  def set_pid_enabled(self, enabled: bool) -> None:
    self.enable_pid = enabled

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralCurvatureState.new_message()
    if not active:
      output_curvature = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      roll_compensation = -VM.roll_compensation(params.roll, CS.vEgo)
      actual_curvature_vm_no_roll = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, 0.)
      actual_curvature_vm = actual_curvature_vm_no_roll - roll_compensation

      actual_curvature = actual_curvature_vm
      if calibrated_pose is not None and CS.vEgo > 5.0:
        actual_curvature_pose = calibrated_pose.angular_velocity.yaw / max(CS.vEgo, 0.1)
        actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])

      desired_curvature_with_ff = desired_curvature + self.curvature_correction

      if self.enable_pid:
        pid_log.error = float(desired_curvature - actual_curvature)
        freeze_integrator = steer_limited_by_safety or CS.vEgo < 5 or CS.steeringPressed

        pid_curvature = self.pid.update(pid_log.error, speed=CS.vEgo,
                                        feedforward=desired_curvature_with_ff,
                                        freeze_integrator=freeze_integrator, override=CS.steeringPressed)
      else:
        pid_curvature = desired_curvature_with_ff
        pid_log.error = float(desired_curvature - actual_curvature)
        pid_log.p = 0.0
        pid_log.i = 0.0
        pid_log.f = float(desired_curvature_with_ff)

      output_curvature = pid_curvature + (CS.steeringCurvature - actual_curvature_vm_no_roll) if self.useCarSteerCurvature else pid_curvature

      saturated = abs(output_curvature) >= self.curvature_max or abs(pid_curvature) >= self.curvature_max

      pid_log.active = True
      if self.enable_pid:
        pid_log.p = float(self.pid.p)
        pid_log.i = float(self.pid.i)
        pid_log.f = float(self.pid.f)
      pid_log.output = float(output_curvature)
      pid_log.actualCurvature = float(actual_curvature)
      pid_log.desiredCurvature = float(desired_curvature)
      pid_log.saturated = bool(self._check_saturation(saturated, CS, steer_limited_by_safety, curvature_limited))

    return 0.0, 0.0, output_curvature, pid_log
