import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import MultiplicativeUnwindPID

CURVATURE_SATURATION_THRESHOLD = 5e-4  # rad/m
LAT_CURVATURE_SATURATION_ACCEL = 0.4  # m/s², lateral accel equivalent for curvature controller saturation


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

  def set_curvature_correction(self, correction: float) -> None:
    self.curvature_correction = correction

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.Deprecated.LateralCurvatureState.new_message()
    angle_steers_des = float(CS.steeringAngleDeg)
    if not active:
      output_curvature = 0.0
      pid_log.active = False
      self.pid.reset()
    else:

      roll_compensation = -VM.roll_compensation(params.roll, CS.vEgo)
      actual_curvature_vm_no_roll = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, 0.)
      actual_curvature_vm = actual_curvature_vm_no_roll - roll_compensation

      # Blend VM curvature with pose-derived curvature at higher speeds
      actual_curvature = actual_curvature_vm
      if calibrated_pose is not None and CS.vEgo > 5.0:
        actual_curvature_pose = calibrated_pose.angular_velocity.yaw / max(CS.vEgo, 0.1)
        actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])

      # Feed CurvatureD correction as additive feedforward (not setpoint shift)
      desired_curvature_with_ff = desired_curvature + self.curvature_correction

      pid_log.error = float(desired_curvature - actual_curvature)
      freeze_integrator = steer_limited_by_safety or CS.vEgo < 5 or CS.steeringPressed

      pid_curvature = self.pid.update(pid_log.error, feedforward=desired_curvature_with_ff, speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator, override=CS.steeringPressed)

      # Closed-loop curvature correction: align controller reference with car's physical curvature.
      output_curvature = pid_curvature + (CS.steeringCurvature - actual_curvature_vm_no_roll) if self.useCarSteerCurvature else pid_curvature

      # Saturation: controller limited by PID limits OR by car safety limits OR by curvature clip
      saturated = abs(output_curvature) >= self.curvature_max or abs(pid_curvature) >= self.curvature_max

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_curvature)
      pid_log.actualCurvature = float(actual_curvature)
      pid_log.desiredCurvature = float(desired_curvature)
      pid_log.saturated = bool(self._check_saturation(saturated, CS, steer_limited_by_safety, curvature_limited))

    return 0.0, 0.0, output_curvature, pid_log
