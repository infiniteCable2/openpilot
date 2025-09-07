import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

CURVATURE_SATURATION_THRESHOLD = 5e-4 # rad/m


class LatControlCurvature(LatControl):
  def __init__(self, CP, CP_SP, CI):
    super().__init__(CP, CP_SP, CI)
    assert False

  def reset(self):
    super().reset()
  
  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()
    actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    output_curvature = desired_curvature
    
    curvature_log.active = active
    curvature_log.output = float(output_curvature)
    curvature_log.actualCurvature = float(actual_curvature)
    curvature_log.desiredCurvature = float(output_curvature)
    curvature_log.saturated = bool(self._check_saturation(steer_limited_by_safety, CS, False, curvature_limited)) if active else False

    return 0.0, 0.0, output_curvature, curvature_log
