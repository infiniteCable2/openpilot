from enum import Enum

from openpilot.selfdrive.car.steer_control import is_curvature_steer_control


class SteerControlType(Enum):
  torque = 0
  angle = 1
  curvatureDEPRECATED = 2
  curvature = 3


def test_curvature_steer_control_compatibility():
  assert is_curvature_steer_control(SteerControlType.curvature)
  assert is_curvature_steer_control(SteerControlType.curvatureDEPRECATED)
  assert not is_curvature_steer_control(SteerControlType.torque)
  assert not is_curvature_steer_control(SteerControlType.angle)
