from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.latcontrol_curvature import (SLIGHT_STEERING_OVERRIDE_UNWIND_TIME,
                                                                   STEERING_OVERRIDE_UNWIND_TIME, LatControlCurvature)


DT = 0.01
INITIAL_I = 1e-3
MIN_I = 1e-6


class DummyLateralTuning:
  kpBP = [0.0]
  kpV = [0.0]
  kiBP = [0.0]
  kiV = [0.0]
  kf = 0.0

  @staticmethod
  def which():
    return 'pid'

  @property
  def pid(self):
    return self


class DummyVehicleModel:
  @staticmethod
  def roll_compensation(roll, v_ego):
    return 0.0

  @staticmethod
  def calc_curvature(steering_angle, v_ego, roll):
    return 0.0


def make_controller():
  CP = SimpleNamespace(steerLimitTimer=1.0, lateralTuning=DummyLateralTuning())
  controller = LatControlCurvature(CP, None, None, DT)
  controller.set_pid_enabled(True)
  controller.pid.i = INITIAL_I
  return controller


def update_controller(controller, steering_pressed=False):
  CS = SimpleNamespace(vEgo=20.0, steeringAngleDeg=0.0, steeringPressed=steering_pressed)
  params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
  controller.update(True, CS, DummyVehicleModel(), params, False, 0.0, None, False, 0.0)


def test_slight_steering_override_unwinds_integrator_over_ten_seconds():
  controller = make_controller()
  controller.set_steering_slightly_pressed(True)

  half_steps = round(SLIGHT_STEERING_OVERRIDE_UNWIND_TIME / DT / 2)
  for _ in range(half_steps):
    update_controller(controller)

  assert np.isclose(controller.pid.i, np.sqrt(INITIAL_I * MIN_I), rtol=1e-6)

  for _ in range(half_steps):
    update_controller(controller)

  assert abs(controller.pid.i) <= MIN_I * (1.0 + 1e-12)


def test_full_steering_override_keeps_fast_unwind():
  controller = make_controller()
  controller.set_steering_slightly_pressed(True)

  for _ in range(round(1.0 / DT)):
    update_controller(controller)
  assert controller.pid.i > MIN_I

  for _ in range(round(STEERING_OVERRIDE_UNWIND_TIME / DT)):
    update_controller(controller, steering_pressed=True)

  assert abs(controller.pid.i) <= MIN_I * (1.0 + 1e-12)


def test_override_uses_default_fast_unwind_time():
  controller = make_controller()

  for _ in range(round(STEERING_OVERRIDE_UNWIND_TIME / DT)):
    controller.pid.update(0.0, override=True)

  assert abs(controller.pid.i) <= MIN_I * (1.0 + 1e-12)
