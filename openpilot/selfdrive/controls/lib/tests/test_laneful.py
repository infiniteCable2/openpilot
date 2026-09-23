import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.laneful import (ARM_TIME, BRIEF_LINE_HOLD, DT_CTRL, ENGAGE_RATE, MAX_ADDED_ACCEL,
                                                       MAX_MODEL_AGE, RELEASE_RATE, LanefulController, lane_target)
from openpilot.selfdrive.modeld.constants import ModelConstants


def model_frame(lane_center=0.0, lane_heading=0.0, e2e_offset=0.0, left_prob=0.98, right_prob=0.98,
                left_std=0.08, right_std=0.08, width=3.6):
  x = np.asarray(ModelConstants.X_IDXS)
  plan_x = np.linspace(0.0, 90.0, 33)
  lines = [SimpleNamespace(x=x, y=np.zeros_like(x)) for _ in range(4)]
  lines[1] = SimpleNamespace(x=x, y=lane_center + lane_heading * x - width / 2.0)
  lines[2] = SimpleNamespace(x=x, y=lane_center + lane_heading * x + width / 2.0)
  return SimpleNamespace(laneLines=lines, laneLineProbs=[0.0, left_prob, right_prob, 0.0],
                         laneLineStds=[0.0, left_std, right_std, 0.0],
                         position=SimpleNamespace(x=plan_x, y=np.full_like(plan_x, e2e_offset)))


class TestLaneful(unittest.TestCase):
  def run_frames(self, controller, model, frames=30, speed=20.0, start_ns=1_000_000_000):
    for i in range(frames):
      model_time = start_ns + i * 50_000_000
      for j in range(5):
        controller.update(model, model_time, model_time + j * 10_000_000, speed, True, True, False)
    return model_time

  def test_spatial_fit_centers_both_directions(self):
    right, strong, quality = lane_target(model_frame(lane_center=0.2), 20.0)
    left, _, _ = lane_target(model_frame(lane_center=-0.2), 20.0)
    self.assertTrue(strong)
    self.assertEqual(quality, 1.0)
    self.assertAlmostEqual(right, -left)
    self.assertGreater(right, 0.0)
    self.assertAlmostEqual(lane_target(model_frame(lane_center=0.2, e2e_offset=0.2), 20.0)[0], 0.0)

  def test_real_modelv2_message_layout(self):
    message = messaging.new_message('modelV2')
    model = message.modelV2
    lines = model.init('laneLines', 4)
    x = ModelConstants.X_IDXS
    for i, line in enumerate(lines):
      line.x = x
      line.y = [0.2 + (-1.8 if i == 1 else 1.8 if i == 2 else 0.0)] * len(x)
    model.laneLineProbs = [0.0, 0.98, 0.98, 0.0]
    model.laneLineStds = [0.0, 0.08, 0.08, 0.0]
    model.position.x = np.linspace(0.0, 90.0, 33).tolist()
    model.position.y = [0.0] * 33
    self.assertGreater(lane_target(model, 20.0)[0], 0.0)

  def test_arms_on_new_frames_and_limits_high_speed_acceleration(self):
    controller = LanefulController()
    model = model_frame(lane_center=0.6)
    self.run_frames(controller, model, frames=10, speed=30.0)
    self.assertFalse(controller.active)
    self.assertEqual(controller.correction, 0.0)
    self.run_frames(controller, model, frames=20, speed=30.0, start_ns=1_500_000_000)
    self.assertTrue(controller.active)
    self.assertGreater(controller.correction, 0.0)
    self.assertLessEqual(controller.correction * 30.0 ** 2, MAX_ADDED_ACCEL + 1e-8)

  def test_invalid_geometry_and_model_staleness_release_smoothly(self):
    controller = LanefulController()
    last = self.run_frames(controller, model_frame(lane_center=0.25))
    previous = controller.correction
    self.assertGreater(previous, 0.0)
    next_time = last + 50_000_000
    held = controller.update(model_frame(lane_center=0.25, width=6.0), next_time, next_time, 20.0, True, True, False)
    self.assertTrue(controller.active)
    self.assertTrue(controller.holding)
    self.assertAlmostEqual(held, previous)
    released = controller.update(model_frame(), next_time, next_time + int((BRIEF_LINE_HOLD + 0.01) * 1e9),
                                 20.0, True, True, False)
    self.assertFalse(controller.active)
    self.assertLessEqual(previous - released, RELEASE_RATE * DT_CTRL + 1e-10)
    for i in range(40):
      controller.update(model_frame(), next_time, next_time + int((MAX_MODEL_AGE + i * DT_CTRL) * 1e9),
                        20.0, True, True, False)
    self.assertEqual(controller.correction, 0.0)

  def test_one_bad_model_frame_recovers_without_rearming(self):
    controller = LanefulController()
    last = self.run_frames(controller, model_frame(lane_center=0.25))
    invalid_time = last + 50_000_000
    controller.update(model_frame(left_prob=0.1), invalid_time, invalid_time, 20.0, True, True, False)
    self.assertTrue(controller.holding)
    good_time = invalid_time + 50_000_000
    controller.update(model_frame(lane_center=0.25), good_time, good_time, 20.0, True, True, False)
    self.assertTrue(controller.active)
    self.assertFalse(controller.holding)

  def test_toggle_and_maneuver_release_without_step(self):
    for enabled, maneuvering in ((False, False), (True, True)):
      controller = LanefulController()
      last = self.run_frames(controller, model_frame(lane_center=0.25))
      previous = controller.correction
      next_time = last + 50_000_000
      released = controller.update(model_frame(lane_center=0.25), next_time, next_time, 20.0,
                                   enabled, True, maneuvering)
      self.assertFalse(controller.active)
      self.assertLessEqual(previous - released, RELEASE_RATE * DT_CTRL + 1e-10)
      self.assertGreater(released, 0.0)

  def test_quality_and_disagreement_suppress_uncertain_lines(self):
    for model in (model_frame(left_prob=0.4), model_frame(right_std=0.4),
                  model_frame(e2e_offset=1.0), model_frame(width=5.0)):
      controller = LanefulController()
      self.run_frames(controller, model)
      self.assertFalse(controller.active)
      self.assertEqual(controller.correction, 0.0)

  def test_model_gap_requires_new_arming_and_new_frames_only(self):
    controller = LanefulController()
    model = model_frame(lane_center=0.2)
    self.run_frames(controller, model, frames=10)
    self.assertLess(controller.arm_time, ARM_TIME)
    controller.update(model, 2_000_000_000, 2_000_000_000, 20.0, True, True, False)
    self.assertFalse(controller.active)
    self.assertLessEqual(controller.arm_time, 0.1)
    for i in range(20):
      controller.update(model, 2_000_000_000, 2_000_000_000 + i * 10_000_000, 20.0, True, True, False)
    self.assertFalse(controller.active)

  def test_entry_rate_is_bounded(self):
    controller = LanefulController()
    model = model_frame(lane_center=0.3)
    last = self.run_frames(controller, model)
    previous = controller.correction
    next_time = last + 50_000_000
    current = controller.update(model, next_time, next_time, 20.0, True, True, False)
    self.assertLessEqual(current - previous, ENGAGE_RATE * DT_CTRL + 1e-10)

  def test_delayed_straight_lane_response_does_not_ping_pong(self):
    controller = LanefulController()
    speed = 20.0
    lateral_offset, heading = 0.2, 0.0
    actuator_delay = deque([0.0] * 15)
    model = model_frame(lane_center=-lateral_offset)
    model_time = 1_000_000_000
    offsets = []
    for tick in range(1400):
      now = 1_000_000_000 + tick * 10_000_000
      if tick % 5 == 0:
        model = model_frame(lane_center=-lateral_offset, lane_heading=-heading)
        model_time = now
      desired = controller.update(model, model_time, now, speed, True, True, False)
      actuator_delay.append(desired)
      applied = actuator_delay.popleft()
      heading += speed * applied * DT_CTRL
      lateral_offset += speed * heading * DT_CTRL
      offsets.append(lateral_offset)
    self.assertLess(abs(lateral_offset), 0.08)
    self.assertLess(max(abs(v) for v in offsets), 0.30)
    self.assertLess(sum(a * b < 0 for a, b in zip(offsets, offsets[1:], strict=False)), 4)


if __name__ == '__main__':
  unittest.main()
