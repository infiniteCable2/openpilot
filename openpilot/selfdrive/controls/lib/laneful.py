"""Bounded lane geometry correction for the model's lateral action.

The model action remains the feedforward command. This controller uses the
model's own inner lane lines to bias that command toward the lane midpoint.
It is independent of the vehicle interface and of steering-wheel input.
"""

import math

import numpy as np

from openpilot.common.realtime import DT_CTRL


MIN_SPEED = 8.0
ARM_TIME = 0.75
MAX_MODEL_AGE = 0.25
BRIEF_LINE_HOLD = 0.15
MIN_WIDTH, MAX_WIDTH = 2.6, 4.6
MAX_WIDTH_CHANGE = 0.6
MAX_PATH_DISAGREEMENT = 0.8
MAX_PATH_SHIFT = 0.30
MAX_CORRECTION = 0.00045
MAX_ADDED_ACCEL = 0.35
ENGAGE_RATE = 0.0008   # curvature / s; full entry takes at least 0.5 s
RELEASE_RATE = 0.0015  # curvature / s; release is quicker than entry
BLEND_START, BLEND_END = 10.0, 45.0
HEADING_GAIN = 0.55


def smoothstep(x: np.ndarray) -> np.ndarray:
  x = np.clip(x, 0.0, 1.0)
  return x ** 3 * (10.0 + x * (-15.0 + 6.0 * x))


def virtual_path(center: np.ndarray, e2e: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
  """Align E2E to the lane midpoint, then blend its residual path shape."""
  heading, offset = np.polyfit(x, center - e2e, 1)
  gain = smoothstep((x - BLEND_START) / (BLEND_END - BLEND_START))
  aligned = e2e + offset + heading * x
  return aligned + gain * (center - aligned), float(heading), gain


def lane_target(model, speed: float) -> tuple[float, bool, float]:
  """Return curvature correction, strong-confidence flag and quality.

  All distances and lateral coordinates come from the same modelV2 message.
  Lane geometry is never extrapolated beyond the E2E path's coverage.
  """
  left, right = model.laneLines[1], model.laneLines[2]
  x = np.asarray(left.x, dtype=np.float64)
  right_x = np.asarray(right.x, dtype=np.float64)
  left_y = np.asarray(left.y, dtype=np.float64)
  right_y = np.asarray(right.y, dtype=np.float64)
  plan_x = np.asarray(model.position.x, dtype=np.float64)
  plan_y = np.asarray(model.position.y, dtype=np.float64)
  if (len(x) < 8 or len(x) != len(right_x) or len(x) != len(left_y) or len(x) != len(right_y) or
      len(plan_x) < 4 or len(plan_x) != len(plan_y) or
      not all(np.all(np.isfinite(a)) for a in (x, right_x, left_y, right_y, plan_x, plan_y)) or
      np.any(np.diff(x) <= 0.0) or np.any(np.diff(plan_x) <= 0.0) or
      not np.allclose(x, right_x, atol=0.1) or plan_x[0] > 8.0 or plan_x[-1] < 30.0):
    raise ValueError("invalid or uncovered lane/E2E path")

  probs = model.laneLineProbs
  stds = model.laneLineStds
  if len(probs) < 3 or len(stds) < 3:
    raise ValueError("missing lane confidence")
  probability = min(float(probs[1]), float(probs[2]))
  std = max(float(stds[1]), float(stds[2]))
  if not math.isfinite(probability) or not math.isfinite(std) or probability < 0.70 or std > 0.30:
    raise ValueError("weak lane confidence")

  fit = (x >= 8.0) & (x <= min(55.0, float(plan_x[-1])))
  if np.count_nonzero(fit) < 5 or x[fit][-1] < 28.0:
    raise ValueError("short shared path horizon")
  widths = right_y[fit] - left_y[fit]
  low, median, high = np.percentile(widths, (10.0, 50.0, 90.0))
  if low < MIN_WIDTH or high > MAX_WIDTH or high - low > MAX_WIDTH_CHANGE:
    raise ValueError("implausible lane width")

  center = (left_y[fit] + right_y[fit]) / 2.0
  if abs(float((left_y[0] + right_y[0]) / 2.0)) > 0.9:
    raise ValueError("vehicle far from lane center")
  fit_x = x[fit]
  e2e = np.interp(fit_x, plan_x, plan_y)
  relative = center - e2e
  disagreement = float(np.percentile(np.abs(relative), 90.0))
  if disagreement >= MAX_PATH_DISAGREEMENT:
    raise ValueError("lane and E2E disagree")

  # Match the E2E path's lateral offset and heading to the lane center across
  # the shared horizon, then blend only its remaining shape toward the lanes.
  target, heading, gain = virtual_path(center, e2e, fit_x)
  field = gain * (target - e2e - (1.0 - HEADING_GAIN) * heading * fit_x)

  lookahead = min(float(np.clip(1.5 * speed, 20.0, 45.0)), float(fit_x[-1]))
  weights = np.exp(-0.5 * ((fit_x - lookahead) / (0.25 * lookahead)) ** 2)
  weights *= np.gradient(fit_x) * fit_x ** 2
  denominator = float(np.dot(weights, gain))
  if denominator <= 1e-9:
    raise ValueError("empty lane fit")
  error = float(smoothstep(np.array((lookahead - BLEND_START) / (BLEND_END - BLEND_START))) *
                np.dot(weights, field) / denominator)
  if not math.isfinite(error):
    raise ValueError("invalid lane fit")

  strong = probability >= 0.90 and std <= 0.20
  prob_quality = float(np.clip((probability - 0.70) / 0.20, 0.0, 1.0))
  std_quality = float(np.clip((0.30 - std) / 0.15, 0.0, 1.0))
  disagreement_quality = float(np.clip((MAX_PATH_DISAGREEMENT - disagreement) / 0.45, 0.0, 1.0))
  quality = min(prob_quality, std_quality, disagreement_quality)
  correction = 2.0 * float(np.clip(error, -MAX_PATH_SHIFT, MAX_PATH_SHIFT)) / lookahead ** 2
  return correction, strong, quality


class LanefulController:
  def __init__(self):
    self.correction = 0.0
    self.target = 0.0
    self.quality = 0.0
    self.arm_time = 0.0
    self.active = False
    self.last_model_time = 0
    self.last_good_time = 0
    self.holding = False

  def update(self, model, model_time_ns: int, now_ns: int, speed: float, enabled: bool,
             model_valid: bool, maneuvering: bool) -> float:
    age = (now_ns - model_time_ns) * 1e-9 if model_time_ns else float('inf')
    usable = (enabled and model_valid and not maneuvering and math.isfinite(speed) and speed >= MIN_SPEED and
              -0.05 <= age <= MAX_MODEL_AGE)
    if not usable:
      self.active = False
      self.arm_time = self.target = self.quality = 0.0
      self.holding = False
    elif model_time_ns != self.last_model_time:
      gap = (model_time_ns - self.last_model_time) * 1e-9 if self.last_model_time else 0.05
      self.last_model_time = model_time_ns
      if gap <= 0.0 or gap > MAX_MODEL_AGE:
        self.active = False
        self.arm_time = 0.0
        gap = 0.05
      try:
        correction, strong, quality = lane_target(model, speed)
      except (AttributeError, IndexError, TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError):
        # One missed detection should not cause a release and a second 0.75 s
        # arm cycle. Retain the last bounded command briefly, then fade it.
        self.holding = self.active and 0 <= (now_ns - self.last_good_time) * 1e-9 <= BRIEF_LINE_HOLD
        self.quality = 0.0
        if not self.holding:
          self.active = False
          self.arm_time = self.target = 0.0
      else:
        self.holding = False
        self.last_good_time = model_time_ns
        if not self.active:
          self.arm_time = min(ARM_TIME, self.arm_time + min(gap, 0.1)) if strong else 0.0
          self.active = self.arm_time >= ARM_TIME
        self.quality = quality if self.active else 0.0
        self.target = correction * self.quality if self.active else 0.0

    if self.holding and (now_ns - self.last_good_time) * 1e-9 > BRIEF_LINE_HOLD:
      self.holding = self.active = False
      self.arm_time = self.target = 0.0

    limit = min(MAX_CORRECTION, MAX_ADDED_ACCEL / max(speed ** 2, MIN_SPEED ** 2)) if math.isfinite(speed) else 0.0
    target = float(np.clip(self.target, -limit, limit))
    rate = RELEASE_RATE if abs(target) < abs(self.correction) or target * self.correction < 0.0 else ENGAGE_RATE
    self.correction += float(np.clip(target - self.correction, -rate * DT_CTRL, rate * DT_CTRL))
    self.correction = float(np.clip(self.correction, -limit, limit))
    return self.correction
