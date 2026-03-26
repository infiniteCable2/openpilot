import numpy as np

from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup, VERSION


class CurvatureDController(CurvatureDLookup):
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.use_params = False
    self.live_valid = False
    self.fit_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_valid = np.zeros(self.bucket_shape(), dtype=bool)

  def update_live_params(self, msg) -> None:
    expected_size = self.total_size()
    if (msg.version != VERSION or
        len(msg.corrections) != expected_size or
        len(msg.counts) != expected_size or
        len(msg.biases) != expected_size):
      self.reset()
      return

    self.use_params = bool(msg.useParams)
    self.live_valid = bool(msg.liveValid)
    self.fit_corrections = self.unflatten_bucket(msg.corrections, dtype=np.float32)

    if len(msg.fitValid) == expected_size:
      self.fit_valid = self.unflatten_bucket(msg.fitValid, dtype=bool)
    else:
      self.fit_valid = np.abs(self.fit_corrections) > 0.0

    if not self.live_valid:
      self.reset()

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not self.use_params or not self.live_valid:
      return 0.0

    abs_curvature = abs(float(desired_curvature))
    if abs_curvature < self.CURVATURE_MIN_STEP or abs_curvature > self.CURVATURE_MAX:
      return 0.0
    if abs_curvature * (float(v_ego) ** 2) > self.MAX_LAT_ACCEL:
      return 0.0

    projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs_curvature)
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * projected)

  def apply(self, desired_curvature: float, v_ego: float) -> float:
    return float(desired_curvature + self.get_correction(desired_curvature, v_ego))
