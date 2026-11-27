import math

import numpy as np

from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup, VERSION

CURVATURE_SERVICE = "lateralCurvatureParameters"


class CurvatureDController(CurvatureDLookup):
  def __init__(self) -> None:
    self._expected_size = self.total_size()
    self.reset()

  def reset(self) -> None:
    self.use_params = False
    self.live_valid = False
    self.fit_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_valid = np.zeros(self.bucket_shape(), dtype=bool)
    self._params_frame: int | None = None

  def update_from_submaster(self, sm) -> None:
    if not sm.all_checks([CURVATURE_SERVICE]):
      if self._params_frame is not None:
        self.reset()
      return

    frame = sm.recv_frame[CURVATURE_SERVICE]
    if frame != self._params_frame:
      self.update_live_params(sm[CURVATURE_SERVICE])
      self._params_frame = frame

  def update_live_params(self, msg) -> None:
    if (msg.version != VERSION or not msg.liveValid or not msg.useParams or
        len(msg.corrections) != self._expected_size or len(msg.fitValid) != self._expected_size):
      self.reset()
      return

    corrections = self.unflatten_bucket(msg.corrections)
    if not np.isfinite(corrections).all():
      self.reset()
      return

    self.fit_corrections = corrections
    self.fit_valid = self.unflatten_bucket(msg.fitValid, dtype=bool)
    self.use_params = True
    self.live_valid = True

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not self.use_params or not self.live_valid:
      return 0.0

    projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs(desired_curvature))
    return float(projected) if math.copysign(1.0, desired_curvature) > 0 else -float(projected)
