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
    # Cache state for get_correction(): avoids recomputing the (vectorized) interpolation
    # when the (v_ego, abs_curvature) inputs are stable across consecutive 100Hz calls.
    # Quantization: v_ego rounded to 0.01 m/s (~0.04 km/h), abs_curvature to 1e-7 (~sub-cm radius).
    # These levels are well below steering precision but high enough to ride out sensor noise.
    self._cached_v_ego_q: float | None = None
    self._cached_curvature_q: float | None = None
    self._cached_projected: float = 0.0

  def update_live_params(self, msg) -> None:
    expected_size = self.total_size()
    if msg.version != VERSION or len(msg.corrections) != expected_size:
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
      return

    # fit_corrections / fit_valid just changed; previous cached correction is now stale.
    self._invalidate_correction_cache()

  def _invalidate_correction_cache(self) -> None:
    self._cached_v_ego_q = None
    self._cached_curvature_q = None
    self._cached_projected = 0.0

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not self.use_params or not self.live_valid:
      return 0.0

    abs_curvature = abs(float(desired_curvature))
    if abs_curvature < self.CURVATURE_MIN or abs_curvature > self.CURVATURE_MAX:
      return 0.0
    if abs_curvature * (float(v_ego) ** 2) > self.MAX_LAT_ACCEL_APPLY:
      return 0.0

    # Quantize inputs to ride out sensor noise. Both round() operations are O(1).
    v_ego_q = round(v_ego, 2)
    curvature_q = round(abs_curvature, 7)
    if v_ego_q == self._cached_v_ego_q and curvature_q == self._cached_curvature_q:
      projected = self._cached_projected
    else:
      projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs_curvature)
      self._cached_v_ego_q = v_ego_q
      self._cached_curvature_q = curvature_q
      self._cached_projected = projected

    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * projected)

  def apply(self, desired_curvature: float, v_ego: float) -> float:
    return float(desired_curvature + self.get_correction(desired_curvature, v_ego))
