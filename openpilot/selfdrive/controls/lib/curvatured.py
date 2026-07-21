import numpy as np

from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup, VERSION

# Cache granularity for get_correction(). Inputs are rounded before comparison so that
# sensor noise does not invalidate the cache between consecutive 100Hz calls.
#
# Rationale per input:
#   CACHE_V_EGO_DECIMALS = 1   (0.1 m/s granularity)
#     - carState.vEgo sensor noise is typically ~0.001 m/s, but during normal
#       driving v_ego rarely changes faster than 0.5 m/s per frame.
#     - 0.1 m/s is loose enough to absorb realistic sensor jitter but still
#       sensitive to actual acceleration/deceleration events.
#   CACHE_CURVATURE_DECIMALS = 7   (1e-7 granularity)
#     - controlsState.modelDesiredCurvature model output has ~1e-6 noise.
#     - 1e-7 is below the model's noise floor so it rounds to the same value
#       across consecutive frames; coarser would miss genuine steering changes.
# Both values are well below steering precision (1 deg of steering ≈ 1e-3 curvature).
CACHE_V_EGO_DECIMALS = 1        # 0.1 m/s
CACHE_CURVATURE_DECIMALS = 7    # 1e-7


class CurvatureDController(CurvatureDLookup):
  def __init__(self) -> None:
    # Cache total_size once; the bucket shape is fixed for the process lifetime.
    self._expected_size: int = self.total_size()
    self.reset()

  def reset(self) -> None:
    self.use_params = False
    self.live_valid = False
    self.fit_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_valid = np.zeros(self.bucket_shape(), dtype=bool)
    self._cached_v_ego_q: float | None = None
    self._cached_curvature_q: float | None = None
    self._cached_projected: float = 0.0

  def update_live_params(self, msg) -> None:
    if msg.version != VERSION or len(msg.corrections) != self._expected_size:
      self.reset()
      return

    # Build all new state from the message first. Then invalidate the cache
    # BEFORE swapping fields, so any concurrent reader sees either the old
    # state with a fresh cache miss (forcing a recompute) or the new state
    # (with cache already empty). The opposite order could briefly leave a
    # reader with the new state but a stale cache hit.
    new_use_params = bool(msg.useParams)
    new_live_valid = bool(msg.liveValid)
    new_fit_corrections = self.unflatten_bucket(msg.corrections, dtype=np.float32)
    if len(msg.fitValid) == self._expected_size:
      new_fit_valid = self.unflatten_bucket(msg.fitValid, dtype=bool)
    else:
      new_fit_valid = np.abs(new_fit_corrections) > 0.0

    if not new_live_valid:
      self.reset()
      return

    # Invalidate-first, then coherent field swap. The tuple assignment makes
    # the cache reset a single bytecode operation, so no reader can observe
    # a half-zeroed cache.
    self._invalidate_correction_cache()
    self.use_params = new_use_params
    self.live_valid = new_live_valid
    self.fit_corrections = new_fit_corrections
    self.fit_valid = new_fit_valid

  def _invalidate_correction_cache(self) -> None:
    # Tuple assignment is a single bytecode op in CPython, so this is
    # effectively atomic with respect to readers (no half-zeroed state).
    self._cached_v_ego_q, self._cached_curvature_q, self._cached_projected = None, None, 0.0

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not self.use_params or not self.live_valid:
      return 0.0

    abs_curvature = abs(float(desired_curvature))
    if self._exceeds_safety_bounds(abs_curvature, v_ego):
      return 0.0

    v_ego_q = round(v_ego, CACHE_V_EGO_DECIMALS)
    curvature_q = round(abs_curvature, CACHE_CURVATURE_DECIMALS)
    if v_ego_q == self._cached_v_ego_q and curvature_q == self._cached_curvature_q:
      projected = self._cached_projected
    else:
      projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs_curvature)
      self._cached_v_ego_q = v_ego_q
      self._cached_curvature_q = curvature_q
      self._cached_projected = projected

    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * projected)
