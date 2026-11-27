import math
import time
from bisect import bisect_right
from collections import deque
from functools import cache

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

VERSION = 2  # The learned bias now includes the previous correction.
HISTORY = 5.0
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 2.0
ALLOWED_CARS = ['volkswagen']
STATUS_LOG_INTERVAL = 10.0
MAX_LEARN_ROLL_LATERAL_ACCEL = 0.10
FIT_REFRESH_EVERY_N_UPDATES = 4
PREVIEW_REFRESH_EVERY_N_UPDATES = 20
MAX_SAMPLE_AGE = 2 * DT_MDL

# CurvatureD learns a small, center-focused curvature correction on top of the model/controller target.
# The goal is not to replace the steering model, but to reduce subtle dynamic-steering mismatch that can
# show up as light ping-pong or center softness around straight driving and shallow bends.
#
# Important magnitude intuition:
# - The corrected range mainly targets small steering wheel angles around center, not large cornering input.
# - Rough real-world feel for this vehicle family:
#   - regular straight / gentle highway lane-keeping tends to stay below ~5e-4
#   - ~1e-3 is still only around a few degrees at the steering wheel (~3 deg order of magnitude)
#   - ~5e-3 is already a clearly visible steering input (~16 deg order of magnitude)
# - In other words, CurvatureD mainly works from near-center out into modest bends, while larger low-speed cornering
#   curvature is only part of the outer fade range and not the primary target.
#
# Safety / scope:
# - Learning is gated by valid upstream pose/calibration, low roll, low yaw uncertainty, and no steering override.
# - TODO: Track 10 s override-free data quality and use it to weight sample counts/EMA updates after the hard override gate.
# - TODO: Latch override events/durations so brief overrides cannot be missed by conflated messaging.
# - Apply is limited to 50% of current curvature and tapers toward the lateral acceleration gate.


class CurvatureDLookup:
  SPEED_ANCHORS = np.array([20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], dtype=np.float32) / 3.6
  CURVATURE_BUCKET_EDGES = np.array([
    1.0e-6,
    2.0e-6,
    4.0e-6,
    8.0e-6,
    1.6e-5,
    3.2e-5,
    6.4e-5,
    1.28e-4,
    2.56e-4,
    5.12e-4,
    1.024e-3,
    2.048e-3,
    4.096e-3,
  ], dtype=np.float32)
  CURVATURE_BUCKET_CENTERS = np.sqrt(CURVATURE_BUCKET_EDGES[:-1] * CURVATURE_BUCKET_EDGES[1:]).astype(np.float32)
  CURVATURE_BUCKET_MIN = float(CURVATURE_BUCKET_EDGES[0])
  CURVATURE_MIN = 0.0
  CURVATURE_BUCKET_MAX = float(CURVATURE_BUCKET_EDGES[-1])
  LAST_BUCKET_WIDTH = float(CURVATURE_BUCKET_EDGES[-1] - CURVATURE_BUCKET_EDGES[-2])
  CURVATURE_MAX = CURVATURE_BUCKET_MAX + LAST_BUCKET_WIDTH

  # Precomputed log of bucket centers for sample interpolation. Avoids recomputing
  # np.log on every interp_curve_value / _interp_curve_impl call.
  _LOG_CENTERS = np.log(CURVATURE_BUCKET_CENTERS.astype(np.float64))
  _EDGES = tuple(CURVATURE_BUCKET_EDGES.tolist())
  _CENTERS = tuple(CURVATURE_BUCKET_CENTERS.tolist())
  _SPEEDS = tuple(SPEED_ANCHORS.tolist())

  MIN_SPEED = float(SPEED_ANCHORS[0] * 0.5)  # learning speed floor
  MAX_LAT_ACCEL_APPLY = 1.0  # apply accel gate
  LAT_ACCEL_FADE = 0.2  # taper over the last 0.2 m/s^2
  RELATIVE_CAP_FULL_RATIO = 0.50  # inner relative cap

  MAX_SAMPLES = 600  # cap on recorded support, not on adaptation
  MEAN_WINDOW = 180.0  # bias EMA horizon
  MIN_REQUIRED_SUPPORT_BUCKETS = 4  # support floor per speed
  SUPPORT_REFERENCE_LAT_ACCEL = 0.05  # maps speed to support width
  MIN_BUCKET_POINTS = np.array([20, 20, 18, 16, 14, 12, 10, 8, 6, 6, 4, 4], dtype=np.float32)  # bucket fit-valid threshold
  FULL_BUCKET_STRENGTH_SAMPLES = MIN_BUCKET_POINTS + MIN_BUCKET_POINTS  # local_strength == 1

  @classmethod
  def bucket_shape(cls) -> tuple[int, int]:
    return len(cls.SPEED_ANCHORS), len(cls.CURVATURE_BUCKET_CENTERS)

  @classmethod
  def total_size(cls) -> int:
    a, b = cls.bucket_shape()
    return a * b

  @classmethod
  def flatten(cls, arr: np.ndarray) -> list:
    return arr.reshape(-1).tolist()

  @classmethod
  def unflatten_bucket(cls, values, dtype=np.float32) -> np.ndarray:
    return np.asarray(values, dtype=dtype).reshape(cls.bucket_shape())

  @classmethod
  def curvature_index(cls, curvature: float) -> int | None:
    abs_curvature = abs(float(curvature))
    if abs_curvature < cls.CURVATURE_BUCKET_MIN or abs_curvature > cls.CURVATURE_BUCKET_MAX:
      return None

    return min(bisect_right(cls._EDGES, abs_curvature) - 1, len(cls._CENTERS) - 1)

  @classmethod
  def speed_index(cls, v_ego: float) -> int | None:
    v = float(v_ego)
    if v < cls.MIN_SPEED:
      return None
    return int(np.argmin(np.abs(cls.SPEED_ANCHORS - v)))

  @classmethod
  def learning_speed_weights(cls, v_ego: float) -> list[tuple[int, float]]:
    v = float(v_ego)
    if v < cls.MIN_SPEED:
      return []

    low, high, alpha = cls.speed_interp(v)
    if low == high:
      return [(low, 1.0)]
    return [(low, 1.0 - alpha), (high, alpha)]

  @classmethod
  def indices(cls, curvature: float, v_ego: float) -> tuple[int, int] | None:
    speed_idx = cls.speed_index(v_ego)
    curvature_idx = cls.curvature_index(curvature)
    if speed_idx is None or curvature_idx is None:
      return None
    return speed_idx, curvature_idx

  @classmethod
  def speed_interp(cls, v_ego: float) -> tuple[int, int, float]:
    v = float(v_ego)
    if v <= cls._SPEEDS[0]:
      return 0, 0, 0.0
    if v >= cls._SPEEDS[-1]:
      last = len(cls._SPEEDS) - 1
      return last, last, 0.0

    high = bisect_right(cls._SPEEDS, v)
    low = high - 1
    alpha = (v - cls._SPEEDS[low]) / (cls._SPEEDS[high] - cls._SPEEDS[low])
    return low, high, alpha

  @classmethod
  def fit_local_strength(cls, bucket_counts: np.ndarray, valid_idx: np.ndarray) -> np.ndarray:
    bucket_conf_start = cls.MIN_BUCKET_POINTS[valid_idx]
    bucket_conf_full = np.asarray(cls.FULL_BUCKET_STRENGTH_SAMPLES[valid_idx], dtype=np.float64)
    bucket_conf_span = bucket_conf_full - bucket_conf_start
    return np.clip(
      (bucket_counts[valid_idx] - bucket_conf_start) / np.maximum(bucket_conf_span, 1.0),
      0.0, 1.0
    ).astype(np.float64)

  @classmethod
  def _build_curve_row(cls, bias: np.ndarray, counts: np.ndarray, speed_idx: int, *, apply: bool) -> tuple[np.ndarray, np.ndarray]:
    if apply:
      caps = cls.RELATIVE_CAP_FULL_RATIO * cls.CURVATURE_BUCKET_CENTERS
      bias = np.clip(bias, -caps, caps)
      valid = (counts >= cls.MIN_BUCKET_POINTS) & cls.apply_bucket_mask(speed_idx)
      idx = np.flatnonzero(valid)
      strength = cls.fit_local_strength(counts, idx)
      width = cls.required_support_bucket_count(speed_idx)
      speed_strength = np.sort(strength)[-width:].sum() / width
    else:
      valid = counts > 0

    row = np.where(valid, bias, 0.0)
    # Smooth only the interior of connected runs.
    interior = valid[:-2] & valid[1:-1] & valid[2:]
    smoothed = 0.25 * row[:-2] + 0.5 * row[1:-1] + 0.25 * row[2:]
    row[1:-1] = np.where(interior, smoothed, row[1:-1])
    if apply:
      row[idx] *= strength * speed_strength
      row = np.clip(row, -caps, caps)
    return row, valid

  @classmethod
  def _build_curve_corrections(cls, bias: np.ndarray, counts: np.ndarray, *, apply: bool) -> tuple[np.ndarray, np.ndarray]:
    corrections = np.empty(cls.bucket_shape(), dtype=np.float32)
    valid = np.empty(cls.bucket_shape(), dtype=bool)
    for speed_idx in range(len(cls._SPEEDS)):
      corrections[speed_idx], valid[speed_idx] = cls._build_curve_row(bias[speed_idx], counts[speed_idx], speed_idx, apply=apply)
    return corrections, valid

  @classmethod
  def bucket_points_for_index(cls, counts: np.ndarray, idx: tuple[int, int] | None) -> int:
    if idx is None:
      return 0
    return int(round(float(counts[idx])))

  @classmethod
  def actual_curvature_from_yaw_rate(cls, yaw_rate: float, v_ego: float) -> float:
    return float(yaw_rate / max(v_ego, 0.1))

  @classmethod
  @cache
  def apply_bucket_mask(cls, speed_idx: int) -> np.ndarray:
    mask = np.zeros(len(cls.CURVATURE_BUCKET_CENTERS), dtype=bool)
    max_bucket_idx = cls.max_supported_bucket_index(float(cls.SPEED_ANCHORS[speed_idx]))
    if max_bucket_idx is not None:
      mask[:max_bucket_idx + 1] = True
    mask.flags.writeable = False
    return mask

  @classmethod
  def speed_curve_valid(cls, counts: np.ndarray, speed_idx: int) -> bool:
    return cls.speed_curve_strength(counts[speed_idx], speed_idx) > 0.0

  @classmethod
  def speed_curve_strength(cls, speed_counts: np.ndarray, speed_idx: int) -> float:
    valid_mask = np.asarray(speed_counts >= cls.MIN_BUCKET_POINTS, dtype=bool) & cls.apply_bucket_mask(speed_idx)
    valid_idx = np.flatnonzero(valid_mask)
    if len(valid_idx) == 0:
      return 0.0

    local_strength = cls.fit_local_strength(speed_counts, valid_idx)
    required_bucket_count = cls.required_support_bucket_count(speed_idx)
    top_strengths = np.sort(local_strength)[-required_bucket_count:]
    return float(np.sum(top_strengths) / float(required_bucket_count))

  @classmethod
  def speed_curve_fully_calibrated(cls, counts: np.ndarray, speed_idx: int) -> bool:
    fully_calibrated_mask = np.asarray(counts[speed_idx] >= cls.FULL_BUCKET_STRENGTH_SAMPLES, dtype=bool) & cls.apply_bucket_mask(speed_idx)
    required_bucket_count = cls.required_support_bucket_count(speed_idx)
    return int(np.count_nonzero(fully_calibrated_mask)) >= required_bucket_count

  @classmethod
  @cache
  def required_support_bucket_count(cls, speed_idx: int) -> int:
    v_ego = float(cls.SPEED_ANCHORS[speed_idx])
    typical_curvature = cls.SUPPORT_REFERENCE_LAT_ACCEL / max(v_ego ** 2, 1e-6)
    bucket_count = int(np.searchsorted(cls.CURVATURE_BUCKET_CENTERS, typical_curvature, side='right'))
    return int(np.clip(bucket_count, cls.MIN_REQUIRED_SUPPORT_BUCKETS, len(cls.CURVATURE_BUCKET_CENTERS)))

  @classmethod
  def calibration_percent(cls, counts: np.ndarray) -> int:
    fully_calibrated_speeds = sum(cls.speed_curve_fully_calibrated(counts, speed_idx) for speed_idx in range(len(cls.SPEED_ANCHORS)))
    return int(round(100.0 * fully_calibrated_speeds / float(len(cls.SPEED_ANCHORS))))

  @classmethod
  def smoothstep(cls, x) -> np.ndarray:
    """Smoothstep, vectorized. Accepts scalar or array input.

    Note: the original scalar-only form is preserved implicitly via np.clip
    on the input - it just no longer raises on arrays.
    """
    y = np.clip(x, 0.0, 1.0)
    return y * y * (3.0 - 2.0 * y)

  @classmethod
  def max_supported_bucket_index(cls, v_ego: float) -> int | None:
    max_curvature = min(cls.MAX_LAT_ACCEL_APPLY / max(float(v_ego) ** 2, 1e-6), cls.CURVATURE_BUCKET_MAX)
    return cls.curvature_index(max_curvature)


  @classmethod
  def learning_error_cap(cls, curvature: float) -> float:
    return float(cls.RELATIVE_CAP_FULL_RATIO * abs(float(curvature)))

  @classmethod
  def projected_error(cls, desired_curvature: float, actual_curvature: float) -> float:
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * (desired_curvature - actual_curvature))

  @classmethod
  def build_fit_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return cls._build_curve_corrections(bias, counts, apply=True)

  @classmethod
  def build_preview_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return cls._build_curve_corrections(bias, counts, apply=False)

  @classmethod
  def valid_runs(cls, valid_mask: np.ndarray) -> list[tuple[int, int]]:
    idx = np.flatnonzero(valid_mask)
    if len(idx) == 0:
      return []

    runs: list[tuple[int, int]] = []
    start = int(idx[0])
    end = start
    for current in idx[1:]:
      current = int(current)
      if current == end + 1:
        end = current
      else:
        runs.append((start, end))
        start = end = current
    runs.append((start, end))
    return runs

  @classmethod
  def interp_curve_value(cls, fit_corrections: np.ndarray, fit_valid: np.ndarray,
                         v_ego: float, abs_curvature: float | np.ndarray, *, apply: bool = True) -> float | np.ndarray:
    if np.ndim(abs_curvature) != 0:
      curvatures = np.asarray(abs_curvature, dtype=np.float64)
      if not math.isfinite(v_ego):
        return np.zeros(curvatures.shape, dtype=np.float32)
      result = cls._interp_curve_impl(fit_corrections, fit_valid, v_ego, curvatures)
      if apply:
        cap = cls.RELATIVE_CAP_FULL_RATIO * curvatures
        fade = cls.smoothstep((cls.MAX_LAT_ACCEL_APPLY - curvatures * v_ego ** 2) / cls.LAT_ACCEL_FADE)
        result = np.clip(result, -cap, cap) * fade
      return np.where(np.isfinite(curvatures) & (curvatures >= cls.CURVATURE_MIN) &
                      (curvatures <= cls.CURVATURE_MAX), result, 0.0)

    k = float(abs_curvature)
    if not math.isfinite(v_ego) or not cls.CURVATURE_MIN <= k <= cls.CURVATURE_MAX:
      return 0.0
    if apply and cls._exceeds_safety_bounds(k, v_ego):
      return 0.0
    low, high, alpha = cls.speed_interp(v_ego)
    idx = bisect_right(cls._EDGES, k) - 1
    value = cls._interp_row_scalar(fit_corrections[low], fit_valid[low], idx, k)
    if alpha > 0.0:
      value += alpha * (cls._interp_row_scalar(fit_corrections[high], fit_valid[high], idx, k) - value)
    if apply:
      cap = cls.RELATIVE_CAP_FULL_RATIO * k
      fade = cls._smoothstep_scalar((cls.MAX_LAT_ACCEL_APPLY - k * v_ego ** 2) / cls.LAT_ACCEL_FADE)
      value = max(-cap, min(cap, value)) * fade
    return value

  @staticmethod
  def _smoothstep_scalar(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)

  @classmethod
  def _interp_row_scalar(cls, curve: np.ndarray, valid: np.ndarray, idx: int, k: float) -> float:
    edges, centers = cls._EDGES, cls._CENTERS
    if idx < 0:
      return float(curve[0]) * cls._smoothstep_scalar(k / edges[0]) if valid[0] else 0.0
    if idx == len(centers):
      return float(curve[-1]) * cls._smoothstep_scalar((cls.CURVATURE_MAX - k) / cls.LAST_BUCKET_WIDTH) if valid[-1] else 0.0
    neighbor = idx - 1 if k < centers[idx] else idx + 1
    neighbor_valid = 0 <= neighbor < len(centers) and valid[neighbor]
    if valid[idx]:
      value = float(curve[idx])
      if neighbor_valid:
        alpha = (math.log(k) - cls._LOG_CENTERS[idx]) / (cls._LOG_CENTERS[neighbor] - cls._LOG_CENTERS[idx])
        value += float(alpha) * (float(curve[neighbor]) - value)
      return value
    if neighbor_valid:
      edge = edges[max(idx, neighbor)]
      alpha = (k - centers[idx]) / (edge - centers[idx])
      return float(curve[neighbor]) * cls._smoothstep_scalar(alpha)
    return 0.0

  @classmethod
  def _exceeds_safety_bounds(cls, abs_curvature: float, v_ego: float) -> bool:
    """Single source of truth for the physical safety bounds.

    Returns True if the requested curvature should not be applied because
    it would exceed the per-vehicle lateral acceleration limit or is
    outside the learned bucket range.
    """
    return (not math.isfinite(v_ego) or not cls.CURVATURE_MIN <= abs_curvature <= cls.CURVATURE_MAX or
            abs_curvature * v_ego ** 2 > cls.MAX_LAT_ACCEL_APPLY)

  @classmethod
  def _interp_curve_impl(cls, fit_corrections: np.ndarray, fit_valid: np.ndarray,
                         v_ego: float, abs_curvatures: np.ndarray) -> np.ndarray:
    """Vectorized core. Always receives a 1-D abs_curvatures array of len >= 1."""
    n = len(abs_curvatures)
    out = np.zeros(n, dtype=np.float32)

    log_curvatures = np.log(np.maximum(abs_curvatures, cls.CURVATURE_BUCKET_MIN))

    # Compute speed interpolation indices once (avoid per-sample calls)
    low_speed, high_speed, speed_alpha = cls.speed_interp(v_ego)
    low_curve = fit_corrections[low_speed]
    high_curve = fit_corrections[high_speed]
    low_valid = fit_valid[low_speed]
    high_valid = fit_valid[high_speed]

    if not low_valid.any() and not high_valid.any():
      return out

    log_centers = cls._LOG_CENTERS
    curvature_edges = cls.CURVATURE_BUCKET_EDGES
    curvature_max = cls.CURVATURE_MAX
    curvature_min = cls.CURVATURE_MIN
    n_centers = len(cls.CURVATURE_BUCKET_CENTERS)

    def interp_speed_row(curve: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
      row_out = np.zeros(n, dtype=np.float32)
      runs = cls.valid_runs(valid_mask)
      if len(runs) == 0:
        return row_out

      for start, end in runs:
        run_idx = np.arange(start, end + 1)
        run_log_x = log_centers[run_idx]
        run_curve = curve[run_idx]
        first_edge = float(curvature_edges[start])
        last_edge = float(curvature_edges[end + 1])

        # Main interpolation: curvature within run range
        in_range = (abs_curvatures >= first_edge) & (abs_curvatures <= last_edge)
        if in_range.any():
          row_out[in_range] = np.interp(log_curvatures[in_range], run_log_x, run_curve).astype(np.float32)

        # Fade in (between previous bucket and first_edge)
        if start > 0:
          fade_in_start = cls._CENTERS[start - 1]
          fade_in_mask = (abs_curvatures >= fade_in_start) & (abs_curvatures < first_edge)
        else:
          fade_in_start = curvature_min
          fade_in_mask = (abs_curvatures >= curvature_min) & (abs_curvatures < first_edge)
        if fade_in_mask.any():
          fade_span = max(first_edge - fade_in_start, 1e-9)
          fade_vals = cls.smoothstep((abs_curvatures[fade_in_mask] - fade_in_start) / fade_span)
          row_out[fade_in_mask] = (run_curve[0] * fade_vals).astype(np.float32)

        # Fade out (between last_edge and next bucket)
        if end < n_centers - 1:
          fade_out_end = cls._CENTERS[end + 1]
          fade_out_mask = (abs_curvatures > last_edge) & (abs_curvatures <= fade_out_end)
        else:
          fade_out_end = curvature_max
          fade_out_mask = (abs_curvatures > last_edge) & (abs_curvatures <= fade_out_end)
        if fade_out_mask.any():
          fade_span = max(fade_out_end - last_edge, 1e-9)
          fade_vals = 1.0 - cls.smoothstep((abs_curvatures[fade_out_mask] - last_edge) / fade_span)
          row_out[fade_out_mask] = (run_curve[-1] * fade_vals).astype(np.float32)

      return row_out

    low_vals = interp_speed_row(low_curve, low_valid)
    if low_speed == high_speed:
      return low_vals
    high_vals = interp_speed_row(high_curve, high_valid)
    return ((1.0 - speed_alpha) * low_vals + speed_alpha * high_vals).astype(np.float32)


class CurvatureEstimator(CurvatureDLookup):
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self.params = Params()
    self.frame = -1
    self.lag = 0.0
    self.hist_len = int(HISTORY / DT_MDL)
    self.calibrator = PoseCalibrator()

    self.bias = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.counts = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_valid = np.zeros(self.bucket_shape(), dtype=bool)
    self.preview_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.preview_valid = np.zeros(self.bucket_shape(), dtype=bool)

    self.car_control_t = deque(maxlen=self.hist_len)
    self.lat_active = deque(maxlen=self.hist_len)
    self.car_state_t = deque(maxlen=self.hist_len)
    self.vego = deque(maxlen=self.hist_len)
    self.steering_pressed = deque(maxlen=self.hist_len)
    self.car_state_ic_t = deque(maxlen=self.hist_len)
    self.steering_slightly_pressed = deque(maxlen=self.hist_len)
    self.controls_state_ic_t = deque(maxlen=self.hist_len)
    self.learning_targets: deque[tuple[float, float] | None] = deque(maxlen=self.hist_len)

    self.last_lat_inactive_t = 0.0
    self.last_override_t = 0.0

    self.current_bucket = (-1, -1)
    self.current_correction = 0.0
    self.current_bias = 0.0
    self.current_bucket_points = 0

    self.use_params = False
    self.enable_curvatured = False
    self.publish_debug_data = False
    self.publish_preview_data = False
    self.prev_use_params = None
    self.last_status_log_t = 0.0
    self.fit_refresh_pending_rows: set[int] = set()
    self.preview_refresh_pending_rows: set[int] = set()
    self.live_pose_update_index = 0
    self.last_fit_refresh_update = -FIT_REFRESH_EVERY_N_UPDATES
    self.last_preview_refresh_update = -PREVIEW_REFRESH_EVERY_N_UPDATES

    self._restore_cached_params()
    self.update_use_params(force=True)

    cloudlog.info(f"curvatured init brand={self.CP.brand} fingerprint={self.CP.carFingerprint} " +
                  f"steerControlType={self.CP.steerControlType} history={HISTORY:.2f}s")

  @staticmethod
  def get_restore_key(CP: car.CarParams, version: int):
    return (CP.carFingerprint, CP.brand, CP.steerControlType.raw, version)

  def _restore_cached_params(self) -> None:
    params_cache = self.params.get("CarParamsPrevRoute")
    curvature_cache = self.params.get("LiveCurvatureParameters")
    if params_cache is None or curvature_cache is None:
      return

    try:
      with log.Event.from_bytes(curvature_cache) as log_evt:
        cache_lcp = log_evt.lateralCurvatureParameters
      with car.CarParams.from_bytes(params_cache) as msg:
        cache_CP = msg

      if self.get_restore_key(cache_CP, cache_lcp.version) != self.get_restore_key(self.CP, VERSION):
        return

      biases = list(cache_lcp.biases)
      counts = list(cache_lcp.counts)
      if len(biases) != self.total_size() or len(counts) != self.total_size():
        raise ValueError("invalid curvature cache shape")

      bias = self.unflatten_bucket(biases)
      support = self.unflatten_bucket(counts)
      if not np.isfinite(bias).all() or np.any(support > self.MAX_SAMPLES):
        raise ValueError("invalid curvature cache values")

      self.bias = bias
      self.counts = support
      self.fit_corrections, self.fit_valid = self.build_fit_corrections(self.bias, self.counts)
      self.preview_refresh_pending_rows.update(np.flatnonzero(self.counts.any(axis=1)).tolist())
      cloudlog.info("restored curvature params from cache")
    except Exception:
      cloudlog.exception("failed to restore cached curvature params")
      self.params.remove("LiveCurvatureParameters")

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
      self.publish_debug_data = self.params.get_bool("CurvatureDDebugData")
      self.publish_preview_data = self.publish_debug_data or self.params.get_bool("ShowDynamicSteeringLearnerGraph")
      self.use_params = self.enable_curvatured and self.CP.brand in ALLOWED_CARS and \
                        self.CP.steerControlType == car.CarParams.SteerControlType.curvature
      if self.prev_use_params != self.use_params:
        cloudlog.info(f"curvatured use_params={self.use_params} toggle={self.enable_curvatured} " +
                      f"brand={self.CP.brand} allowed={self.CP.brand in ALLOWED_CARS} " +
                      f"steerControlType={self.CP.steerControlType}")
        if not self.use_params:
          for d in [self.car_control_t, self.lat_active,
                    self.car_state_t, self.vego, self.steering_pressed,
                    self.car_state_ic_t, self.steering_slightly_pressed,
                     self.controls_state_ic_t, self.learning_targets]:
            d.clear()
          self.last_lat_inactive_t = 0.0
          self.last_override_t = 0.0
        self.prev_use_params = self.use_params
    self.frame += 1

  def _history_ready(self) -> bool:
    histories = (self.car_control_t, self.car_state_t, self.car_state_ic_t, self.controls_state_ic_t)
    return min(map(len, histories)) == self.hist_len

  @staticmethod
  def _sample_at_or_before(target_t: float, ts: deque, values: deque, max_age: float = math.inf):
    for t, value in zip(reversed(ts), reversed(values), strict=True):
      if t <= target_t:
        return value if target_t - t <= max_age else None
    return None

  def _steering_override_at(self, t: float) -> bool | None:
    steering_pressed = self._sample_at_or_before(t, self.car_state_t, self.steering_pressed)
    steering_slightly_pressed = self._sample_at_or_before(t, self.car_state_ic_t, self.steering_slightly_pressed)
    if steering_pressed is None or steering_slightly_pressed is None:
      return None
    return bool(steering_pressed or steering_slightly_pressed)

  def add_measurement(self, desired_curvature: float, actual_curvature: float, v_ego: float,
                      schedule_only: bool = False, *, commanded_correction: float = 0.0) -> None:
    if not all(map(math.isfinite, (desired_curvature, actual_curvature, v_ego, commanded_correction))):
      return
    curvature_idx = self.curvature_index(desired_curvature)
    speed_weights = self.learning_speed_weights(v_ego)
    if curvature_idx is None or len(speed_weights) == 0:
      return

    cap = self.learning_error_cap(desired_curvature)
    # For a local unit-gain response, the previous correction plus the remaining error estimates the total bias.
    estimate = self.projected_error(desired_curvature, actual_curvature - commanded_correction)
    estimate = max(-cap, min(cap, estimate))

    for speed_idx, weight in speed_weights:
      if weight <= 0.0:
        continue

      idx = (speed_idx, curvature_idx)
      prev_count, prev_bias = float(self.counts[idx]), float(self.bias[idx])
      sample_count = min(prev_count + weight, self.MAX_SAMPLES)
      alpha = weight / min(sample_count, self.MEAN_WINDOW)
      self.counts[idx] = sample_count
      self.bias[idx] = prev_bias + alpha * (estimate - prev_bias)
      bias_changed = self.bias[idx] != prev_bias
      if bias_changed or prev_count == 0.0:
        self.preview_refresh_pending_rows.add(speed_idx)
      strength_changed = sample_count != prev_count and prev_count < self.FULL_BUCKET_STRENGTH_SAMPLES[curvature_idx]
      if (bias_changed or strength_changed) and sample_count >= self.MIN_BUCKET_POINTS[curvature_idx] and self.apply_bucket_mask(speed_idx)[curvature_idx]:
        self.fit_refresh_pending_rows.add(speed_idx)

    if not schedule_only:
      self.refresh_curve_lookups(self.live_pose_update_index, force_fit=True, force_preview=True)

  def refresh_curve_lookups(self, update_index: int, force_fit: bool = False, force_preview: bool = False) -> None:
    if self.fit_refresh_pending_rows and (force_fit or update_index - self.last_fit_refresh_update >= FIT_REFRESH_EVERY_N_UPDATES):
      for idx in self.fit_refresh_pending_rows:
        self.fit_corrections[idx], self.fit_valid[idx] = self._build_curve_row(self.bias[idx], self.counts[idx], idx, apply=True)
      self.fit_refresh_pending_rows.clear()
      self.last_fit_refresh_update = update_index

    if (self.publish_preview_data or force_preview) and self.preview_refresh_pending_rows and (
        force_preview or update_index - self.last_preview_refresh_update >= PREVIEW_REFRESH_EVERY_N_UPDATES):
      for idx in self.preview_refresh_pending_rows:
        self.preview_corrections[idx], self.preview_valid[idx] = self._build_curve_row(self.bias[idx], self.counts[idx], idx, apply=False)
      self.preview_refresh_pending_rows.clear()
      self.last_preview_refresh_update = update_index


  def _update_current_lookup(self, desired_curvature: float, v_ego: float) -> None:
    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      self.current_bucket = (-1, -1)
      self.current_correction = 0.0
      self.current_bias = 0.0
      self.current_bucket_points = 0
      return

    speed_idx, curvature_idx = idx
    self.current_bucket = idx
    self.current_bias = float(self.bias[speed_idx, curvature_idx])
    self.current_bucket_points = self.bucket_points_for_index(self.counts, idx)

    if not self.use_params:
      self.current_correction = 0.0
      return
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs(desired_curvature))
    self.current_correction = float(direction * projected)

  def handle_log(self, t: float, which: str, msg) -> None:
    if not self.use_params:
      if which == "extrinsicsCalibration":
        self.calibrator.feed_extrinsics_calibration(msg)
      elif which == "lateralDelay":
        self.lag = get_lat_delay(self.params, msg.lateralDelay)
      return

    if which == "carControl":
      self.car_control_t.append(t)
      self.lat_active.append(msg.latActive)
      if not msg.latActive:
        self.last_lat_inactive_t = t
    elif which == "carState":
      self.car_state_t.append(t)
      self.vego.append(msg.vEgo)
      self.steering_pressed.append(bool(msg.steeringPressed))
      if msg.steeringPressed:
        self.last_override_t = t
    elif which == "carStateIC":
      self.car_state_ic_t.append(t)
      self.steering_slightly_pressed.append(bool(msg.steeringSlightlyPressed))
      if msg.steeringSlightlyPressed:
        self.last_override_t = t
    elif which == "controlsStateIC":
      self.controls_state_ic_t.append(t)
      learning = msg.curvatureLearning
      self.learning_targets.append((learning.desiredCurvature, learning.correction) if learning.valid else None)
      v_ego = self._sample_at_or_before(t, self.car_state_t, self.vego)
      if v_ego is not None:
        self._update_current_lookup(learning.desiredCurvature if learning.valid else msg.modelDesiredCurvature, v_ego)
    elif which == "extrinsicsCalibration":
      self.calibrator.feed_extrinsics_calibration(msg)
    elif which == "lateralDelay":
      self.lag = get_lat_delay(self.params, msg.lateralDelay)
    elif which == "deviceMotion" and self.use_params:
      self.live_pose_update_index += 1
      if not self._history_ready():
        return
      if not (msg.angularVelocityDevice.valid and msg.orientationNED.valid and msg.posenetOK and msg.inputsOK and
              msg.sensorsOK and self.calibrator.calib_valid):
        return
      pose_t = msg.timestamp * 1e-9
      if not 0.0 < pose_t <= t + 1e-6 or t - pose_t > MAX_SAMPLE_AGE:
        return
      t = pose_t
      if (t - self.last_lat_inactive_t) < MIN_ENGAGE_BUFFER or (t - self.last_override_t) < MIN_ENGAGE_BUFFER:
        return

      target_t = t - self.lag
      lat_active = self._sample_at_or_before(target_t, self.car_control_t, self.lat_active, MAX_SAMPLE_AGE)
      v_ego = self._sample_at_or_before(target_t, self.car_state_t, self.vego, MAX_SAMPLE_AGE)
      measured_v_ego = self._sample_at_or_before(t, self.car_state_t, self.vego, MAX_SAMPLE_AGE)
      learning = self._sample_at_or_before(target_t, self.controls_state_ic_t, self.learning_targets, MAX_SAMPLE_AGE)
      # Driver overrides apply immediately; only command-derived signals are lag aligned.
      steering_override = self._steering_override_at(t)

      if v_ego is None or measured_v_ego is None or learning is None or steering_override is None:
        return

      if not lat_active or steering_override or min(v_ego, measured_v_ego) < self.MIN_SPEED:
        return

      calibrated_pose = self.calibrator.build_calibrated_pose(Pose.from_device_motion(msg))
      if not self.roll_learning_allowed(calibrated_pose.orientation.roll):
        return
      yaw_rate = calibrated_pose.angular_velocity.yaw
      yaw_rate_std = calibrated_pose.angular_velocity.yaw_std
      if yaw_rate_std >= MAX_YAW_RATE_STD:
        return

      desired_curvature, correction = learning
      # The controller already compensates roll in the steering command. Compare the realized path to the path target.
      actual_curvature = self.actual_curvature_from_yaw_rate(yaw_rate, measured_v_ego)
      self.add_measurement(desired_curvature, actual_curvature, v_ego, schedule_only=True, commanded_correction=correction)

  def get_msg(self, valid: bool = True, live_valid: bool = True,
              include_debug: bool = False, include_preview: bool = False):
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.valid = valid

    curvature_params = msg.lateralCurvatureParameters
    curvature_params.liveValid = bool(live_valid) and bool(np.isfinite(self.bias).all()) and bool(np.isfinite(self.fit_corrections).all())
    curvature_params.version = VERSION
    curvature_params.useParams = self.use_params
    curvature_params.currentCorrection = self.current_correction if self.use_params else 0.0
    curvature_params.currentBias = self.current_bias if self.use_params else 0.0
    curvature_params.currentBucketPoints = self.current_bucket_points if self.use_params else 0
    curvature_params.totalBucketPoints = int(round(float(self.counts.sum())))
    curvature_params.calPerc = self.calibration_percent(self.counts)
    curvature_params.bucketSpeed = int(self.current_bucket[0]) if self.use_params else -1
    curvature_params.bucketCurvature = int(self.current_bucket[1]) if self.use_params else -1
    curvature_params.corrections = self.flatten(self.fit_corrections)
    curvature_params.fitValid = self.flatten(self.fit_valid)
    if include_debug:
      curvature_params.counts = self.flatten(np.rint(self.counts).astype(np.uint16))
      curvature_params.biases = self.flatten(self.bias)
    if include_preview:
      curvature_params.previewCorrections = self.flatten(self.preview_corrections)
      curvature_params.previewValid = self.flatten(self.preview_valid)
    return msg

  @staticmethod
  def roll_learning_allowed(roll: float) -> bool:
    return abs(np.sin(float(roll)) * ACCELERATION_DUE_TO_GRAVITY) <= MAX_LEARN_ROLL_LATERAL_ACCEL

  def maybe_log_status(self, t: float, sm, services: list[str] | None = None, valid: bool | None = None) -> None:
    if t < self.last_status_log_t + STATUS_LOG_INTERVAL:
      return

    tracked_services = list(sm.valid.keys()) if services is None else services
    invalid = [s for s in tracked_services if not sm.valid[s]]
    not_alive = [s for s in tracked_services if not sm.alive[s]]
    self.last_status_log_t = t

    checks = sm.all_checks(tracked_services) if valid is None else valid
    cloudlog.info(f"curvatured status use_params={self.use_params} checks={checks} " +
                  f"lag={self.lag:.3f} total_points={int(round(float(self.counts.sum())))} " +
                  f"bucket={self.current_bucket} bucket_points={self.current_bucket_points} " +
                  f"corr={self.current_correction:.8f} cal={self.calibration_percent(self.counts)} " +
                   f"invalid={invalid} not_alive={not_alive}")


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['lateralCurvatureParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'carStateIC', 'extrinsicsCalibration', 'lateralDelay',
                            'controlsStateIC', 'deviceMotion'], poll='deviceMotion')

  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  curvature_estimator = CurvatureEstimator(CP)

  while True:
    sm.update()

    if sm.all_checks():
      for which in sm.updated.keys():
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          try:
            curvature_estimator.handle_log(t, which, sm[which])
          except Exception:
            cloudlog.exception(f"curvatured handle_log failed service={which}")

    curvature_estimator.update_use_params()
    curvature_estimator.refresh_curve_lookups(curvature_estimator.live_pose_update_index)

    # 4Hz driven by deviceMotion
    if sm.frame % 5 == 0:
      live_valid = sm.all_checks() and curvature_estimator.use_params
      curvature_estimator.maybe_log_status(time.monotonic(), sm)
      pm.send('lateralCurvatureParameters',
              curvature_estimator.get_msg(valid=sm.all_checks(),
                                          live_valid=live_valid,
                                          include_debug=curvature_estimator.publish_debug_data,
                                          include_preview=curvature_estimator.publish_preview_data))

    # Cache every 240 deviceMotion ticks (about 12 seconds).
    if sm.frame % 240 == 0:
      live_valid = sm.all_checks() and curvature_estimator.use_params
      params.put("LiveCurvatureParameters", curvature_estimator.get_msg(valid=sm.all_checks(),
                                                                        live_valid=live_valid,
                                                                        include_debug=True,
                                                                        include_preview=False).to_bytes())


if __name__ == "__main__":
  main()
