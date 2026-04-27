import math
from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

VERSION = 1
HISTORY = 5.0
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 2.0
ALLOWED_CARS = ['volkswagen']
STATUS_LOG_INTERVAL = 10.0
MAX_LEARN_ROLL_LATERAL_ACCEL = 0.10
FIT_REFRESH_EVERY_N_UPDATES = 4
PREVIEW_REFRESH_EVERY_N_UPDATES = 20

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
# - Corrections are bounded by a relative cap envelope over the speed-available buckets:
#   - up to 50% of local curvature through the last still-supported bucket
#   - from there, the cap fades toward 0 at the next outer bucket center
# - Apply magnitude is limited by the relative cap envelope and the lateral-accel apply gate.


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

  MIN_SPEED = float(SPEED_ANCHORS[0] * 0.5)  # learning/apply speed floor
  MAX_LAT_ACCEL_APPLY = 1.0  # apply accel gate
  RELATIVE_CAP_FULL_RATIO = 0.50  # inner relative cap

  MAX_SAMPLES = 600  # per-bucket saturation
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

    idx = int(np.searchsorted(cls.CURVATURE_BUCKET_EDGES, abs_curvature, side='right') - 1)
    return min(max(idx, 0), len(cls.CURVATURE_BUCKET_CENTERS) - 1)

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
    if v <= cls.SPEED_ANCHORS[0]:
      return 0, 0, 0.0
    if v >= cls.SPEED_ANCHORS[-1]:
      last = len(cls.SPEED_ANCHORS) - 1
      return last, last, 0.0

    high = int(np.searchsorted(cls.SPEED_ANCHORS, v, side='right'))
    low = high - 1
    span = float(cls.SPEED_ANCHORS[high] - cls.SPEED_ANCHORS[low])
    alpha = (v - float(cls.SPEED_ANCHORS[low])) / max(span, 1e-6)
    return low, high, float(np.clip(alpha, 0.0, 1.0))

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
  def preview_local_strength(cls, bucket_counts: np.ndarray, valid_idx: np.ndarray) -> np.ndarray:
    return np.ones(len(valid_idx), dtype=np.float64)

  @classmethod
  def _build_curve_corrections(cls, bias: np.ndarray, counts: np.ndarray,
                               valid_mask_fn,
                               min_valid_buckets_fn,
                               local_strength_fn,
                               speed_strength_fn,
                               apply_cap: bool = True,
                               zero_invalid_buckets: bool = False) -> tuple[np.ndarray, np.ndarray]:
    corrections = np.zeros(cls.bucket_shape(), dtype=np.float32)
    valid = np.zeros(cls.bucket_shape(), dtype=bool)

    for speed_idx in range(len(cls.SPEED_ANCHORS)):
      curve_valid = np.asarray(valid_mask_fn(counts[speed_idx]), dtype=bool)
      if apply_cap:
        curve_valid &= cls.apply_bucket_mask(speed_idx)
      if int(np.count_nonzero(curve_valid)) < int(min_valid_buckets_fn(speed_idx)):
        continue

      valid_idx = np.flatnonzero(curve_valid)
      if apply_cap:
        bucket_caps = np.asarray([cls.correction_cap(float(curvature), float(cls.SPEED_ANCHORS[speed_idx]))
                                  for curvature in cls.CURVATURE_BUCKET_CENTERS], dtype=np.float64)
      else:
        bucket_caps = np.full(len(cls.CURVATURE_BUCKET_CENTERS), np.inf, dtype=np.float64)
      local_strength = local_strength_fn(counts[speed_idx], valid_idx)
      speed_strength = float(speed_strength_fn(counts[speed_idx], speed_idx, valid_idx, local_strength))
      row = np.zeros(len(cls.CURVATURE_BUCKET_CENTERS), dtype=np.float32)

      for start, end in cls.valid_runs(curve_valid):
        run_idx = np.arange(start, end + 1)
        run_curve = np.clip(bias[speed_idx, run_idx], -bucket_caps[run_idx], bucket_caps[run_idx]).astype(np.float32)
        run_strength = local_strength_fn(counts[speed_idx], run_idx).astype(np.float32)

        if len(run_curve) >= 3:
          smoothed_run = run_curve.copy()
          smoothed_run[1:-1] = 0.25 * run_curve[:-2] + 0.5 * run_curve[1:-1] + 0.25 * run_curve[2:]
        else:
          smoothed_run = run_curve

        run_values = speed_strength * run_strength * smoothed_run
        row[run_idx] = np.clip(run_values, -bucket_caps[run_idx], bucket_caps[run_idx]) if apply_cap else run_values

      if zero_invalid_buckets:
        row = np.where(curve_valid, row, 0.0)

      corrections[speed_idx] = row.astype(np.float32)
      valid[speed_idx] = curve_valid

    return corrections, valid

  @classmethod
  def bucket_points_for_index(cls, counts: np.ndarray, idx: tuple[int, int] | None) -> int:
    if idx is None:
      return 0
    return int(round(float(counts[idx])))

  @classmethod
  def actual_curvature_from_yaw_rate(cls, yaw_rate: float, v_ego: float, roll_compensation: float = 0.0) -> float:
    return float(yaw_rate / max(float(v_ego), 0.1) - float(roll_compensation))

  @classmethod
  def apply_bucket_mask(cls, speed_idx: int) -> np.ndarray:
    mask = np.zeros(len(cls.CURVATURE_BUCKET_CENTERS), dtype=bool)
    max_bucket_idx = cls.max_supported_bucket_index(float(cls.SPEED_ANCHORS[speed_idx]))
    if max_bucket_idx is None:
      return mask
    mask[:max_bucket_idx + 1] = True
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
  def smoothstep(cls, x: float) -> float:
    y = float(np.clip(x, 0.0, 1.0))
    return y * y * (3.0 - 2.0 * y)

  @classmethod
  def max_supported_bucket_index(cls, v_ego: float) -> int | None:
    max_curvature = min(cls.MAX_LAT_ACCEL_APPLY / max(float(v_ego) ** 2, 1e-6), cls.CURVATURE_BUCKET_MAX)
    return cls.curvature_index(max_curvature)

  @classmethod
  def cap_zero_curvature(cls, v_ego: float) -> float:
    max_bucket_idx = cls.max_supported_bucket_index(v_ego)
    if max_bucket_idx is None:
      return cls.CURVATURE_BUCKET_MIN

    next_idx = max_bucket_idx + 1
    if next_idx < len(cls.CURVATURE_BUCKET_CENTERS):
      return float(cls.CURVATURE_BUCKET_CENTERS[next_idx])
    return cls.CURVATURE_MAX

  @classmethod
  def correction_cap_ratio(cls, curvature: float, v_ego: float) -> float:
    abs_curvature = abs(float(curvature))
    if abs_curvature <= cls.CURVATURE_MIN:
      return 0.0

    if abs_curvature < cls.CURVATURE_BUCKET_MIN:
      inner_alpha = cls.smoothstep((abs_curvature - cls.CURVATURE_MIN) /
                                   max(cls.CURVATURE_BUCKET_MIN - cls.CURVATURE_MIN, 1e-9))
      return float(inner_alpha * cls.RELATIVE_CAP_FULL_RATIO)

    bucket_idx = cls.curvature_index(abs_curvature)
    if bucket_idx is None:
      return 0.0

    max_bucket_idx = cls.max_supported_bucket_index(v_ego)
    if max_bucket_idx is None:
      return 0.0

    if bucket_idx <= max_bucket_idx:
      return cls.RELATIVE_CAP_FULL_RATIO

    fade_start = float(cls.CURVATURE_BUCKET_CENTERS[max_bucket_idx])
    fade_end = cls.cap_zero_curvature(v_ego)
    if abs_curvature >= fade_end:
      return 0.0

    alpha = cls.smoothstep((abs_curvature - fade_start) / max(fade_end - fade_start, 1e-9))
    return float((1.0 - alpha) * cls.RELATIVE_CAP_FULL_RATIO)

  @classmethod
  def correction_cap(cls, curvature: float, v_ego: float) -> float:
    abs_curvature = abs(float(curvature))
    return float(cls.correction_cap_ratio(abs_curvature, v_ego) * abs_curvature)

  @classmethod
  def learning_error_cap(cls, curvature: float) -> float:
    return float(cls.RELATIVE_CAP_FULL_RATIO * abs(float(curvature)))

  @classmethod
  def projected_error(cls, desired_curvature: float, actual_curvature: float) -> float:
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * (desired_curvature - actual_curvature))

  @classmethod
  def build_fit_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return cls._build_curve_corrections(
      bias,
      counts,
      lambda speed_counts: speed_counts >= cls.MIN_BUCKET_POINTS,
      lambda _speed_idx: 1,
      cls.fit_local_strength,
      lambda speed_counts, speed_idx, _valid_idx, _local_strength: cls.speed_curve_strength(speed_counts, speed_idx),
      apply_cap=True,
      zero_invalid_buckets=True,
    )

  @classmethod
  def build_preview_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return cls._build_curve_corrections(
      bias,
      counts,
      lambda speed_counts: speed_counts > 0.0,
      lambda _speed_idx: 1,
      cls.preview_local_strength,
      lambda _all_counts, _speed_idx, _valid_idx, _local_strength: 1.0,
      apply_cap=False,
    )

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
                         v_ego: float, abs_curvature: float) -> float:
    if abs_curvature < cls.CURVATURE_MIN or abs_curvature > cls.CURVATURE_MAX:
      return 0.0

    low_speed, high_speed, speed_alpha = cls.speed_interp(v_ego)
    low_curve = fit_corrections[low_speed]
    high_curve = fit_corrections[high_speed]
    low_valid = fit_valid[low_speed]
    high_valid = fit_valid[high_speed]

    if not low_valid.any() and not high_valid.any():
      return 0.0

    log_centers = np.log(cls.CURVATURE_BUCKET_CENTERS.astype(np.float64))
    log_curvature = math.log(max(abs_curvature, cls.CURVATURE_BUCKET_MIN))

    def curve_value(curve: np.ndarray, valid_mask: np.ndarray) -> float:
      runs = cls.valid_runs(valid_mask)
      if len(runs) == 0:
        return 0.0

      for start, end in runs:
        run_idx = np.arange(start, end + 1)
        run_log_x = log_centers[run_idx]
        run_curve = curve[run_idx]
        first_edge = cls.CURVATURE_BUCKET_EDGES[start]
        last_edge = cls.CURVATURE_BUCKET_EDGES[end + 1]

        if first_edge <= abs_curvature <= last_edge:
          return float(np.interp(log_curvature, run_log_x, run_curve))

        if start > 0:
          fade_in_start = cls.CURVATURE_BUCKET_EDGES[start - 1]
          if fade_in_start <= abs_curvature < first_edge:
            fade_span = first_edge - fade_in_start
            fade = cls.smoothstep((abs_curvature - fade_in_start) / max(fade_span, 1e-9))
            return float(run_curve[0] * np.clip(fade, 0.0, 1.0))
        elif cls.CURVATURE_MIN <= abs_curvature < first_edge:
          fade_span = first_edge - cls.CURVATURE_MIN
          fade = cls.smoothstep((abs_curvature - cls.CURVATURE_MIN) / max(fade_span, 1e-9))
          return float(run_curve[0] * np.clip(fade, 0.0, 1.0))

        if end < len(cls.CURVATURE_BUCKET_CENTERS) - 1:
          fade_out_end = cls.CURVATURE_BUCKET_EDGES[end + 2]
          if last_edge < abs_curvature <= fade_out_end:
            fade_span = fade_out_end - last_edge
            fade = 1.0 - cls.smoothstep((abs_curvature - last_edge) / max(fade_span, 1e-9))
            return float(run_curve[-1] * np.clip(fade, 0.0, 1.0))
        elif last_edge < abs_curvature <= cls.CURVATURE_MAX:
          fade_span = cls.CURVATURE_MAX - last_edge
          fade = 1.0 - cls.smoothstep((abs_curvature - last_edge) / max(fade_span, 1e-9))
          return float(run_curve[-1] * np.clip(fade, 0.0, 1.0))

      return 0.0

    low_val = curve_value(low_curve, low_valid)
    high_val = curve_value(high_curve, high_valid)
    if low_speed == high_speed:
      return low_val
    return float((1.0 - speed_alpha) * low_val + speed_alpha * high_val)


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
    self.fit_speed_strength = np.zeros(len(self.SPEED_ANCHORS), dtype=np.float32)

    self.car_control_t = deque(maxlen=self.hist_len)
    self.lat_active = deque(maxlen=self.hist_len)
    self.roll_compensation = deque(maxlen=self.hist_len)
    self.car_state_t = deque(maxlen=self.hist_len)
    self.vego = deque(maxlen=self.hist_len)
    self.steering_pressed = deque(maxlen=self.hist_len)
    self.controls_state_t = deque(maxlen=self.hist_len)
    self.model_desired_curvature = deque(maxlen=self.hist_len)

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
    self.fit_refresh_pending_rows: dict[int, set[int]] = {}
    self.preview_refresh_pending_rows: dict[int, set[int]] = {}
    self.live_pose_update_index = 0
    self.last_fit_refresh_update = -FIT_REFRESH_EVERY_N_UPDATES
    self.last_preview_refresh_update = -PREVIEW_REFRESH_EVERY_N_UPDATES

    self._restore_cached_params()
    self.update_use_params(force=True)

    cloudlog.info(f"curvatured init brand={self.CP.brand} fingerprint={self.CP.carFingerprint} "
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
        cache_lcp = log_evt.liveCurvatureParameters
      with car.CarParams.from_bytes(params_cache) as msg:
        cache_CP = msg

      if self.get_restore_key(cache_CP, cache_lcp.version) != self.get_restore_key(self.CP, VERSION):
        return

      biases = list(cache_lcp.biases)
      counts = list(cache_lcp.counts)
      if len(biases) != self.total_size() or len(counts) != self.total_size():
        raise ValueError("invalid curvature cache shape")

      self.bias = self.unflatten_bucket(biases).astype(np.float32)
      self.counts = self.unflatten_bucket(counts).astype(np.float32)
      self.fit_corrections, self.fit_valid = self.build_fit_corrections(self.bias, self.counts)
      self.preview_corrections, self.preview_valid = self.build_preview_corrections(self.bias, self.counts)
      self.fit_speed_strength = np.asarray([self.speed_curve_strength(self.counts[speed_idx], speed_idx)
                                            for speed_idx in range(len(self.SPEED_ANCHORS))], dtype=np.float32)
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
                        self.CP.steerControlType == car.CarParams.SteerControlType.curvatureDEPRECATED
      if self.prev_use_params != self.use_params:
        cloudlog.info(f"curvatured use_params={self.use_params} toggle={self.enable_curvatured} "
                      f"brand={self.CP.brand} allowed={self.CP.brand in ALLOWED_CARS} "
                      f"steerControlType={self.CP.steerControlType}")
        self.prev_use_params = self.use_params
      if not self.use_params:
        self.current_bucket = (-1, -1)
        self.current_correction = 0.0
        self.current_bias = 0.0
        self.current_bucket_points = 0
        if self.prev_use_params:
          for d in [self.car_control_t, self.lat_active, self.roll_compensation,
                    self.car_state_t, self.vego, self.steering_pressed,
                    self.controls_state_t, self.model_desired_curvature]:
            d.clear()
          self.last_lat_inactive_t = 0.0
          self.last_override_t = 0.0
    self.frame += 1

  def _history_ready(self) -> bool:
    return min(len(self.car_control_t), len(self.car_state_t), len(self.controls_state_t)) == self.hist_len

  @staticmethod
  def _sample_at_or_before(target_t: float, ts: deque, values: deque):
    if len(ts) == 0:
      return None
    if target_t < ts[0]:
      return None

    for i in range(len(ts) - 1, -1, -1):
      if ts[i] <= target_t:
        return values[i]
    return None

  def add_measurement(self, desired_curvature: float, actual_curvature: float, v_ego: float,
                      schedule_only: bool = False) -> None:
    curvature_idx = self.curvature_index(desired_curvature)
    speed_weights = self.learning_speed_weights(v_ego)
    if curvature_idx is None or len(speed_weights) == 0:
      return

    error_cap = self.learning_error_cap(desired_curvature)
    error = float(np.clip(self.projected_error(desired_curvature, actual_curvature), -error_cap, error_cap))

    for speed_idx, weight in speed_weights:
      if weight <= 0.0:
        continue

      prev_count = float(self.counts[speed_idx, curvature_idx])
      sample_count = min(prev_count + float(weight), self.MAX_SAMPLES)
      delta = sample_count - prev_count
      if delta <= 0.0:
        continue

      self.counts[speed_idx, curvature_idx] = sample_count
      alpha = delta / min(sample_count, self.MEAN_WINDOW)
      prev_bias = float(self.bias[speed_idx, curvature_idx])
      self.bias[speed_idx, curvature_idx] = prev_bias + alpha * (error - prev_bias)
      self._mark_curve_refresh_pending(speed_idx, curvature_idx)

    if not schedule_only:
      self.refresh_curve_lookups(self.live_pose_update_index, force_fit=True, force_preview=True)

  def _mark_curve_refresh_pending(self, speed_idx: int, curvature_idx: int) -> None:
    self.fit_refresh_pending_rows.setdefault(speed_idx, set()).add(curvature_idx)
    self.preview_refresh_pending_rows.setdefault(speed_idx, set()).add(curvature_idx)

  def _row_bucket_caps(self, speed_idx: int, apply_cap: bool) -> np.ndarray:
    if not apply_cap:
      return np.full(len(self.CURVATURE_BUCKET_CENTERS), np.inf, dtype=np.float32)
    return np.asarray([self.correction_cap(float(curvature), float(self.SPEED_ANCHORS[speed_idx]))
                       for curvature in self.CURVATURE_BUCKET_CENTERS], dtype=np.float32)

  def _row_curve_valid(self, speed_idx: int, valid_mask_fn, min_valid_buckets_fn, apply_cap: bool) -> np.ndarray:
    curve_valid = np.asarray(valid_mask_fn(self.counts[speed_idx]), dtype=bool)
    if apply_cap:
      curve_valid &= self.apply_bucket_mask(speed_idx)
    if int(np.count_nonzero(curve_valid)) < int(min_valid_buckets_fn(speed_idx)):
      return np.zeros(len(self.CURVATURE_BUCKET_CENTERS), dtype=bool)
    return curve_valid

  @staticmethod
  def _merge_bounds(bounds: list[tuple[int, int]]) -> list[tuple[int, int]]:
    if len(bounds) == 0:
      return []
    bounds = sorted(bounds)
    merged = [bounds[0]]
    for start, end in bounds[1:]:
      prev_start, prev_end = merged[-1]
      if start <= prev_end + 1:
        merged[-1] = (prev_start, max(prev_end, end))
      else:
        merged.append((start, end))
    return merged

  def _affected_run_bounds(self, previous_valid: np.ndarray, curve_valid: np.ndarray,
                           changed_indices: set[int]) -> list[tuple[int, int]]:
    bounds: list[tuple[int, int]] = []
    for idx in changed_indices:
      bounds.append((max(0, idx - 1), min(len(self.CURVATURE_BUCKET_CENTERS) - 1, idx + 1)))
    for mask in (previous_valid, curve_valid):
      for start, end in self.valid_runs(mask):
        if any(start <= idx <= end for idx in changed_indices):
          bounds.append((start, end))
    return self._merge_bounds(bounds)

  def _run_values(self, speed_idx: int, run_idx: np.ndarray, speed_strength: float,
                  local_strength_fn, bucket_caps: np.ndarray, apply_cap: bool) -> np.ndarray:
    run_curve = np.clip(self.bias[speed_idx, run_idx], -bucket_caps[run_idx], bucket_caps[run_idx]).astype(np.float32)
    run_strength = local_strength_fn(self.counts[speed_idx], run_idx).astype(np.float32)

    if len(run_curve) >= 3:
      smoothed_run = run_curve.copy()
      smoothed_run[1:-1] = 0.25 * run_curve[:-2] + 0.5 * run_curve[1:-1] + 0.25 * run_curve[2:]
    else:
      smoothed_run = run_curve

    run_values = speed_strength * run_strength * smoothed_run
    return np.clip(run_values, -bucket_caps[run_idx], bucket_caps[run_idx]) if apply_cap else run_values

  def _refresh_row(self, speed_idx: int, changed_indices: set[int],
                   valid_mask_fn,
                   min_valid_buckets_fn,
                   local_strength_fn,
                   speed_strength_fn,
                   apply_cap: bool,
                   zero_invalid_buckets: bool,
                   previous_row: np.ndarray,
                   previous_valid: np.ndarray,
                   previous_speed_strength: float) -> tuple[np.ndarray, np.ndarray, float]:
    curve_valid = self._row_curve_valid(speed_idx, valid_mask_fn, min_valid_buckets_fn, apply_cap)
    bucket_caps = self._row_bucket_caps(speed_idx, apply_cap)

    if not curve_valid.any():
      return np.zeros(len(self.CURVATURE_BUCKET_CENTERS), dtype=np.float32), curve_valid, 0.0

    valid_idx = np.flatnonzero(curve_valid)
    local_strength = local_strength_fn(self.counts[speed_idx], valid_idx)
    speed_strength = float(speed_strength_fn(self.counts[speed_idx], speed_idx, valid_idx, local_strength))
    force_full = not np.isclose(speed_strength, previous_speed_strength)

    if force_full:
      row = np.zeros(len(self.CURVATURE_BUCKET_CENTERS), dtype=np.float32)
      rebuild_bounds = self.valid_runs(curve_valid)
    else:
      row = previous_row.copy()
      rebuild_bounds = self._affected_run_bounds(previous_valid, curve_valid, changed_indices)

    for start, end in rebuild_bounds:
      row[start:end + 1] = 0.0

    current_runs = self.valid_runs(curve_valid)
    for start, end in current_runs:
      if not force_full and all(end < bound_start or start > bound_end for bound_start, bound_end in rebuild_bounds):
        continue
      run_idx = np.arange(start, end + 1)
      row[run_idx] = self._run_values(speed_idx, run_idx, speed_strength, local_strength_fn, bucket_caps, apply_cap)

    if zero_invalid_buckets:
      row = np.where(curve_valid, row, 0.0)

    return row.astype(np.float32), curve_valid, speed_strength

  def refresh_curve_lookups(self, update_index: int, force_fit: bool = False, force_preview: bool = False) -> None:
    fit_due = force_fit or ((update_index - self.last_fit_refresh_update) >= FIT_REFRESH_EVERY_N_UPDATES)
    preview_due = force_preview or ((update_index - self.last_preview_refresh_update) >= PREVIEW_REFRESH_EVERY_N_UPDATES)

    if fit_due and self.fit_refresh_pending_rows:
      for speed_idx, changed_indices in list(self.fit_refresh_pending_rows.items()):
        row, valid, speed_strength = self._refresh_row(
          speed_idx,
          changed_indices,
          lambda speed_counts: speed_counts >= self.MIN_BUCKET_POINTS,
          lambda _speed_idx: 1,
          self.fit_local_strength,
          lambda speed_counts, row_idx, _valid_idx, _local_strength: self.speed_curve_strength(speed_counts, row_idx),
          True,
          True,
          self.fit_corrections[speed_idx],
          self.fit_valid[speed_idx],
          float(self.fit_speed_strength[speed_idx]),
        )
        self.fit_corrections[speed_idx] = row
        self.fit_valid[speed_idx] = valid
        self.fit_speed_strength[speed_idx] = speed_strength
      self.fit_refresh_pending_rows.clear()
      self.last_fit_refresh_update = update_index

    if preview_due and (self.publish_preview_data or force_preview) and self.preview_refresh_pending_rows:
      for speed_idx, changed_indices in list(self.preview_refresh_pending_rows.items()):
        row, valid, _ = self._refresh_row(
          speed_idx,
          changed_indices,
          lambda speed_counts: speed_counts > 0.0,
          lambda _speed_idx: 1,
          self.preview_local_strength,
          lambda _all_counts, _row_idx, _valid_idx, _local_strength: 1.0,
          False,
          False,
          self.preview_corrections[speed_idx],
          self.preview_valid[speed_idx],
          1.0,
        )
        self.preview_corrections[speed_idx] = row
        self.preview_valid[speed_idx] = valid
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

    if not self.fit_valid[speed_idx].any():
      self.current_correction = 0.0
      return

    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs(desired_curvature))
    self.current_correction = float(direction * projected)

  def handle_log(self, t: float, which: str, msg) -> None:
    if not self.use_params:
      if which == "liveCalibration":
        self.calibrator.feed_live_calib(msg)
      elif which == "liveDelay":
        self.lag = get_lat_delay(self.params, msg.lateralDelay)
      return

    if which == "carControl":
      self.car_control_t.append(t)
      self.lat_active.append(msg.latActive)
      self.roll_compensation.append(msg.rollCompensation)
      if not msg.latActive:
        self.last_lat_inactive_t = t
    elif which == "carState":
      steering_override = bool(msg.steeringPressed or msg.steeringSlightlyPressed)
      self.car_state_t.append(t)
      self.vego.append(msg.vEgo)
      self.steering_pressed.append(steering_override)
      if steering_override:
        self.last_override_t = t
    elif which == "controlsState":
      self.controls_state_t.append(t)
      self.model_desired_curvature.append(msg.modelDesiredCurvature)
      if self.car_state_t:
        self._update_current_lookup(self.model_desired_curvature[-1], self.vego[-1])
    elif which == "liveCalibration":
      self.calibrator.feed_live_calib(msg)
    elif which == "liveDelay":
      self.lag = get_lat_delay(self.params, msg.lateralDelay)
    elif which == "livePose" and self.use_params:
      self.live_pose_update_index += 1
      if not self._history_ready():
        return
      if not (msg.angularVelocityDevice.valid and msg.posenetOK and msg.inputsOK and self.calibrator.calib_valid):
        return
      if (t - self.last_lat_inactive_t) < MIN_ENGAGE_BUFFER or (t - self.last_override_t) < MIN_ENGAGE_BUFFER:
        return

      target_t = t - self.lag
      lat_active = self._sample_at_or_before(target_t, self.car_control_t, self.lat_active)
      roll_comp = self._sample_at_or_before(target_t, self.car_control_t, self.roll_compensation)
      steering_pressed = self._sample_at_or_before(target_t, self.car_state_t, self.steering_pressed)
      v_ego = self._sample_at_or_before(target_t, self.car_state_t, self.vego)
      desired_curvature = self._sample_at_or_before(target_t, self.controls_state_t, self.model_desired_curvature)

      if any(x is None for x in (lat_active, roll_comp, steering_pressed, v_ego, desired_curvature)):
        return

      if not bool(lat_active) or bool(steering_pressed) or float(v_ego) < self.MIN_SPEED:
        return

      device_pose = Pose.from_live_pose(msg)
      if not self.roll_learning_allowed(device_pose.orientation.roll):
        return
      calibrated_pose = self.calibrator.build_calibrated_pose(device_pose)
      yaw_rate = calibrated_pose.angular_velocity.yaw
      yaw_rate_std = calibrated_pose.angular_velocity.yaw_std
      if yaw_rate_std >= MAX_YAW_RATE_STD:
        return

      v_ego = float(v_ego)
      desired_curvature = float(desired_curvature)
      actual_curvature = self.actual_curvature_from_yaw_rate(yaw_rate, v_ego, roll_compensation=float(roll_comp))

      self.add_measurement(desired_curvature, actual_curvature, v_ego, schedule_only=True)
      self.refresh_curve_lookups(self.live_pose_update_index)

  def get_msg(self, valid: bool = True, live_valid: bool = True,
              include_debug: bool = False, include_preview: bool = False):
    msg = messaging.new_message('liveCurvatureParameters')
    msg.valid = valid

    curvature_params = msg.liveCurvatureParameters
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
    cloudlog.info(f"curvatured status use_params={self.use_params} checks={checks} "
                  f"lag={self.lag:.3f} total_points={int(round(float(self.counts.sum())))} "
                  f"bucket={self.current_bucket} bucket_points={self.current_bucket_points} "
                  f"corr={self.current_correction:.8f} cal={self.calibration_percent(self.counts)} "
                  f"invalid={invalid} not_alive={not_alive}")
