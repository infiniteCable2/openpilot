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
HISTORY = 1.5
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 1.5
ALLOWED_CARS = ['volkswagen']
STATUS_LOG_INTERVAL = 10.0
MAX_LEARN_ROLL_LATERAL_ACCEL = 0.10

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
# - Learning is gated by valid upstream pose/calibration, low roll, low yaw uncertainty, no steering override,
#   and bounded lateral acceleration.
# - Corrections are bounded by a relative cap envelope on log-curvature:
#   - up to 50% of local curvature in the center-focused range
#   - smoothly reduced through medium curvature
#   - tapering toward 0 near the outer learning limit
# - A separate hard absolute cap remains only as a safety backstop.


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
  CURVATURE_MIN_STEP = float(CURVATURE_BUCKET_EDGES[0])
  IMPORTANT_CURVATURE_MAX = float(CURVATURE_BUCKET_EDGES[10])
  CURVATURE_MAX = float(CURVATURE_BUCKET_EDGES[-1])

  MIN_SPEED = float(SPEED_ANCHORS[0] * 0.5)
  MAX_LAT_ACCEL = 1.0
  HARD_MAX_CORRECTION = 3.0e-4
  RELATIVE_CAP_FULL_RATIO = 0.50
  RELATIVE_CAP_FULL_CURVATURE = 1.0e-4
  RELATIVE_CAP_MID_CURVATURE = 1.024e-3
  RELATIVE_CAP_MID_RATIO = 0.25

  MAX_SAMPLES = 600
  MEAN_WINDOW = 180.0
  FIT_MIN_TOTAL_SAMPLES = 480.0
  FULL_CONFIDENCE_TOTAL_SAMPLES = 960.0
  FULL_CONFIDENCE_BUCKET_SAMPLES = 180.0
  FIT_MIN_VALID_BUCKETS = 4
  INITIAL_VALID_LATERAL_ACCEL = 0.05
  MIN_BUCKET_POINTS = np.array([20, 20, 18, 16, 14, 12, 10, 8, 6, 6, 4, 4], dtype=np.float32)

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
    if abs_curvature < cls.CURVATURE_MIN_STEP or abs_curvature > cls.CURVATURE_MAX:
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
  def confidence(cls, total_points: float) -> float:
    return float(np.clip(
      (total_points - cls.FIT_MIN_TOTAL_SAMPLES) /
      max(cls.FULL_CONFIDENCE_TOTAL_SAMPLES - cls.FIT_MIN_TOTAL_SAMPLES, 1.0),
      0.0, 1.0
    ))

  @classmethod
  def bucket_points_for_index(cls, counts: np.ndarray, idx: tuple[int, int] | None) -> int:
    if idx is None:
      return 0
    return int(round(float(counts[idx])))

  @classmethod
  def speed_curve_valid(cls, counts: np.ndarray, speed_idx: int) -> bool:
    valid_mask = np.asarray(counts[speed_idx] >= cls.MIN_BUCKET_POINTS, dtype=bool)
    valid_bucket_count = int(np.count_nonzero(valid_mask))
    total_points = float(counts[speed_idx].sum())
    required_bucket_count = cls.required_valid_bucket_count(speed_idx)
    return valid_bucket_count >= required_bucket_count and total_points >= cls.FIT_MIN_TOTAL_SAMPLES

  @classmethod
  def required_valid_bucket_count(cls, speed_idx: int) -> int:
    v_ego = float(cls.SPEED_ANCHORS[speed_idx])
    typical_curvature = cls.INITIAL_VALID_LATERAL_ACCEL / max(v_ego ** 2, 1e-6)
    bucket_count = int(np.searchsorted(cls.CURVATURE_BUCKET_CENTERS, typical_curvature, side='right'))
    return int(np.clip(bucket_count, cls.FIT_MIN_VALID_BUCKETS, len(cls.CURVATURE_BUCKET_CENTERS)))

  @classmethod
  def calibration_percent(cls, counts: np.ndarray) -> int:
    valid_speeds = sum(cls.speed_curve_valid(counts, speed_idx) for speed_idx in range(len(cls.SPEED_ANCHORS)))
    return int(round(100.0 * valid_speeds / float(len(cls.SPEED_ANCHORS))))

  @classmethod
  def smoothstep(cls, x: float) -> float:
    y = float(np.clip(x, 0.0, 1.0))
    return y * y * (3.0 - 2.0 * y)

  @classmethod
  def curvature_window(cls, curvature: float) -> float:
    abs_curvature = abs(float(curvature))
    if abs_curvature <= cls.CURVATURE_MIN_STEP or abs_curvature >= cls.CURVATURE_MAX:
      return 0.0

    fade_in = cls.smoothstep((abs_curvature - cls.CURVATURE_MIN_STEP) /
                             max(cls.CURVATURE_BUCKET_EDGES[2] - cls.CURVATURE_MIN_STEP, 1e-9))
    fade_out = 1.0 - cls.smoothstep((abs_curvature - cls.IMPORTANT_CURVATURE_MAX) /
                                    max(cls.CURVATURE_MAX - cls.IMPORTANT_CURVATURE_MAX, 1e-9))
    return float(np.clip(fade_in * fade_out, 0.0, 1.0))

  @classmethod
  def correction_cap(cls, curvature: float) -> float:
    abs_curvature = abs(float(curvature))
    if abs_curvature <= cls.CURVATURE_MIN_STEP:
      return 0.0

    if abs_curvature <= cls.RELATIVE_CAP_FULL_CURVATURE:
      relative_cap = cls.RELATIVE_CAP_FULL_RATIO
    elif abs_curvature <= cls.RELATIVE_CAP_MID_CURVATURE:
      log_low = math.log(cls.RELATIVE_CAP_FULL_CURVATURE)
      log_high = math.log(cls.RELATIVE_CAP_MID_CURVATURE)
      alpha = cls.smoothstep((math.log(abs_curvature) - log_low) / max(log_high - log_low, 1e-9))
      relative_cap = (1.0 - alpha) * cls.RELATIVE_CAP_FULL_RATIO + alpha * cls.RELATIVE_CAP_MID_RATIO
    elif abs_curvature < cls.CURVATURE_MAX:
      log_low = math.log(cls.RELATIVE_CAP_MID_CURVATURE)
      log_high = math.log(cls.CURVATURE_MAX)
      alpha = cls.smoothstep((math.log(abs_curvature) - log_low) / max(log_high - log_low, 1e-9))
      relative_cap = (1.0 - alpha) * cls.RELATIVE_CAP_MID_RATIO
    else:
      relative_cap = 0.0

    return float(min(cls.HARD_MAX_CORRECTION, max(relative_cap, 0.0) * abs_curvature))

  @classmethod
  def projected_error(cls, desired_curvature: float, actual_curvature: float) -> float:
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * (desired_curvature - actual_curvature))

  @classmethod
  def build_fit_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    fit_corrections = np.zeros(cls.bucket_shape(), dtype=np.float32)
    fit_valid = np.zeros(cls.bucket_shape(), dtype=bool)
    log_centers = np.log(cls.CURVATURE_BUCKET_CENTERS.astype(np.float64))
    bucket_caps = np.asarray([cls.correction_cap(float(curvature)) for curvature in cls.CURVATURE_BUCKET_CENTERS], dtype=np.float64)

    for speed_idx in range(len(cls.SPEED_ANCHORS)):
      curve_valid = np.asarray(counts[speed_idx] >= cls.MIN_BUCKET_POINTS, dtype=bool)
      if int(np.count_nonzero(curve_valid)) < cls.required_valid_bucket_count(speed_idx):
        continue
      total_points = float(counts[speed_idx].sum())
      if total_points < cls.FIT_MIN_TOTAL_SAMPLES:
        continue

      valid_idx = np.flatnonzero(curve_valid)
      valid_log_x = log_centers[valid_idx]
      valid_y = np.clip(bias[speed_idx, valid_idx], -bucket_caps[valid_idx], bucket_caps[valid_idx]).astype(np.float64)

      interp = np.interp(log_centers, valid_log_x, valid_y).astype(np.float32)
      smoothed = interp.copy()
      if len(smoothed) >= 3:
        smoothed[1:-1] = 0.25 * interp[:-2] + 0.5 * interp[1:-1] + 0.25 * interp[2:]

      local_strength = np.clip(
        (counts[speed_idx, valid_idx] - cls.MIN_BUCKET_POINTS[valid_idx]) /
        np.maximum(cls.FULL_CONFIDENCE_BUCKET_SAMPLES - cls.MIN_BUCKET_POINTS[valid_idx], 1.0),
        0.0, 1.0
      ).astype(np.float64)
      interpolated_strength = np.interp(log_centers, valid_log_x, local_strength).astype(np.float32)
      confidence = cls.confidence(total_points)
      smoothed *= confidence * interpolated_strength
      for curvature_idx, curvature in enumerate(cls.CURVATURE_BUCKET_CENTERS):
        smoothed[curvature_idx] *= cls.curvature_window(float(curvature))

      fit_corrections[speed_idx] = np.clip(smoothed, -bucket_caps, bucket_caps)
      fit_valid[speed_idx] = curve_valid

    return fit_corrections, fit_valid

  @classmethod
  def build_preview_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    preview_corrections = np.zeros(cls.bucket_shape(), dtype=np.float32)
    preview_valid = np.zeros(cls.bucket_shape(), dtype=bool)
    log_centers = np.log(cls.CURVATURE_BUCKET_CENTERS.astype(np.float64))
    bucket_caps = np.asarray([cls.correction_cap(float(curvature)) for curvature in cls.CURVATURE_BUCKET_CENTERS], dtype=np.float64)

    for speed_idx in range(len(cls.SPEED_ANCHORS)):
      curve_valid = np.asarray(counts[speed_idx] > 0.0, dtype=bool)
      if not curve_valid.any():
        continue

      total_points = float(counts[speed_idx].sum())
      if total_points <= 0.0:
        continue

      valid_idx = np.flatnonzero(curve_valid)
      valid_log_x = log_centers[valid_idx]
      valid_y = np.clip(bias[speed_idx, valid_idx], -bucket_caps[valid_idx], bucket_caps[valid_idx]).astype(np.float64)

      interp = np.interp(log_centers, valid_log_x, valid_y).astype(np.float32)
      smoothed = interp.copy()
      if len(smoothed) >= 3:
        smoothed[1:-1] = 0.25 * interp[:-2] + 0.5 * interp[1:-1] + 0.25 * interp[2:]

      local_strength = np.clip(
        counts[speed_idx, valid_idx] / np.maximum(cls.MIN_BUCKET_POINTS[valid_idx], 1.0),
        0.0, 1.0
      ).astype(np.float64)
      interpolated_strength = np.interp(log_centers, valid_log_x, local_strength).astype(np.float32)
      confidence = cls.confidence(total_points)
      smoothed *= confidence * interpolated_strength
      for curvature_idx, curvature in enumerate(cls.CURVATURE_BUCKET_CENTERS):
        smoothed[curvature_idx] *= cls.curvature_window(float(curvature))

      preview_corrections[speed_idx] = np.clip(smoothed, -bucket_caps, bucket_caps)
      preview_valid[speed_idx] = curve_valid

    return preview_corrections, preview_valid

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
    if abs_curvature < cls.CURVATURE_MIN_STEP or abs_curvature > cls.CURVATURE_MAX:
      return 0.0

    low_speed, high_speed, speed_alpha = cls.speed_interp(v_ego)
    low_curve = fit_corrections[low_speed]
    high_curve = fit_corrections[high_speed]
    low_valid = fit_valid[low_speed]
    high_valid = fit_valid[high_speed]

    if not low_valid.any() and not high_valid.any():
      return 0.0

    log_centers = np.log(cls.CURVATURE_BUCKET_CENTERS.astype(np.float64))
    log_curvature = math.log(max(abs_curvature, cls.CURVATURE_MIN_STEP))

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

        if end < len(cls.CURVATURE_BUCKET_CENTERS) - 1:
          fade_out_end = cls.CURVATURE_BUCKET_EDGES[end + 2]
          if last_edge < abs_curvature <= fade_out_end:
            fade_span = fade_out_end - last_edge
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

    self.car_control_t = deque(maxlen=self.hist_len)
    self.lat_active = deque(maxlen=self.hist_len)
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
    self.prev_use_params = None
    self.last_status_log_t = 0.0

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
      cloudlog.info("restored curvature params from cache")
    except Exception:
      cloudlog.exception("failed to restore cached curvature params")
      self.params.remove("LiveCurvatureParameters")

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
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

  def add_measurement(self, desired_curvature: float, actual_curvature: float, v_ego: float) -> None:
    curvature_idx = self.curvature_index(desired_curvature)
    speed_weights = self.learning_speed_weights(v_ego)
    if curvature_idx is None or len(speed_weights) == 0:
      return

    error_cap = self.correction_cap(desired_curvature)
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

    self.fit_corrections, self.fit_valid = self.build_fit_corrections(self.bias, self.counts)

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
      if not msg.latActive:
        self.last_lat_inactive_t = t
    elif which == "carState":
      self.car_state_t.append(t)
      self.vego.append(msg.vEgo)
      self.steering_pressed.append(msg.steeringPressed)
      if msg.steeringPressed:
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
      if not self._history_ready():
        return
      if not (msg.angularVelocityDevice.valid and msg.posenetOK and msg.inputsOK and self.calibrator.calib_valid):
        return
      if (t - self.last_lat_inactive_t) < MIN_ENGAGE_BUFFER or (t - self.last_override_t) < MIN_ENGAGE_BUFFER:
        return

      target_t = t - self.lag
      lat_active = self._sample_at_or_before(target_t, self.car_control_t, self.lat_active)
      steering_pressed = self._sample_at_or_before(target_t, self.car_state_t, self.steering_pressed)
      v_ego = self._sample_at_or_before(target_t, self.car_state_t, self.vego)
      desired_curvature = self._sample_at_or_before(target_t, self.controls_state_t, self.model_desired_curvature)

      if any(x is None for x in (lat_active, steering_pressed, v_ego, desired_curvature)):
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
      actual_curvature = yaw_rate / max(v_ego, 0.1)
      if max(abs(desired_curvature), abs(actual_curvature)) * (v_ego ** 2) > self.MAX_LAT_ACCEL:
        return

      self.add_measurement(desired_curvature, actual_curvature, v_ego)

  def get_msg(self, valid: bool = True, live_valid: bool = True):
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
    curvature_params.bucketSign = 0
    curvature_params.bucketSpeed = int(self.current_bucket[0]) if self.use_params else -1
    curvature_params.bucketCurvature = int(self.current_bucket[1]) if self.use_params else -1
    curvature_params.corrections = self.flatten(self.fit_corrections)
    curvature_params.counts = self.flatten(np.rint(self.counts).astype(np.int32))
    curvature_params.biases = self.flatten(self.bias)
    curvature_params.fitValid = self.flatten(self.fit_valid)
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


# Standalone CurvatureD reference:
# CurvatureD currently runs inside torqued. To restore it as a dedicated process later,
# add a small main() here that mirrors the old pattern:
# - config_realtime_process(...)
# - SubMaster(curvature_services, poll='livePose')
# - PubMaster(['liveCurvatureParameters'])
# - CurvatureEstimator(CP)
# - sm.update() loop with handle_log(...), update_use_params(), get_msg(...), cache writes
#
# When doing that, comment out the CurvatureEstimator integration in selfdrive/locationd/torqued.py
# so liveCurvatureParameters is only produced from one place.
