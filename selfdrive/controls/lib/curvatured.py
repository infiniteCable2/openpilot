import math

import numpy as np

VERSION = 3


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
  ], dtype=np.float32)
  CURVATURE_BUCKET_CENTERS = np.sqrt(CURVATURE_BUCKET_EDGES[:-1] * CURVATURE_BUCKET_EDGES[1:]).astype(np.float32)
  CURVATURE_MIN_STEP = float(CURVATURE_BUCKET_EDGES[0])
  IMPORTANT_CURVATURE_MAX = float(CURVATURE_BUCKET_EDGES[8])
  CURVATURE_MAX = float(CURVATURE_BUCKET_EDGES[-1])

  MIN_SPEED = float(SPEED_ANCHORS[0] * 0.5)
  MAX_LAT_ACCEL = 2.5
  MAX_CORRECTION = 3.0e-5
  CORRECTION_CAP = MAX_CORRECTION

  MAX_SAMPLES = 600
  MEAN_WINDOW = 180.0
  FULL_CONFIDENCE_SAMPLES = 180.0
  FIT_MIN_TOTAL_SAMPLES = 120.0
  FIT_MIN_VALID_BUCKETS = 4
  MIN_BUCKET_POINTS = np.array([20, 20, 18, 16, 14, 12, 10, 8, 6, 6], dtype=np.float32)

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
    return float(np.clip(total_points / cls.FULL_CONFIDENCE_SAMPLES, 0.0, 1.0))

  @classmethod
  def bucket_points_for_index(cls, counts: np.ndarray, idx: tuple[int, int] | None) -> int:
    if idx is None:
      return 0
    return int(round(float(counts[idx])))

  @classmethod
  def speed_curve_valid(cls, counts: np.ndarray, speed_idx: int) -> bool:
    bucket_counts = counts[speed_idx]
    valid_bucket_count = int(np.count_nonzero(bucket_counts >= cls.MIN_BUCKET_POINTS))
    total_points = float(bucket_counts.sum())
    return valid_bucket_count >= cls.FIT_MIN_VALID_BUCKETS and total_points >= cls.FIT_MIN_TOTAL_SAMPLES

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
  def projected_error(cls, desired_curvature: float, actual_curvature: float) -> float:
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    return float(direction * (desired_curvature - actual_curvature))

  @classmethod
  def build_fit_corrections(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    fit_corrections = np.zeros(cls.bucket_shape(), dtype=np.float32)
    fit_valid = np.zeros(cls.bucket_shape(), dtype=bool)
    log_centers = np.log(cls.CURVATURE_BUCKET_CENTERS.astype(np.float64))

    for speed_idx in range(len(cls.SPEED_ANCHORS)):
      curve_valid = counts[speed_idx] >= cls.MIN_BUCKET_POINTS
      if int(np.count_nonzero(curve_valid)) < cls.FIT_MIN_VALID_BUCKETS:
        continue
      total_points = float(counts[speed_idx].sum())
      if total_points < cls.FIT_MIN_TOTAL_SAMPLES:
        continue

      valid_idx = np.flatnonzero(curve_valid)
      valid_log_x = log_centers[valid_idx]
      valid_y = np.clip(bias[speed_idx, valid_idx], -cls.CORRECTION_CAP, cls.CORRECTION_CAP).astype(np.float64)

      interp = np.interp(log_centers, valid_log_x, valid_y).astype(np.float32)
      smoothed = interp.copy()
      if len(smoothed) >= 3:
        smoothed[1:-1] = 0.25 * interp[:-2] + 0.5 * interp[1:-1] + 0.25 * interp[2:]

      local_strength = 0.5 + 0.5 * np.clip(
        (counts[speed_idx, valid_idx] - cls.MIN_BUCKET_POINTS[valid_idx]) /
        np.maximum(cls.FULL_CONFIDENCE_SAMPLES - cls.MIN_BUCKET_POINTS[valid_idx], 1.0),
        0.0, 1.0
      ).astype(np.float64)
      interpolated_strength = np.interp(log_centers, valid_log_x, local_strength).astype(np.float32)
      confidence = cls.confidence(total_points)
      smoothed *= confidence * interpolated_strength
      for curvature_idx, curvature in enumerate(cls.CURVATURE_BUCKET_CENTERS):
        smoothed[curvature_idx] *= cls.curvature_window(float(curvature))

      fit_corrections[speed_idx] = np.clip(smoothed, -cls.MAX_CORRECTION, cls.MAX_CORRECTION)
      fit_valid[speed_idx, valid_idx[0]:valid_idx[-1] + 1] = True

    return fit_corrections, fit_valid

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
      idx = np.flatnonzero(valid_mask)
      if len(idx) == 0:
        return 0.0

      base_value = float(np.interp(log_curvature, log_centers[idx], curve[idx]))
      first_valid = int(idx[0])
      last_valid = int(idx[-1])

      fade = 1.0
      if abs_curvature < cls.CURVATURE_BUCKET_EDGES[first_valid]:
        if first_valid == 0:
          return 0.0
        fade_span = cls.CURVATURE_BUCKET_EDGES[first_valid] - cls.CURVATURE_BUCKET_EDGES[first_valid - 1]
        fade = cls.smoothstep((abs_curvature - cls.CURVATURE_BUCKET_EDGES[first_valid - 1]) / max(fade_span, 1e-9))
      elif abs_curvature > cls.CURVATURE_BUCKET_EDGES[last_valid + 1]:
        if last_valid >= len(cls.CURVATURE_BUCKET_CENTERS) - 1:
          return 0.0
        fade_span = cls.CURVATURE_BUCKET_EDGES[last_valid + 2] - cls.CURVATURE_BUCKET_EDGES[last_valid + 1]
        fade = 1.0 - cls.smoothstep((abs_curvature - cls.CURVATURE_BUCKET_EDGES[last_valid + 1]) / max(fade_span, 1e-9))

      return float(base_value * np.clip(fade, 0.0, 1.0))

    low_val = curve_value(low_curve, low_valid)
    high_val = curve_value(high_curve, high_valid)
    if low_speed == high_speed:
      return low_val
    return float((1.0 - speed_alpha) * low_val + speed_alpha * high_val)


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
