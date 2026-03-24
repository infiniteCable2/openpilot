import math

import numpy as np

VERSION = 2


class CurvatureDLookup:
  # 20 km/h anchors up to 140 km/h, used as fit points and interpolated in the controller.
  SPEED_ANCHORS = np.array([20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], dtype=np.float32) / 3.6

  # Log-like curvature regions, with most resolution in the sensitive center range.
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

  # Fit and learning limits. The fit stays intentionally conservative.
  FIT_EXPONENT = 1.75
  AMP_MIN = -0.30
  AMP_MAX = 0.90
  SCALE_MIN = float(CURVATURE_BUCKET_EDGES[1])
  SCALE_MAX = 3.2e-4
  MAX_CORRECTION = 3.0e-5
  CORRECTION_CAP = MAX_CORRECTION

  MAX_SAMPLES = 600
  MEAN_WINDOW = 180.0
  FULL_CONFIDENCE_SAMPLES = 180.0
  FIT_MIN_TOTAL_SAMPLES = 120.0
  FIT_MIN_VALID_BUCKETS = 4
  MIN_BUCKET_POINTS = np.array([20, 20, 18, 16, 14, 12, 10, 8, 6, 6], dtype=np.float32)

  @classmethod
  def fit_shape(cls) -> tuple[int, int]:
    return 2, len(cls.SPEED_ANCHORS)

  @classmethod
  def bucket_shape(cls) -> tuple[int, int, int]:
    return 2, len(cls.SPEED_ANCHORS), len(cls.CURVATURE_BUCKET_CENTERS)

  @classmethod
  def fit_total_size(cls) -> int:
    a, b = cls.fit_shape()
    return a * b

  @classmethod
  def total_size(cls) -> int:
    a, b, c = cls.bucket_shape()
    return a * b * c

  @classmethod
  def flatten(cls, arr: np.ndarray) -> list:
    return arr.reshape(-1).tolist()

  @classmethod
  def unflatten_fit(cls, values, dtype=np.float32) -> np.ndarray:
    return np.asarray(values, dtype=dtype).reshape(cls.fit_shape())

  @classmethod
  def unflatten_bucket(cls, values, dtype=np.float32) -> np.ndarray:
    return np.asarray(values, dtype=dtype).reshape(cls.bucket_shape())

  @staticmethod
  def sign_index(curvature: float) -> int:
    return 1 if curvature >= 0.0 else 0

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
    idx = int(np.argmin(np.abs(cls.SPEED_ANCHORS - v)))
    return idx

  @classmethod
  def indices(cls, curvature: float, v_ego: float) -> tuple[int, int, int] | None:
    speed_idx = cls.speed_index(v_ego)
    curvature_idx = cls.curvature_index(curvature)
    if speed_idx is None or curvature_idx is None:
      return None
    return cls.sign_index(curvature), speed_idx, curvature_idx

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
  def calibration_percent(cls, fit_valid: np.ndarray) -> int:
    return int(round(100.0 * float(np.count_nonzero(fit_valid)) / float(fit_valid.size)))

  @classmethod
  def bucket_points_for_index(cls, counts: np.ndarray, idx: tuple[int, int, int] | None) -> int:
    if idx is None:
      return 0
    return int(round(float(counts[idx])))

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
  def center_boost_ratio(cls, abs_curvature: float, amplitude: float, scale: float) -> float:
    scale = float(np.clip(scale, cls.SCALE_MIN, cls.SCALE_MAX))
    amplitude = float(np.clip(amplitude, cls.AMP_MIN, cls.AMP_MAX))
    ratio = amplitude * math.exp(-((max(abs_curvature, cls.CURVATURE_MIN_STEP) / scale) ** cls.FIT_EXPONENT))
    return float(ratio)

  @classmethod
  def correction_from_fit(cls, curvature: float, amplitude: float, scale: float, confidence: float = 1.0) -> float:
    abs_curvature = abs(float(curvature))
    if abs_curvature < cls.CURVATURE_MIN_STEP or abs_curvature > cls.CURVATURE_MAX:
      return 0.0

    ratio = cls.center_boost_ratio(abs_curvature, amplitude, scale)
    correction = float(curvature) * ratio * cls.curvature_window(curvature) * float(np.clip(confidence, 0.0, 1.0))
    return float(np.clip(correction, -cls.MAX_CORRECTION, cls.MAX_CORRECTION))

  @classmethod
  def fit_params_from_buckets(cls, bias: np.ndarray, counts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    fit_amplitudes = np.zeros(cls.fit_shape(), dtype=np.float32)
    fit_scales = np.full(cls.fit_shape(), cls.CURVATURE_BUCKET_EDGES[5], dtype=np.float32)
    fit_valid = np.zeros(cls.fit_shape(), dtype=bool)

    scale_grid = np.geomspace(cls.SCALE_MIN, cls.SCALE_MAX, 24, dtype=np.float32)
    centers = cls.CURVATURE_BUCKET_CENTERS.astype(np.float64)

    for sign_idx in range(cls.fit_shape()[0]):
      sign = -1.0 if sign_idx == 0 else 1.0
      signed_centers = centers * sign
      for speed_idx in range(cls.fit_shape()[1]):
        bucket_counts = counts[sign_idx, speed_idx].astype(np.float64)
        valid_mask = bucket_counts >= cls.MIN_BUCKET_POINTS
        if int(np.count_nonzero(valid_mask)) < cls.FIT_MIN_VALID_BUCKETS:
          continue

        total_points = float(bucket_counts[valid_mask].sum())
        if total_points < cls.FIT_MIN_TOTAL_SAMPLES:
          continue

        target_ratio = np.divide(bias[sign_idx, speed_idx], signed_centers,
                                 out=np.zeros_like(bucket_counts, dtype=np.float64),
                                 where=np.abs(signed_centers) > 1e-12)
        target_ratio = np.clip(target_ratio, cls.AMP_MIN, cls.AMP_MAX)

        best_error = math.inf
        best_amp = 0.0
        best_scale = float(cls.CURVATURE_BUCKET_EDGES[5])
        weights = bucket_counts[valid_mask]
        valid_centers = centers[valid_mask]
        valid_targets = target_ratio[valid_mask]
        valid_weights = weights / max(float(weights.sum()), 1e-6)

        for scale in scale_grid:
          decay = np.exp(-((valid_centers / float(scale)) ** cls.FIT_EXPONENT))
          denom = float(np.sum(valid_weights * decay * decay))
          if denom < 1e-9:
            continue

          amp = float(np.sum(valid_weights * valid_targets * decay) / denom)
          amp = float(np.clip(amp, cls.AMP_MIN, cls.AMP_MAX))
          residual = valid_targets - (amp * decay)
          error = float(np.sum(valid_weights * residual * residual))
          if error < best_error:
            best_error = error
            best_amp = amp
            best_scale = float(scale)

        if not math.isfinite(best_error):
          continue

        fit_amplitudes[sign_idx, speed_idx] = best_amp
        fit_scales[sign_idx, speed_idx] = best_scale
        fit_valid[sign_idx, speed_idx] = True

    return fit_amplitudes, fit_scales, fit_valid

  @classmethod
  def corrections_from_fit(cls, fit_amplitudes: np.ndarray, fit_scales: np.ndarray, fit_valid: np.ndarray,
                           counts: np.ndarray) -> np.ndarray:
    corrections = np.zeros(cls.bucket_shape(), dtype=np.float32)
    centers = cls.CURVATURE_BUCKET_CENTERS.astype(np.float32)

    for sign_idx in range(cls.fit_shape()[0]):
      sign = -1.0 if sign_idx == 0 else 1.0
      signed_centers = sign * centers
      for speed_idx in range(cls.fit_shape()[1]):
        if not fit_valid[sign_idx, speed_idx]:
          continue
        total_points = float(counts[sign_idx, speed_idx].sum())
        confidence = cls.confidence(total_points)
        for curvature_idx, curvature in enumerate(signed_centers):
          corrections[sign_idx, speed_idx, curvature_idx] = cls.correction_from_fit(
            float(curvature),
            float(fit_amplitudes[sign_idx, speed_idx]),
            float(fit_scales[sign_idx, speed_idx]),
            confidence,
          )

    return corrections


class CurvatureDController(CurvatureDLookup):
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.use_params = False
    self.live_valid = False
    self.fit_amplitudes = np.zeros(self.fit_shape(), dtype=np.float32)
    self.fit_scales = np.full(self.fit_shape(), self.CURVATURE_BUCKET_EDGES[5], dtype=np.float32)
    self.fit_valid = np.zeros(self.fit_shape(), dtype=bool)

  def update_live_params(self, msg) -> None:
    expected_fit_size = self.fit_total_size()
    if (msg.version != VERSION or
        len(msg.fitAmplitudes) != expected_fit_size or
        len(msg.fitScales) != expected_fit_size or
        len(msg.fitValid) != expected_fit_size):
      self.reset()
      return

    self.use_params = bool(msg.useParams)
    self.live_valid = bool(msg.liveValid)
    self.fit_amplitudes = self.unflatten_fit(msg.fitAmplitudes, dtype=np.float32)
    self.fit_scales = self.unflatten_fit(msg.fitScales, dtype=np.float32)
    self.fit_valid = self.unflatten_fit(msg.fitValid, dtype=bool)

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

    sign_idx = self.sign_index(desired_curvature)
    low_idx, high_idx, alpha = self.speed_interp(v_ego)

    if not self.fit_valid[sign_idx, low_idx] and not self.fit_valid[sign_idx, high_idx]:
      return 0.0

    corr_low = 0.0
    corr_high = 0.0
    if self.fit_valid[sign_idx, low_idx]:
      corr_low = self.correction_from_fit(
        desired_curvature,
        float(self.fit_amplitudes[sign_idx, low_idx]),
        float(self.fit_scales[sign_idx, low_idx]),
      )
    if self.fit_valid[sign_idx, high_idx]:
      corr_high = self.correction_from_fit(
        desired_curvature,
        float(self.fit_amplitudes[sign_idx, high_idx]),
        float(self.fit_scales[sign_idx, high_idx]),
      )
    if low_idx == high_idx:
      return corr_low
    return float((1.0 - alpha) * corr_low + alpha * corr_high)

  def apply(self, desired_curvature: float, v_ego: float) -> float:
    return float(desired_curvature + self.get_correction(desired_curvature, v_ego))
