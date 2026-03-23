import numpy as np


VERSION = 1


class CurvatureDLookup:
  CURVATURE_MIN_STEP = 1.0e-6
  DBC_CURVATURE_STEP = 6.7e-6  # HCA_03 Curvature scale in vw_meb.dbc
  SPEED_BUCKETS = np.array([0.0, 15.0, 25.0, 35.0, 55.0])
  CENTER_CURVATURE_MAX = 5.0e-5
  CORRECTION_CAP = 1.2e-5

  MIN_SPEED = 5.0
  MAX_LAT_ACCEL = 2.0
  MIN_APPLY_SAMPLES = 120
  FULL_CONFIDENCE_SAMPLES = 800
  MEAN_WINDOW = 400
  MAX_SAMPLES = 5000

  @classmethod
  def shape(cls) -> tuple[int, int]:
    return 2, len(cls.SPEED_BUCKETS) - 1

  @classmethod
  def total_size(cls) -> int:
    sign_dim, speed_dim = cls.shape()
    return sign_dim * speed_dim

  @staticmethod
  def _sign_idx(curvature: float) -> int | None:
    if curvature > 0.0:
      return 1
    if curvature < 0.0:
      return 0
    return None

  @staticmethod
  def _bucket_idx(edges: np.ndarray, value: float) -> int | None:
    if value <= 0.0 or value > float(edges[-1]):
      return None
    idx = int(np.searchsorted(edges, value, side='right') - 1)
    return int(np.clip(idx, 0, len(edges) - 2))

  @classmethod
  def in_center_range(cls, desired_curvature: float) -> bool:
    abs_curvature = abs(desired_curvature)
    return cls.CURVATURE_MIN_STEP <= abs_curvature <= cls.CENTER_CURVATURE_MAX

  @classmethod
  def indices(cls, desired_curvature: float, v_ego: float) -> tuple[int, int] | None:
    sign_idx = cls._sign_idx(desired_curvature)
    speed_idx = cls._bucket_idx(cls.SPEED_BUCKETS, v_ego)

    if sign_idx is None or speed_idx is None or not cls.in_center_range(desired_curvature):
      return None
    return sign_idx, speed_idx

  @classmethod
  def flatten(cls, values: np.ndarray) -> list:
    return values.reshape(cls.total_size()).tolist()

  @classmethod
  def unflatten(cls, values: list | tuple) -> np.ndarray:
    return np.array(values).reshape(cls.shape())

  @classmethod
  def confidence(cls, sample_count: np.ndarray | float | int) -> np.ndarray | float:
    confidence = np.interp(sample_count,
                           [cls.MIN_APPLY_SAMPLES, cls.FULL_CONFIDENCE_SAMPLES],
                           [0.0, 1.0])
    return np.clip(confidence, 0.0, 1.0)

  @classmethod
  def corrections_from_bias(cls, bias: np.ndarray, counts: np.ndarray) -> np.ndarray:
    confidence = cls.confidence(counts.astype(np.float64))
    confidence = np.where(counts >= cls.MIN_APPLY_SAMPLES, confidence, 0.0)
    return np.clip(bias, -cls.CORRECTION_CAP, cls.CORRECTION_CAP) * confidence

  @classmethod
  def calibration_percent(cls, counts: np.ndarray) -> int:
    covered = np.count_nonzero(counts >= cls.MIN_APPLY_SAMPLES)
    return int(round(100.0 * covered / counts.size))


class CurvatureDController(CurvatureDLookup):
  def __init__(self):
    self.use_params = False
    self.live_valid = False
    self.current_correction = 0.0
    self.bucket_sign = -1
    self.bucket_speed = -1

  def reset(self):
    self.use_params = False
    self.live_valid = False
    self.current_correction = 0.0
    self.bucket_sign = -1
    self.bucket_speed = -1

  def update_live_params(self, msg) -> None:
    self.use_params = bool(msg.useParams)
    valid_bucket = (
      msg.bucketSign in (-1, 0, 1) and
      -1 <= msg.bucketSpeed < len(self.SPEED_BUCKETS) - 1
    )
    self.live_valid = bool(msg.liveValid) and msg.version == VERSION and valid_bucket

    if not self.live_valid:
      self.current_correction = 0.0
      self.bucket_sign = -1
      self.bucket_speed = -1
      return

    self.current_correction = float(msg.currentCorrection)
    self.bucket_sign = int(msg.bucketSign)
    self.bucket_speed = int(msg.bucketSpeed)

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not (self.use_params and self.live_valid):
      return 0.0

    if abs(desired_curvature) * (v_ego ** 2) > self.MAX_LAT_ACCEL:
      return 0.0

    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      return 0.0

    sign_idx, speed_idx = idx
    if (sign_idx, speed_idx) != (self.bucket_sign, self.bucket_speed):
      return 0.0
    return self.current_correction

  def apply(self, desired_curvature: float, v_ego: float) -> float:
    return float(desired_curvature + self.get_correction(desired_curvature, v_ego))
