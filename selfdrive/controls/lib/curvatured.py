import numpy as np


VERSION = 1


class CurvatureDLookup:
  SPEED_BUCKETS = np.array([0.0, 8.0, 16.0, 24.0, 32.0, 40.0, 55.0])
  CURVATURE_BUCKETS = np.array([
    0.0,
    1e-6,
    2e-6,
    4e-6,
    8e-6,
    16e-6,
    32e-6,
    64e-6,
    128e-6,
    256e-6,
    512e-6,
    1e-3,
    2e-3,
    4e-3,
  ])
  CORRECTION_CAPS = np.array([
    2e-6,
    3e-6,
    4e-6,
    6e-6,
    8e-6,
    1.2e-5,
    1.8e-5,
    2.6e-5,
    3.8e-5,
    5.0e-5,
    6.2e-5,
    7.5e-5,
    9.0e-5,
  ])

  MIN_SPEED = 5.0
  MAX_LAT_ACCEL = 2.0
  MIN_APPLY_SAMPLES = 120
  FULL_CONFIDENCE_SAMPLES = 800
  MEAN_WINDOW = 400
  MAX_SAMPLES = 5000
  IMPORTANT_CURVATURE_BUCKETS = 8

  @classmethod
  def shape(cls) -> tuple[int, int, int]:
    return 2, len(cls.SPEED_BUCKETS) - 1, len(cls.CURVATURE_BUCKETS) - 1

  @classmethod
  def total_size(cls) -> int:
    sign_dim, speed_dim, curvature_dim = cls.shape()
    return sign_dim * speed_dim * curvature_dim

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
  def indices(cls, desired_curvature: float, v_ego: float) -> tuple[int, int, int] | None:
    sign_idx = cls._sign_idx(desired_curvature)
    speed_idx = cls._bucket_idx(cls.SPEED_BUCKETS, v_ego)
    curvature_idx = cls._bucket_idx(cls.CURVATURE_BUCKETS, abs(desired_curvature))

    if sign_idx is None or speed_idx is None or curvature_idx is None:
      return None
    return sign_idx, speed_idx, curvature_idx

  @classmethod
  def cap_for_bucket(cls, curvature_idx: int) -> float:
    return float(cls.CORRECTION_CAPS[curvature_idx])

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
    caps = cls.CORRECTION_CAPS[np.newaxis, np.newaxis, :]
    confidence = cls.confidence(counts.astype(np.float64))
    confidence = np.where(counts >= cls.MIN_APPLY_SAMPLES, confidence, 0.0)
    return np.clip(bias, -caps, caps) * confidence

  @classmethod
  def calibration_percent(cls, counts: np.ndarray) -> int:
    important = counts[:, :, :cls.IMPORTANT_CURVATURE_BUCKETS]
    covered = np.count_nonzero(important >= cls.MIN_APPLY_SAMPLES)
    return int(round(100.0 * covered / important.size))


class CurvatureDController(CurvatureDLookup):
  def __init__(self):
    self.bias = np.zeros(self.shape(), dtype=np.float64)
    self.counts = np.zeros(self.shape(), dtype=np.int32)
    self.corrections = np.zeros(self.shape(), dtype=np.float64)
    self.use_params = False
    self.live_valid = False

  def reset(self):
    self.bias.fill(0.0)
    self.counts.fill(0)
    self.corrections.fill(0.0)
    self.use_params = False
    self.live_valid = False

  def update_live_params(self, msg) -> None:
    valid_shape = len(msg.bias) == self.total_size() and len(msg.counts) == self.total_size()
    self.use_params = bool(msg.useParams)
    self.live_valid = bool(msg.liveValid) and msg.version == VERSION and valid_shape

    if not self.live_valid:
      self.bias.fill(0.0)
      self.counts.fill(0)
      self.corrections.fill(0.0)
      return

    self.bias = self.unflatten(msg.bias).astype(np.float64)
    self.counts = self.unflatten(msg.counts).astype(np.int32)
    self.corrections = self.corrections_from_bias(self.bias, self.counts)

  def get_correction(self, desired_curvature: float, v_ego: float) -> float:
    if not (self.use_params and self.live_valid):
      return 0.0

    if abs(desired_curvature) * (v_ego ** 2) > self.MAX_LAT_ACCEL:
      return 0.0

    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      return 0.0

    sign_idx, speed_idx, curvature_idx = idx
    return float(self.corrections[sign_idx, speed_idx, curvature_idx])

  def apply(self, desired_curvature: float, v_ego: float) -> float:
    return float(desired_curvature + self.get_correction(desired_curvature, v_ego))
