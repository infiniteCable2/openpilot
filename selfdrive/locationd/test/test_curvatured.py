import numpy as np

from cereal import car

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.locationd.curvatured import CurvatureEstimator, CurvatureDLookup, MAX_LEARN_ROLL_LATERAL_ACCEL


def get_estimator():
  CP = car.CarParams.new_message()
  CP.carFingerprint = "VOLKSWAGEN_TEST"
  CP.brand = "volkswagen"
  CP.steerControlType = car.CarParams.SteerControlType.curvatureDEPRECATED
  return CurvatureEstimator(CP)


class TestCurvatureEstimator:
  @staticmethod
  def _train_speed_curve(estimator, v_ego: float):
    for desired_curvature in (8e-6, 2e-5, 5e-5, 1.2e-4):
      for sign in (-1.0, 1.0):
        for _ in range(int(CurvatureDLookup.FIT_MIN_TOTAL_SAMPLES / 4)):
          desired = sign * desired_curvature
          estimator.add_measurement(desired, desired * 0.6, v_ego)

  def test_left_and_right_feed_the_same_bucket_curve(self):
    estimator = get_estimator()
    desired_curvature = 32e-6
    v_ego = 22.0

    for _ in range(40):
      estimator.add_measurement(desired_curvature, desired_curvature * 0.7, v_ego)
      estimator.add_measurement(-desired_curvature, -desired_curvature * 0.7, v_ego)

    curvature_idx = CurvatureDLookup.curvature_index(desired_curvature)
    assert curvature_idx is not None
    speed_weights = CurvatureDLookup.learning_speed_weights(v_ego)
    assert len(speed_weights) == 2

    total = 0.0
    for speed_idx, weight in speed_weights:
      bucket_count = float(estimator.counts[speed_idx, curvature_idx])
      total += bucket_count
      assert np.isclose(bucket_count, 80.0 * weight)
      assert estimator.bias[speed_idx, curvature_idx] > 0.0

    assert np.isclose(total, 80.0)

  def test_learning_is_weighted_between_neighbor_speed_anchors(self):
    estimator = get_estimator()
    desired_curvature = 32e-6
    low_speed = float(CurvatureDLookup.SPEED_ANCHORS[2])
    high_speed = float(CurvatureDLookup.SPEED_ANCHORS[3])
    v_ego = 0.25 * low_speed + 0.75 * high_speed

    estimator.add_measurement(desired_curvature, desired_curvature * 0.6, v_ego)

    curvature_idx = CurvatureDLookup.curvature_index(desired_curvature)
    assert curvature_idx is not None
    speed_weights = CurvatureDLookup.learning_speed_weights(v_ego)
    assert len(speed_weights) == 2

    for speed_idx, weight in speed_weights:
      assert np.isclose(float(estimator.counts[speed_idx, curvature_idx]), weight)

  def test_bucket_bias_cap_remains_tiny(self):
    estimator = get_estimator()
    desired_curvature = 8e-6

    for _ in range(CurvatureDLookup.MAX_SAMPLES):
      estimator.add_measurement(desired_curvature, -1e-3, 16.0)

    idx = CurvatureDLookup.indices(desired_curvature, 16.0)
    assert idx is not None
    assert estimator.bias[idx] <= CurvatureDLookup.CORRECTION_CAP

  def test_calibration_percent_tracks_valid_speed_curves(self):
    estimator = get_estimator()
    assert estimator.get_msg().liveCurvatureParameters.calPerc == 0

    for v_ego in CurvatureDLookup.SPEED_ANCHORS:
      self._train_speed_curve(estimator, float(v_ego))

    assert estimator.get_msg().liveCurvatureParameters.calPerc == 100

  def test_message_contains_symmetric_fit_curve(self):
    estimator = get_estimator()
    desired_curvature = 32e-6
    v_ego = 22.0

    self._train_speed_curve(estimator, v_ego)
    estimator._update_current_lookup(desired_curvature, v_ego)
    msg = estimator.get_msg()
    idx = CurvatureDLookup.indices(desired_curvature, v_ego)

    assert idx is not None
    assert msg.liveCurvatureParameters.bucketSpeed == idx[0]
    assert msg.liveCurvatureParameters.bucketCurvature == idx[1]
    assert msg.liveCurvatureParameters.currentCorrection > 0.0
    assert len(msg.liveCurvatureParameters.corrections) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.counts) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.biases) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.fitValid) == CurvatureDLookup.total_size()

  def test_fit_valid_is_limited_to_supported_curvature_range(self):
    estimator = get_estimator()
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])

    for desired_curvature in (8e-6, 2e-5, 5e-5, 1.2e-4):
      for _ in range(40):
        estimator.add_measurement(desired_curvature, desired_curvature * 0.6, v_ego)

    msg = estimator.get_msg().liveCurvatureParameters
    fit_valid = CurvatureDLookup.unflatten_bucket(list(msg.fitValid), dtype=bool)
    speed_idx = 3
    valid_idx = np.flatnonzero(fit_valid[speed_idx])

    assert len(valid_idx) > 0
    assert valid_idx[0] > 0
    assert valid_idx[-1] < len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS) - 1

  def test_learning_is_blocked_for_larger_roll(self):
    estimator = get_estimator()

    small_roll = np.arcsin(0.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)
    large_roll = np.arcsin(1.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)

    assert estimator.roll_learning_allowed(float(small_roll))
    assert not estimator.roll_learning_allowed(float(large_roll))
