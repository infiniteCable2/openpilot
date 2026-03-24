import numpy as np

from cereal import car

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.locationd.curvatured import CurvatureEstimator, MAX_LEARN_ROLL_LATERAL_ACCEL
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDLookup


def get_estimator():
  CP = car.CarParams.new_message()
  CP.carFingerprint = "VOLKSWAGEN_TEST"
  CP.brand = "volkswagen"
  CP.steerControlType = car.CarParams.SteerControlType.curvatureDEPRECATED
  return CurvatureEstimator(CP)


class TestCurvatureEstimator:

  def test_speed_buckets_are_independent(self):
    estimator = get_estimator()
    desired_curvature = 32e-6

    for _ in range(CurvatureDLookup.FULL_CONFIDENCE_SAMPLES):
      estimator.add_measurement(desired_curvature, 0.0, 22.0)

    msg = estimator.get_msg()
    idx = CurvatureDLookup.indices(desired_curvature, 22.0)
    other_idx = CurvatureDLookup.indices(desired_curvature, 38.0)

    assert idx is not None
    assert other_idx is not None
    assert estimator.counts[idx] == CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    assert estimator.bias[idx] > 0.0
    assert estimator.counts[other_idx] == 0

  def test_center_caps_remain_tiny(self):
    estimator = get_estimator()
    desired_curvature = CurvatureDLookup.CURVATURE_MIN_STEP

    for _ in range(CurvatureDLookup.MAX_SAMPLES):
      estimator.add_measurement(desired_curvature, -1e-3, 16.0)

    idx = CurvatureDLookup.indices(desired_curvature, 16.0)
    assert idx is not None
    assert estimator.bias[idx] <= CurvatureDLookup.CORRECTION_CAP

  def test_calibration_percent_tracks_coverage(self):
    estimator = get_estimator()
    assert estimator.get_msg().liveCurvatureParameters.calPerc == 0

    for sign in (-1.0, 1.0):
      for speed_idx in range(len(CurvatureDLookup.SPEED_BUCKETS) - 1):
        v_ego = 0.5 * (CurvatureDLookup.SPEED_BUCKETS[speed_idx] + CurvatureDLookup.SPEED_BUCKETS[speed_idx + 1])
        desired_curvature = sign * 32e-6
        for _ in range(CurvatureDLookup.MIN_APPLY_SAMPLES):
          estimator.add_measurement(desired_curvature, 0.0, v_ego)

    assert estimator.get_msg().liveCurvatureParameters.calPerc == 100

  def test_message_contains_stable_bucket_arrays(self):
    estimator = get_estimator()
    desired_curvature = 32e-6
    v_ego = 22.0

    for _ in range(CurvatureDLookup.FULL_CONFIDENCE_SAMPLES):
      estimator.add_measurement(desired_curvature, 0.0, v_ego)

    estimator._update_current_lookup(desired_curvature, v_ego)
    msg = estimator.get_msg()
    idx = CurvatureDLookup.indices(desired_curvature, v_ego)

    assert idx is not None
    assert msg.liveCurvatureParameters.bucketSign == idx[0]
    assert msg.liveCurvatureParameters.bucketSpeed == idx[1]
    assert msg.liveCurvatureParameters.currentCorrection > 0.0
    assert len(msg.liveCurvatureParameters.corrections) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.counts) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.biases) == CurvatureDLookup.total_size()
    flat_idx = idx[0] * (len(CurvatureDLookup.SPEED_BUCKETS) - 1) + idx[1]
    assert msg.liveCurvatureParameters.corrections[flat_idx] > 0.0
    assert msg.liveCurvatureParameters.counts[flat_idx] == CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    assert msg.liveCurvatureParameters.biases[flat_idx] > 0.0

  def test_learning_is_blocked_for_larger_roll(self):
    estimator = get_estimator()

    small_roll = np.arcsin(0.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)
    large_roll = np.arcsin(1.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)

    assert estimator.roll_learning_allowed(float(small_roll))
    assert not estimator.roll_learning_allowed(float(large_roll))
