import numpy as np

from cereal import car

from opendbc.car.volkswagen.values import CAR
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.locationd.curvatured import CurvatureEstimator, CurvatureDLookup, MAX_LEARN_ROLL_LATERAL_ACCEL


def get_estimator():
  CP = car.CarParams.new_message()
  CP.carFingerprint = CAR.CUPRA_BORN_MK1
  CP.brand = "volkswagen"
  CP.steerControlType = car.CarParams.SteerControlType.curvatureDEPRECATED
  return CurvatureEstimator(CP)


class TestCurvatureEstimator:
  @staticmethod
  def _train_speed_curve(estimator, v_ego: float):
    for desired_curvature in CurvatureDLookup.CURVATURE_BUCKET_CENTERS:
      for sign in (-1.0, 1.0):
        for _ in range(int(CurvatureDLookup.MIN_BUCKET_POINTS[CurvatureDLookup.curvature_index(float(desired_curvature))]) + 2):
          desired = sign * float(desired_curvature)
          estimator.add_measurement(desired, desired * 0.6, v_ego)

  @staticmethod
  def _train_speed_curve_full(estimator, v_ego: float):
    for desired_curvature in CurvatureDLookup.CURVATURE_BUCKET_CENTERS:
      for sign in (-1.0, 1.0):
        curvature_idx = CurvatureDLookup.curvature_index(float(desired_curvature))
        assert curvature_idx is not None
        for _ in range(int(CurvatureDLookup.FULL_CONFIDENCE_BUCKET_SAMPLES[curvature_idx]) + 2):
          desired = sign * float(desired_curvature)
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
    assert estimator.bias[idx] <= CurvatureDLookup.correction_cap(desired_curvature)

  def test_relative_correction_cap_envelope(self):
    assert np.isclose(CurvatureDLookup.correction_cap(1.0e-5), 0.5e-5)
    assert np.isclose(CurvatureDLookup.correction_cap(1.0e-4), 0.5e-4)
    assert np.isclose(CurvatureDLookup.correction_cap(1.024e-3), 0.25 * 1.024e-3)
    assert CurvatureDLookup.correction_cap(3.0e-3) < CurvatureDLookup.correction_cap(1.024e-3)
    assert CurvatureDLookup.correction_cap(CurvatureDLookup.CURVATURE_MAX) == 0.0

  def test_calibration_percent_tracks_valid_speed_curves(self):
    estimator = get_estimator()
    assert estimator.get_msg().liveCurvatureParameters.calPerc == 0

    for v_ego in CurvatureDLookup.SPEED_ANCHORS:
      self._train_speed_curve_full(estimator, float(v_ego))

    assert estimator.get_msg().liveCurvatureParameters.calPerc == 100

  def test_required_valid_bucket_count_decreases_with_speed(self):
    low = CurvatureDLookup.required_valid_bucket_count(0)
    mid = CurvatureDLookup.required_valid_bucket_count(3)
    high = CurvatureDLookup.required_valid_bucket_count(6)

    assert low == len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS)
    assert low >= mid >= high >= CurvatureDLookup.FIT_MIN_VALID_BUCKETS

  def test_fit_valid_no_longer_requires_global_total_samples(self):
    speed_idx = 3
    bucket_idx = 5
    counts = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    bias = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)

    counts[speed_idx, bucket_idx] = float(CurvatureDLookup.MIN_BUCKET_POINTS[bucket_idx] + 1.0)
    bias[speed_idx, bucket_idx] = float(0.5 * CurvatureDLookup.correction_cap(
      float(CurvatureDLookup.CURVATURE_BUCKET_CENTERS[bucket_idx])
    ))

    filler_idx = np.arange(CurvatureDLookup.required_valid_bucket_count(speed_idx), dtype=int)
    filler_idx = filler_idx[filler_idx != bucket_idx]
    counts[speed_idx, filler_idx] = CurvatureDLookup.MIN_BUCKET_POINTS[filler_idx] + 1.0

    fit_corrections, fit_valid = CurvatureDLookup.build_fit_corrections(bias, counts)

    assert fit_valid[speed_idx, bucket_idx]
    assert float(fit_corrections[speed_idx, bucket_idx]) > 0.0

  def test_calibration_percent_requires_full_local_strength(self):
    speed_idx = 3
    counts = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    required = CurvatureDLookup.required_valid_bucket_count(speed_idx)
    selected = np.arange(required, dtype=int)

    counts[speed_idx, selected] = CurvatureDLookup.MIN_BUCKET_POINTS[selected]
    assert np.all(counts[speed_idx, selected] >= CurvatureDLookup.MIN_BUCKET_POINTS[selected])
    assert not CurvatureDLookup.speed_curve_valid(counts, speed_idx)
    assert not CurvatureDLookup.speed_curve_fully_calibrated(counts, speed_idx)
    assert CurvatureDLookup.calibration_percent(counts) == 0

    counts[speed_idx, selected] = CurvatureDLookup.FULL_CONFIDENCE_BUCKET_SAMPLES[selected]
    assert CurvatureDLookup.speed_curve_fully_calibrated(counts, speed_idx)
    assert CurvatureDLookup.calibration_percent(counts) > 0

  def test_speed_curve_strength_grows_smoothly_from_bucket_strengths(self):
    speed_idx = 4
    counts = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    required = CurvatureDLookup.required_valid_bucket_count(speed_idx)
    selected = np.arange(required, dtype=int)

    counts[speed_idx, selected] = CurvatureDLookup.MIN_BUCKET_POINTS[selected]
    assert np.isclose(CurvatureDLookup.speed_curve_strength(counts[speed_idx], speed_idx), 0.0)

    counts[speed_idx, selected] = CurvatureDLookup.MIN_BUCKET_POINTS[selected] + 0.5 * (
      CurvatureDLookup.FULL_CONFIDENCE_BUCKET_SAMPLES[selected] - CurvatureDLookup.MIN_BUCKET_POINTS[selected]
    )
    assert np.isclose(CurvatureDLookup.speed_curve_strength(counts[speed_idx], speed_idx), 0.5)

  def test_message_contains_symmetric_fit_curve(self):
    estimator = get_estimator()
    desired_curvature = 32e-6
    v_ego = 22.0

    self._train_speed_curve(estimator, v_ego)
    estimator._update_current_lookup(desired_curvature, v_ego)
    msg = estimator.get_msg(include_debug=True, include_preview=True)
    idx = CurvatureDLookup.indices(desired_curvature, v_ego)

    assert idx is not None
    assert msg.liveCurvatureParameters.bucketSpeed == idx[0]
    assert msg.liveCurvatureParameters.bucketCurvature == idx[1]
    assert msg.liveCurvatureParameters.currentCorrection > 0.0
    assert len(msg.liveCurvatureParameters.corrections) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.counts) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.biases) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.fitValid) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.previewCorrections) == CurvatureDLookup.total_size()
    assert len(msg.liveCurvatureParameters.previewValid) == CurvatureDLookup.total_size()

  def test_fit_valid_allows_noncontiguous_supported_buckets(self):
    estimator = get_estimator()
    speed_idx = len(CurvatureDLookup.SPEED_ANCHORS) - 1
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[speed_idx])
    required = CurvatureDLookup.required_valid_bucket_count(speed_idx)
    selected_indices = list(range(0, required - 1)) + [required]

    for bucket_idx in selected_indices:
      desired_curvature = float(CurvatureDLookup.CURVATURE_BUCKET_CENTERS[bucket_idx])
      for _ in range(int(CurvatureDLookup.MIN_BUCKET_POINTS[bucket_idx]) + 120):
        estimator.add_measurement(desired_curvature, desired_curvature * 0.6, v_ego)

    msg = estimator.get_msg().liveCurvatureParameters
    fit_valid = CurvatureDLookup.unflatten_bucket(list(msg.fitValid), dtype=bool)

    assert fit_valid[speed_idx, selected_indices].all()
    assert not fit_valid[speed_idx, required - 1]

  def test_fit_corrections_are_zero_outside_fit_valid(self):
    speed_idx = 3
    counts = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    bias = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    selected = np.array([5, 6, 7, 8], dtype=int)

    counts[speed_idx, selected] = CurvatureDLookup.MIN_BUCKET_POINTS[selected] + 40.0
    bias[speed_idx, selected] = np.array([2.0e-6, 6.0e-6, 1.2e-5, 2.0e-5], dtype=np.float32)

    fit_corrections, fit_valid = CurvatureDLookup.build_fit_corrections(bias, counts)

    assert fit_valid[speed_idx, selected].all()
    assert np.allclose(fit_corrections[speed_idx, ~fit_valid[speed_idx]], 0.0)

  def test_interp_curve_value_does_not_bridge_invalid_gap(self):
    speed_idx = 3
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[speed_idx])
    fit_corrections = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    fit_valid = np.zeros(CurvatureDLookup.bucket_shape(), dtype=bool)

    fit_valid[speed_idx, 3] = True
    fit_valid[speed_idx, 6] = True
    fit_corrections[speed_idx, 3] = 1.0e-6
    fit_corrections[speed_idx, 6] = 8.0e-6

    gap_curvature = float(CurvatureDLookup.CURVATURE_BUCKET_CENTERS[4])
    valid_curvature = float(CurvatureDLookup.CURVATURE_BUCKET_CENTERS[3])

    assert CurvatureDLookup.interp_curve_value(fit_corrections, fit_valid, v_ego, gap_curvature) == 0.0
    assert CurvatureDLookup.interp_curve_value(fit_corrections, fit_valid, v_ego, valid_curvature) > 0.0

  def test_learning_is_blocked_for_larger_roll(self):
    estimator = get_estimator()

    small_roll = np.arcsin(0.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)
    large_roll = np.arcsin(1.5 * MAX_LEARN_ROLL_LATERAL_ACCEL / ACCELERATION_DUE_TO_GRAVITY)

    assert estimator.roll_learning_allowed(float(small_roll))
    assert not estimator.roll_learning_allowed(float(large_roll))

  def test_actual_curvature_subtracts_roll_compensation(self):
    yaw_rate = 0.03
    v_ego = 20.0
    roll_comp = 4.0e-4

    raw_curvature = yaw_rate / v_ego
    corrected_curvature = CurvatureDLookup.actual_curvature_from_yaw_rate(yaw_rate, v_ego, roll_comp)

    assert np.isclose(corrected_curvature, raw_curvature - roll_comp)
