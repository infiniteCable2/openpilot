from cereal import car

from openpilot.selfdrive.locationd.curvatured import CurvatureEstimator
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
    desired_curvature = 64e-6

    for _ in range(CurvatureDLookup.FULL_CONFIDENCE_SAMPLES):
      estimator.add_measurement(desired_curvature, 0.0, 22.0)

    msg = estimator.get_msg()
    idx = CurvatureDLookup.indices(desired_curvature, 22.0)
    other_idx = CurvatureDLookup.indices(desired_curvature, 38.0)

    bias = CurvatureDLookup.unflatten(msg.liveCurvatureParameters.bias)
    counts = CurvatureDLookup.unflatten(msg.liveCurvatureParameters.counts)

    assert idx is not None
    assert other_idx is not None
    assert counts[idx] == CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    assert bias[idx] > 0.0
    assert counts[other_idx] == 0

  def test_center_caps_remain_tiny(self):
    estimator = get_estimator()
    desired_curvature = 1.5e-6

    for _ in range(CurvatureDLookup.MAX_SAMPLES):
      estimator.add_measurement(desired_curvature, -1e-3, 16.0)

    idx = CurvatureDLookup.indices(desired_curvature, 16.0)
    assert idx is not None
    assert estimator.bias[idx] <= CurvatureDLookup.CORRECTION_CAPS[1]

  def test_calibration_percent_tracks_coverage(self):
    estimator = get_estimator()
    assert estimator.get_msg().liveCurvatureParameters.calPerc == 0

    important_bucket_count = 2 * (len(CurvatureDLookup.SPEED_BUCKETS) - 1) * CurvatureDLookup.IMPORTANT_CURVATURE_BUCKETS
    for sign in (-1.0, 1.0):
      for speed_idx in range(len(CurvatureDLookup.SPEED_BUCKETS) - 1):
        v_ego = 0.5 * (CurvatureDLookup.SPEED_BUCKETS[speed_idx] + CurvatureDLookup.SPEED_BUCKETS[speed_idx + 1])
        for curvature_idx in range(CurvatureDLookup.IMPORTANT_CURVATURE_BUCKETS):
          desired_curvature = sign * 0.5 * (CurvatureDLookup.CURVATURE_BUCKETS[curvature_idx] +
                                            CurvatureDLookup.CURVATURE_BUCKETS[curvature_idx + 1])
          for _ in range(CurvatureDLookup.MIN_APPLY_SAMPLES):
            estimator.add_measurement(desired_curvature, 0.0, v_ego)

    assert important_bucket_count > 0
    assert estimator.get_msg().liveCurvatureParameters.calPerc == 100
