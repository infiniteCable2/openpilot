import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController, CurvatureDLookup, VERSION


class TestCurvatureDController:
  @staticmethod
  def _set_bucket(msg, idx, correction):
    corrections = [0.0] * CurvatureDLookup.total_size()
    counts = [0] * CurvatureDLookup.total_size()
    flat_idx = idx[0] * (len(CurvatureDLookup.SPEED_BUCKETS) - 1) + idx[1]
    corrections[flat_idx] = correction
    counts[flat_idx] = CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    msg.liveCurvatureParameters.corrections = corrections
    msg.liveCurvatureParameters.counts = counts
    msg.liveCurvatureParameters.biases = corrections
    msg.liveCurvatureParameters.currentCorrection = correction
    msg.liveCurvatureParameters.currentBias = correction
    msg.liveCurvatureParameters.currentBucketPoints = CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    msg.liveCurvatureParameters.bucketSign = idx[0]
    msg.liveCurvatureParameters.bucketSpeed = idx[1]

  def test_apply_uses_matching_bucket_only(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    idx = CurvatureDLookup.indices(32e-6, 22.0)
    assert idx is not None
    self._set_bucket(msg, idx, 20e-6)

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 22.0) > 32e-6
    assert controller.apply(32e-6, 38.0) == 32e-6
    assert controller.apply(2e-4, 22.0) == 2e-4

  def test_invalid_message_disables_corrections(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = False
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.corrections = [0.0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.bucketSign = -1
    msg.liveCurvatureParameters.bucketSpeed = -1

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 20.0) == 32e-6

  def test_apply_fades_near_center_limits(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    idx = CurvatureDLookup.indices(32e-6, 22.0)
    assert idx is not None
    self._set_bucket(msg, idx, 10e-6)

    controller.update_live_params(msg.liveCurvatureParameters)

    mid = controller.get_correction(32e-6, 22.0)
    low = controller.get_correction(2e-6, 22.0)
    high = controller.get_correction(4.5e-5, 22.0)

    assert mid > 0.0
    assert 0.0 < low < mid
    assert 0.0 < high < mid
    assert controller.get_correction(CurvatureDLookup.CURVATURE_MIN_STEP, 22.0) == 0.0
    assert controller.get_correction(CurvatureDLookup.CENTER_CURVATURE_MAX, 22.0) == 0.0
