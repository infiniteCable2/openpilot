import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController, CurvatureDLookup, VERSION


class TestCurvatureDController:
  @staticmethod
  def _set_curve(msg, speed_idx: int, values: dict[int, float]):
    corrections = list(msg.liveCurvatureParameters.corrections)
    fit_valid = list(msg.liveCurvatureParameters.fitValid)
    if len(corrections) != CurvatureDLookup.total_size():
      corrections = [0.0] * CurvatureDLookup.total_size()
    if len(fit_valid) != CurvatureDLookup.total_size():
      fit_valid = [False] * CurvatureDLookup.total_size()

    width = len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS)
    for curvature_idx, value in values.items():
      flat_idx = speed_idx * width + curvature_idx
      corrections[flat_idx] = value
      fit_valid[flat_idx] = True

    msg.liveCurvatureParameters.corrections = corrections
    msg.liveCurvatureParameters.fitValid = fit_valid

  def test_apply_interpolates_between_neighbor_speed_curves(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 2, {curvature_idx: 4e-6})
    self._set_curve(msg, 3, {curvature_idx: 12e-6})
    controller.update_live_params(msg.liveCurvatureParameters)

    low_speed = float(CurvatureDLookup.SPEED_ANCHORS[2])
    high_speed = float(CurvatureDLookup.SPEED_ANCHORS[3])
    mid_speed = 0.5 * (low_speed + high_speed)

    low = controller.get_correction(32e-6, low_speed)
    mid = controller.get_correction(32e-6, mid_speed)
    high = controller.get_correction(32e-6, high_speed)

    assert low > 0.0
    assert high > low
    assert low < mid < high

  def test_negative_curvature_uses_same_curve_with_negative_sign(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 3, {curvature_idx: 8e-6})
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    pos = controller.get_correction(32e-6, v_ego)
    neg = controller.get_correction(-32e-6, v_ego)

    assert pos > 0.0
    assert neg < 0.0
    assert abs(pos + neg) < 1e-12

  def test_invalid_message_disables_corrections(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = False
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.corrections = [0.0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.fitValid = [False] * CurvatureDLookup.total_size()

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 20.0) == 32e-6
