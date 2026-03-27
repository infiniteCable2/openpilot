import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController
from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup, VERSION


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

  def test_correction_fades_outside_supported_curvature_range(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    self._set_curve(msg, 3, {
      4: 4e-6,
      5: 8e-6,
      6: 6e-6,
    })
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    inside = controller.get_correction(5.0e-5, v_ego)
    lower_fade = controller.get_correction(1.0e-5, v_ego)
    upper_fade = controller.get_correction(2.0e-4, v_ego)

    assert inside > 0.0
    assert 0.0 <= lower_fade < inside
    assert 0.0 <= upper_fade < inside

  def test_outer_bucket_range_is_supported(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    outer_idx = CurvatureDLookup.curvature_index(3.0e-3)
    assert outer_idx is not None
    self._set_curve(msg, 3, {outer_idx: 8.0e-5})
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    outer = controller.get_correction(3.0e-3, v_ego)

    assert outer > 0.0
