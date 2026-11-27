import openpilot.cereal.messaging as messaging

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController
from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup, VERSION


class TestCurvatureDController(OpenpilotTestCase):
  @staticmethod
  def _set_curve(msg, speed_idx: int, values: dict[int, float]):
    corrections = list(msg.lateralCurvatureParameters.corrections)
    fit_valid = list(msg.lateralCurvatureParameters.fitValid)
    if len(corrections) != CurvatureDLookup.total_size():
      corrections = [0.0] * CurvatureDLookup.total_size()
    if len(fit_valid) != CurvatureDLookup.total_size():
      fit_valid = [False] * CurvatureDLookup.total_size()

    width = len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS)
    for curvature_idx, value in values.items():
      flat_idx = speed_idx * width + curvature_idx
      corrections[flat_idx] = value
      fit_valid[flat_idx] = True

    msg.lateralCurvatureParameters.corrections = corrections
    msg.lateralCurvatureParameters.fitValid = fit_valid

  def test_apply_interpolates_between_neighbor_speed_curves(self):
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 2, {curvature_idx: 4e-6})
    self._set_curve(msg, 3, {curvature_idx: 12e-6})
    controller.update_live_params(msg.lateralCurvatureParameters)

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
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 3, {curvature_idx: 8e-6})
    controller.update_live_params(msg.lateralCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    pos = controller.get_correction(32e-6, v_ego)
    neg = controller.get_correction(-32e-6, v_ego)

    assert pos > 0.0
    assert neg < 0.0
    assert abs(pos + neg) < 1e-12

  def test_live_valid_false_disables_corrections(self):
    """When the message's liveValid flag is False, get_correction must return 0.0
    regardless of any cached state, since the upstream signal is invalid.
    """
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = False
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.corrections = [0.0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.fitValid = [False] * CurvatureDLookup.total_size()

    controller.update_live_params(msg.lateralCurvatureParameters)

    # Without a learnable curve and no live_valid, get_correction returns 0.0.
    assert controller.get_correction(32e-6, 20.0) == 0.0

  def test_version_mismatch_resets_controller(self):
    """A message with the wrong version number must trigger a full reset, not
    a partial state update. This is the actual 'invalid message' path.
    """
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION + 99  # intentionally wrong
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.corrections = [0.0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.fitValid = [False] * CurvatureDLookup.total_size()

    # Pretend we had a valid state before the bad message
    controller.use_params = True
    controller.live_valid = True

    controller.update_live_params(msg.lateralCurvatureParameters)

    # Bad version -> full reset -> both flags back to False
    assert not controller.use_params
    assert not controller.live_valid
    assert controller.get_correction(32e-6, 20.0) == 0.0

  def test_size_mismatch_resets_controller(self):
    """A message with the wrong corrections array size must also trigger a full reset."""
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    # Wrong size (1 element instead of total_size())
    msg.lateralCurvatureParameters.corrections = [0.0]
    msg.lateralCurvatureParameters.counts = [0]
    msg.lateralCurvatureParameters.biases = [0.0]
    msg.lateralCurvatureParameters.fitValid = [False]

    controller.use_params = True
    controller.live_valid = True

    controller.update_live_params(msg.lateralCurvatureParameters)

    assert not controller.use_params
    assert not controller.live_valid
    assert controller.get_correction(32e-6, 20.0) == 0.0

  def test_correction_fades_outside_supported_curvature_range(self):
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    self._set_curve(msg, 3, {
      4: 4e-6,
      5: 8e-6,
      6: 6e-6,
    })
    controller.update_live_params(msg.lateralCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    inside = controller.get_correction(5.0e-5, v_ego)
    lower_fade = controller.get_correction(1.0e-5, v_ego)
    upper_fade = controller.get_correction(2.0e-4, v_ego)

    assert inside > 0.0
    assert 0.0 <= lower_fade < inside
    assert 0.0 <= upper_fade < inside

  def test_outer_bucket_range_is_supported(self):
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    outer_idx = CurvatureDLookup.curvature_index(1.5e-3)
    assert outer_idx is not None
    self._set_curve(msg, 0, {outer_idx: 8.0e-5})
    controller.update_live_params(msg.lateralCurvatureParameters)

    # Use the lowest speed anchor so the global outer fade range remains
    # inside the independent lateral-acceleration safety gate.
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[0])
    outer = controller.get_correction(1.5e-3, v_ego)

    assert outer > 0.0

  def test_outer_range_fades_to_zero_past_last_bucket_edge(self):
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    outer_idx = len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS) - 1
    self._set_curve(msg, 0, {outer_idx: 8.0e-5})
    controller.update_live_params(msg.lateralCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[0])
    last_edge = float(CurvatureDLookup.CURVATURE_BUCKET_MAX)
    fade_mid = 0.5 * (last_edge + float(CurvatureDLookup.CURVATURE_MAX))

    at_last_edge = controller.get_correction(last_edge, v_ego)
    in_fade = controller.get_correction(fade_mid, v_ego)
    at_max = controller.get_correction(float(CurvatureDLookup.CURVATURE_MAX), v_ego)

    assert at_last_edge > 0.0
    assert 0.0 < in_fade < at_last_edge
    assert at_max == 0.0

  def test_fit_is_decoded_once_per_message_and_cleared_on_service_loss(self, mocker):
    controller = CurvatureDController()
    event = messaging.new_message('lateralCurvatureParameters')
    fit = event.lateralCurvatureParameters
    fit.version = VERSION
    fit.useParams = True
    fit.liveValid = True
    k = float(CurvatureDLookup.CURVATURE_BUCKET_CENTERS[6])
    v = float(CurvatureDLookup.SPEED_ANCHORS[3])
    self._set_curve(event, 3, {6: 0.3 * k})
    sm = mocker.MagicMock()
    sm.all_checks.return_value = True
    sm.recv_frame = {'lateralCurvatureParameters': 1}
    sm.__getitem__.return_value = fit
    decode = mocker.patch.object(controller, 'update_live_params', wraps=controller.update_live_params)

    for _ in range(25):
      controller.update_from_submaster(sm)
    assert decode.call_count == 1
    assert controller.get_correction(k, v) > 0.0

    sm.all_checks.return_value = False
    controller.update_from_submaster(sm)
    assert controller.get_correction(k, v) == 0.0
    sm.all_checks.return_value = True
    controller.update_from_submaster(sm)
    assert decode.call_count == 2
    assert controller.get_correction(k, v) > 0.0

    self._set_curve(event, 3, {6: 0.0})
    sm.recv_frame['lateralCurvatureParameters'] += 1
    controller.update_from_submaster(sm)
    assert decode.call_count == 3
    assert controller.get_correction(k, v) == 0.0

  def test_rejects_nonfinite_fit(self):
    controller = CurvatureDController()
    event = messaging.new_message('lateralCurvatureParameters')
    fit = event.lateralCurvatureParameters
    fit.version = VERSION
    fit.useParams = True
    fit.liveValid = True
    self._set_curve(event, 3, {6: float('nan')})
    controller.update_live_params(fit)
    assert not controller.live_valid
    assert controller.get_correction(1e-4, 22.0) == 0.0
