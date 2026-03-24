import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController, CurvatureDLookup, VERSION


class TestCurvatureDController:
  @staticmethod
  def _set_fit(msg, sign_idx: int, speed_idx: int, amplitude: float, scale: float):
    fit_amplitudes = list(msg.liveCurvatureParameters.fitAmplitudes)
    fit_scales = list(msg.liveCurvatureParameters.fitScales)
    fit_valid = list(msg.liveCurvatureParameters.fitValid)
    if len(fit_amplitudes) != CurvatureDLookup.fit_total_size():
      fit_amplitudes = [0.0] * CurvatureDLookup.fit_total_size()
    if len(fit_scales) != CurvatureDLookup.fit_total_size():
      fit_scales = [CurvatureDLookup.CURVATURE_BUCKET_EDGES[5]] * CurvatureDLookup.fit_total_size()
    if len(fit_valid) != CurvatureDLookup.fit_total_size():
      fit_valid = [False] * CurvatureDLookup.fit_total_size()
    flat_idx = sign_idx * len(CurvatureDLookup.SPEED_ANCHORS) + speed_idx
    fit_amplitudes[flat_idx] = amplitude
    fit_scales[flat_idx] = scale
    fit_valid[flat_idx] = True
    msg.liveCurvatureParameters.fitAmplitudes = fit_amplitudes
    msg.liveCurvatureParameters.fitScales = fit_scales
    msg.liveCurvatureParameters.fitValid = fit_valid

  def test_apply_interpolates_between_neighbor_speed_fits(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    sign_idx = CurvatureDLookup.sign_index(32e-6)
    self._set_fit(msg, sign_idx, 2, 0.15, 5e-5)
    self._set_fit(msg, sign_idx, 3, 0.45, 5e-5)
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

  def test_invalid_message_disables_corrections(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = False
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.fitAmplitudes = [0.0] * CurvatureDLookup.fit_total_size()
    msg.liveCurvatureParameters.fitScales = [CurvatureDLookup.CURVATURE_BUCKET_EDGES[5]] * CurvatureDLookup.fit_total_size()
    msg.liveCurvatureParameters.fitValid = [False] * CurvatureDLookup.fit_total_size()

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 20.0) == 32e-6

  def test_apply_fades_near_center_limits(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    sign_idx = CurvatureDLookup.sign_index(32e-6)
    self._set_fit(msg, sign_idx, 3, 0.35, 5e-5)
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    mid = controller.get_correction(32e-6, v_ego)
    low = controller.get_correction(1.5e-6, v_ego)
    high = controller.get_correction(8e-4, v_ego)

    assert mid > 0.0
    assert 0.0 < low < mid
    assert 0.0 < high < mid
    assert controller.get_correction(CurvatureDLookup.CURVATURE_MIN_STEP, v_ego) == 0.0
    assert controller.get_correction(CurvatureDLookup.CURVATURE_MAX, v_ego) == 0.0
