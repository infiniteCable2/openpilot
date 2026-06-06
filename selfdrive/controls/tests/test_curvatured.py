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

    outer_idx = CurvatureDLookup.curvature_index(1.5e-3)
    assert outer_idx is not None
    self._set_curve(msg, 3, {outer_idx: 8.0e-5})
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    outer = controller.get_correction(1.5e-3, v_ego)

    assert outer > 0.0

  def test_outer_range_fades_to_zero_past_last_bucket_edge(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    outer_idx = len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS) - 1
    self._set_curve(msg, 3, {outer_idx: 8.0e-5})
    controller.update_live_params(msg.liveCurvatureParameters)

    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])
    last_edge = float(CurvatureDLookup.CURVATURE_BUCKET_MAX)
    fade_mid = 0.5 * (last_edge + float(CurvatureDLookup.CURVATURE_MAX))

    at_last_edge = controller.get_correction(last_edge, v_ego)
    in_fade = controller.get_correction(fade_mid, v_ego)
    at_max = controller.get_correction(float(CurvatureDLookup.CURVATURE_MAX), v_ego)

    assert at_last_edge > 0.0
    assert 0.0 < in_fade < at_last_edge
    assert at_max == 0.0

  def test_get_correction_caches_within_quantization_window(self):
    """Identical (v_ego, abs_curvature) within quantization granularity must
    hit the cache. The interp_curve_value source must not be called twice.
    """
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

    # Wrap the source to count calls
    call_count = {"n": 0}
    original = CurvatureDLookup.interp_curve_value
    def counting(*args, **kwargs):
      call_count["n"] += 1
      return original(*args, **kwargs)
    CurvatureDLookup.interp_curve_value = counting
    try:
      # First call: cache miss, calls interp_curve_value once
      first = controller.get_correction(32e-6, v_ego)
      assert call_count["n"] == 1
      # Subsequent identical calls: cache hit, no further invocations
      for _ in range(5):
        cached = controller.get_correction(32e-6, v_ego)
        assert cached == first
      assert call_count["n"] == 1

      # v_ego noise below quantization (0.01 m/s) must still hit the cache
      noised = controller.get_correction(32e-6, v_ego + 0.005)
      assert noised == first
      assert call_count["n"] == 1

      # Curvature noise below quantization (1e-7) must still hit the cache
      noised = controller.get_correction(32e-6 + 5e-8, v_ego)
      assert noised == first
      assert call_count["n"] == 1
    finally:
      CurvatureDLookup.interp_curve_value = original

  def test_get_correction_cache_invalidates_on_live_params_update(self):
    """Cache must be invalidated when fit_corrections / fit_valid change,
    otherwise stale corrections would be served after a params update.
    """
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.liveCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 3, {curvature_idx: 4e-6})
    controller.update_live_params(msg.liveCurvatureParameters)
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])

    first = controller.get_correction(32e-6, v_ego)

    # Update the underlying curve to a different value
    self._set_curve(msg, 3, {curvature_idx: 16e-6})
    controller.update_live_params(msg.liveCurvatureParameters)

    second = controller.get_correction(32e-6, v_ego)
    assert second > first
    # Specifically: must not be the cached value
    assert second != first

  def test_get_correction_cache_invalidates_on_reset(self):
    """reset() must clear the cache to avoid stale hits after disengage/engage."""
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

    # Warm the cache
    controller.get_correction(32e-6, v_ego)
    assert controller._cached_v_ego_q is not None

    controller.reset()
    # Cache state must be reset
    assert controller._cached_v_ego_q is None
    assert controller._cached_curvature_q is None
    assert controller._cached_projected == 0.0

  def test_get_correction_bypasses_cache_when_params_disabled(self):
    """When use_params is False, the early-return at the top of get_correction
    must not interfere with cache invariants (the cache may legitimately be stale).
    """
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

    # Warm the cache
    controller.get_correction(32e-6, v_ego)
    assert controller._cached_projected != 0.0

    # Disable params -> early return 0.0
    controller.use_params = False
    assert controller.get_correction(32e-6, v_ego) == 0.0
    # The cache still holds the old value but is not consulted on this path
    assert controller._cached_projected != 0.0  # cache not touched
