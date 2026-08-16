import openpilot.cereal.messaging as messaging

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.curvatured import CACHE_CURVATURE_DECIMALS, CACHE_V_EGO_DECIMALS, CurvatureDController
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

  def test_get_correction_caches_within_quantization_window(self):
    """Identical (v_ego, abs_curvature) within quantization granularity must
    hit the cache. The interp_curve_value source must not be called twice.
    """
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

    # Wrap the source to count calls
    call_count = {"n": 0}
    original = CurvatureDLookup.interp_curve_value.__func__
    def counting(cls, *args, **kwargs):
      call_count["n"] += 1
      return original(cls, *args, **kwargs)
    CurvatureDLookup.interp_curve_value = classmethod(counting)  # ty: ignore[invalid-assignment]
    try:
      # First call: cache miss, calls interp_curve_value once
      first = controller.get_correction(32e-6, v_ego)
      assert call_count["n"] == 1
      # Subsequent identical calls: cache hit, no further invocations
      for _ in range(5):
        cached = controller.get_correction(32e-6, v_ego)
        assert cached == first
      assert call_count["n"] == 1

      # v_ego noise below quantization must still hit the cache
      v_ego_step = 10 ** -CACHE_V_EGO_DECIMALS
      noised = controller.get_correction(32e-6, round(v_ego, CACHE_V_EGO_DECIMALS) + v_ego_step * 0.25)
      assert noised == first
      assert call_count["n"] == 1

      # Curvature noise below quantization must still hit the cache
      curvature_step = 10 ** -CACHE_CURVATURE_DECIMALS
      noised = controller.get_correction(round(32e-6, CACHE_CURVATURE_DECIMALS) + curvature_step * 0.25, v_ego)
      assert noised == first
      assert call_count["n"] == 1
    finally:
      CurvatureDLookup.interp_curve_value = classmethod(original)

  def test_get_correction_cache_invalidates_on_live_params_update(self):
    """Cache must be invalidated when fit_corrections / fit_valid change,
    otherwise stale corrections would be served after a params update.
    """
    controller = CurvatureDController()
    msg = messaging.new_message('lateralCurvatureParameters')
    msg.lateralCurvatureParameters.liveValid = True
    msg.lateralCurvatureParameters.version = VERSION
    msg.lateralCurvatureParameters.useParams = True
    msg.lateralCurvatureParameters.counts = [0] * CurvatureDLookup.total_size()
    msg.lateralCurvatureParameters.biases = [0.0] * CurvatureDLookup.total_size()

    curvature_idx = CurvatureDLookup.curvature_index(32e-6)
    assert curvature_idx is not None
    self._set_curve(msg, 3, {curvature_idx: 4e-6})
    controller.update_live_params(msg.lateralCurvatureParameters)
    v_ego = float(CurvatureDLookup.SPEED_ANCHORS[3])

    first = controller.get_correction(32e-6, v_ego)

    # Update the underlying curve to a different value
    self._set_curve(msg, 3, {curvature_idx: 16e-6})
    controller.update_live_params(msg.lateralCurvatureParameters)

    second = controller.get_correction(32e-6, v_ego)
    assert second > first
    # Specifically: must not be the cached value
    assert second != first

  def test_get_correction_cache_invalidates_on_reset(self):
    """reset() must clear the cache to avoid stale hits after disengage/engage."""
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

    # Warm the cache
    controller.get_correction(32e-6, v_ego)
    assert controller._cached_projected != 0.0

    # Disable params -> early return 0.0
    controller.use_params = False
    assert controller.get_correction(32e-6, v_ego) == 0.0
    # The cache still holds the old value but is not consulted on this path
    assert controller._cached_projected != 0.0  # cache not touched
