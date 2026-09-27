import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.sunnypilot.livedelay.helpers import LateralDelayCache
from openpilot.sunnypilot.livedelay.lagd_toggle import CACHE_PERIODIC_INTERVAL, LagdToggle


class FakeParams:
  def __init__(self, *, live_learning=False, software_delay=0.05, cached_delay=0.25):
    self.values = {
      "LagdToggle": live_learning,
      "LagdToggleDelay": software_delay,
      "LagdValueCache": cached_delay,
    }
    self.read_count = 0
    self.writes = []

  def get_bool(self, key):
    self.read_count += 1
    return self.values[key]

  def get(self, key, return_default=False):
    self.read_count += 1
    return self.values[key]

  def put(self, key, value):
    self.values[key] = value
    self.writes.append((key, value))


class TestLateralDelayCache(unittest.TestCase):
  def test_model_reads_cached_values_and_refreshes_settings(self):
    params = FakeParams()
    with patch('openpilot.sunnypilot.livedelay.helpers.drop_realtime'):
      cache = LateralDelayCache(0.2, refresh_interval=0.01, params=params)
      try:
        reads_at_start = params.read_count
        for _ in range(100):
          self.assertEqual(cache.get(0.4), 0.25)
        self.assertEqual(params.read_count, reads_at_start)

        params.values["LagdToggleDelay"] = 0.1
        deadline = time.monotonic() + 1
        while abs(cache.get(0.4) - 0.3) > 1e-9 and time.monotonic() < deadline:
          time.sleep(0.01)
        self.assertAlmostEqual(cache.get(0.4), 0.3)
        self.assertEqual(params.values["LagdValueCache"], 0.25)

        params.values["LagdToggle"] = True
        deadline = time.monotonic() + 1
        while cache.get(0.4) != 0.4 and time.monotonic() < deadline:
          time.sleep(0.01)
        self.assertEqual(cache.get(0.4), 0.4)
      finally:
        cache.close()


class TestLagdTogglePersistence(unittest.TestCase):
  def test_fixed_delay_is_saved_only_when_changed(self):
    params = FakeParams()
    with patch('openpilot.sunnypilot.livedelay.lagd_toggle.Params', return_value=params):
      toggle = LagdToggle(SimpleNamespace(steerActuatorDelay=0.2))
    message = SimpleNamespace(lateralDelay=SimpleNamespace(lateralDelay=0.4))
    for _ in range(5):
      toggle.update(message)
    self.assertEqual(params.writes, [])

    params.values["LagdToggleDelay"] = 0.0505
    toggle.update(message)
    self.assertEqual(len(params.writes), 1)
    self.assertEqual(params.writes[0][0], "LagdValueCache")
    self.assertAlmostEqual(params.writes[0][1], 0.2505)

  def test_small_learned_changes_are_saved_periodically(self):
    params = FakeParams(live_learning=True, cached_delay=0.2)
    with patch('openpilot.sunnypilot.livedelay.lagd_toggle.Params', return_value=params):
      toggle = LagdToggle(SimpleNamespace(steerActuatorDelay=0.2))
    message = SimpleNamespace(lateralDelay=SimpleNamespace(lateralDelay=0.2))
    toggle.update(message)
    message.lateralDelay.lateralDelay = 0.203
    toggle.update(message)
    self.assertEqual(params.writes, [])

    toggle._last_cache_write -= CACHE_PERIODIC_INTERVAL
    toggle.update(message)
    self.assertEqual(params.writes, [("LagdValueCache", 0.203)])

    toggle._last_cache_write -= CACHE_PERIODIC_INTERVAL
    toggle.update(message)
    self.assertEqual(params.writes[-1], ("LagdValueCache", 0.203))
    self.assertEqual(len(params.writes), 2)

    message.lateralDelay.lateralDelay = 0.21
    toggle.update(message)
    self.assertEqual(params.writes[-1], ("LagdValueCache", 0.21))
