import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


class SlowParams:
  def __init__(self):
    self.value = 0
    self.write_started = threading.Event()
    self.release_write = threading.Event()
    self.writes = []

  def get_bool(self, _key):
    return False

  def get(self, key, return_default=False):
    assert key == 'LongitudinalPersonality'
    return self.value

  def put(self, key, value, block=False):
    assert key == 'LongitudinalPersonality' and block
    self.write_started.set()
    assert self.release_write.wait(timeout=3)
    self.value = value
    self.writes.append(value)


class TestPersonalityPersistence(unittest.TestCase):
  def test_button_survives_slow_param_write(self):
    params = SlowParams()
    selfdrive = SelfdriveD.__new__(SelfdriveD)
    selfdrive.params = params
    selfdrive.CP = SimpleNamespace(openpilotLongitudinalControl=True)
    selfdrive.mads = SimpleNamespace(read_params=lambda: None)
    selfdrive.personality = 0
    selfdrive.personality_lock = threading.Lock()
    selfdrive.personality_write_event = threading.Event()
    selfdrive.personality_write_pending = None
    selfdrive.personality_generation = 0

    stop = threading.Event()
    reader = threading.Thread(target=selfdrive.params_thread, args=(stop,))
    writer = threading.Thread(target=selfdrive.personality_write_thread, args=(stop,))
    with patch('openpilot.selfdrive.selfdrived.selfdrived.cloudlog.event'):
      reader.start()
      writer.start()
      try:
        self.assertEqual(selfdrive._change_personality(), (0, 2))
        self.assertTrue(params.write_started.wait(timeout=1))
        self.assertEqual(selfdrive._change_personality(), (2, 1))
        time.sleep(0.25)  # Reader observes the old on-disk value during the blocked write.
        self.assertEqual(params.value, 0)
        self.assertEqual(selfdrive.personality, 1)

        params.release_write.set()
        deadline = time.monotonic() + 2
        while selfdrive.personality_write_pending is not None and time.monotonic() < deadline:
          time.sleep(0.01)
        self.assertIsNone(selfdrive.personality_write_pending)
        self.assertEqual(selfdrive.personality, 1)
        self.assertEqual(params.value, 1)
        self.assertEqual(params.writes, [2, 1])
      finally:
        params.release_write.set()
        stop.set()
        selfdrive.personality_write_event.set()
        reader.join(timeout=2)
        writer.join(timeout=2)
        self.assertFalse(reader.is_alive() or writer.is_alive())
