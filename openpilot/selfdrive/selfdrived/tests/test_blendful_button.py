import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from openpilot.cereal import custom
from openpilot.selfdrive.selfdrived.events import ET, EventName
from openpilot.selfdrive.selfdrived.selfdrived import ButtonType, SelfdriveD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP


class TestBlendfulButton(unittest.TestCase):
  def setUp(self):
    self.selfdrive = SelfdriveD.__new__(SelfdriveD)
    self.selfdrive.CP = SimpleNamespace(openpilotLongitudinalControl=True)
    self.selfdrive.sm = SimpleNamespace(frame=0)
    self.selfdrive.distance_button_pressed_frame = None
    self.selfdrive.distance_button_long_press = False
    self.values = {"Blendful": False, "ModelRunnerTypeCache": custom.ModelManagerSP.Runner.tinygrad}
    self.selfdrive.params = Mock()
    self.selfdrive.params.get.side_effect = self.values.get
    self.selfdrive.params.get_bool.side_effect = self.values.get
    self.selfdrive.params.put_bool.side_effect = self.values.__setitem__
    self.selfdrive._change_personality = Mock()
    self.events, self.events_sp = [], []
    self.selfdrive.events = SimpleNamespace(add=self.events.append)
    self.selfdrive.events_sp = SimpleNamespace(add=self.events_sp.append)

  def tick(self, frame, pressed=None, valid=True):
    self.selfdrive.sm.frame = frame
    buttons = [] if pressed is None else [SimpleNamespace(type=ButtonType.gapAdjustCruise, pressed=pressed)]
    self.selfdrive._update_distance_button(SimpleNamespace(buttonEvents=buttons, canValid=valid))

  def test_short_press_changes_personality_only(self):
    self.tick(0, True)
    self.tick(99, False)
    self.selfdrive._change_personality.assert_called_once_with()
    self.assertEqual(self.events, [EventName.personalityChanged])
    self.assertEqual(self.events_sp, [])
    self.selfdrive.params.put_bool.assert_not_called()

  def test_long_press_toggles_once_until_released(self):
    self.tick(0, True)
    self.tick(99)
    self.assertFalse(self.values["Blendful"])
    self.tick(100)
    self.tick(200, True)  # Repeated press event must not restart the hold.
    self.tick(300)
    self.tick(301, False)
    self.selfdrive.params.put_bool.assert_called_once_with("Blendful", True)
    self.assertEqual(self.events_sp, [custom.OnroadEventSP.EventName.blendfulEnabled])
    self.selfdrive._change_personality.assert_not_called()

  def test_second_long_press_turns_off(self):
    self.tick(0, True)
    self.tick(100, False)  # Release at the threshold is still a long press.
    self.tick(101, True)
    self.tick(201, False)
    self.assertFalse(self.values["Blendful"])
    self.assertEqual(self.events_sp, [custom.OnroadEventSP.EventName.blendfulEnabled,
                                    custom.OnroadEventSP.EventName.blendfulDisabled])
    self.selfdrive._change_personality.assert_not_called()

  def test_unsupported_model_is_silent(self):
    self.values["ModelRunnerTypeCache"] = None
    self.tick(0, True)
    self.tick(100)
    self.tick(200, False)
    self.selfdrive.params.put_bool.assert_not_called()
    self.selfdrive._change_personality.assert_not_called()
    self.assertEqual(self.events_sp, [])
    self.assertEqual(self.events, [])

  def test_stock_long_still_allows_blendful(self):
    self.selfdrive.CP.openpilotLongitudinalControl = False
    self.tick(0, True)
    self.tick(20, False)
    self.selfdrive._change_personality.assert_not_called()
    self.tick(30, True)
    self.tick(130, False)
    self.assertTrue(self.values["Blendful"])

  def test_invalid_can_cancels_hold(self):
    self.tick(0, True)
    self.tick(50, valid=False)
    self.tick(150, False)
    self.selfdrive.params.put_bool.assert_not_called()
    self.selfdrive._change_personality.assert_not_called()

  def test_release_without_press_is_ignored(self):
    self.tick(0, False)
    self.selfdrive._change_personality.assert_not_called()
    self.selfdrive.params.put_bool.assert_not_called()

  def test_alerts_are_silent_status_messages(self):
    for event, text in ((custom.OnroadEventSP.EventName.blendfulEnabled, "Blendful ON"),
                        (custom.OnroadEventSP.EventName.blendfulDisabled, "Blendful OFF")):
      alert = EVENTS_SP[event][ET.PERMANENT]
      self.assertEqual(alert.alert_text_1, text)
      self.assertEqual(alert.audible_alert, 0)
