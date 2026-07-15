from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import (
  E2EAlertsHelper,
  E2EStates,
  LEAD_DEPART_TRIGGER_DELAY,
)


class FakeSubMaster(dict):
  pass


def build_helper() -> E2EAlertsHelper:
  helper = E2EAlertsHelper.__new__(E2EAlertsHelper)
  helper.frame = 0
  helper.green_light_state = E2EStates.ARMED
  helper.lead_depart_state = E2EStates.ARMED
  helper.green_light_trigger_timer = 0
  helper.lead_depart_trigger_timer = 0
  helper.last_lead_distance = 5.0
  helper.last_moving_frame = -1000
  helper.allowed = True
  helper.last_allowed = True
  helper.has_lead = True
  helper.lead_depart_arm_timer = 0
  helper.lead_depart_confirmed_lead = True
  helper.lead_depart_armed = True
  return helper


def build_sm(*, lead_distance: float, model_distance: float = 0.0) -> FakeSubMaster:
  return FakeSubMaster({
    'carState': SimpleNamespace(standstill=True, vEgo=0.0, gasPressed=False),
    'carControl': SimpleNamespace(enabled=False),
    'modelV2': SimpleNamespace(position=SimpleNamespace(x=[0.0, model_distance])),
    'radarState': SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=lead_distance)),
  })


def test_lead_departure_uses_explicit_delay():
  helper = build_helper()
  sm = build_sm(lead_distance=6.5)
  delay_frames = round(LEAD_DEPART_TRIGGER_DELAY / DT_MDL)

  for frame in range(delay_frames - 1):
    helper.frame = frame
    _, lead_trigger = helper.update_alert_trigger(sm)
    assert not lead_trigger

  helper.frame = delay_frames - 1
  _, lead_trigger = helper.update_alert_trigger(sm)
  assert lead_trigger


def test_lead_departure_suppresses_simultaneous_green_light():
  helper = build_helper()
  helper.green_light_trigger_timer = round(LEAD_DEPART_TRIGGER_DELAY / DT_MDL)
  helper.lead_depart_trigger_timer = round(LEAD_DEPART_TRIGGER_DELAY / DT_MDL) - 1
  sm = build_sm(lead_distance=6.5, model_distance=40.0)

  green_trigger, lead_trigger = helper.update_alert_trigger(sm)

  assert lead_trigger
  assert not green_trigger
