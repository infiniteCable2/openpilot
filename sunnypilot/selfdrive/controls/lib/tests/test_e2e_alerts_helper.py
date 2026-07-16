from collections import deque
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import (
  E2EAlertsHelper,
  E2EStates,
  LEAD_DEPART_VREL_THRESHOLD,
  LEAD_FILTER_FRAMES,
  TRIGGER_TIMER_THRESHOLD,
)


def test_lead_departure_debounce_thresholds():
  assert LEAD_FILTER_FRAMES == 10
  assert TRIGGER_TIMER_THRESHOLD == 0.8
  assert LEAD_DEPART_VREL_THRESHOLD == 0.2


def _make_helper():
  helper = E2EAlertsHelper.__new__(E2EAlertsHelper)
  helper.frame = 100
  helper.lead_depart_state = E2EStates.ARMED
  helper.lead_depart_trigger_timer = 0
  helper.last_lead_distance = -1
  helper.lead_distance_history = deque(maxlen=LEAD_FILTER_FRAMES)
  helper.lead_vrel_history = deque(maxlen=LEAD_FILTER_FRAMES)
  helper.last_moving_frame = -100
  helper.last_allowed = False
  helper.lead_depart_confirmed_lead = False
  helper.lead_depart_arm_timer = 0
  helper.lead_depart_armed = False
  helper.green_light_state = E2EStates.INACTIVE
  helper.green_light_trigger_timer = 0
  return helper


def _sm(d_rel, v_rel):
  return {
    "carState": SimpleNamespace(standstill=True, vEgo=0.0, gasPressed=False),
    "carControl": SimpleNamespace(enabled=False),
    "modelV2": SimpleNamespace(position=SimpleNamespace(x=[0.0])),
    "radarState": SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=d_rel, vRel=v_rel)),
  }


def test_lead_departure_requires_positive_relative_speed():
  helper = _make_helper()

  for _ in range(20):
    assert helper.update_alert_trigger(_sm(5.0, 0.0))[1] is False
    helper.frame += 1

  for _ in range(20):
    assert helper.update_alert_trigger(_sm(6.2, -0.1))[1] is False
    helper.frame += 1


def test_lead_departure_triggers_after_filtered_positive_motion():
  helper = _make_helper()

  for _ in range(20):
    helper.update_alert_trigger(_sm(5.0, 0.0))
    helper.frame += 1

  triggered = False
  for _ in range(40):
    triggered = triggered or helper.update_alert_trigger(_sm(6.2, 0.3))[1]
    helper.frame += 1

  assert triggered
