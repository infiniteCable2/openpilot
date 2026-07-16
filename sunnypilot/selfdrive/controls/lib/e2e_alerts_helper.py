"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import deque
from statistics import median

from cereal import messaging, custom

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

GREEN_LIGHT_X_THRESHOLD = 30
LEAD_DEPART_DIST_THRESHOLD = 1.0
TRIGGER_TIMER_THRESHOLD = 0.8
LEAD_DEPART_VREL_THRESHOLD = 0.2
LEAD_FILTER_FRAMES = 10  # 0.5 s at DT_MDL


class E2EStates:
  INACTIVE = 0
  ARMED = 1
  CONSUMED = 2


class E2EAlertsHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.green_light_state = E2EStates.INACTIVE
    self.prev_green_light_state = E2EStates.INACTIVE
    self.lead_depart_state = E2EStates.INACTIVE
    self.prev_lead_depart_state = E2EStates.INACTIVE

    self.green_light_alert = False
    self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
    self.lead_depart_alert = False
    self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

    self.green_light_trigger_timer = 0
    self.lead_depart_trigger_timer = 0
    self.last_lead_distance = -1
    self.lead_distance_history = deque(maxlen=LEAD_FILTER_FRAMES)
    self.lead_vrel_history = deque(maxlen=LEAD_FILTER_FRAMES)
    self.last_moving_frame = -1

    self.allowed = False
    self.last_allowed = False
    self.has_lead = False

    self.lead_depart_arm_timer = 0
    self.lead_depart_confirmed_lead = False
    self.lead_depart_armed = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
      self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

  def update_alert_trigger(self, sm: messaging.SubMaster):
    CS = sm['carState']
    CC = sm['carControl']

    model_x = sm['modelV2'].position.x
    max_idx = len(model_x) - 1
    self.has_lead = sm['radarState'].leadOne.status
    lead_dRel = sm['radarState'].leadOne.dRel
    lead_vRel = sm['radarState'].leadOne.vRel

    standstill = CS.standstill
    moving = not standstill and CS.vEgo > 0.1

    if moving:
      self.last_moving_frame = self.frame
    recent_moving = self.last_moving_frame == -1 or (self.frame - self.last_moving_frame) * DT_MDL < 2.0

    self.allowed = not moving and not CS.gasPressed and not CC.enabled and not recent_moving

    # Green Light Alert
    green_light_trigger = False
    if self.green_light_state == E2EStates.ARMED:
      if model_x[max_idx] > GREEN_LIGHT_X_THRESHOLD:
        self.green_light_trigger_timer += 1
      else:
        self.green_light_trigger_timer = 0

      if self.green_light_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        green_light_trigger = True
    elif self.green_light_state != E2EStates.ARMED:
      self.green_light_trigger_timer = 0

    # Lead Departure Alert
    close_lead_valid = self.has_lead and lead_dRel < 8.0
    if close_lead_valid:
      self.lead_distance_history.append(lead_dRel)
      self.lead_vrel_history.append(lead_vRel)
    else:
      self.lead_distance_history.clear()
      self.lead_vrel_history.clear()

    filtered_dRel = median(self.lead_distance_history) if self.lead_distance_history else -1
    filtered_vRel = median(self.lead_vrel_history) if self.lead_vrel_history else 0
    if self.allowed and not self.last_allowed and close_lead_valid:
      self.lead_depart_confirmed_lead = True
    elif not self.allowed:
      self.lead_depart_confirmed_lead = False

    if self.allowed and self.lead_depart_confirmed_lead and close_lead_valid:
      self.lead_depart_arm_timer += 1

      if self.lead_depart_arm_timer * DT_MDL >= 1.0:
        self.lead_depart_armed = True
    else:
      self.lead_depart_arm_timer = 0
      self.lead_depart_armed = False

    lead_depart_trigger = False
    if self.lead_depart_state == E2EStates.ARMED:
      if self.last_lead_distance == -1 or filtered_dRel < self.last_lead_distance:
        self.last_lead_distance = filtered_dRel

      if (self.last_lead_distance != -1 and filtered_vRel > LEAD_DEPART_VREL_THRESHOLD and
          filtered_dRel - self.last_lead_distance > LEAD_DEPART_DIST_THRESHOLD):
        self.lead_depart_trigger_timer += 1
      else:
        self.lead_depart_trigger_timer = 0

      if self.lead_depart_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        lead_depart_trigger = True
    elif self.lead_depart_state != E2EStates.ARMED:
      self.last_lead_distance = -1
      self.lead_depart_trigger_timer = 0
      self.lead_distance_history.clear()
      self.lead_vrel_history.clear()

    self.last_allowed = self.allowed

    return green_light_trigger, lead_depart_trigger

  @staticmethod
  def update_state_machine(state: int, enabled: bool, allowed: bool, triggered: bool) -> tuple[int, bool]:
    if state != E2EStates.INACTIVE:
      if not allowed or not enabled:
        state = E2EStates.INACTIVE

      else:
        if state == E2EStates.ARMED:
          if triggered:
            state = E2EStates.CONSUMED

        elif state == E2EStates.CONSUMED:
          pass

    elif state == E2EStates.INACTIVE:
      if allowed and enabled:
        state = E2EStates.ARMED

    return state, triggered

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()

    green_light_trigger, lead_depart_trigger = self.update_alert_trigger(sm)

    self.prev_green_light_state = self.green_light_state
    self.prev_lead_depart_state = self.lead_depart_state

    self.green_light_state, self.green_light_alert = self.update_state_machine(
      self.green_light_state,
      self.green_light_alert_enabled,
      self.allowed and not self.has_lead,
      green_light_trigger
    )

    self.lead_depart_state, self.lead_depart_alert = self.update_state_machine(
      self.lead_depart_state,
      self.lead_depart_alert_enabled,
      self.allowed and self.lead_depart_armed,
      lead_depart_trigger
    )

    if self.green_light_alert or self.lead_depart_alert:
      events_sp.add(custom.OnroadEventSP.EventName.e2eChime)

    self.frame += 1
