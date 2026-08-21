import math
import numpy as np

from opendbc.car.structs import car, CarStateIC
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 20
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 20
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
PREDICTIVE_TYPE_SPEED_LIMIT = 1
PREDICTIVE_TYPE_CURVE = 2
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    self.v_speed_limit_kph = 0
    self.curve_speed_cap_active = False
    self.curve_speed_cap_baseline_kph = V_CRUISE_UNSET
    self.curve_speed_cap_kph = V_CRUISE_UNSET

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, CS_IC: CarStateIC, enabled, is_metric, speed_limit_control=False, speed_limit_predicative=False):
    self.v_cruise_kph_last = self.v_cruise_kph

    self.get_minimum_set_speed(is_metric)

    _enabled = self.update_enabled_state(CS, enabled)

    if CS.cruiseState.available:
      if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_speed_limit(CS, CS_IC, _enabled, speed_limit_control, speed_limit_predicative)
        self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
        self.update_speed_limit_assist_v_cruise_non_pcm()
        self._apply_curve_speed_cap()
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self._clear_curve_speed_cap()
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self._clear_curve_speed_cap()
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

    if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
      self.update_button_timers(CS, enabled)

  def _update_v_speed_limit(self, CS, CS_IC: CarStateIC, enabled, speed_limit_control, predicative):
    if not speed_limit_control: # or not enabled # always set speed limit
      self._clear_curve_speed_cap()
      return

    speed_limit_current = CS_IC.cruiseSpeedLimit * CV.MS_TO_KPH
    speed_limit_predicative = CS_IC.cruiseSpeedLimitPredicative * CV.MS_TO_KPH
    predicative_type = CS_IC.cruiseSpeedLimitPredicativeType
    # Only an explicitly typed speed-limit event may use setpoint semantics.
    # Unknown/mismatched interface versions fail safe as a cap.
    curve_cap = (predicative and speed_limit_predicative != 0 and
                 predicative_type != PREDICTIVE_TYPE_SPEED_LIMIT)

    if curve_cap:
      previous_cap = self.curve_speed_cap_kph
      if not self.curve_speed_cap_active:
        self.curve_speed_cap_baseline_kph = self.v_cruise_kph
      cap_candidates = [speed_limit_predicative]
      if speed_limit_current != 0:
        cap_candidates.append(speed_limit_current)
      if self.curve_speed_cap_baseline_kph != V_CRUISE_UNSET:
        cap_candidates.append(self.curve_speed_cap_baseline_kph)
      self.curve_speed_cap_kph = np.clip(round(min(cap_candidates), 1), V_CRUISE_MIN, V_CRUISE_MAX)
      if (not self.curve_speed_cap_active or previous_cap == V_CRUISE_UNSET or
          math.isclose(self.v_cruise_kph, previous_cap, abs_tol=0.1)):
        # Follow an opening curve corridor only when cruise still equals the
        # previously applied cap. A lower driver/SLA choice becomes the new
        # ceiling and is never raised by the curve controller.
        self.v_cruise_kph = self.curve_speed_cap_kph
      else:
        self.v_cruise_kph = min(self.v_cruise_kph, self.curve_speed_cap_kph)
      self.v_speed_limit_kph = speed_limit_predicative
      self.curve_speed_cap_active = True
      return

    curve_baseline = self.curve_speed_cap_baseline_kph
    leaving_curve_cap = self.curve_speed_cap_active
    self._clear_curve_speed_cap()

    speed_limit = speed_limit_predicative if predicative and speed_limit_predicative != 0 else speed_limit_current
    if (leaving_curve_cap and speed_limit_predicative == 0 and curve_baseline != V_CRUISE_UNSET and
        speed_limit != 0):
      # Restore at most the pre-curve setpoint. A curve is a temporary cap and
      # must never raise cruise to a higher current legal limit on release.
      speed_limit = min(speed_limit, curve_baseline)

    if leaving_curve_cap or speed_limit != self.v_speed_limit_kph:
      if speed_limit != 0:
        self.v_cruise_kph = speed_limit
        self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)
      self.v_speed_limit_kph = speed_limit

  def _apply_curve_speed_cap(self):
    if not self.curve_speed_cap_active or self.curve_speed_cap_kph == V_CRUISE_UNSET:
      return
    if self.v_cruise_kph < self.curve_speed_cap_kph:
      self.curve_speed_cap_baseline_kph = min(self.curve_speed_cap_baseline_kph, self.v_cruise_kph)
      self.curve_speed_cap_kph = min(self.curve_speed_cap_kph, self.curve_speed_cap_baseline_kph)
    self.v_cruise_kph = min(self.v_cruise_kph, self.curve_speed_cap_kph)

  def _clear_curve_speed_cap(self):
    self.curve_speed_cap_active = False
    self.curve_speed_cap_baseline_kph = V_CRUISE_UNSET
    self.curve_speed_cap_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # Speed Limit Assist for Non PCM long cars.
    # True: Disallow set speed changes when user confirmed the target set speed during preActive state
    # False: Allow set speed changes as SLA is not requesting user confirmation
    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial_experimental_mode = experimental_mode and not dynamic_experimental_control
    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if initial_experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
