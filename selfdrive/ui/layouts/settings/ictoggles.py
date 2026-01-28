from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.selfdrive.ui.ui_state import ui_state

if gui_app.sunnypilot_ui():
  from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp as toggle_item

# Description constants
DESCRIPTIONS = {
  "EnableCurvatureController": tr_noop(
    "Enables curvature PID post-processing additionally to QFK curvature offset"
  ),
  "EnableLongComfortMode": tr_noop(
    "Enables longitudinal jerk and accel deviation limit control for safe and comfortable driving"
  ),
  "EnableSpeedLimitControl": tr_noop(
    "Enables setting maximum speed by speed limit detection"
  ),
  "EnableSpeedLimitPredicative": tr_noop(
    "Enables setting predicative speed limit"
  ),
  "EnableSLPredReactToSL": tr_noop(
    "Enables reaction to speed limits as predicative speed limit"
  ),
  "EnableSLPredReactToCurves": tr_noop(
    "Enables reaction to curves as predicative speed limit (Max Speed as per lateral ISO limits)"
  ),
  "BatteryDetails": tr_noop(
    "Display battery detail panel"
  ),
  "ForceRHDForBSM": tr_noop(
    "Switch BSM detection side to RHD. Driver is on the right side."
  ),
  "EnableSmoothSteer": tr_noop(
    "Enables S-curving on lateral control for smoother steering"
  ),
  "DarkMode": tr_noop(
    "Force brightness to a minimal value"
  ),
  "DisableScreenTimer": tr_noop(
    "The onroad screen is turned of after 10 seconds. It will be temporarily enabled on alerts"
  ),
  "DisableCarSteerAlerts": tr_noop(
    "Disables audible steering alerts from car"
  ),
}


class ICTogglesLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      "EnableCurvatureController": (
        lambda: tr("VW: Lateral Correction (Recommended)"),
        DESCRIPTIONS["EnableCurvatureController"],
        "chffr_wheel.png",
        False,
      ),
      "EnableLongComfortMode": (
        lambda: tr("VW: Longitudinal Comfort Mode"),
        DESCRIPTIONS["EnableLongComfortMode"],
        "chffr_wheel.png",
        False,
      ),
      "EnableSpeedLimitControl": (
        lambda: tr("VW: Speed Limit Control"),
        DESCRIPTIONS["EnableSpeedLimitControl"],
        "speed_limit.png",
        False,
      ),
      "EnableSpeedLimitPredicative": (
        lambda: tr("VW: Predicative Speed Limit (pACC)"),
        DESCRIPTIONS["EnableSpeedLimitPredicative"],
        "speed_limit.png",
        False,
      ),
      "EnableSLPredReactToSL": (
        lambda: tr("VW: Predicative - Reaction to Speed Limits"),
        DESCRIPTIONS["EnableSLPredReactToSL"],
        "speed_limit.png",
        False,
      ),
      "EnableSLPredReactToCurves": (
        lambda: tr("VW: Predicative - Reaction to Curves"),
        DESCRIPTIONS["EnableSLPredReactToCurves"],
        "speed_limit.png",
        False,
      ),
      "BatteryDetails": (
        lambda: tr("VW MEB: Display Battery Details"),
        DESCRIPTIONS["BatteryDetails"],
        "capslock-fill.png",
        False,
      ),
      "ForceRHDForBSM": (
        lambda: tr("VW: Force RHD for BSM"),
        DESCRIPTIONS["ForceRHDForBSM"],
        "chffr_wheel.png",
        False,
      ),
      "EnableSmoothSteer": (
        lambda: tr("Steer Smoothing"),
        DESCRIPTIONS["EnableSmoothSteer"],
        "chffr_wheel.png",
        False,
      ),
      "DarkMode": (
        lambda: tr("Dark Mode"),
        DESCRIPTIONS["DarkMode"],
        "eye_closed.png",
        False,
      ),
      "DisableScreenTimer": (
        lambda: tr("Onroad Screen Timeout"),
        DESCRIPTIONS["DisableScreenTimer"],
        "eye_closed.png",
        False,
      ),
      "DisableCarSteerAlerts": (
        lambda: tr("VW: Disable Car Steer Alert Chime"),
        DESCRIPTIONS["DisableCarSteerAlerts"],
        "chffr_wheel.png",
        False,
      ),
    }

    self._toggles = {}
    self._locked_toggles = set()
    for param, (title, desc, icon, needs_restart) in self._toggle_defs.items():
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      # Make description callable for live translation
      additional_desc = ""
      if needs_restart and not locked:
        additional_desc = tr("Changing this setting will restart openpilot if the car is powered on.")
      toggle.set_description(lambda og_desc=toggle.description, add_desc=additional_desc: tr(og_desc) + (" " + tr(add_desc) if add_desc else ""))

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    return

  def show_event(self):
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

  def _render(self, rect):
    self._scroller.render(rect)

  def _toggle_callback(self, state: bool, param: str):
    self._params.put_bool(param, state)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True)
