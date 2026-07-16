from openpilot.selfdrive.ui.mici.widgets.button import BigMultiParamToggle, BigParamControl, BigToggle
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import MousePos
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class AutoDarkModeControl(BigToggle):
  """Expose only the safe Auto and Auto Dark brightness modes."""

  def __init__(self):
    super().__init__(tr("Dark Mode"))
    self.refresh()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    ui_state.params.put("OnroadScreenOffBrightness", 1 if self._checked else 0, block=True)

  def refresh(self):
    self.set_checked(ui_state.params.get("OnroadScreenOffBrightness", return_default=True) == 1)


class AdvancedSettingsLayoutMici(NavScroller):
  """Current SP/C4 equivalents of the former C4 advanced settings."""

  SPEED_LIMIT_OPTIONS = [tr("Off"), tr("Info"), tr("Warning"), tr("Assist")]

  def __init__(self):
    super().__init__()
    self._speed_limit_assist_available = False

    vw_curvature = BigDialogButton(
      tr("VW: Lateral Correction (Recommended)"),
      tr("Automatic"),
      description=tr(
        "VW curvature steering is enabled automatically for supported Caddy and MQB EVO vehicles. "
        "The frozen-opendbc compatibility layer remains active; no separate toggle is required."
      ),
    )

    self._icbm = BigParamControl(
      tr("Intelligent Cruise Button Management (ICBM) (Alpha)"),
      "IntelligentCruiseButtonManagement",
      toggle_callback=self._set_predictive_controls_enabled,
    )
    self._speed_limit_mode = BigMultiParamToggle(
      tr("VW: Speed Limit Control"),
      "SpeedLimitMode",
      self.SPEED_LIMIT_OPTIONS,
      select_callback=self._on_speed_limit_mode_selected,
    )
    self._smart_cruise_vision = BigParamControl(
      tr("VW: Predicative - Reaction to Curves"),
      "SmartCruiseControlVision",
    )
    self._smart_cruise_map = BigParamControl(
      tr("Smart Cruise Control - Map"),
      "SmartCruiseControlMap",
    )
    self._blind_spot = BigParamControl(tr("Show Blind Spot Warnings"), "BlindSpot")
    self._dark_mode = AutoDarkModeControl()
    self._accel_bar = BigParamControl(tr("Enable Accel Bar"), "RocketFuel")

    self._scroller.add_widgets([
      vw_curvature,
      self._icbm,
      self._speed_limit_mode,
      self._smart_cruise_vision,
      self._smart_cruise_map,
      self._blind_spot,
      self._dark_mode,
      self._accel_bar,
    ])

    self._refresh_controls = (
      ("IntelligentCruiseButtonManagement", self._icbm),
      ("SmartCruiseControlVision", self._smart_cruise_vision),
      ("SmartCruiseControlMap", self._smart_cruise_map),
      ("BlindSpot", self._blind_spot),
      ("RocketFuel", self._accel_bar),
    )

    ui_state.add_offroad_transition_callback(self._update_controls)

  def show_event(self):
    super().show_event()
    self._update_controls()

  def _update_controls(self):
    ui_state.update_params()

    for key, control in self._refresh_controls:
      control.set_checked(ui_state.params.get_bool(key))
    self._speed_limit_mode._load_value()
    self._dark_mode.refresh()

    has_long = ui_state.has_longitudinal_control
    icbm_available = (ui_state.CP_SP is not None and
                      ui_state.CP_SP.intelligentCruiseButtonManagementAvailable and not has_long)
    self._icbm.set_enabled(ui_state.is_offroad() and icbm_available)

    predictive_available = has_long or ui_state.has_icbm
    self._set_predictive_controls_enabled(predictive_available)
    self._blind_spot.set_enabled(ui_state.CP is not None and ui_state.CP.enableBsm)

    if not predictive_available and self._speed_limit_mode.get_value() == tr("Assist"):
      self._speed_limit_mode.set_value(tr("Warning"))
      ui_state.params.put("SpeedLimitMode", 2, block=True)

  def _set_predictive_controls_enabled(self, enabled: bool):
    self._speed_limit_assist_available = enabled
    self._smart_cruise_vision.set_enabled(enabled)
    self._smart_cruise_map.set_enabled(enabled)

  def _on_speed_limit_mode_selected(self, value: str):
    if value == tr("Assist") and not self._speed_limit_assist_available:
      self._speed_limit_mode.set_value(tr("Warning"))
