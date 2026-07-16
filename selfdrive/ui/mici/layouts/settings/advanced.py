from openpilot.selfdrive.ui.mici.widgets.button import BigMultiParamToggle, BigParamControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class SpeedLimitDisplayControl(BigMultiParamToggle):
  """Speed-limit modes that remain useful without openpilot longitudinal control."""

  def _load_value(self):
    value = int(self._params.get(self._param) or 0)
    self.set_value(self._options[min(max(value, 0), len(self._options) - 1)])


class AdvancedSettingsLayoutMici(NavScroller):
  """Small set of working C4/VW controls for the mici settings UI."""

  def __init__(self):
    super().__init__()

    self._blind_spot = BigParamControl(tr("Show Blind Spot Warnings"), "BlindSpot")
    self._bsm_side = BigMultiParamToggle(
      tr("VW: Blind Spot Driving Side"),
      "ForceRHDForBSM",
      [tr("Left-Hand Drive"), tr("Right-Hand Drive")],
    )
    self._disable_steer_chime = BigParamControl(
      tr("VW: Disable Car Steer Alert Chime"),
      "DisableCarSteerAlerts",
    )
    self._speed_limit_mode = SpeedLimitDisplayControl(
      tr("Speed Limit Display & Warning"),
      "SpeedLimitMode",
      [tr("Off"), tr("Information"), tr("Warning")],
    )
    self._accel_bar = BigParamControl(tr("Real-time Acceleration Bar"), "RocketFuel")

    self._scroller.add_widgets([
      self._blind_spot,
      self._bsm_side,
      self._disable_steer_chime,
      self._speed_limit_mode,
      self._accel_bar,
    ])

    self._refresh_controls = (
      ("BlindSpot", self._blind_spot),
      ("DisableCarSteerAlerts", self._disable_steer_chime),
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
    self._bsm_side._load_value()
    self._speed_limit_mode._load_value()

    is_vw = ui_state.CP is not None and ui_state.CP.brand == "volkswagen"
    self._blind_spot.set_enabled(ui_state.CP is not None and ui_state.CP.enableBsm)
    self._bsm_side.set_enabled(is_vw and ui_state.is_offroad())
    self._disable_steer_chime.set_enabled(is_vw and ui_state.is_offroad())
