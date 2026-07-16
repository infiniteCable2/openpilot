from openpilot.selfdrive.ui.mici.widgets.button import BigMultiParamToggle, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class AdvancedSettingsLayoutMici(NavScroller):
  """Small-screen controls backed by settings that remain active in current SP/C4."""

  def __init__(self):
    super().__init__()

    vw_status = BigDialogButton(
      tr("VW / Caddy Support"),
      tr("Active"),
      description=tr(
        "Caddy Mk5 and MQB EVO support remains active. Curvature steering and the VW-specific "
        "MADS adapter are selected automatically from the detected vehicle; they are not separate toggles."
      ),
    )

    self._mads = BigParamControl(
      tr("Modular Assistive Driving System (MADS)"),
      "Mads",
      toggle_callback=self._set_mads_controls_enabled,
    )
    self._mads_main = BigParamControl(tr("Toggle with Main Cruise"), "MadsMainCruiseAllowed")
    self._mads_uem = BigParamControl(tr("Unified Engagement Mode (UEM)"), "MadsUnifiedEngagementMode")
    self._mads_steering_mode = BigMultiParamToggle(
      tr("Steering Mode on Brake Pedal"),
      "MadsSteeringMode",
      [tr("Remain Active"), tr("Pause"), tr("Disengage")],
    )
    self._blind_spot = BigParamControl(tr("Show Blind Spot Warnings"), "BlindSpot")

    self._scroller.add_widgets([
      vw_status,
      self._mads,
      self._mads_main,
      self._mads_uem,
      self._mads_steering_mode,
      self._blind_spot,
    ])

    self._refresh_controls = (
      ("Mads", self._mads),
      ("MadsMainCruiseAllowed", self._mads_main),
      ("MadsUnifiedEngagementMode", self._mads_uem),
      ("BlindSpot", self._blind_spot),
    )

    ui_state.add_offroad_transition_callback(self._update_controls)

  def show_event(self):
    super().show_event()
    self._update_controls()

  def _update_controls(self):
    ui_state.update_params()

    for key, control in self._refresh_controls:
      control.set_checked(ui_state.params.get_bool(key))
    self._mads_steering_mode._load_value()

    offroad = ui_state.is_offroad()
    mads_enabled = ui_state.params.get_bool("Mads")
    self._mads.set_enabled(offroad)
    self._set_mads_controls_enabled(mads_enabled)

  def _set_mads_controls_enabled(self, enabled: bool):
    enabled &= ui_state.is_offroad()
    self._mads_main.set_enabled(enabled)
    self._mads_uem.set_enabled(enabled)
    self._mads_steering_mode.set_enabled(enabled)
