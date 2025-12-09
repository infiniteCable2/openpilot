import pyray as rl
from collections.abc import Callable
from cereal import log

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state


class ICTogglesLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    enable_curvature_correction = BigParamControl("VW: Lateral Correction (Recommended)", "EnableCurvatureController")
    enable_long_comfort_mode    = BigParamControl("VW: Longitudinal Comfort Mode", "EnableLongComfortMode")
    enable_sl_control           = BigParamControl("VW: Speed Limit Control", "EnableSpeedLimitControl")
    enable_sl_pred_control      = BigParamControl("VW: Predicative Speed Limit (pACC)", "EnableSpeedLimitPredicative")
    enable_sl_pred_sl           = BigParamControl("VW: Predicative - Reaction to Speed Limits", "EnableSLPredReactToSL")
    enable_sl_pred_curve        = BigParamControl("VW: Predicative - Reaction to Curves", "EnableSLPredReactToCurves")
    force_rhd_bsm               = BigParamControl("VW: Force RHD for BSM", "ForceRHDForBSM")
    enable_smooth_steer         = BigParamControl("Steer Smoothing", "EnableSmoothSteer")
    enable_dark_mode            = BigParamControl("Dark Mode", "DarkMode")
    enable_onroad_screen_timer  = BigParamControl("Onroad Screen Timeout", "DisableScreenTimer")
    

    self._scroller = Scroller([
      enable_curvature_correction,
      enable_long_comfort_mode,
      enable_sl_control,
      enable_sl_pred_control,
      enable_sl_pred_sl,
      enable_sl_pred_curve,
      force_rhd_bsm,
      enable_smooth_steer,
      enable_dark_mode,
      enable_onroad_screen_timer,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("EnableCurvatureController", enable_curvature_correction),
      ("EnableLongComfortMode", enable_long_comfort_mode),
      ("EnableSpeedLimitControl", enable_sl_control),
      ("EnableSpeedLimitPredicative", enable_sl_pred_control),
      ("EnableSLPredReactToSL", enable_sl_pred_sl),
      ("EnableSLPredReactToCurves", enable_sl_pred_curve),
      ("ForceRHDForBSM", force_rhd_bsm),
      ("EnableSmoothSteer", enable_smooth_steer),
      ("DarkMode", enable_dark_mode),
      ("DisableScreenTimer", enable_onroad_screen_timer),
    )

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
