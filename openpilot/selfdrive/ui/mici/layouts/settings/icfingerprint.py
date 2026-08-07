import json
import os

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigInputDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller

CAR_LIST_JSON_OUT = os.path.join(BASEDIR, "openpilot", "sunnypilot", "selfdrive", "car", "car_list.json")


def _load_car_list() -> dict:
  with open(CAR_LIST_JSON_OUT) as f:
    return json.load(f)


def _get_current_platform_name() -> str:
  bundle = ui_state.params.get("CarPlatformBundle")
  if bundle:
    return bundle.get("name", "")
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    return ui_state.CP.carFingerprint
  return ""


def _is_manual() -> bool:
  return ui_state.params.get("CarPlatformBundle") is not None


def _reset_to_auto():
  ui_state.params.remove("CarPlatformBundle")


def _set_platform(platform_name: str, car_list: dict):
  data = car_list.get(platform_name)
  if data:
    ui_state.params.put("CarPlatformBundle", {**data, "name": platform_name})


class ManualSelectPage(NavScroller):
  def __init__(self, car_list: dict, on_selected: callable):
    super().__init__()
    self._car_list = car_list
    self._on_selected = on_selected
    self._search_query = ""
    self._search_btn = BigButton(tr("search"))
    self._search_btn.set_click_callback(self._on_search_clicked)
    self._scroller.add_widget(self._search_btn)
    self._rebuild_platform_buttons()

  def _on_search_clicked(self):
    dlg = BigInputDialog(
      tr("search vehicle"),
      self._search_query,
      minimum_length=0,
      confirm_callback=self._on_search_confirm,
    )
    gui_app.push_widget(dlg)

  def _on_search_confirm(self, text: str):
    self._search_query = text.strip()
    self._search_btn.set_value(self._search_query if self._search_query else "")
    self._rebuild_platform_buttons()

  def _rebuild_platform_buttons(self):
    query = self._search_query.lower()
    names = sorted(self._car_list.keys())
    if query:
      names = [n for n in names if query in n.lower()]

    existing = [w for w in self._scroller._items if w is not self._search_btn]
    for w in existing:
      self._scroller._items.remove(w)

    for name in names:
      btn = BigButton(name)
      btn.set_click_callback(lambda n=name: self._select_platform(n))
      self._scroller.add_widget(btn)

  def _select_platform(self, platform_name: str):
    _set_platform(platform_name, self._car_list)
    self._on_selected()


class FingerprintLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._car_list = _load_car_list()

    self._platform_info = BigButton(tr("current fingerprint"))
    self._platform_info.set_value(_get_current_platform_name() or tr("unrecognized vehicle"))
    self._platform_info.set_enabled(False)

    self._select_btn = BigButton(tr("select"))
    self._select_btn.set_click_callback(self._show_manual_select)

    self._reset_btn = BigButton(tr("reset to auto"))
    self._reset_btn.set_click_callback(self._confirm_reset)

    self._scroller.add_widgets([
      self._platform_info,
      self._select_btn,
      self._reset_btn,
    ])

  def _show_manual_select(self):
    page = ManualSelectPage(self._car_list, self._on_platform_selected)
    gui_app.push_widget(page)

  def _on_platform_selected(self):
    gui_app.pop_widgets_to(self)
    self._refresh_state()

  def _confirm_reset(self):
    icon = gui_app.texture("icons_mici/settings/device/update.png", 64, 64)
    dlg = BigConfirmationDialog(tr("reset to auto"), icon, confirm_callback=self._do_reset)
    gui_app.push_widget(dlg)

  def _do_reset(self):
    _reset_to_auto()
    self._refresh_state()

  def _refresh_state(self):
    manual = _is_manual()
    self._platform_info.set_value(_get_current_platform_name() or tr("unrecognized vehicle"))
    self._reset_btn.set_enabled(manual)

  def show_event(self):
    super().show_event()
    self._refresh_state()

  def _update_state(self):
    super()._update_state()
    self._refresh_state()
