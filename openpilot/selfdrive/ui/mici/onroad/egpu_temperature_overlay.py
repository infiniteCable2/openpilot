import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.dynamic_steering_learner_graph import CONFIG as STEERING_GRAPH_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class EgpuTemperatureOverlay(Widget):
  PANEL_WIDTH = 145
  PANEL_HEIGHT = 86
  GAP_TO_STEERING_GRAPH = 8
  HORIZONTAL_PADDING = 12
  BACKGROUND_ALPHA = 108

  TITLE_FONT_SIZE = 20
  ROW_FONT_SIZE = 18

  def __init__(self):
    super().__init__()
    self._font_title = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_label = gui_app.font(FontWeight.MEDIUM)
    self._font_value = gui_app.font(FontWeight.SEMI_BOLD)

    self._core_temp: float | None = None
    self._vram_temp: float | None = None
    self._available = False
    self._shown_this_drive = False
    self._started_frame = -1

  def _update_state(self) -> None:
    sm = ui_state.sm
    if ui_state.started_frame != self._started_frame:
      self._started_frame = ui_state.started_frame
      self._shown_this_drive = False
      self._core_temp = None
      self._vram_temp = None

    self._available = (sm.seen["chestnutState"] and sm.alive["chestnutState"] and sm.valid["chestnutState"] and
                       sm.recv_frame["chestnutState"] >= ui_state.started_frame)

    if self._available and (sm.updated["chestnutState"] or self._core_temp is None or self._vram_temp is None):
      self._core_temp = float(sm["chestnutState"].tempC)
      self._vram_temp = float(sm["chestnutState"].memoryTempC)

    if self._available and ui_state.status != UIStatus.DISENGAGED:
      self._shown_this_drive = True

  @staticmethod
  def _format_temperature(temperature: float | None) -> str:
    return "—" if temperature is None else f"{temperature:.0f}°C"

  def _draw_row(self, panel: rl.Rectangle, y: float, label: str, value: str) -> None:
    label_pos = rl.Vector2(panel.x + self.HORIZONTAL_PADDING, y)
    rl.draw_text_ex(self._font_label, label, label_pos, self.ROW_FONT_SIZE, 0, rl.Color(205, 205, 205, 255))

    value_size = measure_text_cached(self._font_value, value, self.ROW_FONT_SIZE)
    value_pos = rl.Vector2(panel.x + panel.width - self.HORIZONTAL_PADDING - value_size.x, y)
    value_color = rl.WHITE if self._available else rl.Color(166, 166, 166, 255)
    rl.draw_text_ex(self._font_value, value, value_pos, self.ROW_FONT_SIZE, 0, value_color)

  def _render(self, rect: rl.Rectangle) -> None:
    if not ui_state.ic_show_egpu_temperatures or not ui_state.usbgpu or not self._shown_this_drive:
      return

    graph_top = rect.y + rect.height * STEERING_GRAPH_CONFIG.zero_line_screen_y_frac - STEERING_GRAPH_CONFIG.height * 0.5
    panel = rl.Rectangle(
      rect.x + rect.width - STEERING_GRAPH_CONFIG.right_margin - self.PANEL_WIDTH,
      graph_top - self.GAP_TO_STEERING_GRAPH - self.PANEL_HEIGHT,
      self.PANEL_WIDTH,
      self.PANEL_HEIGHT,
    )

    rl.draw_rectangle_rounded(panel, 0.18, 10, rl.Color(0, 0, 0, self.BACKGROUND_ALPHA))
    rl.draw_rectangle_rounded_lines_ex(panel, 0.18, 10, 1, rl.Color(255, 255, 255, 45))

    title_pos = rl.Vector2(panel.x + self.HORIZONTAL_PADDING, panel.y + 7)
    rl.draw_text_ex(self._font_title, "eGPU", title_pos, self.TITLE_FONT_SIZE, 0, rl.WHITE)

    core = self._format_temperature(self._core_temp if self._available else None)
    vram = self._format_temperature(self._vram_temp if self._available else None)
    self._draw_row(panel, panel.y + 34, "CORE", core)
    self._draw_row(panel, panel.y + 58, "VRAM", vram)
