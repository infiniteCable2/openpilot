import pyray as rl
from dataclasses import dataclass

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget


@dataclass(frozen=True)
class BatteryPanelConfig:
  scale_factor: float = 1.1
  panel_width_ratio: float = 0.7   # 70 % der Breite von rechts
  panel_margin: int = 30           # Abstand zum Rand
  line_height: int = 48            # Basis-Zeilenhöhe
  label_width: int = 320
  text_margin: int = 25            # Abstand Label → Wert
  
CONFIG = BatteryPanelConfig()

class BatteryDetails(Widget):
  def __init__(self) -> None:
    super().__init__()
    self._params = Params()
    
    self._capacity: float = 0.0
    self._charge: float = 0.0
    self._soc: float = 0.0
    self._temperature: float = 0.0
    self._heaterActive: bool = False
    self._voltage: float = 0.0
    self._current: float = 0.0
    self._power: float = 0.0

    self._font: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._panel_bg: rl.Color = rl.Color(0, 0, 0, 128)
    self._text_color: rl.Color = rl.Color(255, 255, 255, 255)
    
    self._display_enabled: bool = False

  def _update_state(self) -> None:
    self._display_enabled = self._params.get_bool("BatteryDetails")
    if not self._display_enabled:
      return
      
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self._reset_values()
      return

    car_state = sm["carState"]
    
    battery_data = car_state.batteryDetails
    self._capacity      = float(battery_data.capacity)
    self._charge        = float(battery_data.charge)
    self._soc           = float(battery_data.soc)
    self._temperature   = float(battery_data.temperature)
    self._heater_active = bool(battery_data.heaterActive)
    self._voltage       = float(battery_data.voltage)
    self._current       = float(battery_data.current)
    self._power         = float(battery_data.power)
    
  def _reset_values(self) -> None:
    self._capacity = 0.0
    self._charge = 0.0
    self._soc = 0.0
    self._temperature = 0.0
    self._heater_active = False
    self._voltage = 0.0
    self._current = 0.0
    self._power = 0.0

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._display_enabled:
      return

    scale = CONFIG.scale_factor
    base_line_h = CONFIG.line_height
    line_h = int(base_line_h * scale)

    panel_width = int(rect.width * CONFIG.panel_width_ratio)
    panel_height = line_h * 4

    panel_margin = CONFIG.panel_margin

    x_start = int(rect.x + rect.width - panel_width - panel_margin)
    y_start = int(rect.y + rect.height - panel_margin - panel_height)

    panel_rect = rl.Rectangle(x_start, y_start, panel_width, panel_height)
    rl.draw_rectangle_rounded(panel_rect, 0.1, 8, self._panel_bg)

    label_width = CONFIG.label_width
    text_margin = CONFIG.text_margin
    column_spacing = panel_width // 2 - 40
    value_width = column_spacing - label_width - text_margin

    labels = [
      "Capacity:", "Charge:", "SoC:", "Temperature:",
      "Heater Active:", "Voltage:", "Current:", "Power:",
    ]

    values = [
      f"{self._capacity:.2f} Wh",
      f"{self._charge:.2f} Wh",
      f"{self._soc:.2f} %",
      f"{self._temperature:.2f} °C",
      "True" if self._heater_active else "False",
      f"{self._voltage:.2f} V",
      f"{self._current:.2f} A",
      f"{self._power:.2f} kW",
    ]

    rl.draw_text_ex

    for i, (label, value) in enumerate(zip(labels, values)):
      column = i // 4
      row = i % 4

      base_x = x_start + column * column_spacing
      base_y = y_start + row * line_h

      label_pos = rl.Vector2(float(base_x), float(base_y))
      rl.draw_text_ex(
        self._font,
        label,
        label_pos,
        40 * scale,
        0,
        self._text_color,
      )

      value_x = base_x + label_width + text_margin
      value_pos = rl.Vector2(float(value_x), float(base_y))
      rl.draw_text_ex(
        self._font,
        value,
        value_pos,
        40 * scale,
        0,
        self._text_color,
      )
