import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH, blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter


ACCEL_MAX = 2.0
ACCEL_MIN = -3.5


def clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


class LongitudinalAccelBar(Widget):
  """
  Vertical acceleration bar (straight edges) shown in the right side panel,
  positioned left of the confidence ball.

  - Thick fill: actual accel (carState.aEgo)
  - Thin marker: desired accel (carControl.actuators.accel)
  """
  def __init__(self, demo: bool = False, scale: float = 1.0, always: bool = False):
    super().__init__()
    self._demo = demo
    self._scale = scale
    self._always = always

    # filtering for smooth UI
    self._aego_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._ades_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._alpha_f = FirstOrderFilter(0.0, 0.10, 1 / gui_app.target_fps)

  def update_filter(self, aego: float, ades: float = 0.0):
    """Demo mode helper."""
    self._aego_f.update(aego)
    self._ades_f.update(ades)

  def _update_state(self):
    if self._demo:
      return

    car_state = ui_state.sm['carState']
    car_control = ui_state.sm['carControl']

    self._aego_f.update(car_state.aEgo)
    self._ades_f.update(car_control.actuators.accel)

  def _render(self, _):
    # Side panel rect (same as ConfidenceBall)
    content_rect = rl.Rectangle(
      self.rect.x + self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    # confidence ball footprint (from your reference)
    status_dot_radius = int(24 * self._scale)

    # bar geometry (smaller than full height)
    bar_w = int(14 * self._scale)
    bar_h = int(160 * self._scale)
    top_pad = int(22 * self._scale)
    gap_to_ball = int(10 * self._scale)

    # Place bar: left of the ball (ball is drawn at far right edge)
    bar_x = int(content_rect.x + content_rect.width - (2 * status_dot_radius) - gap_to_ball - bar_w)
    bar_y = int(content_rect.y + top_pad)

    # If you want it vertically centered instead, swap line above with:
    # bar_y = int(content_rect.y + (content_rect.height - bar_h) / 2)

    # alpha behavior (similar to torque bar)
    if self._demo:
      self._alpha_f.update(1.0)
    else:
      visible = self._always or (ui_state.status not in (UIStatus.DISENGAGED,))
      self._alpha_f.update(1.0 if visible else 0.0)

    alpha = clamp(self._alpha_f.x, 0.0, 1.0)

    # status-based dimming
    engaged_like = (ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY, UIStatus.LONG_ONLY, UIStatus.OVERRIDE))
    dim = 1.0 if (self._demo or engaged_like) else 0.55

    # clamp accels
    aego = clamp(self._aego_f.x, ACCEL_MIN, ACCEL_MAX)
    ades = clamp(self._ades_f.x, ACCEL_MIN, ACCEL_MAX)

    # map accel to [-1..1] around 0, but with asymmetric range
    # -> use separate scaling for + and - so visual "full scale" hits at ACCEL_MAX/ACCEL_MIN
    def norm_acc(a: float) -> float:
      if a >= 0:
        return a / max(1e-3, ACCEL_MAX)
      else:
        return -a / max(1e-3, -ACCEL_MIN) * -1.0  # keep sign

    naego = clamp(norm_acc(aego), -1.0, 1.0)
    nades = clamp(norm_acc(ades), -1.0, 1.0)

    # colors
    bg_alpha = int(255 * 0.22 * alpha * dim)
    border_alpha = int(255 * 0.28 * alpha * dim)
    mid_alpha = int(255 * 0.35 * alpha * dim)

    bg = rl.Color(255, 255, 255, bg_alpha)
    border = rl.Color(255, 255, 255, border_alpha)
    midline = rl.Color(255, 255, 255, mid_alpha)

    # base fill colors (fade to orange/red with magnitude)
    # accel (positive): white -> yellow
    # brake (negative): white -> orange/red
    mag = abs(naego)
    if naego >= 0:
      c0 = rl.Color(255, 255, 255, int(255 * 0.90 * alpha * dim))
      c1 = rl.Color(255, 200, 0,   int(255 * 0.95 * alpha * dim))
      fill = blend_colors(c0, c1, clamp((mag - 0.55) / 0.45, 0.0, 1.0))
    else:
      c0 = rl.Color(255, 255, 255, int(255 * 0.90 * alpha * dim))
      c1 = rl.Color(255, 115, 0,   int(255 * 0.95 * alpha * dim))
      fill = blend_colors(c0, c1, clamp((mag - 0.55) / 0.45, 0.0, 1.0))

    # if not actively engaged (but visible), gray it out
    if (not self._demo) and (ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY, UIStatus.LONG_ONLY)):
      fill = rl.Color(255, 255, 255, int(255 * 0.35 * alpha * dim))

    # draw background bar (straight)
    rl.draw_rectangle(bar_x, bar_y, bar_w, bar_h, bg)
    rl.draw_rectangle_lines(bar_x, bar_y, bar_w, bar_h, border)

    # midline (0 accel)
    mid_y = bar_y + bar_h // 2
    rl.draw_line(bar_x, mid_y, bar_x + bar_w, mid_y, midline)

    # actual fill from midline up/down
    # compute fill height (half bar)
    half = bar_h / 2.0
    fill_h = int(abs(naego) * half)

    if fill_h > 0:
      if naego >= 0:
        fy = int(mid_y - fill_h)
        rl.draw_rectangle(bar_x, fy, bar_w, fill_h, fill)
      else:
        fy = int(mid_y)
        rl.draw_rectangle(bar_x, fy, bar_w, fill_h, fill)

    # desired marker (thin horizontal line)
    # map desired to y position
    des_off = int((-nades) * half)  # -1 => down, +1 => up
    des_y = int(mid_y + des_off)
    des_color = rl.Color(255, 255, 255, int(255 * 0.70 * alpha * dim))
    rl.draw_line(bar_x - int(3 * self._scale), des_y, bar_x + bar_w + int(3 * self._scale), des_y, des_color)

    # optional: little "tick" at top/bottom to show limits
    tick_alpha = int(255 * 0.25 * alpha * dim)
    tick = rl.Color(255, 255, 255, tick_alpha)
    rl.draw_line(bar_x, bar_y, bar_x + bar_w, bar_y, tick)
    rl.draw_line(bar_x, bar_y + bar_h, bar_x + bar_w, bar_y + bar_h, tick)
