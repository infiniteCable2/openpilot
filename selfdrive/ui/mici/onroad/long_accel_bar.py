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
  def __init__(self, demo: bool = False, scale: float = 1.0, always: bool = False):
    super().__init__()
    self._demo = demo
    self._scale = scale
    self._always = always

    # Smooth motion
    self._aego_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # actual accel (effect)
    self._ades_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # desired accel (request)

    self._mag_f = FirstOrderFilter(0.0, 0.20, 1 / gui_app.target_fps)

  def update_filter(self, aego: float, ades: float = 0.0):
    self._aego_f.update(aego)
    self._ades_f.update(ades)

  def _update_state(self):
    if self._demo:
      return

    car_state = ui_state.sm['carState']
    car_control = ui_state.sm['carControl']

    # actual accel
    aego = float(car_state.aEgo)

    # desired accel (command)
    ades = float(car_control.actuators.accel)

    self._aego_f.update(aego)
    self._ades_f.update(ades)

  @staticmethod
  def _norm_acc(a: float) -> float:
    # Map to [-1..1] with asymmetric scaling around 0
    if a >= 0.0:
      return a / max(1e-3, ACCEL_MAX)
    else:
      return a / max(1e-3, -ACCEL_MIN)  # negative / positive -> negative

  def _render(self, rect: rl.Rectangle):
    content_rect = rl.Rectangle(
      rect.x + rect.width - SIDE_PANEL_WIDTH,
      rect.y,
      SIDE_PANEL_WIDTH,
      rect.height,
    )

    # "rightmost" confidence ball sits at (content_rect.x + content_rect.width - status_dot_radius)
    status_dot_radius = int(24 * self._scale)

    bar_w_base = 16
    bar_w = int(bar_w_base * self._scale)

    # Height roughly the confidence ball vertical span (2*radius) plus some breathing room.
    # Also clamp to not exceed panel height.
    bar_h = int((2 * status_dot_radius + 110 * self._scale))
    bar_h = int(clamp(bar_h, 140 * self._scale, content_rect.height - 40 * self._scale))

    # Place near the top like ConfidenceBall uses, but keep it nicely within panel.
    top_pad = int(18 * self._scale)

    # Slightly closer to the confidence ball than before
    gap_to_ball = int(6 * self._scale)

    bar_x = int(content_rect.x + content_rect.width - (2 * status_dot_radius) - gap_to_ball - bar_w)
    bar_y = int(content_rect.y + top_pad)

    # If we run out of space at bottom, shift upward a bit
    if bar_y + bar_h > content_rect.y + content_rect.height - int(12 * self._scale):
      bar_y = int(content_rect.y + content_rect.height - int(12 * self._scale) - bar_h)

    # Visibility: always drawn
    alpha = 1.0 if (self._demo or self._always or True) else 0.0

    # Color policy: only demo/ENGAGED/LONG_ONLY are colored; everything else is gray
    colored = self._demo or (ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY))
    dim = 1.0 if colored else 0.55

    # Values
    aego = clamp(self._aego_f.x, ACCEL_MIN, ACCEL_MAX)  # effect
    ades = clamp(self._ades_f.x, ACCEL_MIN, ACCEL_MAX)  # request

    naego = clamp(self._norm_acc(aego), -1.0, 1.0)
    nades = clamp(self._norm_acc(ades), -1.0, 1.0)

    mag = clamp(abs(nades), 0.0, 1.0)
    self._mag_f.update(mag)

    # Slight intensity growth near extremes (similar "bigger/stronger" feel)
    # This increases background alpha a bit and makes the bar look more "loaded".
    load = self._mag_f.x
    bg_alpha = int(255 * (0.20 + 0.08 * load) * alpha * dim)
    border_alpha = int(255 * (0.26 + 0.10 * load) * alpha * dim)
    mid_alpha = int(255 * 0.35 * alpha * dim)

    bg = rl.Color(255, 255, 255, bg_alpha)
    border = rl.Color(255, 255, 255, border_alpha)
    midline = rl.Color(255, 255, 255, mid_alpha)

    # Background
    rl.draw_rectangle(bar_x, bar_y, bar_w, bar_h, bg)
    rl.draw_rectangle_lines(bar_x, bar_y, bar_w, bar_h, border)

    # Midline at 0 m/s²
    mid_y = bar_y + bar_h // 2
    rl.draw_line(bar_x, mid_y, bar_x + bar_w, mid_y, midline)

    # Fill = desired/request (what the model/controller wants)
    # Marker = actual/effect (what happens)
    half = bar_h / 2.0
    fill_h = int(abs(nades) * half)

    if colored:
      # Fade to orange as we approach max request magnitude
      # Positive request -> more yellow, Negative request -> more orange
      t = clamp((abs(nades) - 0.75) * 4.0, 0.0, 1.0)
      base_white = rl.Color(255, 255, 255, int(255 * (0.90 + 0.05 * load) * alpha * dim))

      if nades >= 0:
        hi = rl.Color(255, 200, 0, int(255 * (0.95 + 0.03 * load) * alpha * dim))   # yellow
      else:
        hi = rl.Color(255, 115, 0, int(255 * (0.95 + 0.03 * load) * alpha * dim))   # orange

      fill = blend_colors(base_white, hi, t)
    else:
      fill = rl.Color(255, 255, 255, int(255 * 0.35 * alpha * dim))

    if fill_h > 0:
      if nades >= 0:
        fy = int(mid_y - fill_h)
        rl.draw_rectangle(bar_x, fy, bar_w, fill_h, fill)
      else:
        fy = int(mid_y)
        rl.draw_rectangle(bar_x, fy, bar_w, fill_h, fill)

    # Actual marker line (effect): thin line across bar (and a tiny bit outside)
    # Keep it visible even in gray mode, but slightly dimmed
    marker_alpha = int(255 * (0.70 if colored else 0.45) * alpha * dim)
    marker = rl.Color(255, 255, 255, marker_alpha)

    a_off = int((-naego) * half)
    a_y = int(mid_y + a_off)
    ext = int(3 * self._scale)
    rl.draw_line(bar_x - ext, a_y, bar_x + bar_w + ext, a_y, marker)

    # Optional limit ticks (subtle)
    tick_alpha = int(255 * (0.22 + 0.08 * load) * alpha * dim)
    tick = rl.Color(255, 255, 255, tick_alpha)
    rl.draw_line(bar_x, bar_y, bar_x + bar_w, bar_y, tick)
    rl.draw_line(bar_x, bar_y + bar_h, bar_x + bar_w, bar_y + bar_h, tick)
