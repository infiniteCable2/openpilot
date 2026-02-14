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

    self._aego_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # actual (effect)
    self._ades_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # desired (request)

    # intensity / "loaded" feel
    self._mag_f = FirstOrderFilter(0.0, 0.20, 1 / gui_app.target_fps)

    # TorqueBar-like show/hide smoothing
    self._alpha_f = FirstOrderFilter(0.0, 0.10, 1 / gui_app.target_fps)

  def update_filter(self, aego: float, ades: float = 0.0):
    self._aego_f.update(aego)
    self._ades_f.update(ades)

  def _update_state(self):
    if self._demo:
      return

    car_state = ui_state.sm['carState']
    car_control = ui_state.sm['carControl']

    self._aego_f.update(float(car_state.aEgo))
    self._ades_f.update(float(car_control.actuators.accel))

  @staticmethod
  def _norm_acc(a: float) -> float:
    # Map to [-1..1] with asymmetric scaling around 0
    if a >= 0.0:
      return a / max(1e-3, ACCEL_MAX)
    else:
      return a / max(1e-3, -ACCEL_MIN)

  def _render(self, rect: rl.Rectangle):
    content_rect = rl.Rectangle(
      rect.x + rect.width - SIDE_PANEL_WIDTH,
      rect.y,
      SIDE_PANEL_WIDTH,
      rect.height,
    )

    # ConfidenceBall uses this radius; we align to its footprint
    status_dot_radius = int(24 * self._scale)

    # --- Geometry: same "length scale" as the ball column ---
    bar_w = int(18 * self._scale)  # slightly thicker, closer to torque-bar visual weight
    gap_to_ball = int(3 * self._scale)  # move further right (smaller gap)

    # Make height match the confidence ball travel span (same formula as ConfidenceBall)
    # ConfidenceBall: dot_height spans [status_dot_radius .. content_rect.height - status_dot_radius]
    bar_h = int(content_rect.height - 2 * status_dot_radius)
    bar_h = int(clamp(bar_h, 160 * self._scale, content_rect.height - int(2 * status_dot_radius)))

    # Align bar top to the same band where the ball travels
    bar_y = int(content_rect.y + status_dot_radius)

    # Place bar left of ball footprint
    bar_x = int(content_rect.x + content_rect.width - (2 * status_dot_radius) - gap_to_ball - bar_w)

    # Visibility
    # fade out only when DISENGAGED (unless always=True).
    if self._demo:
      self._alpha_f.update(1.0)
    else:
      visible = self._always or (ui_state.status != UIStatus.DISENGAGED)
      self._alpha_f.update(1.0 if visible else 0.0)

    alpha = clamp(self._alpha_f.x, 0.0, 1.0)
    if alpha <= 0.001:
      return

    # --- Color policy: colored only for demo / ENGAGED / LONG_ONLY, otherwise gray ---
    colored = self._demo or (ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY))
    dim = 1.0 if colored else 0.55

    # Values
    aego = clamp(self._aego_f.x, ACCEL_MIN, ACCEL_MAX)  # effect
    ades = clamp(self._ades_f.x, ACCEL_MIN, ACCEL_MAX)  # request

    naego = clamp(self._norm_acc(aego), -1.0, 1.0)
    nades = clamp(self._norm_acc(ades), -1.0, 1.0)

    mag = clamp(abs(nades), 0.0, 1.0)
    self._mag_f.update(mag)
    load = self._mag_f.x

    # Slightly increase bar thickness with load (subtle) while keeping right edge aligned.
    extra_w = int((2.0 * load) * self._scale)  # 0..~2px
    bar_w_dyn = bar_w + extra_w
    bar_x_dyn = bar_x - extra_w  # expand left only, keeps proximity to ball

    # Background alpha ramps with load
    bg_alpha = int(255 * (0.18 + 0.10 * load) * alpha * dim)
    border_alpha = int(255 * (0.24 + 0.12 * load) * alpha * dim)
    mid_alpha = int(255 * 0.30 * alpha * dim)

    bg = rl.Color(255, 255, 255, bg_alpha)
    border = rl.Color(255, 255, 255, border_alpha)
    midline = rl.Color(255, 255, 255, mid_alpha)

    # Background
    rl.draw_rectangle(bar_x_dyn, bar_y, bar_w_dyn, bar_h, bg)
    rl.draw_rectangle_lines(bar_x_dyn, bar_y, bar_w_dyn, bar_h, border)

    # Midline at 0 m/s²
    mid_y = bar_y + bar_h // 2
    rl.draw_line(bar_x_dyn, mid_y, bar_x_dyn + bar_w_dyn, mid_y, midline)

    # Fill = desired/request (model/controller wants)
    half = bar_h / 2.0
    fill_h = int(abs(nades) * half)

    if colored:
      # fade to yellow/orange near extremes
      t = clamp((abs(nades) - 0.75) * 4.0, 0.0, 1.0)

      base_white = rl.Color(255, 255, 255, int(255 * (0.88 + 0.08 * load) * alpha * dim))
      if nades >= 0:
        hi = rl.Color(255, 200, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim))   # yellow
      else:
        hi = rl.Color(255, 115, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim))   # orange

      fill = blend_colors(base_white, hi, t)
    else:
      fill = rl.Color(255, 255, 255, int(255 * 0.32 * alpha * dim))

    if fill_h > 0:
      if nades >= 0:
        fy = int(mid_y - fill_h)
        rl.draw_rectangle(bar_x_dyn, fy, bar_w_dyn, fill_h, fill)
      else:
        fy = int(mid_y)
        rl.draw_rectangle(bar_x_dyn, fy, bar_w_dyn, fill_h, fill)

    # Actual marker = effect
    marker_alpha = int(255 * (0.70 if colored else 0.45) * alpha * dim)
    marker = rl.Color(255, 255, 255, marker_alpha)

    a_off = int((-naego) * half)
    a_y = int(mid_y + a_off)

    ext = int(3 * self._scale)
    rl.draw_line(bar_x_dyn - ext, a_y, bar_x_dyn + bar_w_dyn + ext, a_y, marker)

    # small centered marker dot
    dot_r = int((4 + 2 * load) * self._scale)  # grows a bit at extremes
    dot_x = int(bar_x_dyn + bar_w_dyn / 2)
    rl.draw_circle(dot_x, a_y, dot_r, marker)

    # Subtle limit ticks
    tick_alpha = int(255 * (0.18 + 0.10 * load) * alpha * dim)
    tick = rl.Color(255, 255, 255, tick_alpha)
    rl.draw_line(bar_x_dyn, bar_y, bar_x_dyn + bar_w_dyn, bar_y, tick)
    rl.draw_line(bar_x_dyn, bar_y + bar_h, bar_x_dyn + bar_w_dyn, bar_y + bar_h, tick)
