import math
from functools import wraps
from collections import OrderedDict

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH, blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient


ACCEL_MAX = 2.0
ACCEL_MIN = -3.5


def clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


def quantized_lru_cache(maxsize=256):
  def decorator(func):
    cache = OrderedDict()
    @wraps(func)
    def wrapper(*args, **kwargs):
      # Quantize floats for cache efficiency
      q_args = []
      for a in args:
        if isinstance(a, float):
          q_args.append(round(a * 2) / 2)   # 0.5px precision
        else:
          q_args.append(a)
      key = (tuple(q_args), tuple(sorted(kwargs.items())))
      if key in cache:
        cache.move_to_end(key)
      else:
        if len(cache) >= maxsize:
          cache.popitem(last=False)
        cache[key] = func(*args, **kwargs)
      return cache[key]
    return wrapper
  return decorator


def _arc(cx: float, cy: float, r: float, a0_deg: float, a1_deg: float, segs: int) -> np.ndarray:
  a = np.deg2rad(np.linspace(a0_deg, a1_deg, max(2, segs)))
  return np.c_[cx + np.cos(a) * r, cy + np.sin(a) * r]


@quantized_lru_cache(maxsize=256)
def rounded_rect_pts(x: float, y: float, w: float, h: float, r: float, segs: int = 8) -> np.ndarray:
  r = max(0.0, min(r, w * 0.5, h * 0.5))
  if r <= 0.01:
    pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]], dtype=np.float32)
    return pts

  tr = _arc(x + w - r, y + r, r, 270, 360, segs)
  br = _arc(x + w - r, y + h - r, r, 0, 90, segs)
  bl = _arc(x + r, y + h - r, r, 90, 180, segs)
  tl = _arc(x + r, y + r, r, 180, 270, segs)

  pts = np.vstack([tr, br, bl, tl, tr[:1]]).astype(np.float32)
  return pts


@quantized_lru_cache(maxsize=256)
def rounded_cap_segment_pts(x: float, y: float, w: float, h: float, r: float, *, cap: str, segs: int = 8) -> np.ndarray:
  """
  Segment with ONE rounded end and one flat end.
  cap:
    - "top"    : rounded top corners, flat bottom
    - "bottom" : flat top, rounded bottom corners
  """
  r = max(0.0, min(r, w * 0.5, h * 0.5))
  if r <= 0.01:
    pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]], dtype=np.float32)
    return pts

  if cap == "top":
    # Start at top-right arc, go down to bottom-right (flat), across, up, top-left arc
    tr = _arc(x + w - r, y + r, r, 270, 360, segs)
    tl = _arc(x + r, y + r, r, 180, 270, segs)

    pts = np.vstack([
      tr,                                 # rounded top right
      [x + w, y + r],                     # end of arc (redundant-ish but safe)
      [x + w, y + h],                     # flat bottom-right
      [x,     y + h],                     # flat bottom-left
      [x,     y + r],                     # up left side to start of top-left arc
      tl,                                 # rounded top left
      tr[:1]
    ]).astype(np.float32)
    return pts

  # cap == "bottom"
  br = _arc(x + w - r, y + h - r, r, 0, 90, segs)
  bl = _arc(x + r, y + h - r, r, 90, 180, segs)

  pts = np.vstack([
    [x + w, y],                           # flat top-right
    [x + w, y + h - r],                   # down right side to start of arc
    br,                                   # rounded bottom-right
    bl,                                   # rounded bottom-left
    [x,     y + h - r],                   # end of left arc
    [x,     y],                           # up to flat top-left
    [x + w, y]
  ]).astype(np.float32)
  return pts


class LongitudinalAccelBar(Widget):
  def __init__(self, demo: bool = False, scale: float = 1.0, always: bool = False):
    super().__init__()
    self._demo = demo
    self._scale = scale
    self._always = always

    self._aego_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # actual (effect)
    self._ades_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)   # desired (request)
    self._mag_f = FirstOrderFilter(0.0, 0.20, 1 / gui_app.target_fps)
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
    if a >= 0.0:
      return a / max(1e-3, ACCEL_MAX)
    else:
      return a / max(1e-3, -ACCEL_MIN)

  def _render(self, rect: rl.Rectangle):
    # Right-side panel frame
    content_rect = rl.Rectangle(
      rect.x + rect.width - SIDE_PANEL_WIDTH,
      rect.y,
      SIDE_PANEL_WIDTH,
      rect.height,
    )

    status_dot_radius = int(24 * self._scale)

    # Make it a bit thicker and shift further right (smaller gap)
    bar_w = int(19 * self._scale)
    gap_to_ball = int(1 * self._scale)  # further right than before

    # Match confidence-ball travel span
    bar_h = int(content_rect.height - 2 * status_dot_radius)
    bar_h = int(clamp(bar_h, 160 * self._scale, content_rect.height - 2 * status_dot_radius))

    bar_y = int(content_rect.y + status_dot_radius)
    bar_x = int(content_rect.x + content_rect.width - (2 * status_dot_radius) - gap_to_ball - bar_w)

    # Fade logic like TorqueBar spirit (hide on DISENGAGED unless always/demo)
    if self._demo:
      self._alpha_f.update(1.0)
    else:
      visible = self._always or (ui_state.status != UIStatus.DISENGAGED)
      self._alpha_f.update(1.0 if visible else 0.0)

    alpha = clamp(self._alpha_f.x, 0.0, 1.0)
    if alpha <= 0.001:
      return

    # Colored only in demo/ENGAGED/LONG_ONLY; otherwise gray
    colored = self._demo or (ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY))
    dim = 1.0 if colored else 0.55

    aego = clamp(self._aego_f.x, ACCEL_MIN, ACCEL_MAX)  # effect
    ades = clamp(self._ades_f.x, ACCEL_MIN, ACCEL_MAX)  # request

    naego = clamp(self._norm_acc(aego), -1.0, 1.0)
    nades = clamp(self._norm_acc(ades), -1.0, 1.0)

    mag = clamp(abs(nades), 0.0, 1.0)
    self._mag_f.update(mag)
    load = self._mag_f.x

    # Subtle "loaded" thickness increase (like TorqueBar height growth feel)
    extra_w = int((2.0 * load) * self._scale)  # ~0..2px
    bw = bar_w + extra_w
    bx = bar_x - extra_w  # expand left, keep it near the ball on the right

    # Rounded corners via polygon (TorqueBar-like softness)
    radius = max(2.0, 6.0 * self._scale)

    # Background polygon (no outline!)
    bg_alpha = int(255 * (0.18 + 0.10 * load) * alpha * dim)
    bg_color = rl.Color(255, 255, 255, bg_alpha)
    bg_pts = rounded_rect_pts(float(bx), float(bar_y), float(bw), float(bar_h), float(radius), segs=9)
    draw_polygon(rect, bg_pts, color=bg_color)

    # Midline (0 accel) — keep subtle
    mid_alpha = int(255 * 0.30 * alpha * dim)
    midline = rl.Color(255, 255, 255, mid_alpha)
    mid_y = bar_y + bar_h // 2
    rl.draw_line(bx, mid_y, bx + bw, mid_y, midline)

    # Fill = desired/request segment with a rounded cap at the extreme end
    half = bar_h / 2.0
    fill_h = int(abs(nades) * half)

    if colored:
      t = clamp((abs(nades) - 0.75) * 4.0, 0.0, 1.0)
      base_white = rl.Color(255, 255, 255, int(255 * (0.88 + 0.08 * load) * alpha * dim))
      if nades >= 0:
        hi = rl.Color(255, 200, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim))
      else:
        hi = rl.Color(255, 115, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim))
      fill_start = blend_colors(base_white, hi, t)
      fill_end = blend_colors(base_white, hi, clamp(t + 0.25, 0.0, 1.0))
    else:
      fill_start = fill_end = rl.Color(255, 255, 255, int(255 * 0.32 * alpha * dim))

    if fill_h > 0:
      if nades >= 0:
        fy = int(mid_y - fill_h)
        fh = int(fill_h)
        cap = "top"
      else:
        fy = int(mid_y)
        fh = int(fill_h)
        cap = "bottom"

      # cap radius can't exceed segment height
      seg_r = float(min(radius, fh * 0.5))

      seg_pts = rounded_cap_segment_pts(float(bx), float(fy), float(bw), float(fh), float(seg_r), cap=cap, segs=9)

      # TorqueBar-like gradient: from bar center toward requested direction
      # (horizontal gradient, subtle but gives that "shader" feel)
      cx = (bx + bw / 2.0) / rect.width
      if nades < 0:
        ex = (bx / rect.width)  # left
      else:
        ex = ((bx + bw) / rect.width)  # right

      grad = Gradient(
        start=(cx, 0),
        end=(ex, 0),
        colors=[fill_start, fill_end],
        stops=[0.0, 1.0],
      )

      draw_polygon(rect, seg_pts, gradient=grad)

    # Actual marker (effect): keep the softer “dot” like TorqueBar
    marker_alpha = int(255 * (0.70 if colored else 0.45) * alpha * dim)
    marker = rl.Color(255, 255, 255, marker_alpha)

    a_off = int((-naego) * half)
    a_y = int(mid_y + a_off)

    # Keep marker mostly inside bar for softness
    rl.draw_line(bx, a_y, bx + bw, a_y, marker)

    dot_r = int((4 + 2 * load) * self._scale)
    dot_x = int(bx + bw / 2)
    rl.draw_circle(dot_x, a_y, dot_r, marker)

    # Subtle limit ticks
    tick_alpha = int(255 * (0.18 + 0.10 * load) * alpha * dim)
    tick = rl.Color(255, 255, 255, tick_alpha)
    rl.draw_line(bx, bar_y, bx + bw, bar_y, tick)
    rl.draw_line(bx, bar_y + bar_h, bx + bw, bar_y + bar_h, tick)
