from functools import wraps
from collections import OrderedDict

import numpy as np
import pyray as rl
from cereal import car

from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient


ACCEL_MAX = 2.0
ACCEL_MIN = -3.5

GearShifter = car.CarState.GearShifter


def clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


def quantized_lru_cache(maxsize=256):
  def decorator(func):
    cache = OrderedDict()

    @wraps(func)
    def wrapper(*args, **kwargs):
      q_args = []
      for a in args:
        if isinstance(a, float):
          q_args.append(round(a * 2) / 2)
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
def rounded_rect_pts(x: float, y: float, w: float, h: float, r: float, segs: int = 9) -> np.ndarray:
  r = max(0.0, min(r, w * 0.5, h * 0.5))
  if r <= 0.01:
    return np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]], dtype=np.float32)

  tr = _arc(x + w - r, y + r, r, 270, 360, segs)
  br = _arc(x + w - r, y + h - r, r, 0, 90, segs)
  bl = _arc(x + r, y + h - r, r, 90, 180, segs)
  tl = _arc(x + r, y + r, r, 180, 270, segs)
  return np.vstack([tr, br, bl, tl, tr[:1]]).astype(np.float32)


@quantized_lru_cache(maxsize=256)
def rounded_cap_segment_pts(x: float, y: float, w: float, h: float, r: float, *, cap: str, segs: int = 9) -> np.ndarray:
  r = max(0.0, min(r, w * 0.5, h * 0.5))
  if r <= 0.01:
    return np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]], dtype=np.float32)

  if cap == "top":
    tr = _arc(x + w - r, y + r, r, 270, 360, segs)
    tl = _arc(x + r, y + r, r, 180, 270, segs)
    return np.vstack([
      tr,
      [x + w, y + r],
      [x + w, y + h],
      [x,     y + h],
      [x,     y + r],
      tl,
      tr[:1],
    ]).astype(np.float32)

  br = _arc(x + w - r, y + h - r, r, 0, 90, segs)
  bl = _arc(x + r, y + h - r, r, 90, 180, segs)
  return np.vstack([
    [x + w, y],
    [x + w, y + h - r],
    br,
    bl,
    [x,     y + h - r],
    [x,     y],
    [x + w, y],
  ]).astype(np.float32)


class LongitudinalAccelBar(Widget):
  def __init__(self, demo: bool = False, scale: float = 1.0, always: bool = False):
    super().__init__()
    self._demo = demo
    self._scale = scale
    self._always = always

    # filtered effect and request values
    self._aego_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._ades_f = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)

    # load / visibility filters
    self._mag_f = FirstOrderFilter(0.0, 0.20, 1 / gui_app.target_fps)
    self._alpha_f = FirstOrderFilter(0.0, 0.10, 1 / gui_app.target_fps)

  def update_filter(self, aego: float, ades: float = 0.0):
    self._aego_f.update(aego)
    self._ades_f.update(ades)

  def _update_state(self):
    if self._demo:
      return
    cs = ui_state.sm['carState']
    cc = ui_state.sm['carControl']

    aego = float(cs.aEgo)
    if cs.gearShifter == GearShifter.reverse:
      aego = -aego

    self._aego_f.update(aego)
    self._ades_f.update(float(cc.actuators.accel))

  @staticmethod
  def _norm_acc(a: float) -> float:
    return a / max(1e-3, (ACCEL_MAX if a >= 0.0 else -ACCEL_MIN))

  def _render(self, rect: rl.Rectangle):
    # alignment
    bar_w = int(19 * self._scale)
    right_margin = int(12 * self._scale)
    bar_x = int(rect.x + rect.width - bar_w + right_margin)

    # vertical span similar to confidence ball travel
    status_dot_radius = int(24 * self._scale)
    bar_h_max = int(rect.height - 2 * status_dot_radius)
    bar_h_min = int(160 * self._scale)
    bar_h = int(clamp(bar_h_max, bar_h_min, max(bar_h_min, bar_h_max)))
    bar_y = int(rect.y + status_dot_radius)

    # visibility logic
    if self._demo:
      self._alpha_f.update(1.0)
    else:
      visible = self._always or (ui_state.status != UIStatus.DISENGAGED)
      self._alpha_f.update(1.0 if visible else 0.0)

    alpha = clamp(self._alpha_f.x, 0.0, 1.0)
    if alpha <= 0.001:
      return

    # colored for demo / engaged / long_only
    colored = self._demo or (ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY))
    dim = 1.0 if colored else 0.55

    aego = clamp(self._aego_f.x, ACCEL_MIN, ACCEL_MAX)
    ades = clamp(self._ades_f.x, ACCEL_MIN, ACCEL_MAX)

    naego = clamp(self._norm_acc(aego), -1.0, 1.0)
    nades = clamp(self._norm_acc(ades), -1.0, 1.0)

    # intensity scaling
    self._mag_f.update(clamp(abs(nades), 0.0, 1.0))
    load = self._mag_f.x

    extra_w = int((2.0 * load) * self._scale)
    bw = bar_w + extra_w
    bx = bar_x - extra_w

    radius = max(2.0, 6.0 * self._scale)

    # background shape
    bg_alpha = int(255 * (0.18 + 0.10 * load) * alpha * dim)
    bg_pts = rounded_rect_pts(float(bx), float(bar_y), float(bw), float(bar_h), float(radius), segs=9)
    draw_polygon(rect, bg_pts, color=rl.Color(255, 255, 255, bg_alpha))

    # zero line
    mid_y = bar_y + bar_h // 2
    mid_alpha = int(255 * 0.30 * alpha * dim)
    rl.draw_line(bx, mid_y, bx + bw, mid_y, rl.Color(255, 255, 255, mid_alpha))

    # fill for desired accel
    half = bar_h / 2.0
    fill_h = int(abs(nades) * half)

    if colored:
      t = clamp((abs(nades) - 0.75) * 4.0, 0.0, 1.0)
      base = rl.Color(255, 255, 255, int(255 * (0.88 + 0.08 * load) * alpha * dim))
      hi = rl.Color(255, 200, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim)) if nades >= 0 else \
           rl.Color(255, 115, 0, int(255 * (0.92 + 0.06 * load) * alpha * dim))
      fill_start = blend_colors(base, hi, t)
      fill_end = blend_colors(base, hi, clamp(t + 0.25, 0.0, 1.0))
    else:
      fill_start = fill_end = rl.Color(255, 255, 255, int(255 * 0.32 * alpha * dim))

    if fill_h > 0:
      if nades >= 0:
        fy, fh, cap = int(mid_y - fill_h), int(fill_h), "top"
      else:
        fy, fh, cap = int(mid_y), int(fill_h), "bottom"

      seg_r = float(min(radius, fh * 0.5))
      seg_pts = rounded_cap_segment_pts(float(bx), float(fy), float(bw), float(fh), float(seg_r), cap=cap, segs=9)

      # gradient in widget-local coordinates
      cx = ((bx + bw / 2.0) - rect.x) / rect.width
      ex = (bx - rect.x) / rect.width if (nades < 0) else ((bx + bw) - rect.x) / rect.width

      grad = Gradient(start=(cx, 0), end=(ex, 0), colors=[fill_start, fill_end], stops=[0.0, 1.0])
      draw_polygon(rect, seg_pts, gradient=grad)

    # actual accel dot (effect)
    dot_alpha = int(255 * (0.75 if colored else 0.50) * alpha * dim)
    dot_color = rl.Color(255, 255, 255, dot_alpha)

    a_off = int((-naego) * half)
    a_y = int(mid_y + a_off)

    dot_r = int((6 + 3 * load) * self._scale)
    rl.draw_circle(int(bx + bw / 2), a_y, dot_r, dot_color)
