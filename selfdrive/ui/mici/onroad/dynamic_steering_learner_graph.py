import time
from dataclasses import dataclass

import numpy as np
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget


@dataclass(frozen=True)
class DynamicSteeringLearnerGraphMiciConfig:
  width: int = 144
  height: int = 72
  right_margin: int = 77
  zero_line_screen_y_frac: float = 0.70
  plot_padding_left: int = 0
  plot_padding_right: int = 0
  plot_padding_top: int = 0
  plot_padding_bottom: int = 0
  sample_points: int = 81


CONFIG = DynamicSteeringLearnerGraphMiciConfig()


class DynamicSteeringLearnerGraphMici(Widget):
  def __init__(self) -> None:
    super().__init__()
    self._params = Params()
    self._display_enabled = False
    self._param_update_time = 0.0

    self._preview_curve_color = rl.Color(250, 250, 250, 150)
    self._curve_color = rl.Color(120, 220, 170, 179)
    self._curve_invalid_color = rl.Color(220, 180, 90, 158)

    self._plot_x = np.linspace(-CurvatureDLookup.CURVATURE_MAX, CurvatureDLookup.CURVATURE_MAX, CONFIG.sample_points)
    self._update_params()

  def _update_params(self) -> None:
    self._param_update_time = time.monotonic()
    self._display_enabled = self._params.get_bool("ShowDynamicSteeringLearnerGraph")

  def _update_state(self) -> None:
    if time.monotonic() - self._param_update_time > 2.0:
      self._update_params()

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._display_enabled:
      return
    if ui_state.status in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY):
      return

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame or sm.recv_frame["controlsState"] < ui_state.started_frame:
      return

    zero_line_y = rect.y + rect.height * CONFIG.zero_line_screen_y_frac
    graph_rect = rl.Rectangle(
      rect.x + rect.width - CONFIG.right_margin - CONFIG.width,
      zero_line_y - CONFIG.height * 0.5,
      CONFIG.width,
      CONFIG.height,
    )

    lcp = sm["liveCurvatureParameters"]
    car_state = sm["carState"]

    fit_corrections = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    fit_valid = np.zeros(CurvatureDLookup.bucket_shape(), dtype=bool)
    preview_corrections = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    preview_valid = np.zeros(CurvatureDLookup.bucket_shape(), dtype=bool)
    payload_valid = bool(getattr(lcp, "liveValid", False))

    expected_size = CurvatureDLookup.total_size()
    if len(getattr(lcp, "corrections", [])) == expected_size:
      fit_corrections = CurvatureDLookup.unflatten_bucket(lcp.corrections, dtype=np.float32)
    if len(getattr(lcp, "fitValid", [])) == expected_size:
      fit_valid = CurvatureDLookup.unflatten_bucket(lcp.fitValid, dtype=bool)
    if len(getattr(lcp, "previewCorrections", [])) == expected_size:
      preview_corrections = CurvatureDLookup.unflatten_bucket(lcp.previewCorrections, dtype=np.float32)
    if len(getattr(lcp, "previewValid", [])) == expected_size:
      preview_valid = CurvatureDLookup.unflatten_bucket(lcp.previewValid, dtype=bool)

    plot_rect = rl.Rectangle(
      graph_rect.x + CONFIG.plot_padding_left,
      graph_rect.y + CONFIG.plot_padding_top,
      graph_rect.width - CONFIG.plot_padding_left - CONFIG.plot_padding_right,
      graph_rect.height - CONFIG.plot_padding_top - CONFIG.plot_padding_bottom,
    )

    self._draw_plot(
      plot_rect,
      preview_corrections,
      preview_valid,
      fit_corrections,
      fit_valid,
      float(car_state.vEgo),
      payload_valid,
    )

  def _draw_plot(self, plot_rect: rl.Rectangle,
                 preview_corrections: np.ndarray, preview_valid: np.ndarray,
                 fit_corrections: np.ndarray, fit_valid: np.ndarray,
                 v_ego: float, curve_valid: bool) -> None:
    preview_curve = np.array([
      CurvatureDLookup.interp_curve_value(preview_corrections, preview_valid, v_ego, abs(float(k))) * (1.0 if k >= 0.0 else -1.0)
      for k in self._plot_x
    ], dtype=np.float32)
    corrections = np.array([
      CurvatureDLookup.interp_curve_value(fit_corrections, fit_valid, v_ego, abs(float(k))) * (1.0 if k >= 0.0 else -1.0)
      for k in self._plot_x
    ], dtype=np.float32)
    max_y = max(2e-5, float(max(np.max(np.abs(preview_curve)), np.max(np.abs(corrections)))) * 1.2)

    preview_points = []
    actual_points = []
    for curvature, preview_correction, correction in zip(self._plot_x, preview_curve, corrections, strict=True):
      x = plot_rect.x + (float(curvature + CurvatureDLookup.CURVATURE_MAX) / (2.0 * CurvatureDLookup.CURVATURE_MAX)) * plot_rect.width
      preview_y = plot_rect.y + plot_rect.height * (0.5 - 0.5 * float(preview_correction) / max_y)
      actual_y = plot_rect.y + plot_rect.height * (0.5 - 0.5 * float(correction) / max_y)
      preview_points.append(rl.Vector2(float(x), float(preview_y)))
      actual_points.append(rl.Vector2(float(x), float(actual_y)))

    for p0, p1 in zip(preview_points[:-1], preview_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 1.9, self._preview_curve_color)
    curve_color = self._curve_color if curve_valid else self._curve_invalid_color
    for p0, p1 in zip(actual_points[:-1], actual_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 3.4, curve_color)
