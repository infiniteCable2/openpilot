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
  zero_line_screen_y_frac: float = 0.68
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

    self._preview_glow_color = rl.Color(255, 255, 255, 42)
    self._preview_curve_color = rl.Color(250, 250, 250, 168)
    self._curve_glow_color = rl.Color(0, 255, 64, 56)
    self._curve_color = rl.Color(0, 255, 64, 188)
    self._curve_invalid_glow_color = rl.Color(255, 170, 70, 44)
    self._curve_invalid_color = rl.Color(235, 185, 95, 166)
    self._marker_glow_color = rl.Color(255, 80, 80, 72)
    self._marker_color = rl.Color(255, 90, 90, 240)
    self._plot_x = np.linspace(-CurvatureDLookup.CURVATURE_MAX, CurvatureDLookup.CURVATURE_MAX, CONFIG.sample_points)
    self._cached_lcp_frame = -1
    self._cached_preview_curve = np.zeros(CONFIG.sample_points, dtype=np.float32)
    self._cached_fit_curve = np.zeros(CONFIG.sample_points, dtype=np.float32)
    self._cached_min_y = 0.0
    self._cached_max_y = 2e-5

    self._update_params()

  @staticmethod
  def _compute_y_bounds(preview_curve: np.ndarray, corrections: np.ndarray) -> tuple[float, float]:
    min_val = float(min(np.min(preview_curve), np.min(corrections)))
    max_val = float(max(np.max(preview_curve), np.max(corrections)))
    min_span = 2e-5

    if min_val >= 0.0:
      return 0.0, max(min_span, max_val * 1.2)
    if max_val <= 0.0:
      return min(-min_span, min_val * 1.2), 0.0

    low = min_val * 1.2
    high = max_val * 1.2
    if (high - low) < min_span:
      center = 0.5 * (high + low)
      half = 0.5 * min_span
      return center - half, center + half
    return low, high

  @staticmethod
  def _map_y(plot_rect: rl.Rectangle, value: float, min_y: float, max_y: float) -> float:
    frac = (value - min_y) / max(max_y - min_y, 1e-9)
    return float(plot_rect.y + plot_rect.height * (1.0 - frac))

  def _update_params(self) -> None:
    self._param_update_time = time.monotonic()
    self._display_enabled = self._params.get_bool("ShowDynamicSteeringLearnerGraph")

  def _update_state(self) -> None:
    if time.monotonic() - self._param_update_time > 2.0:
      self._update_params()

  def _get_curve_samples(self, lcp_frame: int,
                         preview_corrections: np.ndarray, preview_valid: np.ndarray,
                         fit_corrections: np.ndarray, fit_valid: np.ndarray,
                         v_ego: float) -> tuple[np.ndarray, np.ndarray, float, float]:
    if lcp_frame != self._cached_lcp_frame:
      abs_curvatures = np.abs(self._plot_x).astype(np.float64)
      # Cached across UI frames; recomputes only when liveCurvatureParameters changes (4Hz).
      self._cached_fit_curve = CurvatureDLookup.interp_curve_value(
        fit_corrections, fit_valid, v_ego, abs_curvatures
      )
      has_preview = preview_corrections.shape == fit_corrections.shape and np.any(preview_corrections)
      if has_preview:
        self._cached_preview_curve = CurvatureDLookup.interp_curve_value(
          preview_corrections, preview_valid, v_ego, abs_curvatures
        )
      self._cached_min_y, self._cached_max_y = self._compute_y_bounds(self._cached_preview_curve, self._cached_fit_curve)
      self._cached_lcp_frame = lcp_frame

    return self._cached_preview_curve, self._cached_fit_curve, self._cached_min_y, self._cached_max_y

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
    lcp_frame = sm.recv_frame["liveCurvatureParameters"]
    car_state = sm["carState"]
    controls_state = sm["controlsState"]

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
    preview_curve, corrections, min_y, max_y = self._get_curve_samples(
      lcp_frame, preview_corrections, preview_valid, fit_corrections, fit_valid, float(car_state.vEgo)
    )

    self._draw_plot(
      plot_rect,
      preview_curve,
      corrections,
      min_y,
      max_y,
      float(controls_state.modelDesiredCurvature),
      payload_valid,
    )

  def _draw_plot(self, plot_rect: rl.Rectangle,
                 preview_curve: np.ndarray, corrections: np.ndarray,
                 min_y: float, max_y: float,
                 desired_curvature: float, curve_valid: bool) -> None:

    preview_points = []
    actual_points = []
    for curvature, preview_correction, correction in zip(self._plot_x, preview_curve, corrections, strict=True):
      x = plot_rect.x + ((float(curvature) + CurvatureDLookup.CURVATURE_MAX) / (2.0 * CurvatureDLookup.CURVATURE_MAX)) * plot_rect.width
      preview_y = self._map_y(plot_rect, float(preview_correction), min_y, max_y)
      actual_y = self._map_y(plot_rect, float(correction), min_y, max_y)
      preview_points.append(rl.Vector2(float(x), float(preview_y)))
      actual_points.append(rl.Vector2(float(x), float(actual_y)))

    for p0, p1 in zip(preview_points[:-1], preview_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 4.2, self._preview_glow_color)
    for p0, p1 in zip(preview_points[:-1], preview_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 2.0, self._preview_curve_color)

    curve_glow_color = self._curve_glow_color if curve_valid else self._curve_invalid_glow_color
    curve_color = self._curve_color if curve_valid else self._curve_invalid_color
    for p0, p1 in zip(actual_points[:-1], actual_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 6.4, curve_glow_color)
    for p0, p1 in zip(actual_points[:-1], actual_points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 3.4, curve_color)

    marker_alpha = float(np.clip(
      (desired_curvature + CurvatureDLookup.CURVATURE_MAX) / (2.0 * CurvatureDLookup.CURVATURE_MAX),
      0.0, 1.0,
    ))
    marker_x = plot_rect.x + marker_alpha * plot_rect.width
    marker_correction = float(np.interp(abs(desired_curvature), np.abs(self._plot_x), self._cached_fit_curve))
    marker_y = self._map_y(plot_rect, marker_correction, min_y, max_y)
    rl.draw_circle(int(marker_x), int(marker_y), 5, self._marker_glow_color)
    rl.draw_circle(int(marker_x), int(marker_y), 3, self._marker_color)
