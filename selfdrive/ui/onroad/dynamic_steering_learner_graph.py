import time
from dataclasses import dataclass

import numpy as np
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup
from openpilot.selfdrive.ui.onroad.battery_details import CONFIG as BATTERY_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget


@dataclass(frozen=True)
class DynamicSteeringLearnerGraphConfig:
  width: int = 896
  height: int = 392
  right_margin: int = 30
  bottom_gap_to_battery: int = 20
  padding: int = 25
  plot_padding_left: int = 73
  plot_padding_right: int = 25
  plot_padding_top: int = 59
  plot_padding_bottom: int = 48
  sample_points: int = 121


CONFIG = DynamicSteeringLearnerGraphConfig()


class DynamicSteeringLearnerGraph(Widget):
  def __init__(self) -> None:
    super().__init__()
    self._params = Params()
    self._display_enabled = False
    self._param_update_time = 0.0

    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._panel_bg = rl.Color(0, 0, 0, 128)
    self._axis_color = rl.Color(255, 255, 255, 90)
    self._grid_color = rl.Color(255, 255, 255, 45)
    self._curve_color = rl.Color(120, 220, 170, 255)
    self._curve_invalid_color = rl.Color(220, 180, 90, 220)
    self._marker_color = rl.Color(255, 80, 80, 255)
    self._text_color = rl.Color(255, 255, 255, 245)
    self._muted_text_color = rl.Color(200, 200, 200, 220)

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

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame or sm.recv_frame["controlsState"] < ui_state.started_frame:
      return

    battery_line_height = int(BATTERY_CONFIG.line_height * BATTERY_CONFIG.scale_factor)
    battery_panel_height = battery_line_height * 4
    battery_panel_margin = BATTERY_CONFIG.panel_margin
    graph_rect = rl.Rectangle(
      rect.x + rect.width - CONFIG.width - battery_panel_margin,
      rect.y + rect.height - CONFIG.height - CONFIG.bottom_gap_to_battery
      - battery_panel_height - battery_panel_margin,
      CONFIG.width,
      CONFIG.height,
    )
    rl.draw_rectangle_rounded(graph_rect, 0.08, 8, self._panel_bg)

    lcp = sm["liveCurvatureParameters"]
    controls_state = sm["controlsState"]
    car_state = sm["carState"]

    fit_corrections = np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
    fit_valid = np.zeros(CurvatureDLookup.bucket_shape(), dtype=bool)
    transport_valid = bool(sm.valid["liveCurvatureParameters"])
    payload_valid = bool(getattr(lcp, "liveValid", False))

    expected_size = CurvatureDLookup.total_size()
    if len(getattr(lcp, "corrections", [])) == expected_size:
      fit_corrections = CurvatureDLookup.unflatten_bucket(lcp.corrections, dtype=np.float32)
    if len(getattr(lcp, "fitValid", [])) == expected_size:
      fit_valid = CurvatureDLookup.unflatten_bucket(lcp.fitValid, dtype=bool)

    plot_rect = rl.Rectangle(
      graph_rect.x + CONFIG.plot_padding_left,
      graph_rect.y + CONFIG.plot_padding_top,
      graph_rect.width - CONFIG.plot_padding_left - CONFIG.plot_padding_right,
      graph_rect.height - CONFIG.plot_padding_top - CONFIG.plot_padding_bottom,
    )
    _, max_y = self._draw_plot(plot_rect, fit_corrections, fit_valid, float(car_state.vEgo), transport_valid and payload_valid)
    self._draw_overlay_info(graph_rect, lcp, float(car_state.vEgo), float(controls_state.modelDesiredCurvature),
                            fit_corrections, fit_valid, max_y, transport_valid, payload_valid)

  def _draw_plot(self, plot_rect: rl.Rectangle, fit_corrections: np.ndarray, fit_valid: np.ndarray,
                 v_ego: float, curve_valid: bool) -> tuple[np.ndarray, float]:
    rl.draw_rectangle_lines_ex(plot_rect, 1.0, self._grid_color)

    zero_x = plot_rect.x + plot_rect.width / 2
    zero_y = plot_rect.y + plot_rect.height / 2
    rl.draw_line_ex(rl.Vector2(float(plot_rect.x), float(zero_y)),
                    rl.Vector2(float(plot_rect.x + plot_rect.width), float(zero_y)), 2.0, self._axis_color)
    rl.draw_line_ex(rl.Vector2(float(zero_x), float(plot_rect.y)),
                    rl.Vector2(float(zero_x), float(plot_rect.y + plot_rect.height)), 2.0, self._axis_color)

    for frac in (0.25, 0.75):
      x = plot_rect.x + plot_rect.width * frac
      y = plot_rect.y + plot_rect.height * frac
      rl.draw_line_ex(rl.Vector2(float(x), float(plot_rect.y)),
                      rl.Vector2(float(x), float(plot_rect.y + plot_rect.height)), 1.0, self._grid_color)
      rl.draw_line_ex(rl.Vector2(float(plot_rect.x), float(y)),
                      rl.Vector2(float(plot_rect.x + plot_rect.width), float(y)), 1.0, self._grid_color)

    corrections = np.array([
      CurvatureDLookup.interp_curve_value(fit_corrections, fit_valid, v_ego, abs(float(k))) * (1.0 if k >= 0.0 else -1.0)
      for k in self._plot_x
    ], dtype=np.float32)
    max_y = max(2e-5, float(np.max(np.abs(corrections))) * 1.2)

    points = []
    for curvature, correction in zip(self._plot_x, corrections, strict=True):
      x = plot_rect.x + (float(curvature + CurvatureDLookup.CURVATURE_MAX) / (2.0 * CurvatureDLookup.CURVATURE_MAX)) * plot_rect.width
      y = plot_rect.y + plot_rect.height * (0.5 - 0.5 * float(correction) / max_y)
      points.append(rl.Vector2(float(x), float(y)))

    curve_color = self._curve_color if curve_valid else self._curve_invalid_color
    for p0, p1 in zip(points[:-1], points[1:], strict=True):
      rl.draw_line_ex(p0, p1, 3.0, curve_color)
    return corrections, max_y

  def _draw_overlay_info(self, graph_rect: rl.Rectangle, lcp, v_ego: float, desired_curvature: float,
                         fit_corrections: np.ndarray, fit_valid: np.ndarray, max_y: float,
                         transport_valid: bool, payload_valid: bool) -> None:
    low_idx, high_idx, alpha = CurvatureDLookup.speed_interp(v_ego)
    current_correction = 0.0
    if transport_valid and payload_valid:
      current_correction = CurvatureDLookup.interp_curve_value(
        fit_corrections, fit_valid, v_ego, abs(desired_curvature)
      ) * (1.0 if desired_curvature >= 0.0 else -1.0)

    marker_alpha = float(np.clip(
      (desired_curvature + CurvatureDLookup.CURVATURE_MAX) / (2.0 * CurvatureDLookup.CURVATURE_MAX),
      0.0, 1.0,
    ))
    marker_x = graph_rect.x + CONFIG.plot_padding_left + marker_alpha * (
      graph_rect.width - CONFIG.plot_padding_left - CONFIG.plot_padding_right
    )
    plot_height = graph_rect.height - CONFIG.plot_padding_top - CONFIG.plot_padding_bottom
    marker_y = graph_rect.y + CONFIG.plot_padding_top + plot_height * (0.5 - 0.5 * float(current_correction) / max_y)
    rl.draw_circle(int(marker_x), int(marker_y), 6, self._marker_color)

    title = "Dynamic Steering Learner"
    rl.draw_text_ex(self._font_bold, title, rl.Vector2(float(graph_rect.x + 18), float(graph_rect.y + 10)), 28, 0, self._text_color)

    status_text = (
      f"live={payload_valid} transport={transport_valid} cal={int(getattr(lcp, 'calPerc', 0))}% "
      f"points={int(getattr(lcp, 'totalBucketPoints', 0))}"
    )
    rl.draw_text_ex(self._font_medium, status_text, rl.Vector2(float(graph_rect.x + 18), float(graph_rect.y + 46)), 20, 0, self._muted_text_color)

    speed_mix = (
      f"v={v_ego * 3.6:.0f} km/h  mix={CurvatureDLookup.SPEED_ANCHORS[low_idx] * 3.6:.0f}/"
      f"{CurvatureDLookup.SPEED_ANCHORS[high_idx] * 3.6:.0f} alpha={alpha:.2f}"
    )
    marker_info = (
      f"k={desired_curvature:.2e}  corr={current_correction:.2e}  "
      f"bucket=({int(getattr(lcp, 'bucketSpeed', -1))}, {int(getattr(lcp, 'bucketCurvature', -1))})"
    )
    rl.draw_text_ex(self._font_medium, speed_mix, rl.Vector2(float(graph_rect.x + 18), float(graph_rect.y + graph_rect.height - 52)), 18, 0, self._muted_text_color)
    rl.draw_text_ex(self._font_medium, marker_info, rl.Vector2(float(graph_rect.x + 18), float(graph_rect.y + graph_rect.height - 28)), 18, 0, self._muted_text_color)
