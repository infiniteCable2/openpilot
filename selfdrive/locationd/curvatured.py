#!/usr/bin/env python3
from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDLookup, VERSION
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

HISTORY = 6.0
MIN_ENGAGE_BUFFER = 1.5
MAX_YAW_RATE_STD = 1.0
ALLOWED_CARS = ['volkswagen']


class CurvatureEstimator(CurvatureDLookup):
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self.params = Params()
    self.hist_len = int(HISTORY / DT_MDL)
    self.frame = -1
    self.lag = 0.0
    self.calibrator = PoseCalibrator()

    self.bias = np.zeros(self.shape(), dtype=np.float64)
    self.counts = np.zeros(self.shape(), dtype=np.int32)
    self.reset_history()

    self.use_params = False
    self.enable_curvatured = False
    self.update_use_params(force=True)

    params_cache = self.params.get("CarParamsPrevRoute")
    curvature_cache = self.params.get("LiveCurvatureParameters")
    if params_cache is not None and curvature_cache is not None:
      try:
        with log.Event.from_bytes(curvature_cache) as log_evt:
          cache_lcp = log_evt.liveCurvatureParameters
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_cp = msg

        if self.get_restore_key(cache_cp, cache_lcp.version) == self.get_restore_key(CP, VERSION):
          if len(cache_lcp.bias) == self.total_size() and len(cache_lcp.counts) == self.total_size():
            self.bias = self.unflatten(cache_lcp.bias).astype(np.float64)
            self.counts = self.unflatten(cache_lcp.counts).astype(np.int32)
            cloudlog.info("restored curvature params from cache")
      except Exception:
        cloudlog.exception("failed to restore cached curvature params")
        self.params.remove("LiveCurvatureParameters")

  @staticmethod
  def get_restore_key(CP: car.CarParams, version: int):
    return CP.carFingerprint, CP.brand, version

  def reset_history(self):
    self.raw_points = {
      "carControl_t": deque(maxlen=self.hist_len),
      "lat_active": deque(maxlen=self.hist_len),
      "carState_t": deque(maxlen=self.hist_len),
      "vego": deque(maxlen=self.hist_len),
      "steering_pressed": deque(maxlen=self.hist_len),
      "controlsState_t": deque(maxlen=self.hist_len),
      "desired_curvature": deque(maxlen=self.hist_len),
      "saturated": deque(maxlen=self.hist_len),
    }

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
      self.use_params = self.enable_curvatured and self.CP.brand in ALLOWED_CARS and \
                        self.CP.steerControlType == car.CarParams.SteerControlType.curvatureDEPRECATED
    self.frame += 1

  def add_measurement(self, desired_curvature: float, actual_curvature: float, v_ego: float) -> None:
    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      return

    sign_idx, speed_idx, curvature_idx = idx
    cap = self.cap_for_bucket(curvature_idx)
    error = float(np.clip(desired_curvature - actual_curvature, -cap, cap))

    sample_count = min(int(self.counts[sign_idx, speed_idx, curvature_idx]) + 1, self.MAX_SAMPLES)
    self.counts[sign_idx, speed_idx, curvature_idx] = sample_count

    alpha = 1.0 / min(sample_count, self.MEAN_WINDOW)
    prev_bias = self.bias[sign_idx, speed_idx, curvature_idx]
    self.bias[sign_idx, speed_idx, curvature_idx] = float(prev_bias + alpha * (error - prev_bias))

  def handle_log(self, t: float, which: str, msg) -> None:
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["lat_active"].append(msg.latActive)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steering_pressed"].append(msg.steeringPressed)
    elif which == "controlsState":
      which_state = msg.lateralControlState.which()
      saturated = False if which_state is None else getattr(msg.lateralControlState, which_state).saturated
      self.raw_points["controlsState_t"].append(t + self.lag)
      self.raw_points["desired_curvature"].append(msg.desiredCurvature)
      self.raw_points["saturated"].append(saturated)
    elif which == "liveCalibration":
      self.calibrator.feed_live_calib(msg)
    elif which == "liveDelay":
      self.lag = get_lat_delay(self.params, msg.lateralDelay)
    elif which == "livePose" and self.use_params:
      if min(len(values) for values in self.raw_points.values()) < self.hist_len:
        return
      if not (msg.angularVelocityDevice.valid and msg.posenetOK and msg.inputsOK and self.calibrator.calib_valid):
        return

      device_pose = Pose.from_live_pose(msg)
      calibrated_pose = self.calibrator.build_calibrated_pose(device_pose)
      yaw_rate = calibrated_pose.angular_velocity.yaw
      yaw_rate_std = calibrated_pose.angular_velocity.yaw_std

      if yaw_rate_std >= MAX_YAW_RATE_STD:
        return

      window = np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL)
      lat_active = np.interp(window, self.raw_points["carControl_t"], self.raw_points["lat_active"]).astype(bool)
      steering_pressed = np.interp(window, self.raw_points["carState_t"], self.raw_points["steering_pressed"]).astype(bool)
      saturated = np.interp(window, self.raw_points["controlsState_t"], self.raw_points["saturated"]).astype(bool)
      v_ego = float(np.interp(t, self.raw_points["carState_t"], self.raw_points["vego"]))
      desired_curvature = float(np.interp(t, self.raw_points["controlsState_t"], self.raw_points["desired_curvature"]))
      actual_curvature = yaw_rate / max(v_ego, 0.1)

      if not all(lat_active) or any(steering_pressed) or any(saturated):
        return
      if v_ego < self.MIN_SPEED:
        return
      if max(abs(desired_curvature), abs(actual_curvature)) * (v_ego ** 2) > self.MAX_LAT_ACCEL:
        return

      self.add_measurement(desired_curvature, actual_curvature, v_ego)

  def get_msg(self, valid: bool = True):
    msg = messaging.new_message('liveCurvatureParameters')
    msg.valid = valid

    live_curvature_parameters = msg.liveCurvatureParameters
    live_curvature_parameters.liveValid = bool(np.isfinite(self.bias).all())
    live_curvature_parameters.version = VERSION
    live_curvature_parameters.useParams = self.use_params
    live_curvature_parameters.bias = self.flatten(self.bias.astype(np.float32))
    live_curvature_parameters.counts = self.flatten(self.counts.astype(np.int32))
    live_curvature_parameters.totalBucketPoints = int(self.counts.sum())
    live_curvature_parameters.calPerc = self.calibration_percent(self.counts)
    return msg


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['liveCurvatureParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'liveCalibration', 'livePose', 'liveDelay'],
                           poll='livePose')

  params = Params()
  estimator = CurvatureEstimator(messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams))

  while True:
    sm.update()
    if sm.all_checks():
      for which in sm.updated.keys():
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          estimator.handle_log(t, which, sm[which])

    estimator.update_use_params()

    if sm.frame % 5 == 0:
      pm.send('liveCurvatureParameters', estimator.get_msg(valid=sm.all_checks()))

    if sm.frame % 240 == 0:
      params.put_nonblocking("LiveCurvatureParameters", estimator.get_msg(valid=sm.all_checks()).to_bytes())


if __name__ == "__main__":
  main()
