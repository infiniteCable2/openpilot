#!/usr/bin/env python3
from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDLookup, VERSION
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

HISTORY = 1.5
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 1.5
ALLOWED_CARS = ['volkswagen']


class CurvatureEstimator(CurvatureDLookup):
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self.params = Params()
    self.frame = -1
    self.lag = 0.0
    self.hist_len = int(HISTORY / DT_MDL)
    self.calibrator = PoseCalibrator()

    self.bias = np.zeros(self.shape(), dtype=np.float32)
    self.counts = np.zeros(self.shape(), dtype=np.int32)

    self.car_control_t = deque(maxlen=self.hist_len)
    self.lat_active = deque(maxlen=self.hist_len)
    self.car_state_t = deque(maxlen=self.hist_len)
    self.vego = deque(maxlen=self.hist_len)
    self.steering_pressed = deque(maxlen=self.hist_len)
    self.model_t = deque(maxlen=self.hist_len)
    self.desired_curvature = deque(maxlen=self.hist_len)

    self.last_lat_inactive_t = 0.0
    self.last_override_t = 0.0

    self.current_bucket = (-1, -1, -1)
    self.current_correction = 0.0
    self.current_bias = 0.0
    self.current_bucket_points = 0

    self.use_params = False
    self.enable_curvatured = False
    self.update_use_params(force=True)

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
      self.use_params = self.enable_curvatured and self.CP.brand in ALLOWED_CARS and \
                        self.CP.steerControlType == car.CarParams.SteerControlType.curvatureDEPRECATED
      if not self.use_params:
        self.current_bucket = (-1, -1, -1)
        self.current_correction = 0.0
        self.current_bias = 0.0
        self.current_bucket_points = 0
    self.frame += 1

  def _history_ready(self) -> bool:
    return min(len(self.car_control_t), len(self.car_state_t), len(self.model_t)) == self.hist_len

  @staticmethod
  def _sample_at_or_before(target_t: float, ts: deque, values: deque):
    if len(ts) == 0:
      return None
    if target_t < ts[0]:
      return None

    for i in range(len(ts) - 1, -1, -1):
      if ts[i] <= target_t:
        return values[i]
    return None

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
    prev_bias = float(self.bias[sign_idx, speed_idx, curvature_idx])
    self.bias[sign_idx, speed_idx, curvature_idx] = prev_bias + alpha * (error - prev_bias)

  def _update_current_lookup(self, desired_curvature: float, v_ego: float) -> None:
    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      self.current_bucket = (-1, -1, -1)
      self.current_correction = 0.0
      self.current_bias = 0.0
      self.current_bucket_points = 0
      return

    sign_idx, speed_idx, curvature_idx = idx
    self.current_bucket = idx
    self.current_bias = float(self.bias[sign_idx, speed_idx, curvature_idx])
    self.current_bucket_points = int(self.counts[sign_idx, speed_idx, curvature_idx])

    if self.current_bucket_points < self.MIN_APPLY_SAMPLES:
      self.current_correction = 0.0
      return

    confidence = float(self.confidence(self.current_bucket_points))
    cap = self.cap_for_bucket(curvature_idx)
    self.current_correction = float(np.clip(self.current_bias, -cap, cap) * confidence)

  def handle_log(self, t: float, which: str, msg) -> None:
    if which == "carControl":
      self.car_control_t.append(t)
      self.lat_active.append(msg.latActive)
      if not msg.latActive:
        self.last_lat_inactive_t = t
    elif which == "carState":
      self.car_state_t.append(t)
      self.vego.append(msg.vEgo)
      self.steering_pressed.append(msg.steeringPressed)
      if msg.steeringPressed:
        self.last_override_t = t
    elif which == "modelV2":
      self.model_t.append(t)
      self.desired_curvature.append(msg.action.desiredCurvature)
      if self.car_state_t:
        self._update_current_lookup(self.desired_curvature[-1], self.vego[-1])
    elif which == "liveCalibration":
      self.calibrator.feed_live_calib(msg)
    elif which == "liveDelay":
      self.lag = get_lat_delay(self.params, msg.lateralDelay)
    elif which == "livePose" and self.use_params:
      if not self._history_ready():
        return
      if not (msg.angularVelocityDevice.valid and msg.posenetOK and msg.inputsOK and self.calibrator.calib_valid):
        return
      if (t - self.last_lat_inactive_t) < MIN_ENGAGE_BUFFER or (t - self.last_override_t) < MIN_ENGAGE_BUFFER:
        return

      target_t = t - self.lag
      lat_active = self._sample_at_or_before(target_t, self.car_control_t, self.lat_active)
      steering_pressed = self._sample_at_or_before(target_t, self.car_state_t, self.steering_pressed)
      v_ego = self._sample_at_or_before(target_t, self.car_state_t, self.vego)
      desired_curvature = self._sample_at_or_before(target_t, self.model_t, self.desired_curvature)

      if None in (lat_active, steering_pressed, v_ego, desired_curvature):
        return

      if not bool(lat_active) or bool(steering_pressed) or float(v_ego) < self.MIN_SPEED:
        return

      device_pose = Pose.from_live_pose(msg)
      calibrated_pose = self.calibrator.build_calibrated_pose(device_pose)
      yaw_rate = calibrated_pose.angular_velocity.yaw
      yaw_rate_std = calibrated_pose.angular_velocity.yaw_std
      if yaw_rate_std >= MAX_YAW_RATE_STD:
        return

      v_ego = float(v_ego)
      desired_curvature = float(desired_curvature)
      actual_curvature = yaw_rate / max(v_ego, 0.1)
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
    live_curvature_parameters.currentCorrection = self.current_correction
    live_curvature_parameters.currentBias = self.current_bias
    live_curvature_parameters.currentBucketPoints = self.current_bucket_points
    live_curvature_parameters.totalBucketPoints = int(self.counts.sum())
    live_curvature_parameters.calPerc = self.calibration_percent(self.counts)
    live_curvature_parameters.bucketSign = int(self.current_bucket[0])
    live_curvature_parameters.bucketSpeed = int(self.current_bucket[1])
    live_curvature_parameters.bucketCurvature = int(self.current_bucket[2])
    return msg


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['liveCurvatureParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'modelV2', 'liveCalibration', 'livePose', 'liveDelay'],
                           poll='livePose')

  params = Params()
  estimator = CurvatureEstimator(messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams))

  while True:
    sm.update()
    estimator.update_use_params()

    if estimator.use_params and sm.all_checks():
      for which in sm.updated.keys():
        if sm.updated[which]:
          estimator.handle_log(sm.logMonoTime[which] * 1e-9, which, sm[which])

    if sm.frame % 10 == 0:
      pm.send('liveCurvatureParameters', estimator.get_msg(valid=sm.all_checks()))


if __name__ == "__main__":
  main()
