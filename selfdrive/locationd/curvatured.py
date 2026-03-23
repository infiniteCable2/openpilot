#!/usr/bin/env python3
from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDLookup, VERSION
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

HISTORY = 1.5
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 1.5
ALLOWED_CARS = ['volkswagen']
STATUS_LOG_INTERVAL = 10.0


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

    self.current_bucket = (-1, -1)
    self.current_correction = 0.0
    self.current_bias = 0.0
    self.current_bucket_points = 0

    self.use_params = False
    self.enable_curvatured = False
    self.prev_use_params = None
    self.last_status_log_t = 0.0
    self.update_use_params(force=True)

    cloudlog.info(f"curvatured init brand={self.CP.brand} fingerprint={self.CP.carFingerprint} "
                  f"steerControlType={self.CP.steerControlType} history={HISTORY:.2f}s")

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
      self.use_params = self.enable_curvatured and self.CP.brand in ALLOWED_CARS and \
                        self.CP.steerControlType == car.CarParams.SteerControlType.curvatureDEPRECATED
      if self.prev_use_params != self.use_params:
        cloudlog.info(f"curvatured use_params={self.use_params} toggle={self.enable_curvatured} "
                      f"brand={self.CP.brand} allowed={self.CP.brand in ALLOWED_CARS} "
                      f"steerControlType={self.CP.steerControlType}")
        self.prev_use_params = self.use_params
      if not self.use_params:
        self.current_bucket = (-1, -1)
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

    sign_idx, speed_idx = idx
    error = float(np.clip(desired_curvature - actual_curvature,
                          -self.CORRECTION_CAP, self.CORRECTION_CAP))

    sample_count = min(int(self.counts[sign_idx, speed_idx]) + 1, self.MAX_SAMPLES)
    self.counts[sign_idx, speed_idx] = sample_count

    alpha = 1.0 / min(sample_count, self.MEAN_WINDOW)
    prev_bias = float(self.bias[sign_idx, speed_idx])
    self.bias[sign_idx, speed_idx] = prev_bias + alpha * (error - prev_bias)

  def _update_current_lookup(self, desired_curvature: float, v_ego: float) -> None:
    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      self.current_bucket = (-1, -1)
      self.current_correction = 0.0
      self.current_bias = 0.0
      self.current_bucket_points = 0
      return

    sign_idx, speed_idx = idx
    self.current_bucket = idx
    self.current_bias = float(self.bias[sign_idx, speed_idx])
    self.current_bucket_points = int(self.counts[sign_idx, speed_idx])

    if self.current_bucket_points < self.MIN_APPLY_SAMPLES:
      self.current_correction = 0.0
      return

    confidence = float(self.confidence(self.current_bucket_points))
    self.current_correction = float(np.clip(self.current_bias, -self.CORRECTION_CAP, self.CORRECTION_CAP) * confidence)

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

      if any(x is None for x in (lat_active, steering_pressed, v_ego, desired_curvature)):
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
    return msg

  def maybe_log_status(self, t: float, sm) -> None:
    if t < self.last_status_log_t + STATUS_LOG_INTERVAL:
      return

    invalid = [s for s in sm.valid.keys() if not sm.valid[s]]
    not_alive = [s for s in sm.alive.keys() if not sm.alive[s]]
    self.last_status_log_t = t

    cloudlog.info(f"curvatured status use_params={self.use_params} checks={sm.all_checks()} "
                  f"lag={self.lag:.3f} total_points={int(self.counts.sum())} "
                  f"bucket={self.current_bucket} bucket_points={self.current_bucket_points} "
                  f"corr={self.current_correction:.8f} cal={self.calibration_percent(self.counts)} "
                  f"invalid={invalid} not_alive={not_alive}")


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['liveCurvatureParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'modelV2', 'liveCalibration', 'livePose', 'liveDelay'],
                           poll='livePose')

  params = Params()
  estimator = CurvatureEstimator(messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams))

  while True:
    try:
      sm.update()
      estimator.update_use_params()

      if estimator.use_params and sm.all_checks():
        for which in sm.updated.keys():
          if sm.updated[which]:
            try:
              estimator.handle_log(sm.logMonoTime[which] * 1e-9, which, sm[which])
            except Exception:
              cloudlog.exception(f"curvatured handle_log failed service={which}")

      if sm.frame % 20 == 0:
        t = sm.logMonoTime['livePose'] * 1e-9 if sm.logMonoTime['livePose'] != 0 else sm.frame * DT_MDL
        estimator.maybe_log_status(t, sm)
        pm.send('liveCurvatureParameters', estimator.get_msg(valid=sm.all_checks()))
    except Exception:
      cloudlog.exception("curvatured main loop failed")


if __name__ == "__main__":
  main()
