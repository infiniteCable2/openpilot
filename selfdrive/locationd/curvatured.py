from collections import deque

import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL
from openpilot.selfdrive.controls.lib.curvatured import CurvatureDLookup, VERSION
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

HISTORY = 1.5
MAX_YAW_RATE_STD = 1.0
MIN_ENGAGE_BUFFER = 1.5
ALLOWED_CARS = ['volkswagen']
STATUS_LOG_INTERVAL = 10.0
ROLL_LEARN_FRACTION_OF_MAX_LAT_ACCEL = 0.2
MAX_LEARN_ROLL_LATERAL_ACCEL = MAX_LATERAL_ACCEL_NO_ROLL * ROLL_LEARN_FRACTION_OF_MAX_LAT_ACCEL


class CurvatureEstimator(CurvatureDLookup):
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self.params = Params()
    self.frame = -1
    self.lag = 0.0
    self.hist_len = int(HISTORY / DT_MDL)
    self.calibrator = PoseCalibrator()

    self.bias = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.counts = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_corrections = np.zeros(self.bucket_shape(), dtype=np.float32)
    self.fit_valid = np.zeros(self.bucket_shape(), dtype=bool)

    self.car_control_t = deque(maxlen=self.hist_len)
    self.lat_active = deque(maxlen=self.hist_len)
    self.car_state_t = deque(maxlen=self.hist_len)
    self.vego = deque(maxlen=self.hist_len)
    self.steering_pressed = deque(maxlen=self.hist_len)
    self.controls_state_t = deque(maxlen=self.hist_len)
    self.model_desired_curvature = deque(maxlen=self.hist_len)

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

    self._restore_cached_params()
    self.update_use_params(force=True)

    cloudlog.info(f"curvatured init brand={self.CP.brand} fingerprint={self.CP.carFingerprint} "
                  f"steerControlType={self.CP.steerControlType} history={HISTORY:.2f}s")

  @staticmethod
  def get_restore_key(CP: car.CarParams, version: int):
    return (CP.carFingerprint, CP.brand, CP.steerControlType.raw, version)

  def _restore_cached_params(self) -> None:
    params_cache = self.params.get("CarParamsPrevRoute")
    curvature_cache = self.params.get("LiveCurvatureParameters")
    if params_cache is None or curvature_cache is None:
      return

    try:
      with log.Event.from_bytes(curvature_cache) as log_evt:
        cache_lcp = log_evt.liveCurvatureParameters
      with car.CarParams.from_bytes(params_cache) as msg:
        cache_CP = msg

      if self.get_restore_key(cache_CP, cache_lcp.version) != self.get_restore_key(self.CP, VERSION):
        return

      biases = list(cache_lcp.biases)
      counts = list(cache_lcp.counts)
      if len(biases) != self.total_size() or len(counts) != self.total_size():
        raise ValueError("invalid curvature cache shape")

      self.bias = self.unflatten_bucket(biases).astype(np.float32)
      self.counts = self.unflatten_bucket(counts).astype(np.float32)
      self.fit_corrections, self.fit_valid = self.build_fit_corrections(self.bias, self.counts)
      cloudlog.info("restored curvature params from cache")
    except Exception:
      cloudlog.exception("failed to restore cached curvature params")
      self.params.remove("LiveCurvatureParameters")

  def update_use_params(self, force: bool = False):
    if force or self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enable_curvatured = self.params.get_bool("EnableCurvatureD")
      self.use_params = self.CP.brand in ALLOWED_CARS and \
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
    return min(len(self.car_control_t), len(self.car_state_t), len(self.controls_state_t)) == self.hist_len

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

    speed_idx, curvature_idx = idx
    error = float(np.clip(self.projected_error(desired_curvature, actual_curvature),
                          -self.CORRECTION_CAP, self.CORRECTION_CAP))

    sample_count = min(float(self.counts[speed_idx, curvature_idx]) + 1.0, self.MAX_SAMPLES)
    self.counts[speed_idx, curvature_idx] = sample_count

    alpha = 1.0 / min(sample_count, self.MEAN_WINDOW)
    prev_bias = float(self.bias[speed_idx, curvature_idx])
    self.bias[speed_idx, curvature_idx] = prev_bias + alpha * (error - prev_bias)
    self.fit_corrections, self.fit_valid = self.build_fit_corrections(self.bias, self.counts)

  def _update_current_lookup(self, desired_curvature: float, v_ego: float) -> None:
    idx = self.indices(desired_curvature, v_ego)
    if idx is None:
      self.current_bucket = (-1, -1)
      self.current_correction = 0.0
      self.current_bias = 0.0
      self.current_bucket_points = 0
      return

    speed_idx, curvature_idx = idx
    self.current_bucket = idx
    self.current_bias = float(self.bias[speed_idx, curvature_idx])
    self.current_bucket_points = self.bucket_points_for_index(self.counts, idx)

    if not self.fit_valid[speed_idx].any():
      self.current_correction = 0.0
      return

    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    projected = self.interp_curve_value(self.fit_corrections, self.fit_valid, v_ego, abs(desired_curvature))
    self.current_correction = float(direction * projected)

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
    elif which == "controlsState":
      self.controls_state_t.append(t)
      self.model_desired_curvature.append(msg.modelDesiredCurvature)
      if self.car_state_t:
        self._update_current_lookup(self.model_desired_curvature[-1], self.vego[-1])
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
      desired_curvature = self._sample_at_or_before(target_t, self.controls_state_t, self.model_desired_curvature)

      if any(x is None for x in (lat_active, steering_pressed, v_ego, desired_curvature)):
        return

      if not bool(lat_active) or bool(steering_pressed) or float(v_ego) < self.MIN_SPEED:
        return

      device_pose = Pose.from_live_pose(msg)
      if not self.roll_learning_allowed(device_pose.orientation.roll):
        return
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

  def get_msg(self, valid: bool = True, live_valid: bool = True):
    msg = messaging.new_message('liveCurvatureParameters')
    msg.valid = valid

    live_curvature_parameters = msg.liveCurvatureParameters
    live_curvature_parameters.liveValid = bool(live_valid) and bool(np.isfinite(self.bias).all()) and bool(np.isfinite(self.fit_corrections).all())
    live_curvature_parameters.version = VERSION
    live_curvature_parameters.useParams = self.use_params
    live_curvature_parameters.currentCorrection = self.current_correction
    live_curvature_parameters.currentBias = self.current_bias
    live_curvature_parameters.currentBucketPoints = self.current_bucket_points
    live_curvature_parameters.totalBucketPoints = int(round(float(self.counts.sum())))
    live_curvature_parameters.calPerc = self.calibration_percent(self.counts)
    live_curvature_parameters.bucketSign = 0
    live_curvature_parameters.bucketSpeed = int(self.current_bucket[0])
    live_curvature_parameters.bucketCurvature = int(self.current_bucket[1])
    live_curvature_parameters.corrections = self.flatten(self.fit_corrections)
    live_curvature_parameters.counts = self.flatten(np.rint(self.counts).astype(np.int32))
    live_curvature_parameters.biases = self.flatten(self.bias)
    live_curvature_parameters.fitValid = self.flatten(self.fit_valid)
    return msg

  @staticmethod
  def roll_learning_allowed(roll: float) -> bool:
    return abs(np.sin(float(roll)) * ACCELERATION_DUE_TO_GRAVITY) <= MAX_LEARN_ROLL_LATERAL_ACCEL

  def maybe_log_status(self, t: float, sm, services: list[str] | None = None, valid: bool | None = None) -> None:
    if t < self.last_status_log_t + STATUS_LOG_INTERVAL:
      return

    tracked_services = list(sm.valid.keys()) if services is None else services
    invalid = [s for s in tracked_services if not sm.valid[s]]
    not_alive = [s for s in tracked_services if not sm.alive[s]]
    self.last_status_log_t = t

    checks = sm.all_checks(tracked_services) if valid is None else valid
    cloudlog.info(f"curvatured status use_params={self.use_params} checks={checks} "
                  f"lag={self.lag:.3f} total_points={int(round(float(self.counts.sum())))} "
                  f"bucket={self.current_bucket} bucket_points={self.current_bucket_points} "
                  f"corr={self.current_correction:.8f} cal={self.calibration_percent(self.counts)} "
                  f"invalid={invalid} not_alive={not_alive}")
