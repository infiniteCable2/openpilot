from cereal import car
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.car import DT_CTRL, apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan, mebcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
from openpilot.selfdrive.controls.lib.drive_helpers import VOLKSWAGEN_V_CRUISE_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    self.CCP = CarControllerParams(CP)
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.last_button_frame = 0
    
    sub_services = ['longitudinalPlanSP']
    if CP.openpilotLongitudinalControl:
      sub_services.append('radarState')
    # TODO: Always true, prep for future conditional refactoring
    if sub_services:
      self.sm = messaging.SubMaster(sub_services)
    
    self.param_s = Params()
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_set_dis_prev = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.last_cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.acc_type = -1
    self.send_count = 0
    self.lead_distance = 0
    self.lead_distance_bars_last = None
    self.lead_distance_bar_timer = 0
    
    self.apply_curvature_last = 0
    self.steering_power_last = 0
    self.accel_last = 0

  def calculate_lead_distance(self, hud_control: car.CarControl.HUDControl) -> float:
    lead_one = self.sm["radarState"].leadOne
    lead_two = self.sm["radarState"].leadTwo

    if lead_one.status and (not lead_two.status or lead_one.dRel < lead_two.dRel):
      return lead_one.dRel
    if lead_two.status:
      return lead_two.dRel

    return 19 if hud_control.leadVisible else 0

  def update(self, CC, CS, now_nanos):
    if not self.CP.pcmCruiseSpeed or (self.CP.openpilotLongitudinalControl and self.frame % 5 == 0):
      self.sm.update(0)

      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      self.v_cruise_min = VOLKSWAGEN_V_CRUISE_MIN[CS.params_list.is_metric] * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1)
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if not self.CP.pcmCruiseSpeed:
      if not self.last_speed_limit_sign_tap_prev and CS.params_list.last_speed_limit_sign_tap:
        self.sl_force_active_timer = self.frame
        self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
      self.last_speed_limit_sign_tap_prev = CS.params_list.last_speed_limit_sign_tap

      sl_force_active = CS.params_list.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
      sl_inactive = not sl_force_active and (not CS.params_list.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
      sl_temp_inactive = not sl_force_active and (CS.params_list.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
      slc_active = not sl_inactive and not sl_temp_inactive

      self.slc_active_stock = slc_active

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state
        #   * steering power as counter near zero before lane assist deactivation
        # MEB rack can be used continously without found time limits yet

        if CC.latActive:
          hca_enabled = True
          # apply rate limits, curvature error limit, and clip to signal range
          current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
          apply_curvature = apply_std_steer_angle_limits(actuators.curvature, self.apply_curvature_last, CS.out.vEgoRaw, self.CCP)
          apply_curvature = clip(apply_curvature, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX)
          if CS.out.steeringPressed: # roughly sync with user input
            apply_angle = clip(apply_curvature, current_curvature - self.CCP.CURVATURE_ERROR, current_curvature + self.CCP.CURVATURE_ERROR)
          
        else:
          if self.steering_power_last > 0: # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
            apply_curvature = current_curvature # synchronize with current steering angle
          else:
            hca_enabled = False
            apply_curvature = 0. # inactive curvature

        steering_power = self.generate_vw_meb_steering_power(CS, CC.latActive, apply_curvature, self.steering_power_last)
        steering_power_boost = True if steering_power == self.CCP.STEERING_POWER_MAX else False
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_curvature, hca_enabled, steering_power, steering_power_boost))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.
        
        if CC.latActive:
          new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
          apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_steer_last == apply_steer:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_steer -= (1, -1)[apply_steer < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_steer) > 0
        else:
          hca_enabled = False
          apply_steer = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_steer_last = apply_steer
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled and CS.out.cruiseState.enabled else 0
      self.accel_last = accel
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      
      if self.CP.flags & VolkswagenFlags.MEB:
        current_speed = CS.out.vEgo * CV.MS_TO_KPH
        reversing = CS.out.gearShifter in [car.CarState.GearShifter.reverse]
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                                 CS.esp_hold_confirmation, CC.cruiseControl.override or CS.out.gasPressed)
        acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                               starting, stopping, CS.esp_hold_confirmation, CC.cruiseControl.override or CS.out.gasPressed)
        required_jerk = min(3, abs(accel - CS.out.aEgo) * 50) ## pfeiferj:openpilot:pfeifer-hkg-long-control-tune
        lower_jerk = required_jerk
        upper_jerk = required_jerk

        if CS.out.aEgo < accel:
          lower_jerk = 0
        else:
          upper_jerk = 0

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.enabled and CS.out.cruiseState.enabled,
                                                           accel, acc_control, acc_hold_type, stopping, starting, lower_jerk, upper_jerk,
                                                           CS.esp_hold_confirmation, CC.cruiseControl.override or CS.out.gasPressed, current_speed,
                                                           reversing, CS.travel_assist_available))

      else:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      sound_alert = 0
        
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
        sound_alert = 1
      if self.CP.flags & VolkswagenFlags.MEB:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control, sound_alert))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    # Parse lead distance from radarState and display the corresponding distance in the car's cluster
    if self.CP.openpilotLongitudinalControl and self.sm.updated['radarState'] and self.frame % 5 == 0:
      self.lead_distance = self.calculate_lead_distance(hud_control)

    if self.frame % 100 == 0 and self.lead_distance_bar_timer <= 3:
      self.lead_distance_bar_timer += 1

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        desired_gap = max(1, CS.out.vEgo * get_T_FOLLOW(hud_control.leadDistanceBars))
        distance = min(self.lead_distance, 140)
        
        change_distance_bar = False
        if hud_control.leadDistanceBars != self.lead_distance_bars_last:
          self.lead_distance_bar_timer = 0
        self.lead_distance_bars_last = hud_control.leadDistanceBars
        change_distance_bar = True if self.lead_distance_bar_timer <= 3 else False
          
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                                       CS.esp_hold_confirmation, CC.cruiseControl.override or CS.out.gasPressed)
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars, change_distance_bar,
                                                         desired_gap, distance, CS.esp_hold_confirmation))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
    if not (CC.cruiseControl.cancel or CC.cruiseControl.resume) and CS.out.cruiseState.enabled:
      if not self.CP.pcmCruiseSpeed:
        self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)
        if self.cruise_button is not None:
          if self.acc_type == -1:
            if self.button_count >= 2 and self.v_set_dis_prev != self.v_set_dis:
              self.acc_type = 1 if abs(self.v_set_dis - self.v_set_dis_prev) >= 10 and self.last_cruise_button in (1, 2) else \
                              0 if abs(self.v_set_dis - self.v_set_dis_prev) < 10 and self.last_cruise_button not in (1, 2) else 1
            if self.send_count >= 10 and self.v_set_dis_prev == self.v_set_dis:
              self.cruise_button = 3 if self.cruise_button == 1 else 4
          if self.acc_type == 0:
            self.cruise_button = 1 if self.cruise_button == 1 else 2  # accel, decel
          elif self.acc_type == 1:
            self.cruise_button = 3 if self.cruise_button == 1 else 4  # resume, set
          if self.frame % self.CCP.BTN_STEP == 0:
            can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CS.gra_stock_values, frame=(self.frame // self.CCP.BTN_STEP),
                                                                 buttons=self.cruise_button, custom_stock_long=True))
            self.send_count += 1
        else:
          self.send_count = 0
        self.last_cruise_button = self.cruise_button

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.v_set_dis_prev = self.v_set_dis
    self.frame += 1
    return new_actuators, can_sends

  def generate_vw_meb_steering_power(self, CS, lat_active, apply_curvature, steering_power_prev):
    # Steering power counter is used to:
    #   * prevent sudden fluctuations at low speeds
    #   * avoid HCA refused
    #   * easy user intervention
    #   * keep it near maximum regarding speed to get full steering power in shortest time
    if lat_active:
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
      steering_power_min_by_speed = interp(CS.out.vEgoRaw, [0, self.CCP.STEERING_POWER_MAX_BY_SPEED], [self.CCP.STEERING_POWER_MIN, self.CCP.STEERING_POWER_MAX])
      steering_curvature_diff = abs(apply_curvature - current_curvature)
      steering_power_target_curvature = steering_power_min_by_speed + self.CCP.CURVATURE_POWER_FACTOR * (steering_curvature_diff + abs(apply_curvature))
      steering_power_target = clip(steering_power_target_curvature, self.CCP.STEERING_POWER_MIN, self.CCP.STEERING_POWER_MAX)

      if steering_power_prev < self.CCP.STEERING_POWER_MIN:  # OP lane assist just activated
        steering_power = min(steering_power_prev + self.CCP.STEERING_POWER_STEPS, self.CCP.STEERING_POWER_MIN)
      elif CS.out.steeringPressed and steering_power_prev > self.CCP.STEERING_POWER_MIN:  # user action results in decreasing the steering power
        steering_power = max(steering_power_prev - self.CCP.STEERING_POWER_STEPS, self.CCP.STEERING_POWER_MIN)
      else: # following desired target
        if steering_power_prev < steering_power_target:
          steering_power = min(steering_power_prev + self.CCP.STEERING_POWER_STEPS, steering_power_target)
        elif steering_power_prev > steering_power_target:
          steering_power = max(steering_power_prev - self.CCP.STEERING_POWER_STEPS, steering_power_target)
        else:
          steering_power = steering_power_prev
        
    else:
      if steering_power_prev > 0: # monotonously decrement power to zero before disabling lane assist to prevent EPS fault
        steering_power = max(steering_power_prev - self.CCP.STEERING_POWER_STEPS, 0)
      else:
        steering_power = 0
        
    return steering_power

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled:
      for be in CS.out.buttonEvents:
        if be.type in (ButtonType.accelCruise, ButtonType.resumeCruise,
                       ButtonType.decelCruise, ButtonType.setCruise) and be.pressed:
          self.timer = 40
        elif be.type == ButtonType.gapAdjustCruise and be.pressed:
          self.timer = 300
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda: "default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    self.speed_diff = self.target_speed - self.v_set_dis
    if self.target_speed > self.v_set_dis:
      self.button_type = 1
    elif self.target_speed < self.v_set_dis and self.v_set_dis > self.v_cruise_min:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed <= self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed >= self.v_set_dis or self.v_set_dis <= self.v_cruise_min:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1))
    self.v_set_dis = round(CS.out.cruiseState.speed * (CV.MS_TO_MPH if not CS.params_list.is_metric else CV.MS_TO_KPH))
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def curve_speed_hysteresis(self, cur_speed: float, hyst=(0.75 * CV.MPH_TO_KPH)):
    if cur_speed > self.steady_speed:
      self.steady_speed = cur_speed
    elif cur_speed < self.steady_speed - hyst:
      self.steady_speed = cur_speed
    return self.steady_speed

  def get_cruise_buttons(self, CS, v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.out.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if self.slc_state > 1:
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if self.v_tsc_state != 0 or self.m_tsc_state > 1:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph, v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph, v_cruise_kph_prev)  # MPH/KPH based button presses
    return cruise_button
