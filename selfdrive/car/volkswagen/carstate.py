import numpy as np
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.volkswagen.values import DBC, CANBUS, NetworkLocation, TransmissionType, GearShifter, \
                                            CarControllerParams, VolkswagenFlags, VolkswagenFlagsSP


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.eps_init_complete = False
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.esp_hold_confirmation = False
    self.upscale_lead_car_signal = False
    self.eps_stock_values = False
    self.v_limit = 0
    self.v_limit_speed_factor = 0
    self.v_limit_receive = False

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, pt_cp, cam_cp, ext_cp, trans_type):
    if self.CP.flags & VolkswagenFlags.PQ:
      return self.update_pq(pt_cp, cam_cp, ext_cp, trans_type)
    elif self.CP.flags & VolkswagenFlags.MEB:
      return self.update_meb(pt_cp, cam_cp, ext_cp, trans_type)

    ret = car.CarState.new_message()

    self.prev_mads_enabled = self.mads_enabled

    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # VW Emergency Assist status tracking and mitigation
    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    brake_pedal_pressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
    brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
    ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well
    ret.brakeLightsDEPRECATED = bool(pt_cp.vl["ESP_05"]['ESP_Status_Bremsdruck'])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    elif trans_type == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["EV_Gearshift"]["GearPosition"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_14"]["MO_Kuppl_schalter"]
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    if not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
      ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    if not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR or self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
      _acc_type = 0 if self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY else ext_cp.vl["ACC_06"]["ACC_Typ"]
      self.acc_type = _acc_type

    # ACC okay but disabled (1), ACC ready (2), a radar visibility or other fault/disruption (6 or 7)
    # currently regulating speed (3), driver accel override (4), brake only (5)
    ret.cruiseState.available = pt_cp.vl["TSK_06"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled = pt_cp.vl["TSK_06"]["TSK_Status"] in (3, 4, 5)

    if self.CP.pcmCruise and not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      # Cruise Control mode; check for distance UI setting from the radar.
      # ECM does not manage this, so do not need to check for openpilot longitudinal
      ret.cruiseState.nonAdaptive = ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"] == 0
    else:
      # Speed limiter mode; ECM faults if we command ACC while not pcmCruise
      ret.cruiseState.nonAdaptive = bool(pt_cp.vl["TSK_06"]["TSK_Limiter_ausgewaehlt"])

    ret.accFaulted = pt_cp.vl["TSK_06"]["TSK_Status"] in (6, 7)

    self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise and not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker = ret.leftBlinkerOn = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = ret.rightBlinkerOn = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])
    self.button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0

    # Digital instrument clusters expect the ACC HUD lead car distance to be scaled differently
    self.upscale_lead_car_signal = bool(pt_cp.vl["Kombi_03"]["KBI_Variante"])

    self.frame += 1
    return ret

  def update_pq(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()

    self.prev_mads_enabled = self.mads_enabled

    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["Bremse_3"]["Radgeschw__VL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__VR_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HR_4_1"],
    )

    # vEgo obtained from Bremse_1 vehicle speed rather than Bremse_3 wheel speeds because Bremse_3 isn't present on NSF
    ret.vEgoRaw = pt_cp.vl["Bremse_1"]["Geschwindigkeit_neu__Bremse_1_"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]["Giergeschwindigkeit"] * (1, -1)[int(pt_cp.vl["Bremse_5"]["Vorzeichen_der_Giergeschwindigk"])] * CV.DEG_TO_RAD
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_3"]["Fahrpedal_Rohsignal"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["Bremse_5"]["Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremslichtschalter"])
    ret.parkingBrake = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])
    ret.brakeLightsDEPRECATED = bool(pt_cp.vl["Motor_2"]['Bremstestschalter'])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]["Kupplungsschalter"]
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_BT_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HL_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HR_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HD_Hauptraste"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_Status"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapters on Front Assist with Braking and City Emergency
    # Braking for the 2016 Passat NMS
    # TODO: deferred until we can collect data on pre-MY2016 behavior, AWV message may be shorter with fewer signals
    ret.stockFcw = False
    ret.stockAeb = False

    # Update ACC radar status.
    self.acc_type = ext_cp.vl["ACC_System"]["ACS_Typ_ACC"]
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["GRA_Hauptschalter"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["GRA_Status"] in (1, 2)
    if self.CP.pcmCruise:
      ret.accFaulted = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_StaACC"] in (6, 7)
    else:
      ret.accFaulted = pt_cp.vl["Motor_2"]["GRA_Status"] == 3

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker, ret.rightBlinker = ret.leftBlinkerOn, ret.rightBlinkerOn = self.update_blinker_from_stalk(300, pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"],
                                                                                                                      pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    self.button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_Neu"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    self.frame += 1
    return ret
    
  def update_meb(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    
    self.prev_mads_enabled = self.mads_enabled
    
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["MEB_ESP_01"]["VL_Radgeschw"],
      pt_cp.vl["MEB_ESP_01"]["VR_Radgeschw"],
      pt_cp.vl["MEB_ESP_01"]["HL_Radgeschw"],
      pt_cp.vl["MEB_ESP_01"]["HR_Radgeschw"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    #ret.steeringAngleDeg = pt_cp.vl["MEB_EPS_01"]["Steering_Angle"] * (1, -1)[int(pt_cp.vl["MEB_EPS_01"]["Steering_Angle_VZ"])] # wrong between 0 and 360 deg, also lwi_01
    ret.steeringAngleDeg = pt_cp.vl["LH_EPS_03"]["EPS_Berechneter_LW"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_BLW"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["MEB_ESP_04"]["Yaw_Rate"] * (1, -1)[int(pt_cp.vl["MEB_ESP_04"]["Yaw_Rate_Sign"])] * CV.DEG_TO_RAD

    # Update MEB HCA status
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["MEB_EPS_01"]["LatCon_HCA_Status"]) # HCA_03
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # VW Emergency Assist status tracking and mitigation
    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    #ret.carFaultedNonCritical =

    # Update gas, brakes, and gearshift.
    ret.gasPressed = pt_cp.vl["MEB_ESP_03"]["Accelerator_Pressure"] > 0
    ret.gas = pt_cp.vl["MEB_ESP_03"]["Accelerator_Pressure"]
    ret.brakePressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"]) # includes regen braking by user
    ret.brake = pt_cp.vl["MEB_ESP_01"]["Brake_Pressure"]
    ret.parkingBrake = pt_cp.vl["MEB_EPB_01"]["EPB_Status"] in (1, 4) # EPB closing or closed
    ret.brakeLightsDEPRECATED = bool(pt_cp.vl["MEB_ESP_04"]['Regen_Braking'])

    # Update gear and/or clutch position data.
    ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["ZV_02"]["ZV_FT_offen"],
                        pt_cp.vl["ZV_02"]["ZV_BT_offen"],
                        pt_cp.vl["ZV_02"]["ZV_HFS_offen"],
                        pt_cp.vl["ZV_02"]["ZV_HBFS_offen"],
                        pt_cp.vl["ZV_02"]["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Left"] > 0
      ret.rightBlindspot = ext_cp.vl["MEB_Side_Assist_01"]["Blind_Spot_Right"] > 0

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    if not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      ret.stockFcw = bool(pt_cp.vl["MEB_ESP_05"]["FCW_Active"])
      ret.stockAeb = bool(pt_cp.vl["MEB_ESP_05"]["AEB_Active"])

    # Update ACC radar status.
    if not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR or self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
      _acc_type = 0 if self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY else ext_cp.vl["MEB_ACC_02"]["ACC_Typ"]
      self.acc_type = _acc_type
    
    self.travel_assist_available = bool(ext_cp.vl["MEB_Travel_Assist_01"]["Travel_Assist_Available"])

    ret.cruiseState.available = pt_cp.vl["MEB_Motor_01"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled   = pt_cp.vl["MEB_Motor_01"]["TSK_Status"] in (3, 4, 5)

    if self.CP.pcmCruise and not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      # Cruise Control mode; check for distance UI setting from the radar.
      # ECM does not manage this, so do not need to check for openpilot longitudinal
      ret.cruiseState.nonAdaptive = bool(ext_cp.vl["MEB_ACC_01"]["ACC_Limiter_Mode"])
    else:
      # Speed limiter mode; ECM faults if we command ACC while not pcmCruise
      ret.cruiseState.nonAdaptive = bool(pt_cp.vl["MEB_Motor_01"]["TSK_Limiter_ausgewaehlt"])

    ret.accFaulted = pt_cp.vl["MEB_Motor_01"]["TSK_Status"] in (6, 7)

    self.esp_hold_confirmation = bool(pt_cp.vl["MEB_ESP_05"]["ESP_Hold"])
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise and not self.CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
      ret.cruiseState.speed = int(round(ext_cp.vl["MEB_ACC_01"]["ACC_Wunschgeschw_02"])) * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # speed limit detection
    ret.cruiseState.speedLimit = self.update_traffic_signals(pt_cp)

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker = ret.leftBlinkerOn = bool(pt_cp.vl["Blinkmodi_02"]["BM_links"])
    ret.rightBlinker = ret.rightBlinkerOn = bool(pt_cp.vl["Blinkmodi_02"]["BM_rechts"])
    self.button_events = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"]) # this is also true for ESC Sport mode
    ret.espActive = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])

    # EV battery charge WattHours
    ret.fuelGauge = pt_cp.vl["Motor_16"]["MO_Energieinhalt_BMS"]

    self.frame += 1
    return ret

  def update_traffic_signals(self, cp):
    if self.CP.flags & VolkswagenFlags.MEB:
      psd_06 = cp.vl["PSD_06"]
      if psd_06["PSD_06_Mux"] == 2: # multiplex signal speed limit attribute state
        if (self.v_limit_receive and # receiving allowed
            psd_06["PSD_Ges_Typ"] == 1 and # current plausible speed limit
            psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0): # detected non street type specific speed limit
              
          speed_limit_raw = psd_06["PSD_Ges_Geschwindigkeit"]
          if speed_limit_raw > 0 and speed_limit_raw < 11:
            self.v_limit = (speed_limit_raw - 1) * 5 # speed in steps of five from 0 to 45
          elif speed_limit_raw >= 11 and speed_limit_raw < 23:
            self.v_limit = 50 + (speed_limit_raw - 11) * 10 # speed in steps of ten from 50 to 160
          else:
            self.v_limit = 0
  
          self.v_limit = self.v_limit * self.v_limit_speed_factor
          self.v_limit_receive = False # speed limit has been received, wait for next receive permission
              
      elif psd_06["PSD_06_Mux"] == 0: # multiplex signal in init state
        v_limit_unit = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]
        self.v_limit_speed_factor = CV.MPH_TO_MS if v_limit_unit == 1 else CV.KPH_TO_MS if v_limit_unit == 0 else 0
        self.v_limit_receive = True if psd_06["PSD_Sys_Quali_Tempolimits"] == 7 else False # receive permission by quality "flag"

    return self.v_limit

  def update_hca_state(self, hca_status):
    # Treat INITIALIZING and FAULT as temporary for worst likely EPS recovery time, for cars without factory Lane Assist
    # DISABLED means the EPS hasn't been configured to support Lane Assist
    self.eps_init_complete = self.eps_init_complete or (hca_status in ("DISABLED", "READY", "ACTIVE") or self.frame > 600)
    perm_fault = hca_status == "DISABLED" or (self.eps_init_complete and hca_status in ("INITIALIZING", "FAULT"))
    temp_fault = hca_status in ("REJECTED", "PREEMPTED") or not self.eps_init_complete
    return temp_fault, perm_fault

  @staticmethod
  def get_can_parser(CP):
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_can_parser_pq(CP)
    elif CP.flags & VolkswagenFlags.MEB:
      return CarState.get_can_parser_meb(CP)

    messages = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_19", 100),      # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("ESP_21", 50),       # From J104 ABS/ESP controller
      ("Motor_20", 50),     # From J623 Engine control module
      ("TSK_06", 50),       # From J623 Engine control module
      ("ESP_02", 50),       # From J104 ABS/ESP controller
      ("GRA_ACC_01", 33),   # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
      ("Motor_14", 10),     # From J623 Engine control module
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("Kombi_03", 0),      # From J285 instrument cluster (not present on older cars, 1Hz when present)
    ]

    if CP.transmissionType == TransmissionType.automatic:
      messages.append(("Getriebe_11", 20))  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      messages.append(("EV_Gearshift", 10))  # From J??? unknown EV control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      if not CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
        messages += MqbExtraSignals.fwd_radar_messages
        if CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
          messages.remove(("ACC_06", 50))
      if CP.enableBsm:
        messages += MqbExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_cam_can_parser_pq(CP)
    elif CP.flags & VolkswagenFlags.MEB:
      return CarState.get_cam_can_parser_meb(CP)

    messages = []

    if CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      messages += [
        ("HCA_01", 1),  # From R242 Driver assistance camera, 50Hz if steering/1Hz if not
      ]

    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      if not CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
        messages += MqbExtraSignals.fwd_radar_messages
        if CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
          messages.remove(("ACC_06", 50))
      if CP.enableBsm:
        messages += MqbExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)

  @staticmethod
  def get_can_parser_pq(CP):
    messages = [
      # sig_address, frequency
      ("Bremse_1", 100),    # From J104 ABS/ESP controller
      ("Bremse_3", 100),    # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),  # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),  # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),     # From J623 Engine control module
      ("Airbag_1", 50),     # From J234 Airbag control module
      ("Bremse_5", 50),     # From J104 ABS/ESP controller
      ("GRA_Neu", 50),      # From J??? steering wheel control buttons
      ("Kombi_1", 50),      # From J285 Instrument cluster
      ("Motor_2", 50),      # From J623 Engine control module
      ("Motor_5", 50),      # From J623 Engine control module
      ("Lenkhilfe_2", 20),  # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),  # From J533 CAN gateway
    ]

    if CP.transmissionType == TransmissionType.automatic:
      messages += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.manual:
      messages += [("Motor_1", 100)]  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Extended CAN devices other than the camera are here on CANBUS.pt
      messages += PqExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += PqExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser_pq(CP):

    messages = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        # sig_address, frequency
        ("LDW_Status", 10)      # From R242 Driver assistance camera
      ]

    if CP.networkLocation == NetworkLocation.gateway:
      # Radars are here on CANBUS.cam
      messages += PqExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += PqExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)
    
  @staticmethod
  def get_can_parser_meb(CP):
    messages = [
      # sig_address, frequency
      ("LWI_01", 100),            # From J500 Steering Assist with integrated sensors
      ("GRA_ACC_01", 33),         # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Airbag_02", 5),           # From J234 Airbag control module
      ("Motor_14", 10),           # From J623 Engine control module
      ("Motor_16", 2),            # From J623 Engine control module
      ("Blinkmodi_02", 2),        # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("LH_EPS_03", 100),         # From J500 Steering Assist with integrated sensors
      ("Getriebe_11", 100),       # From J743 Auto transmission control module
      ("ZV_02", 5),               # From ZV
      ("MEB_EPS_01", 100),        #
      ("ESP_21", 50),             #
      ("MEB_ABS_01", 50),         #
      ("MEB_ESP_01", 100),        #
      ("MEB_ESP_03", 10),         #
      ("MEB_ESP_04", 50),         #
      ("MEB_ESP_05", 50),         #
      ("MEB_EPB_01", 20),         #
      ("MEB_Light_01", 5),        #
      ("MEB_Motor_01", 50),       #
      ("PSD_06", 7),              #
    ]

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      if not CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
        messages += MebExtraSignals.fwd_radar_messages
        if CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
          messages.remove(("MEB_ACC_02", 50))
      if CP.enableBsm:
        messages += MebExtraSignals.bsm_radar_messages
    
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser_meb(CP):
    messages = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      if not CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY_NO_RADAR:
        messages += MebExtraSignals.fwd_radar_messages
        if CP.spFlags & VolkswagenFlagsSP.SP_CC_ONLY:
          messages.remove(("MEB_ACC_02", 50))
      if CP.enableBsm:
        messages += MebExtraSignals.bsm_radar_messages
    
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)


class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_messages = [
    ("ACC_06", 50),                              # From J428 ACC radar control module
    ("ACC_10", 50),                              # From J428 ACC radar control module
    ("ACC_02", 17),                              # From J428 ACC radar control module
  ]
  bsm_radar_messages = [
    ("SWA_01", 20),                              # From J1086 Lane Change Assist
  ]

class PqExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_messages = [
    ("ACC_System", 50),                          # From J428 ACC radar control module
    ("ACC_GRA_Anzeige", 25),                     # From J428 ACC radar control module
  ]
  bsm_radar_messages = [
    ("SWA_1", 20),                               # From J1086 Lane Change Assist
  ]

class MebExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_messages = [
    ("MEB_ACC_01", 17),           #
    ("MEB_ACC_02", 50),           #
    ("MEB_Travel_Assist_01", 10), #
    #("MEB_Distance_01", 25),     #
  ]
  bsm_radar_messages = [
    ("MEB_Side_Assist_01", 20),
  ]
