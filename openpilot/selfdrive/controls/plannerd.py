#!/usr/bin/env python3
import time

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
import openpilot.cereal.messaging as messaging


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  cloudlog.info("plannerd is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("plannerd got CarParamsSP")

  gps_location_service = get_gps_location_service(params)
  ignore_services = ["liveMapDataSP", "carStateSP", "selfdriveStateSP", gps_location_service]

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP, CP_SP)
  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'longitudinalPlanSP', 'longitudinalPlanIC'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState', 'modelV2', 'selfdriveState',
                            'liveMapDataSP', 'carStateSP', 'selfdriveStateSP', gps_location_service],
                           poll='modelV2', ignore_alive=ignore_services, ignore_avg_freq=ignore_services, ignore_valid=ignore_services)
  failed_checks_prev = None
  failed_checks_start_ns = None

  while True:
    sm.update()
    longitudinal_planner.sla.update_buttons(sm['selfdriveStateSP'].buttonsReleaseToggle)
    if sm.updated['modelV2']:
      if not sm.all_checks():
        failures = sm.failed_checks()
        signature = {key: failures[key] for key in ('invalid', 'not_alive', 'not_freq_ok')}
        if failed_checks_start_ns is None:
          failed_checks_start_ns = time.monotonic_ns()
        if signature != failed_checks_prev:
          cloudlog.event('plannerd.inputChecksFailed', mono_time_ns=time.monotonic_ns(), error=True,
                         model_mono_time_ns=sm.logMonoTime['modelV2'], **failures)
          failed_checks_prev = signature
      elif failed_checks_start_ns is not None:
        cloudlog.event('plannerd.inputChecksRecovered', mono_time_ns=time.monotonic_ns(),
                       duration_ms=round((time.monotonic_ns() - failed_checks_start_ns) / 1e6, 1))
        failed_checks_start_ns = None
        failed_checks_prev = None

      longitudinal_planner.update(sm)
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks()
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
