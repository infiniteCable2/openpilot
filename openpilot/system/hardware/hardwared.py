#!/usr/bin/env python3
import fcntl
import os
import queue
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import OrderedDict, namedtuple

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.utils import strip_deprecated_keys
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_HW
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.common.hardware import HARDWARE, COMMA_HARDWARE
from openpilot.common.basedir import BASEDIR
from openpilot.common.git import get_short_branch
from openpilot.common.hardware.usb import CHESTNUT_FW_VERSION, CHESTNUT_USB_PRODUCT, get_usb_state, get_usb_topology, is_chestnut_usb_id, set_usb_state
from openpilot.common.linux import LinuxSystemStats
from openpilot.system.loggerd.config import get_available_percent
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.system.statsd import statlog
from openpilot.system.hardware.power_monitoring import PowerMonitoring
from openpilot.system.hardware.fan_controller import FanController
from openpilot.system.hardware.chestnut.status import ChestnutStatus
from openpilot.common.version import terms_version, training_version, get_build_metadata, terms_version_sp

ThermalStatus = log.DeviceState.ThermalStatus
NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength
CURRENT_TAU = 15.   # 15s time constant
TEMP_TAU = 5.   # 5s time constant
DISCONNECT_TIMEOUT = 5.  # wait 5 seconds before going offroad after disconnect so you get an alert
PANDA_STATES_TIMEOUT = round(1000 / SERVICE_LIST['pandaStates'].frequency * 1.5)  # 1.5x the expected pandaState frequency
ONROAD_CYCLE_TIME = 1  # seconds to wait offroad after requesting an onroad cycle

class Chestnut:
  # flash offroad, modeld ignores chestnut until the product string matches
  MAX_ATTEMPTS = 3
  RETRY_INTERVAL = 20.

  def __init__(self):
    self.thread: threading.Thread | None = None
    self.attempts = 0
    self.last_attempt = 0.
    self.flashed = False
    self.mismatch = False

  @property
  def failed(self) -> bool:
    return self.mismatch and self.attempts >= self.MAX_ATTEMPTS and self.thread is not None and not self.thread.is_alive() and not self.flashed

  def flash(self) -> None:
    ret = subprocess.run(["sudo", sys.executable, os.path.join(BASEDIR, "openpilot/system/hardware/chestnut/flash.py"), CHESTNUT_FW_VERSION],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    cloudlog.event("chestnut flash done", returncode=ret.returncode, output=ret.stdout[-1000:], error=ret.returncode != 0)
    self.flashed = ret.returncode == 0

  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    self.mismatch = any(is_chestnut_usb_id(d["vendorId"], d["productId"], include_bootloader=True) and
                        d["product"] != CHESTNUT_USB_PRODUCT for d in usb_state)
    if not self.mismatch:
      self.flashed = False
      return

    if not offroad or self.flashed or self.attempts >= self.MAX_ATTEMPTS:
      return
    if self.thread is not None and self.thread.is_alive():
      return
    if time.monotonic() - self.last_attempt < self.RETRY_INTERVAL:
      return

    self.attempts += 1
    self.last_attempt = time.monotonic()
    cloudlog.warning(f"chestnut firmware out of date, flashing (attempt {self.attempts})")
    self.thread = threading.Thread(target=self.flash, daemon=True)
    self.thread.start()


ThermalBand = namedtuple("ThermalBand", ['min_temp', 'max_temp'])
HardwareState = namedtuple("HardwareState", ['network_type', 'network_info', 'network_strength', 'network_stats',
                                             'network_metered', 'modem_temps', 'usb_state'])

# List of thermal bands. We will stay within this region as long as we are within the bounds.
# When exiting the bounds, we'll jump to the lower or higher band. Bands are ordered in the dict.
if HARDWARE.get_device_type() == "mici":
  THERMAL_BANDS = OrderedDict({
    ThermalStatus.ok: ThermalBand(None, 100.0),
    ThermalStatus.overheated: ThermalBand(92.0, 107.),
    ThermalStatus.critical: ThermalBand(98.0, None),
  })
else:
  THERMAL_BANDS = OrderedDict({
    ThermalStatus.ok: ThermalBand(None, 96.0),
    ThermalStatus.overheated: ThermalBand(88.0, 107.),
    ThermalStatus.critical: ThermalBand(94.0, None),
  })

# Override to highest thermal band when offroad and above this temp
OFFROAD_DANGER_TEMP = 85 if HARDWARE.get_device_type() == "mici" else 75

prev_offroad_states: dict[str, tuple[bool, str | None]] = {}



def set_offroad_alert_if_changed(offroad_alert: str, show_alert: bool, extra_text: str | None=None):
  # Extra text only matters while the alert is visible. Changing temperatures
  # must not turn an unchanged hidden alert into another filesystem removal.
  state = (show_alert, extra_text if show_alert else None)
  if prev_offroad_states.get(offroad_alert, None) == state:
    return
  prev_offroad_states[offroad_alert] = state
  set_offroad_alert(offroad_alert, show_alert, extra_text)

def touch_thread(end_event):
  count = 0

  pm = messaging.PubMaster(["touch"])

  event_format = "llHHi"
  event_size = struct.calcsize(event_format)
  event_frame = []

  with open("/dev/input/by-path/platform-894000.i2c-event", "rb") as event_file:
    fcntl.fcntl(event_file, fcntl.F_SETFL, os.O_NONBLOCK)
    while not end_event.is_set():
      if (count % int(1. / DT_HW)) == 0:
        event = event_file.read(event_size)
        if event:
          (sec, usec, etype, code, value) = struct.unpack(event_format, event)
          if etype != 0 or code != 0 or value != 0:
            touch = log.Touch.new_message()
            touch.sec = sec
            touch.usec = usec
            touch.type = etype
            touch.code = code
            touch.value = value
            event_frame.append(touch)
          else: # end of frame, push new log
            msg = messaging.new_message('touch', len(event_frame), valid=True)
            msg.touch = event_frame
            pm.send('touch', msg)
            event_frame = []
          continue

      count += 1
      time.sleep(DT_HW)


def hw_state_thread(end_event, hw_queue):
  """Handles non critical hardware state, and sends over queue"""
  count = 0
  prev_hw_state = None
  prev_usb_topology = set()

  while not end_event.is_set():
    usb_topology = get_usb_topology()
    usb_changed = usb_topology != prev_usb_topology
    if usb_changed and count > 0:
      cloudlog.event('hardwared.usbTopologyChanged', mono_time_ns=time.monotonic_ns(),
                     added=sorted(usb_topology - prev_usb_topology), removed=sorted(prev_usb_topology - usb_topology))

    # these are expensive calls. update every 10s or when USB devices change
    if (count % int(10. / DT_HW)) == 0 or usb_changed:
      prev_usb_topology = usb_topology
      try:
        network_type = HARDWARE.get_network_type()
        modem_temps = HARDWARE.get_modem_temperatures()
        if len(modem_temps) == 0 and prev_hw_state is not None:
          modem_temps = prev_hw_state.modem_temps

        tx, rx = HARDWARE.get_modem_data_usage()

        hw_state = HardwareState(
          network_type=network_type,
          network_info=HARDWARE.get_network_info(),
          network_strength=HARDWARE.get_network_strength(network_type),
          network_stats={'wwanTx': tx, 'wwanRx': rx},
          network_metered=HARDWARE.get_network_metered(network_type),
          modem_temps=modem_temps,
          usb_state=get_usb_state(),
        )

        try:
          hw_queue.put_nowait(hw_state)
        except queue.Full:
          pass

        prev_hw_state = hw_state
      except Exception:
        cloudlog.exception("Error getting hardware state")

    count += 1
    time.sleep(DT_HW)


def slow_cycle_watchdog(end_event: threading.Event, probe: dict) -> None:
  """Capture the hardware thread's caller while a slow cycle is blocked."""
  reported: set[tuple[int, int]] = set()
  while not end_event.wait(0.1):
    start_ns = probe.get('start_ns', 0)
    now_ns = time.monotonic_ns()
    elapsed_ns = now_ns - start_ns if start_ns else 0
    threshold_ms = 2500 if elapsed_ns >= 2_500_000_000 else 750 if elapsed_ns >= 750_000_000 else 0
    key = (start_ns, threshold_ms)
    if not threshold_ms or key in reported or probe.get('start_ns') != start_ns:
      continue
    reported.add(key)
    if len(reported) > 32:
      reported = {key}
    frame = sys._current_frames().get(probe.get('thread_ident'))
    cloudlog.event('hardwared.slowCycleStack', mono_time_ns=now_ns, cycle_start_mono_time_ns=start_ns,
                   elapsed_ms=round(elapsed_ns / 1e6, 2), stage=probe.get('stage'),
                   stack=''.join(traceback.format_stack(frame, limit=16)) if frame is not None else 'unavailable')


def hardware_thread(end_event, hw_queue, probe: dict) -> None:
  system_stats = LinuxSystemStats()
  pm = messaging.PubMaster(['deviceState'])
  sm = messaging.SubMaster(["peripheralState", "gpsLocationExternal", "selfdriveState", "pandaStates", "chestnutState"], poll="pandaStates")

  count = 0

  onroad_conditions: dict[str, bool] = {
    "ignition": False,
    "not_onroad_cycle": True,
    "device_temp_good": True,
  }
  startup_conditions: dict[str, bool] = {}
  startup_conditions_prev: dict[str, bool] = {}

  off_ts: float | None = None
  started_ts: float | None = None
  started_seen = False
  startup_blocked_ts: float | None = None
  thermal_status = ThermalStatus.ok

  last_hw_state = HardwareState(
    network_type=NetworkType.none,
    network_info=None,
    network_metered=False,
    network_strength=NetworkStrength.unknown,
    network_stats={'wwanTx': -1, 'wwanRx': -1},
    modem_temps=[],
    usb_state=[],
  )

  all_temp_filter = FirstOrderFilter(0., TEMP_TAU, DT_HW, initialized=False)
  offroad_temp_filter = FirstOrderFilter(0., TEMP_TAU, DT_HW, initialized=False)
  should_start_prev = False
  in_car = False
  engaged_prev = False
  pwrsave = False
  offroad_cycle_count = 0

  params = Params()
  power_monitor = PowerMonitoring()

  uptime_offroad: float = params.get("UptimeOffroad", return_default=True)
  uptime_onroad: float = params.get("UptimeOnroad", return_default=True)
  last_uptime_ts: float = time.monotonic()

  HARDWARE.initialize_hardware()
  thermal_config = HARDWARE.get_thermal_config()

  fan_controller = FanController(int(1./DT_HW))
  chestnut = Chestnut()
  chestnut_status = ChestnutStatus()
  branch = get_short_branch()
  last_github_runner_voltage: bool | None = None
  last_network_metered: bool | None = None
  probe['thread_ident'] = threading.get_ident()

  while not end_event.is_set():
    cycle_start_ns = time.monotonic_ns()
    probe['stage'] = 'submaster.update'
    probe['start_ns'] = cycle_start_ns
    cycle_start_cpu_ns = time.thread_time_ns()
    sm.update(PANDA_STATES_TIMEOUT)
    sm_update_end_ns = time.monotonic_ns()

    pandaStates = sm['pandaStates']
    peripheralState = sm['peripheralState']

    # handle requests to cycle system started state
    if params.get_bool("OnroadCycleRequested"):
      params.put_bool("OnroadCycleRequested", False, block=True)
      offroad_cycle_count = sm.frame
    onroad_conditions["not_onroad_cycle"] = (sm.frame - offroad_cycle_count) >= ONROAD_CYCLE_TIME * SERVICE_LIST['pandaStates'].frequency

    if sm.updated['pandaStates'] and len(pandaStates) > 0:

      # Set ignition based on any panda connected
      onroad_conditions["ignition"] = any(ps.ignitionLine or ps.ignitionCan for ps in pandaStates if ps.pandaType != log.PandaState.PandaType.unknown)

      pandaState = pandaStates[0]

      in_car = pandaState.harnessStatus != log.PandaState.HarnessStatus.notConnected

    elif (time.monotonic() - sm.recv_time['pandaStates']) > DISCONNECT_TIMEOUT:
      if onroad_conditions["ignition"]:
        onroad_conditions["ignition"] = False
        cloudlog.error("panda timed out onroad")

    # Run at 2Hz, plus either edge of ignition
    ign_edge = (started_ts is not None) != all(onroad_conditions.values())
    if (sm.frame % round(SERVICE_LIST['pandaStates'].frequency * DT_HW) != 0) and not ign_edge:
      cycle_end_ns = time.monotonic_ns()
      if cycle_end_ns - cycle_start_ns > 500_000_000:
        cloudlog.event('hardwared.slowCycle', mono_time_ns=cycle_end_ns, published_device_state=False,
                       cycle_ms=round((cycle_end_ns - cycle_start_ns) / 1e6, 2),
                       submaster_ms=round((sm_update_end_ns - cycle_start_ns) / 1e6, 2),
                       pre_gate_ms=round((cycle_end_ns - sm_update_end_ns) / 1e6, 2),
                       thread_cpu_ms=round((time.thread_time_ns() - cycle_start_cpu_ns) / 1e6, 2))
      probe['start_ns'] = 0
      continue

    probe['stage'] = 'deviceState.stats'
    msg = messaging.new_message('deviceState', valid=True)
    msg.deviceState = thermal_config.get_msg()
    msg.deviceState.deviceType = HARDWARE.get_device_type()

    try:
      last_hw_state = hw_queue.get_nowait()
    except queue.Empty:
      pass

    msg.deviceState.freeSpacePercent = get_available_percent(default=100.0)
    msg.deviceState.memoryUsagePercent = int(round(system_stats.memory_usage_percent()))
    msg.deviceState.gpuUsagePercent = int(round(HARDWARE.get_gpu_usage_percent()))
    online_cpu_usage = [int(round(n)) for n in system_stats.cpu_usage_percent()]
    offline_cpu_usage = [0., ] * (len(msg.deviceState.cpuTempC) - len(online_cpu_usage))
    msg.deviceState.cpuUsagePercent = online_cpu_usage + offline_cpu_usage

    msg.deviceState.networkType = last_hw_state.network_type
    msg.deviceState.networkMetered = last_hw_state.network_metered
    msg.deviceState.networkStrength = last_hw_state.network_strength
    msg.deviceState.networkStats = last_hw_state.network_stats
    if last_hw_state.network_info is not None:
      msg.deviceState.networkInfo = last_hw_state.network_info

    msg.deviceState.modemTempC = last_hw_state.modem_temps

    msg.deviceState.screenBrightnessPercent = HARDWARE.get_screen_brightness()

    set_usb_state(msg.deviceState, last_hw_state.usb_state)
    chestnut.update(started_ts is None, last_hw_state.usb_state)
    chestnut_state = sm["chestnutState"]
    chestnut_valid = sm.alive["chestnutState"] and sm.valid["chestnutState"]
    chestnut_status.update(started_ts is None, branch, last_hw_state.usb_state, chestnut.failed,
                           params.get_bool("ChestnutLoading"), params.get("ChestnutActive"),
                           chestnut_state if chestnut_valid else None, set_offroad_alert_if_changed)
    stats_end_ns = time.monotonic_ns()
    probe['stage'] = 'deviceState.startupParams'
    # this subset is only used for offroad
    temp_sources = [
      msg.deviceState.memoryTempC,
      max(msg.deviceState.cpuTempC, default=0.),
      max(msg.deviceState.gpuTempC, default=0.),
    ]
    offroad_comp_temp = offroad_temp_filter.update(max(temp_sources))

    # this drives the thermal status while onroad
    temp_sources.append(max(msg.deviceState.pmicTempC, default=0.))
    all_comp_temp = all_temp_filter.update(max(temp_sources))
    msg.deviceState.maxTempC = all_comp_temp

    msg.deviceState.fanSpeedPercentDesired = fan_controller.update(all_comp_temp, onroad_conditions["ignition"])

    is_offroad_for_5_min = (started_ts is None) and ((not started_seen) or (off_ts is None) or (time.monotonic() - off_ts > 60 * 5))
    if is_offroad_for_5_min and offroad_comp_temp > OFFROAD_DANGER_TEMP:
      # if device is offroad and already hot without the extra onroad load,
      # we want to cool down first before increasing load
      thermal_status = ThermalStatus.critical
    else:
      current_band = THERMAL_BANDS[thermal_status]
      band_idx = list(THERMAL_BANDS.keys()).index(thermal_status)
      if current_band.min_temp is not None and all_comp_temp < current_band.min_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx - 1]
      elif current_band.max_temp is not None and all_comp_temp > current_band.max_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx + 1]

    # **** starting logic ****

    startup_conditions["up_to_date"] = params.get("Offroad_ConnectivityNeeded") is None or params.get_bool("DisableUpdates") or params.get_bool("SnoozeUpdate")
    startup_conditions["no_excessive_actuation"] = params.get("Offroad_ExcessiveActuation") is None
    startup_conditions["not_uninstalling"] = not params.get_bool("DoUninstall")
    startup_conditions["accepted_terms"] = params.get("HasAcceptedTerms") == terms_version
    startup_conditions["accepted_terms_sp"] = params.get("HasAcceptedTermsSP") == terms_version_sp

    # with 2% left, we killall, otherwise the phone will take a long time to boot
    startup_conditions["free_space"] = msg.deviceState.freeSpacePercent > 2
    startup_conditions["completed_training"] = params.get("CompletedTrainingVersion") == training_version
    startup_conditions["not_driver_view"] = not params.get_bool("IsDriverViewEnabled")

    # must be at an engageable thermal band to go onroad
    startup_conditions["device_temp_engageable"] = thermal_status < ThermalStatus.overheated

    # ensure device is fully booted
    startup_conditions["device_booted"] = startup_conditions.get("device_booted", False) or HARDWARE.booted()
    startup_params_end_ns = time.monotonic_ns()

    # user-forced status
    probe['stage'] = 'OffroadMode.get'
    offroad_mode = params.get_bool("OffroadMode")
    startup_conditions["not_always_offroad"] = not offroad_mode
    onroad_conditions["not_always_offroad"] = not offroad_mode
    offroad_mode_end_ns = time.monotonic_ns()

    # if an unsupported device and branch is detected, going onroad is blocked
    # only allow going onroad when:
    # - TIZI, or
    # - TICI and channel_type is "tici"
    probe['stage'] = 'get_build_metadata'
    build_metadata = get_build_metadata()
    build_metadata_end_ns = time.monotonic_ns()
    is_unsupported_combo = COMMA_HARDWARE and HARDWARE.get_device_type() == "tici" and build_metadata.channel_type != "tici"
    startup_conditions["not_tici"] = not is_unsupported_combo
    onroad_conditions["not_tici"] = not is_unsupported_combo
    probe['stage'] = 'Offroad_TiciSupport.set_offroad_alert'
    set_offroad_alert_if_changed("Offroad_TiciSupport", is_unsupported_combo, extra_text=build_metadata.channel)
    support_alert_end_ns = time.monotonic_ns()
    probe['stage'] = 'deviceState.startupAndPower'

    # if the temperature enters the danger zone, go offroad to cool down
    onroad_conditions["device_temp_good"] = thermal_status < ThermalStatus.critical
    extra_text = f"{offroad_comp_temp:.1f}C"
    show_alert = (not onroad_conditions["device_temp_good"] or not startup_conditions["device_temp_engageable"]) and onroad_conditions["ignition"]
    set_offroad_alert_if_changed("Offroad_TemperatureTooHigh", show_alert, extra_text=extra_text)
    temperature_alert_end_ns = time.monotonic_ns()

    if show_alert:
      msg.deviceState.fanSpeedPercentDesired = 100

    # Handle offroad/onroad transition
    should_start = all(onroad_conditions.values())
    if started_ts is None:
      should_start = should_start and all(startup_conditions.values())

    if should_start != should_start_prev or (count == 0):
      params.put_bool("IsEngaged", False, block=True)
      engaged_prev = False

    if sm.updated['selfdriveState']:
      engaged = sm['selfdriveState'].enabled
      if engaged != engaged_prev:
        params.put_bool("IsEngaged", engaged, block=True)
        engaged_prev = engaged

      try:
        with open('/dev/kmsg', 'w') as kmsg:
          kmsg.write(f"<3>[hardware] engaged: {engaged}\n")
      except Exception:
        pass
    engagement_end_ns = time.monotonic_ns()

    should_pwrsave = not onroad_conditions["ignition"] and msg.deviceState.screenBrightnessPercent < 1e-3
    if should_pwrsave != pwrsave or (count == 0):
      HARDWARE.set_power_save(should_pwrsave)
    pwrsave = should_pwrsave
    power_save_end_ns = time.monotonic_ns()

    if should_start:
      off_ts = None
      if started_ts is None:
        started_ts = time.monotonic()
        started_seen = True
        if startup_blocked_ts is not None:
          cloudlog.event("Startup after block", block_duration=(time.monotonic() - startup_blocked_ts),
                         startup_conditions=startup_conditions, onroad_conditions=onroad_conditions,
                         startup_conditions_prev=startup_conditions_prev, error=True)
      startup_blocked_ts = None
    else:
      if onroad_conditions["ignition"] and (startup_conditions != startup_conditions_prev):
        cloudlog.event("Startup blocked", startup_conditions=startup_conditions, onroad_conditions=onroad_conditions, error=True)
        startup_conditions_prev = startup_conditions.copy()
        startup_blocked_ts = time.monotonic()

      started_ts = None
      if off_ts is None:
        off_ts = time.monotonic()
    startup_end_ns = time.monotonic_ns()

    # Offroad power monitoring
    voltage = None if peripheralState.pandaType == log.PandaState.PandaType.unknown else peripheralState.voltage

    # GitHub runner auto off: 9V is used as the threshold because most desktop runners
    # will rarely exceed 5V so 9V is set as our buffer between desk use and car use.
    github_runner_voltage = bool((voltage or 0) and voltage > 9000)
    if github_runner_voltage != last_github_runner_voltage:
      params.put_bool("GithubRunnerSufficientVoltage", github_runner_voltage)
      last_github_runner_voltage = github_runner_voltage

    power_monitor.calculate(voltage, onroad_conditions["ignition"])
    msg.deviceState.offroadPowerUsageUwh = power_monitor.get_power_used()
    msg.deviceState.carBatteryCapacityUwh = max(0, power_monitor.get_car_battery_capacity())
    current_power_draw = HARDWARE.get_current_power_draw()
    statlog.sample("power_draw", current_power_draw)
    msg.deviceState.powerDrawW = current_power_draw

    som_power_draw = HARDWARE.get_som_power_draw()
    statlog.sample("som_power_draw", som_power_draw)
    msg.deviceState.somPowerDrawW = som_power_draw

    # Check if we need to shut down
    if power_monitor.should_shutdown(onroad_conditions["ignition"], in_car, off_ts, started_seen):
      cloudlog.warning(f"shutting device down, offroad since {off_ts}")
      params.put_bool("DoShutdown", True, block=True)

    msg.deviceState.started = started_ts is not None and not offroad_mode
    msg.deviceState.startedMonoTime = int(1e9*(started_ts or 0))

    last_ping = params.get("LastAthenaPingTime")
    if last_ping is not None:
      msg.deviceState.lastAthenaPingTime = last_ping

    msg.deviceState.thermalStatus = thermal_status
    probe['stage'] = 'deviceState.publish'
    pm.send("deviceState", msg)
    publish_end_ns = time.monotonic_ns()
    probe['stage'] = 'deviceState.postPublish'

    statlog.gauge("free_space_percent", msg.deviceState.freeSpacePercent)
    statlog.gauge("gpu_usage_percent", msg.deviceState.gpuUsagePercent)
    statlog.gauge("memory_usage_percent", msg.deviceState.memoryUsagePercent)
    for i, usage in enumerate(msg.deviceState.cpuUsagePercent):
      statlog.gauge(f"cpu{i}_usage_percent", usage)
    for i, temp in enumerate(msg.deviceState.cpuTempC):
      statlog.gauge(f"cpu{i}_temperature", temp)
    for i, temp in enumerate(msg.deviceState.gpuTempC):
      statlog.gauge(f"gpu{i}_temperature", temp)
    statlog.gauge("memory_temperature", msg.deviceState.memoryTempC)
    for i, temp in enumerate(msg.deviceState.pmicTempC):
      statlog.gauge(f"pmic{i}_temperature", temp)
    for i, temp in enumerate(last_hw_state.modem_temps):
      statlog.gauge(f"modem_temperature{i}", temp)
    statlog.gauge("fan_speed_percent_desired", msg.deviceState.fanSpeedPercentDesired)
    statlog.gauge("screen_brightness_percent", msg.deviceState.screenBrightnessPercent)

    # report to server once every 10 minutes, or every 1s when thermally blocked
    rising_edge_started = should_start and not should_start_prev
    status_packet_interval = 1. if show_alert else 600.
    probe['stage'] = 'deviceState.statusPacket'
    if rising_edge_started or (count % int(status_packet_interval / DT_HW)) == 0:
      dat = {
        'count': count,
        'pandaStates': [strip_deprecated_keys(p.to_dict()) for p in pandaStates],
        'peripheralState': strip_deprecated_keys(peripheralState.to_dict()),
        'location': (strip_deprecated_keys(sm["gpsLocationExternal"].to_dict()) if sm.alive["gpsLocationExternal"] else None),
        'deviceState': strip_deprecated_keys(msg.to_dict())
      }
      cloudlog.event("STATUS_PACKET", **dat)

      # save last one before going onroad
      if rising_edge_started:
        try:
          params.put("LastOffroadStatusPacket", dat, block=True)
        except Exception:
          cloudlog.exception("failed to save offroad status")
    status_packet_end_ns = time.monotonic_ns()

    probe['stage'] = 'NetworkMetered.put'
    if msg.deviceState.networkMetered != last_network_metered:
      params.put_bool("NetworkMetered", msg.deviceState.networkMetered)
      last_network_metered = msg.deviceState.networkMetered
    network_param_end_ns = time.monotonic_ns()

    now_ts = time.monotonic()
    if off_ts:
      uptime_offroad += now_ts - max(last_uptime_ts, off_ts)
    elif started_ts:
      uptime_onroad += now_ts - max(last_uptime_ts, started_ts)
    last_uptime_ts = now_ts

    if (count % int(60. / DT_HW)) == 0:
      probe['stage'] = 'UptimeOffroad.put'
      params.put("UptimeOffroad", uptime_offroad)
      uptime_offroad_end_ns = time.monotonic_ns()
      probe['stage'] = 'UptimeOnroad.put'
      params.put("UptimeOnroad", uptime_onroad)
      uptime_onroad_end_ns = time.monotonic_ns()
    else:
      uptime_offroad_end_ns = uptime_onroad_end_ns = network_param_end_ns
    uptime_param_end_ns = time.monotonic_ns()
    probe['stage'] = 'deviceState.cycleEnd'

    count += 1
    should_start_prev = should_start
    cycle_end_ns = time.monotonic_ns()
    if cycle_end_ns - cycle_start_ns > 500_000_000:
      cloudlog.event('hardwared.slowCycle', mono_time_ns=cycle_end_ns, published_device_state=True,
                     cycle_ms=round((cycle_end_ns - cycle_start_ns) / 1e6, 2),
                     submaster_ms=round((sm_update_end_ns - cycle_start_ns) / 1e6, 2),
                     stats_ms=round((stats_end_ns - sm_update_end_ns) / 1e6, 2),
                     startup_ms=round((startup_end_ns - stats_end_ns) / 1e6, 2),
                     startup_params_ms=round((startup_params_end_ns - stats_end_ns) / 1e6, 2),
                     support_alert_ms=round((support_alert_end_ns - startup_params_end_ns) / 1e6, 2),
                     offroad_mode_param_ms=round((offroad_mode_end_ns - startup_params_end_ns) / 1e6, 2),
                     build_metadata_ms=round((build_metadata_end_ns - offroad_mode_end_ns) / 1e6, 2),
                     support_alert_call_ms=round((support_alert_end_ns - build_metadata_end_ns) / 1e6, 2),
                     temperature_alert_ms=round((temperature_alert_end_ns - support_alert_end_ns) / 1e6, 2),
                     engagement_ms=round((engagement_end_ns - temperature_alert_end_ns) / 1e6, 2),
                     power_save_ms=round((power_save_end_ns - engagement_end_ns) / 1e6, 2),
                     transition_ms=round((startup_end_ns - power_save_end_ns) / 1e6, 2),
                     pre_publish_ms=round((publish_end_ns - startup_end_ns) / 1e6, 2),
                     post_publish_ms=round((cycle_end_ns - publish_end_ns) / 1e6, 2),
                     status_packet_ms=round((status_packet_end_ns - publish_end_ns) / 1e6, 2),
                     network_param_ms=round((network_param_end_ns - status_packet_end_ns) / 1e6, 2),
                     uptime_param_ms=round((uptime_param_end_ns - network_param_end_ns) / 1e6, 2),
                     uptime_offroad_put_ms=round((uptime_offroad_end_ns - network_param_end_ns) / 1e6, 2),
                     uptime_onroad_put_ms=round((uptime_onroad_end_ns - uptime_offroad_end_ns) / 1e6, 2),
                     thread_cpu_ms=round((time.thread_time_ns() - cycle_start_cpu_ns) / 1e6, 2))
    probe['start_ns'] = 0


def main():
  hw_queue = queue.Queue(maxsize=1)
  end_event = threading.Event()
  slow_cycle_probe: dict = {'start_ns': 0, 'stage': 'initializing', 'thread_ident': None}

  threads = [
    threading.Thread(target=hw_state_thread, args=(end_event, hw_queue)),
    threading.Thread(target=hardware_thread, args=(end_event, hw_queue, slow_cycle_probe)),
  ]
  watchdog = threading.Thread(target=slow_cycle_watchdog, args=(end_event, slow_cycle_probe), daemon=True)

  if COMMA_HARDWARE:
    threads.append(threading.Thread(target=touch_thread, args=(end_event,)))

  for t in threads:
    t.start()
  watchdog.start()

  try:
    while True:
      time.sleep(1)
      if not all(t.is_alive() for t in threads):
        break
  finally:
    end_event.set()

  for t in threads:
    t.join()
  watchdog.join(timeout=1)


if __name__ == "__main__":
  main()
