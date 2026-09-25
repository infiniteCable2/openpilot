#!/usr/bin/env python3
"""Print route events and timing gaps relevant to inter-process comm issues."""
import argparse
import json
from collections import Counter, defaultdict

from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route


TRACKED = ('carControl', 'controlsState', 'controlsStateIC', 'controlsTiming', 'modelV2', 'longitudinalPlan',
           'driverAssistance', 'carState', 'carStateSP', 'carStateIC', 'carOutput', 'deviceState', 'managerState', 'pandaStates')
MIN_GAP_MS = {'carControl': 35, 'controlsState': 35, 'controlsStateIC': 35, 'controlsTiming': 35,
              'modelV2': 120, 'longitudinalPlan': 180, 'driverAssistance': 180,
              'carState': 35, 'carStateSP': 35, 'carStateIC': 35, 'carOutput': 35,
              'deviceState': 1200, 'managerState': 1200, 'pandaStates': 350}


def control_detail(timing) -> str:
  if not timing.vehicleModelEndMonoTime:
    return ''  # first logging revision did not have the finer markers
  times = (timing.updateEndMonoTime, timing.vehicleModelEndMonoTime, timing.longitudinalControlEndMonoTime,
           timing.lanefulEndMonoTime, timing.lateralControlEndMonoTime, timing.controlEndMonoTime)
  stages = [round((end-start)/1e6, 2) for start, end in zip(times, times[1:], strict=False)]
  return f' state_control_stage_ms={stages} thread_cpu_ms={timing.controlThreadCpuTimeNs/1e6:.2f}'


def inspect(url: str, segment: int, focus: float | None):
  rows = defaultdict(list)
  counts = Counter()
  events = []
  spi_log_times = []
  alert_prev = None
  start_ns = None
  for msg in LogReader(url):
    kind = msg.which()
    counts[kind] += 1
    t_ns = msg.logMonoTime
    if start_ns is None or t_ns < start_ns:
      start_ns = t_ns
    if kind in TRACKED:
      rows[kind].append((t_ns, msg.valid, msg))
    elif kind in ('logMessage', 'errorLogMessage'):
      try:
        payload = json.loads(str(getattr(msg, kind))).get('msg', '')
      except json.JSONDecodeError:
        continue
      if isinstance(payload, dict) and payload.get('event') in ('commIssue', 'commIssueRecovered', 'commIssueSuppressed',
                                                                 'plannerd.inputChecksFailed', 'plannerd.inputChecksRecovered',
                                                                 'hardwared.usbTopologyChanged', 'hardwared.slowCycle',
                                                                 'selfdrived.carStateMissing', 'selfdrived.carStateRecovered',
                                                                 'selfdrived.personalityButton', 'selfdrived.personalityParamChanged'):
        events.append((t_ns, payload))
      elif isinstance(payload, str) and 'SPI: got NACK' in payload:
        spi_log_times.append(t_ns)
    elif kind == 'selfdriveState':
      alert = str(msg.selfdriveState.alertType)
      if alert != alert_prev:
        if 'commIssue' in alert or (alert_prev and 'commIssue' in alert_prev):
          events.append((t_ns, f'selfdriveState alert={alert} state={msg.selfdriveState.state}'))
        alert_prev = alert

  print(f'Segment {segment}: {sum(counts.values())} messages, start_ns={start_ns}')
  print('Counts:', {k: counts[k] for k in TRACKED})
  spi_span = [(spi_log_times[0]-start_ns)/1e9, (spi_log_times[-1]-start_ns)/1e9] if spi_log_times else None
  print(f'SPI NACK logs: {len(spi_log_times)}; first/last: {spi_span}')
  last_event_time = {}
  for t_ns, text in events:
    if isinstance(text, dict):
      name = text['event']
      signature = (name, tuple(text.get('invalid', [])), tuple(text.get('not_alive', [])), tuple(text.get('not_freq_ok', [])))
      if signature in last_event_time and abs(t_ns - last_event_time[signature]) < 100_000_000:
        continue
      last_event_time[signature] = t_ns
      if name in ('commIssue', 'commIssueRecovered', 'commIssueSuppressed',
                  'plannerd.inputChecksFailed', 'plannerd.inputChecksRecovered'):
        detail = {s: {'recv_age_ms': d.get('last_recv_age_ms'), 'avg_hz': d.get('average_frequency_hz'),
                      'recent_hz': d.get('recent_frequency_hz')} for s, d in text.get('details', {}).items()}
        text = {k: text[k] for k in ('event', 'invalid', 'not_alive', 'not_freq_ok', 'duration_ms') if k in text}
        if detail:
          text['details'] = detail
      else:
        text = {k: v for k, v in text.items() if k not in ('ctx', 'thread', 'filename', 'lineno', 'funcname')}
    print(f'EVENT +{(t_ns-start_ns)/1e9:.3f}s {text}')
  for kind, seq in rows.items():
    seq.sort(key=lambda row: row[0])
    gaps = [(t0, t1, valid) for (t0, _, _), (t1, valid, _) in zip(seq, seq[1:], strict=False)
            if (t1-t0)/1e6 >= MIN_GAP_MS[kind]]
    for t0, t1, valid in sorted(gaps, key=lambda gap: gap[1]-gap[0], reverse=True)[:12]:
      gap_ms = (t1-t0)/1e6
      print(f'GAP {kind} +{(t0-start_ns)/1e9:.3f}s to +{(t1-start_ns)/1e9:.3f}s {gap_ms:.1f}ms next_valid={valid}')
    if kind in ('longitudinalPlan', 'driverAssistance', 'controlsState', 'carControl'):
      invalid = [t_ns for t_ns, valid, _ in seq if not valid]
      if invalid:
        first, last = (invalid[0]-start_ns)/1e9, (invalid[-1]-start_ns)/1e9
        print(f'INVALID {kind} count={len(invalid)} first=+{first:.3f}s last=+{last:.3f}s')
    if kind == 'controlsTiming':
      slow = []
      for t_ns, _, msg in seq:
        t = msg.controlsTiming
        phases = ((t.subMasterUpdateEndMonoTime-t.cycleStartMonoTime)/1e6,
                  (t.updateEndMonoTime-t.subMasterUpdateEndMonoTime)/1e6,
                  (t.controlEndMonoTime-t.updateEndMonoTime)/1e6,
                  (t.publishEndMonoTime-t.controlEndMonoTime)/1e6,
                  (t.extensionEndMonoTime-t.publishEndMonoTime)/1e6)
        if max(phases) >= 20:
          slow.append((t_ns, phases, t))
      for t_ns, phases, timing in sorted(slow, key=lambda row: max(row[1]), reverse=True)[:15]:
        print(f'SLOW controlsTiming +{(t_ns-start_ns)/1e9:.3f}s phase_ms={tuple(round(x, 2) for x in phases)}'
              + control_detail(timing))
    if kind == 'pandaStates':
      prior = None
      changes = []
      for t_ns, _, msg in seq:
        counts_now = [p.spiErrorCount for p in msg.pandaStates]
        if prior is not None and counts_now != prior:
          changes.append((t_ns, prior, counts_now))
        prior = counts_now
      print(f'SPI_COUNT changes={len(changes)} first={changes[0][1:] if changes else None} last={changes[-1][1:] if changes else None}')
      for t_ns, before, after in changes:
        if any(abs(t_ns - alert_ns) < 1e9 and (isinstance(event, dict) and event.get('event') == 'commIssue' or
                                              isinstance(event, str) and 'commIssue' in event) for alert_ns, event in events):
          print(f'SPI_COUNT_NEAR_EVENT +{(t_ns-start_ns)/1e9:.3f}s {before} -> {after}')

  if focus is not None:
    lo_ns, hi_ns = start_ns + int((focus - 0.35) * 1e9), start_ns + int((focus + 0.5) * 1e9)
    print(f'FOCUS +{focus:.3f}s window -0.35/+0.5s')
    print('SPI NACK times:', [round((t-start_ns)/1e9, 3) for t in spi_log_times if lo_ns <= t <= hi_ns])
    for kind in ('carControl', 'controlsState', 'controlsStateIC', 'carState', 'carOutput',
                 'modelV2', 'longitudinalPlan', 'driverAssistance', 'deviceState', 'pandaStates'):
      near = [(t, valid, msg) for t, valid, msg in rows[kind] if lo_ns <= t <= hi_ns]
      print(f'FOCUS {kind} count={len(near)} invalid={sum(not valid for _, valid, _ in near)}')
      if kind == 'carControl' and near:
        print(f'  latActive false={sum(not msg.carControl.latActive for _, _, msg in near)}')
        print('  latActive first/last:', [(round((t-start_ns)/1e9, 3), msg.carControl.latActive)
                                         for t, _, msg in (near[0], near[-1])])
        transitions = []
        previous_lat_active = near[0][2].carControl.latActive
        for t, _, msg in near[1:]:
          if msg.carControl.latActive != previous_lat_active:
            transitions.append((round((t-start_ns)/1e9, 3), msg.carControl.latActive))
          previous_lat_active = msg.carControl.latActive
        if transitions:
          print('  latActive transitions:', transitions)
      elif kind == 'controlsStateIC' and near:
        print(f'  lanefulActive true={sum(msg.controlsStateIC.lanefulActive for _, _, msg in near)}')
        print('  lanefulActive first/last:', [(round((t-start_ns)/1e9, 3), msg.controlsStateIC.lanefulActive)
                                             for t, _, msg in (near[0], near[-1])])
      elif kind == 'carState' and near:
        print('  canValid first/last:', [(round((t-start_ns)/1e9, 3), msg.carState.canValid)
                                        for t, _, msg in (near[0], near[-1])])
      elif kind == 'deviceState' and near:
        for t, _, msg in near:
          devices = [(hex(d.vendorId), hex(d.productId), d.linkErrorCount) for d in msg.deviceState.usbState.devices]
          print(f'  USB +{(t-start_ns)/1e9:.3f}s {devices}')
    for t, _, msg in rows['controlsTiming']:
      if lo_ns <= t <= hi_ns:
        timing = msg.controlsTiming
        phases = [(timing.subMasterUpdateEndMonoTime-timing.cycleStartMonoTime)/1e6,
                  (timing.updateEndMonoTime-timing.subMasterUpdateEndMonoTime)/1e6,
                  (timing.controlEndMonoTime-timing.updateEndMonoTime)/1e6,
                  (timing.publishEndMonoTime-timing.controlEndMonoTime)/1e6,
                  (timing.extensionEndMonoTime-timing.publishEndMonoTime)/1e6]
        if max(phases) > 10:
          print(f'FOCUS SLOW CYCLE +{(t-start_ns)/1e9:.3f}s phase_ms={[round(x, 2) for x in phases]}'
                + control_detail(timing))

  return {kind: (min(seq, key=lambda row: row[0])[0], max(seq, key=lambda row: row[0])[0])
          for kind, seq in rows.items() if seq}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('route', help='dongle_id/route_id')
  parser.add_argument('segments', nargs='*', type=int, default=[0])
  parser.add_argument('--focus', type=float, help='seconds relative to earliest message timestamp in segment')
  args = parser.parse_args()
  route = Route(args.route)
  previous = None
  for segment in args.segments:
    url = route.log_paths()[segment]
    if url is None:
      print(f'Segment {segment}: no rlog')
      previous = None
      continue
    current = inspect(url, segment, args.focus)
    if previous is not None and segment == previous[0] + 1:
      prev_segment, prev_bounds = previous
      for kind in TRACKED:
        if kind in prev_bounds and kind in current:
          gap_ms = (current[kind][0] - prev_bounds[kind][1]) / 1e6
          if gap_ms >= MIN_GAP_MS[kind]:
            print(f'CROSS_SEGMENT_GAP {kind} segment {prev_segment} to {segment}: {gap_ms:.1f}ms')
    previous = segment, current


if __name__ == '__main__':
  main()
