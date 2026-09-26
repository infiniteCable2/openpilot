#!/usr/bin/env python3
"""Inspect local on-device rlogs for personality changes and slow Params writes."""

import argparse
import json
import re
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader


PERSONALITY_EVENTS = {
  'selfdrived.personalityButton',
  'selfdrived.personalityParamChanged',
  'selfdrived.personalityWriteCompleted',
}
COMM_EVENTS = {'commIssue', 'commIssueRecovered', 'commIssueSuppressed'}
MONO_RE = re.compile(r'\bmono_time_ns=(\d+)')
TOTAL_MS_RE = re.compile(r'\btotal_ms=([\d.]+)')


def inspect(path: Path, start_ns: int | None, end_ns: int | None) -> None:
  segment = int(path.parent.name.rsplit('--', 1)[-1])
  count = 0
  for msg in LogReader(str(path)):
    kind = msg.which()
    if kind not in ('logMessage', 'errorLogMessage', 'selfdriveState'):
      continue
    event_ns = msg.logMonoTime
    payload = None
    if kind == 'selfdriveState':
      state = msg.selfdriveState
      payload = {'alert': str(state.alertType), 'personality': str(state.personality),
                 'state': str(state.state)}
      # Output all personality/alert transitions, not the 100-Hz stream.
      signature = (payload['alert'], payload['personality'], payload['state'])
      if signature == inspect.last_state:
        continue
      inspect.last_state = signature
    else:
      try:
        value = json.loads(str(getattr(msg, kind))).get('msg')
      except (ValueError, TypeError, AttributeError):
        continue
      if isinstance(value, dict) and value.get('event') in PERSONALITY_EVENTS | COMM_EVENTS:
        payload = value
        event_ns = value.get('mono_time_ns') or event_ns
      elif isinstance(value, str) and value.startswith('params.slowOp '):
        duration = TOTAL_MS_RE.search(value)
        if 'key=LongitudinalPersonality' not in value and (duration is None or float(duration[1]) < 1000):
          continue
        payload = value
        timestamp = MONO_RE.search(value)
        event_ns = int(timestamp[1]) if timestamp else event_ns
      else:
        continue
    if (start_ns is not None and event_ns < start_ns) or (end_ns is not None and event_ns > end_ns):
      continue
    print(json.dumps({'segment': segment, 'log_ns': msg.logMonoTime, 'event_ns': event_ns,
                      'type': kind, 'payload': payload}, default=str))
    count += 1
  print(f'SEGMENT {segment} selected={count}')


inspect.last_state = None


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('route', help='local route name, e.g. 000002b6--645aecde2a')
  parser.add_argument('--root', type=Path, default=Path('/data/media/0/realdata'))
  parser.add_argument('--start-ns', type=int)
  parser.add_argument('--end-ns', type=int)
  args = parser.parse_args()
  paths = sorted(args.root.glob(f'{args.route}--*/rlog.zst'),
                 key=lambda path: int(path.parent.name.rsplit('--', 1)[-1]))
  if not paths:
    parser.error(f'no local rlogs for {args.route} in {args.root}')
  for path in paths:
    inspect(path, args.start_ns, args.end_ns)


if __name__ == '__main__':
  main()
