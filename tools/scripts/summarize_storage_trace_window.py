#!/usr/bin/env python3
"""Count storage trace events per second around an observed stall."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path

from analyze_storage_trace import LINE_RE


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('trace', type=Path)
  parser.add_argument('--start-s', type=int, required=True)
  parser.add_argument('--end-s', type=int, required=True)
  args = parser.parse_args()
  buckets = defaultdict(Counter)
  for line in args.trace.open(errors='replace'):
    match = LINE_RE.match(line)
    if match is None:
      continue
    t = float(match['seconds'])
    sec = int(t)
    if not args.start_s <= sec <= args.end_s:
      continue
    event = match['event']
    if event == 'ufshcd_command':
      event += ':' + match['detail'].split(':')[1].strip().split()[0]
    buckets[sec][event] += 1

  names = ('ext4_sync_file_enter', 'ext4_sync_file_exit',
           'block_rq_insert', 'block_rq_issue', 'block_rq_complete',
           'block_rq_requeue', 'block_rq_abort',
           'ufshcd_command:scsi_send', 'ufshcd_command:scsi_cmpl',
           'scsi_dispatch_cmd_timeout')
  print('second ' + ' '.join(names))
  for sec in range(args.start_s, args.end_s + 1):
    print(str(sec) + ' ' + ' '.join(str(buckets[sec][name]) for name in names))


if __name__ == '__main__':
  main()
