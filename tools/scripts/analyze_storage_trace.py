#!/usr/bin/env python3
"""Summarize ext4 fsync and UFS command latencies from an ftrace text file."""

import argparse
import re
from collections import Counter, defaultdict
from pathlib import Path


LINE_RE = re.compile(r'^\s*(?P<comm>.*?)\-(?P<pid>\d+)\s+\[\d+\]\s+\S+\s+'
                     r'(?P<seconds>\d+\.\d+): (?P<event>\w+): (?P<detail>.*)$')
EXT4_RE = re.compile(r'\bdev (?P<dev>\d+,\d+) ino (?P<ino>\d+)\b')
UFS_RE = re.compile(r'^\S+:\s+(?P<phase>scsi_send|scsi_cmpl):\s+tag:\s*(?P<tag>\d+)\s+'
                    r'cmd:\s*(?P<cmd>0x[0-9a-fA-F]+)')


def top_latencies(rows: list[tuple[float, float, str]], count: int) -> None:
  if not rows:
    print('  no matching enter/exit pairs')
    return
  durations = sorted((end - start) * 1000 for start, end, _ in rows)
  print(f'  paired={len(rows)} p50_ms={durations[len(rows)//2]:.2f} '
        f'p95_ms={durations[int((len(rows)-1)*0.95)]:.2f} '
        f'p99_ms={durations[int((len(rows)-1)*0.99)]:.2f} max_ms={durations[-1]:.2f}')
  for start, end, detail in sorted(rows, key=lambda row: row[1] - row[0], reverse=True)[:count]:
    print(f'  start_s={start:.6f} end_s={end:.6f} duration_ms={(end-start)*1000:.2f} {detail}')


def analyze(path: Path, count: int) -> None:
  events = Counter()
  ext4_active: dict[tuple[str, str, str], list[float]] = defaultdict(list)
  ufs_active: dict[str, tuple[float, str]] = {}
  ext4_rows: list[tuple[float, float, str]] = []
  ufs_rows: list[tuple[float, float, str]] = []
  for line in path.open(errors='replace'):
    match = LINE_RE.match(line)
    if not match:
      continue
    event = match['event']
    detail = match['detail']
    seconds = float(match['seconds'])
    events[event] += 1
    if event in ('ext4_sync_file_enter', 'ext4_sync_file_exit'):
      ext4_match = EXT4_RE.search(detail)
      if ext4_match is None:
        continue
      key = match['pid'], ext4_match['dev'], ext4_match['ino']
      if event.endswith('_enter'):
        ext4_active[key].append(seconds)
      elif ext4_active[key]:
        start = ext4_active[key].pop()
        ext4_rows.append((start, seconds, f'pid={key[0]} dev={key[1]} ino={key[2]}'))
    elif event == 'ufshcd_command':
      ufs_match = UFS_RE.match(detail)
      if ufs_match is None:
        continue
      tag = ufs_match['tag']
      if ufs_match['phase'] == 'scsi_send':
        ufs_active[tag] = seconds, ufs_match['cmd']
      elif tag in ufs_active:
        start, cmd = ufs_active.pop(tag)
        ufs_rows.append((start, seconds, f'tag={tag} cmd={cmd}'))

  print(f'trace={path} parsed_events={sum(events.values())}')
  print('event_counts=' + ' '.join(f'{name}:{amount}' for name, amount in events.most_common()))
  print('ext4_sync_file:')
  top_latencies(ext4_rows, count)
  print('ufshcd_command:')
  top_latencies(ufs_rows, count)
  print(f'unpaired_ext4_enters={sum(map(len, ext4_active.values()))} unpaired_ufs_sends={len(ufs_active)}')


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('path', type=Path)
  parser.add_argument('--top', type=int, default=10)
  args = parser.parse_args()
  analyze(args.path, args.top)


if __name__ == '__main__':
  main()
