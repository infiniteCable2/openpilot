#!/usr/bin/env python3
"""Sample read-only storage counters on-device without writing to persistent storage.

Run this from the comma device. The JSONL output goes to /dev/shm (tmpfs) and
uses the same CLOCK_MONOTONIC time base as openpilot's mono_time_ns logs.
"""
import argparse
import json
import time
from pathlib import Path


DISK_FIELDS = ('reads', 'reads_merged', 'read_sectors', 'read_ms',
               'writes', 'writes_merged', 'write_sectors', 'write_ms',
               'in_flight', 'io_ms', 'weighted_io_ms')
MEMINFO_KEYS = {'Dirty', 'Writeback', 'WritebackTmp'}
EXT4_ROOT = Path('/sys/fs/ext4/sda12')
UFS_ROOT = Path('/sys/kernel/debug/1d84000.ufshc')


def read_text(path: Path) -> str | None:
  try:
    return path.read_text().strip()
  except (OSError, UnicodeError):
    return None


def read_diskstats(devices: set[str]) -> dict[str, dict[str, int]]:
  result = {}
  for line in Path('/proc/diskstats').read_text().splitlines():
    fields = line.split()
    if len(fields) >= 14 and fields[2] in devices:
      result[fields[2]] = dict(zip(DISK_FIELDS, map(int, fields[3:14]), strict=True))
  return result


def read_meminfo() -> dict[str, int]:
  result = {}
  for line in Path('/proc/meminfo').read_text().splitlines():
    key, _, value = line.partition(':')
    if key in MEMINFO_KEYS:
      result[key] = int(value.split()[0])
  return result


def snapshot(devices: set[str], include_ufs: bool) -> dict:
  row = {'mono_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(),
         'disk': read_diskstats(devices), 'mem_kb': read_meminfo(),
         'ext4_errors': read_text(EXT4_ROOT / 'errors_count'),
         'ext4_delayed_blocks': read_text(EXT4_ROOT / 'delayed_allocation_blocks')}
  if include_ufs:
    row['ufs_err_state'] = read_text(UFS_ROOT / 'err_state')
    row['ufs_req_stats'] = read_text(UFS_ROOT / 'stats/req_stats')
  return row


def sample(args: argparse.Namespace) -> None:
  if not 0.1 <= args.interval <= 5 or not 1 <= args.seconds <= 3600:
    raise ValueError('interval must be 0.1-5 s and duration 1-3600 s')
  output = Path(args.output or f'/dev/shm/openpilot-storage-{time.monotonic_ns()}.jsonl').resolve()
  if output.parent != Path('/dev/shm'):
    raise ValueError('output must be directly inside /dev/shm')

  devices = set(args.device or ['sda', 'sda12'])
  available = set(read_diskstats(devices))
  if not devices.issubset(available):
    raise ValueError(f'missing diskstats device: {devices - available}')

  start = time.monotonic()
  next_sample = start
  count = 0
  with output.open('x', buffering=1) as stream:
    stream.write(json.dumps({'schema': 1, 'devices': sorted(devices), 'interval_s': args.interval,
                             'boot_id': read_text(Path('/proc/sys/kernel/random/boot_id'))}) + '\n')
    while time.monotonic() - start < args.seconds:
      stream.write(json.dumps(snapshot(devices, include_ufs=(count % max(1, round(1 / args.interval)) == 0))) + '\n')
      count += 1
      next_sample += args.interval
      time.sleep(max(0, next_sample - time.monotonic()))
  print(f'{output} samples={count}')


def summarize(path: Path, device: str) -> None:
  rows = [json.loads(line) for line in path.read_text().splitlines()]
  samples = [row for row in rows if 'mono_ns' in row and device in row['disk']]
  if len(samples) < 2:
    raise ValueError(f'need at least two samples for {device}')
  intervals = []
  for before, after in zip(samples, samples[1:], strict=False):
    old, new = before['disk'][device], after['disk'][device]
    elapsed_ms = (after['mono_ns'] - before['mono_ns']) / 1e6
    intervals.append({'mono_ns': after['mono_ns'], 'elapsed_ms': round(elapsed_ms, 1),
                      'in_flight': new['in_flight'], 'writes': new['writes'] - old['writes'],
                      'write_ms': new['write_ms'] - old['write_ms'],
                      'io_ms': new['io_ms'] - old['io_ms'],
                      'weighted_io_ms': new['weighted_io_ms'] - old['weighted_io_ms'],
                      'dirty_kb': after['mem_kb'].get('Dirty'),
                      'writeback_kb': after['mem_kb'].get('Writeback')})
  print(f'{device}: {len(samples)} samples; mono_ns {samples[0]["mono_ns"]}..{samples[-1]["mono_ns"]}')
  total_writes = sum(row['writes'] for row in intervals)
  total_write_ms = sum(row['write_ms'] for row in intervals)
  max_in_flight = max(row['in_flight'] for row in intervals)
  print(f'total_writes={total_writes} total_write_ms={total_write_ms} max_in_flight={max_in_flight}')
  active = [row for row in intervals if row['weighted_io_ms'] or row['write_ms'] or row['in_flight']]
  if not active:
    print('no measurable queue latency in this sample')
  for row in sorted(active, key=lambda item: (item['weighted_io_ms'], item['write_ms']), reverse=True)[:10]:
    print(json.dumps(row))
  print(f'ext4_errors first={samples[0]["ext4_errors"]} last={samples[-1]["ext4_errors"]}')


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  subparsers = parser.add_subparsers(dest='command', required=True)
  sampler = subparsers.add_parser('sample')
  sampler.add_argument('--seconds', type=float, default=900)
  sampler.add_argument('--interval', type=float, default=0.2)
  sampler.add_argument('--device', action='append', help='repeat for each block device (default: sda and sda12)')
  sampler.add_argument('--output')
  summary = subparsers.add_parser('summary')
  summary.add_argument('path', type=Path)
  summary.add_argument('--device', default='sda')
  args = parser.parse_args()
  if args.command == 'sample':
    sample(args)
  else:
    summarize(args.path, args.device)


if __name__ == '__main__':
  main()
