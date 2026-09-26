#!/usr/bin/env python3
"""Capture a bounded block/ext4/UFS trace in a private tracefs instance.

Run as root on a comma device. Output stays in /dev/shm and must be copied
before reboot. No persistent storage writes are made by this script.
"""

import argparse
import json
import os
import shutil
import signal
import time
from pathlib import Path


TRACEFS = Path('/sys/kernel/tracing')
OUTPUT_DIR = Path('/dev/shm')
INSTANCE_NAME = 'codex_storage'
REQUIRED_EVENTS = (
  'ext4:ext4_sync_file_enter',
  'ext4:ext4_sync_file_exit',
  'block:block_rq_issue',
  'block:block_rq_complete',
)
OPTIONAL_EVENTS = (
  'block:block_rq_insert',
  'block:block_rq_requeue',
  'block:block_rq_abort',
  'scsi:scsi_dispatch_cmd_start',
  'scsi:scsi_dispatch_cmd_done',
  'scsi:scsi_dispatch_cmd_error',
  'scsi:scsi_dispatch_cmd_timeout',
  'scsi:scsi_eh_wakeup',
  'ufs:ufshcd_command',
  'ufs:ufshcd_clk_gating',
  'ufs:ufshcd_hibern8_on_idle',
  'ufs:ufshcd_runtime_resume',
  'ufs:ufshcd_runtime_suspend',
  'ufs:ufshcd_auto_bkops_state',
  'ufs:ufshcd_profile_clk_gating',
  'ufs:ufshcd_profile_hibern8',
  'ufs:ufshcd_profile_clk_scaling',
  'ufs:ufshcd_clk_scaling',
)


def write_control(path: Path, value: str) -> None:
  with path.open('w') as stream:
    stream.write(value + '\n')


def read_optional(path: Path) -> str | None:
  try:
    return path.read_text().strip()
  except OSError:
    return None


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--seconds', type=int, default=180)
  parser.add_argument('--buffer-kb', type=int, default=8192,
                      help='ring buffer KiB per CPU (default: 8192)')
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('run with sudo')
  if not 1 <= args.seconds <= 1200 or not 1024 <= args.buffer_kb <= 16384:
    parser.error('seconds must be 1-1200 and buffer-kb 1024-16384')
  if not (TRACEFS / 'available_events').is_file():
    parser.error('tracefs is not mounted at /sys/kernel/tracing')

  available = set((TRACEFS / 'available_events').read_text().splitlines())
  missing = set(REQUIRED_EVENTS) - available
  if missing:
    parser.error(f'missing required tracepoints: {sorted(missing)}')
  selected = [event for event in REQUIRED_EVENTS + OPTIONAL_EVENTS if event in available]
  instance = TRACEFS / 'instances' / INSTANCE_NAME
  if instance.exists():
    parser.error(f'{instance} already exists; refusing to alter another capture')

  boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  basename = f'openpilot-storage-trace-{boot_id[:8]}-{time.monotonic_ns()}'
  trace_path = OUTPUT_DIR / f'{basename}.trace'
  metadata_path = OUTPUT_DIR / f'{basename}.json'
  stop_requested = False

  def stop_capture(_signal: int, _frame) -> None:
    nonlocal stop_requested
    stop_requested = True

  signal.signal(signal.SIGTERM, stop_capture)
  signal.signal(signal.SIGINT, stop_capture)
  instance.mkdir()
  started_ns = None
  stopped_ns = None
  try:
    write_control(instance / 'tracing_on', '0')
    write_control(instance / 'current_tracer', 'nop')
    write_control(instance / 'trace_clock', 'mono')
    write_control(instance / 'buffer_size_kb', str(args.buffer_kb))
    if (instance / 'options/record-cmd').exists():
      write_control(instance / 'options/record-cmd', '1')
    write_control(instance / 'events/enable', '0')
    for event in selected:
      group, name = event.split(':', 1)
      write_control(instance / 'events' / group / name / 'enable', '1')

    started_ns = time.monotonic_ns()
    write_control(instance / 'tracing_on', '1')
    write_control(instance / 'trace_marker', f'codex_storage_start mono_ns={started_ns}')
    print(f'TRACE_STARTED mono_ns={started_ns} seconds={args.seconds} events={len(selected)}', flush=True)
    deadline = time.monotonic() + args.seconds
    while not stop_requested and time.monotonic() < deadline:
      time.sleep(min(0.25, max(0, deadline - time.monotonic())))
  finally:
    try:
      if started_ns is not None:
        try:
          write_control(instance / 'trace_marker', f'codex_storage_stop mono_ns={time.monotonic_ns()}')
        except OSError:
          pass
      write_control(instance / 'tracing_on', '0')
      stopped_ns = time.monotonic_ns()
      per_cpu_stats = {path.parent.name: read_optional(path)
                       for path in (instance / 'per_cpu').glob('cpu*/stats')}
      with (instance / 'trace').open('rb') as source, trace_path.open('xb') as target:
        shutil.copyfileobj(source, target, length=1024 * 1024)
      metadata = {
        'boot_id': boot_id,
        'start_mono_ns': started_ns,
        'stop_mono_ns': stopped_ns,
        'trace_clock': read_optional(instance / 'trace_clock'),
        'buffer_size_kb_per_cpu': read_optional(instance / 'buffer_size_kb'),
        'events': selected,
        'per_cpu_stats': per_cpu_stats,
        'trace_path': str(trace_path),
        'trace_bytes': trace_path.stat().st_size,
      }
      with metadata_path.open('x') as target:
        json.dump(metadata, target, indent=2)
        target.write('\n')
      print(f'TRACE_SAVED path={trace_path} bytes={metadata["trace_bytes"]} '
            f'metadata={metadata_path} stop_mono_ns={stopped_ns}', flush=True)
    finally:
      try:
        write_control(instance / 'events/enable', '0')
        instance.rmdir()
      except OSError as exc:
        print(f'TRACE_CLEANUP_WARNING instance={instance} error={exc}', flush=True)


if __name__ == '__main__':
  main()
