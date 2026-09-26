#!/usr/bin/env python3
"""Capture storage diagnostics for each onroad session on comma four.

The manager keeps this process alive offroad. All onroad output stays in
tmpfs; completed sessions are copied to /data only after going offroad.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import openpilot.cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


TMP = Path('/dev/shm')
ARCHIVE = Path('/data/media/0/storage_traces')
TRACE_SCRIPT = Path(BASEDIR) / 'tools/scripts/capture_storage_trace.py'
SAMPLE_SCRIPT = Path(BASEDIR) / 'tools/scripts/sample_storage_io.py'
MIN_FREE_BYTES = 1024 ** 3


def fsync_directory(path: Path) -> None:
  fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def copy_durable(source: Path, target: Path) -> None:
  with source.open('rb') as inp, target.open('xb') as out:
    shutil.copyfileobj(inp, out, length=1024 * 1024)
    out.flush()
    os.fsync(out.fileno())


def kernel_messages() -> str:
  result = subprocess.run(['dmesg'], capture_output=True, text=True, timeout=15, check=True)
  return result.stdout


class Session:
  def __init__(self) -> None:
    self.start_ns = time.monotonic_ns()
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    self.name = f'openpilot-storage-{boot_id[:8]}-{self.start_ns}'
    self.processes: dict[str, subprocess.Popen] = {}
    self.files = [TMP / f'{self.name}.trace', TMP / f'{self.name}.json',
                  TMP / f'{self.name}.jsonl']
    self.logs = [TMP / f'{self.name}-{kind}.log' for kind in ('trace', 'sampler')]
    commands = {
      'trace': [sys.executable, str(TRACE_SCRIPT), '--seconds', '0', '--buffer-kb', '8192',
                '--basename', self.name, '--watch-pid', str(os.getpid())],
      'sampler': [sys.executable, str(SAMPLE_SCRIPT), 'sample', '--seconds', '0',
                  '--interval', '0.2', '--output', str(self.files[2]), '--watch-pid', str(os.getpid())],
    }
    try:
      for (kind, command), log_path in zip(commands.items(), self.logs, strict=True):
        with log_path.open('xb') as output:
          self.processes[kind] = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                                 stdout=output, stderr=subprocess.STDOUT,
                                                 start_new_session=True)
    except Exception:
      for process in self.processes.values():
        if process.poll() is None:
          os.killpg(process.pid, signal.SIGTERM)
          process.wait(timeout=10)
      raise
    cloudlog.info(f'storage trace started: {self.name}')

  def finish(self, reason: str, persist: bool) -> None:
    stop_ns = time.monotonic_ns()
    for process in self.processes.values():
      if process.poll() is None:
        try:
          os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
          pass
    exit_codes = {}
    for kind, process in self.processes.items():
      try:
        exit_codes[kind] = process.wait(timeout=45)
      except subprocess.TimeoutExpired:
        cloudlog.error(f'storage trace {kind} did not stop: pid={process.pid}')
        try:
          os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
          pass
        process.wait(timeout=5)
        exit_codes[kind] = None
    if not persist:
      cloudlog.warning(f'storage trace left in tmpfs after {reason}: {self.name}')
      return

    try:
      ARCHIVE.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
      cloudlog.error(f'storage trace archive unavailable: {exc}; tmpfs={self.name}')
      return
    sources = [path for path in self.files + self.logs if path.is_file()]
    required_bytes = sum(path.stat().st_size for path in sources)
    if shutil.disk_usage(ARCHIVE).free < required_bytes + MIN_FREE_BYTES:
      cloudlog.error(f'storage trace not saved: insufficient /data space; tmpfs={self.name}')
      return

    staging = ARCHIVE / f'{self.name}.partial'
    final = ARCHIVE / self.name
    try:
      staging.mkdir()
      for source in sources:
        copy_durable(source, staging / source.name)
      try:
        kernel_log = kernel_messages()
        with (staging / 'kernel.log').open('x') as output:
          output.write(kernel_log)
          output.flush()
          os.fsync(output.fileno())
      except (OSError, subprocess.SubprocessError) as exc:
        cloudlog.warning(f'storage trace kernel log unavailable: {exc}')
      manifest = {'name': self.name, 'route': Params().get('CurrentRoute'),
                  'start_mono_ns': self.start_ns,
                  'stop_mono_ns': stop_ns, 'reason': reason, 'exit_codes': exit_codes,
                  'files': [path.name for path in sources]}
      with (staging / 'manifest.json').open('x') as output:
        json.dump(manifest, output, indent=2)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())
      fsync_directory(staging)
      staging.rename(final)
      fsync_directory(ARCHIVE)
    except Exception:
      cloudlog.exception(f'storage trace save failed; tmpfs={self.name} staging={staging}')
      return
    for source in sources:
      try:
        source.unlink(missing_ok=True)
      except OSError as exc:
        cloudlog.warning(f'storage trace tmpfs cleanup failed: {source}: {exc}')
    cloudlog.info(f'storage trace saved: {final}')


def main() -> None:
  if os.geteuid() != 0:
    raise PermissionError('storage trace supervisor must run as root')
  sm = messaging.SubMaster(['deviceState'], poll='deviceState')
  session: Session | None = None
  started_prev = False
  stopping = False

  def request_stop(_signal: int, _frame) -> None:
    nonlocal stopping
    stopping = True

  signal.signal(signal.SIGINT, request_stop)
  signal.signal(signal.SIGTERM, request_stop)
  cloudlog.info('storage trace supervisor ready')
  try:
    while not stopping:
      sm.update(1000)
      if not sm.updated['deviceState']:
        continue
      started = sm['deviceState'].started
      if started and not started_prev:
        try:
          session = Session()
        except Exception:
          cloudlog.exception('storage trace could not start')
      elif not started and started_prev and session is not None:
        session.finish('offroad', persist=True)
        session = None
      started_prev = started
  finally:
    if session is not None:
      session.finish('supervisor_exit', persist=False)


if __name__ == '__main__':
  main()
