#!/usr/bin/env python3
import argparse
from collections.abc import Iterable
from pathlib import Path

import numpy as np

from cereal import log
from openpilot.selfdrive.locationd.curvatured import CurvatureDLookup
from openpilot.tools.lib.logreader import LogReader, ReadMode


def speed_label(speed_idx: int) -> str:
  speed_kph = CurvatureDLookup.SPEED_ANCHORS[speed_idx] * 3.6
  return f"{speed_kph:.0f} km/h"


def curvature_bucket_label(curvature_idx: int) -> str:
  low = CurvatureDLookup.CURVATURE_BUCKET_EDGES[curvature_idx]
  high = CurvatureDLookup.CURVATURE_BUCKET_EDGES[curvature_idx + 1]
  return f"{low:.2e} .. {high:.2e}"


def iter_param_entries(init_data) -> Iterable:
  params = getattr(init_data, "params", None)
  if params is None:
    return []

  entries = getattr(params, "entries", None)
  if entries is None:
    return []

  return entries


def decode_cached_param(init_data):
  for entry in iter_param_entries(init_data):
    if getattr(entry, "key", None) != "LiveCurvatureParameters":
      continue

    raw_value = bytes(entry.value)
    result = {
      "byte_len": len(raw_value),
      "decoded": None,
      "error": None,
      "redacted": False,
    }

    if len(raw_value) == 0:
      result["redacted"] = True
      return result

    try:
      with log.Event.from_bytes(raw_value) as evt:
        result["decoded"] = evt.liveCurvatureParameters
    except Exception as e:
      result["error"] = repr(e)

    return result

  return None


def message_summary(msg) -> dict:
  payload = msg.liveCurvatureParameters
  return {
    "valid": bool(msg.valid),
    "live_valid": bool(payload.liveValid),
    "version": int(payload.version),
    "use_params": bool(payload.useParams),
    "cal_perc": int(payload.calPerc),
    "total_points": int(payload.totalBucketPoints),
    "bucket_speed": int(payload.bucketSpeed),
    "bucket_curvature": int(payload.bucketCurvature),
    "bucket_points": int(payload.currentBucketPoints),
    "current_bias": float(payload.currentBias),
    "current_correction": float(payload.currentCorrection),
  }


def print_bucket_details(counts: np.ndarray, biases: np.ndarray, corrections: np.ndarray,
                         fit_valid: np.ndarray, speed_idx: int, focus_idx: int | None = None) -> None:
  print("    buckets:")
  for curvature_idx in range(len(CurvatureDLookup.CURVATURE_BUCKET_CENTERS)):
    marker = "*" if focus_idx is not None and curvature_idx == focus_idx else " "
    bucket_range = curvature_bucket_label(curvature_idx)
    bucket_points = int(round(float(counts[speed_idx, curvature_idx])))
    bucket_bias = float(biases[speed_idx, curvature_idx])
    bucket_corr = float(corrections[speed_idx, curvature_idx])
    bucket_valid = bool(fit_valid[speed_idx, curvature_idx])
    print(
      f"    {marker} idx={curvature_idx} range={bucket_range} "
      f"points={bucket_points} fitValid={bucket_valid} "
      f"bias={bucket_bias:.8f} corr={bucket_corr:.8f}"
    )


def print_message_summary(title: str, payload) -> None:
  has_debug_arrays = len(list(payload.counts)) == CurvatureDLookup.total_size() and len(list(payload.biases)) == CurvatureDLookup.total_size()
  counts = CurvatureDLookup.unflatten_bucket(list(payload.counts)) if has_debug_arrays else np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
  biases = CurvatureDLookup.unflatten_bucket(list(payload.biases)) if has_debug_arrays else np.zeros(CurvatureDLookup.bucket_shape(), dtype=np.float32)
  corrections = CurvatureDLookup.unflatten_bucket(list(payload.corrections))
  fit_valid = CurvatureDLookup.unflatten_bucket(list(payload.fitValid), dtype=bool)

  print(title)
  print(f"  version: {int(payload.version)}")
  print(f"  liveValid: {bool(payload.liveValid)}")
  print(f"  useParams: {bool(payload.useParams)}")
  print(f"  calPerc: {int(payload.calPerc)}")
  print(f"  totalBucketPoints: {int(payload.totalBucketPoints)}")
  print(f"  currentCorrection: {float(payload.currentCorrection):.8f}")
  print(f"  currentBias: {float(payload.currentBias):.8f}")
  print(f"  currentBucket: ({int(payload.bucketSpeed)}, {int(payload.bucketCurvature)})")
  if not has_debug_arrays:
    print("  debugArrays: omitted")

  speed_entries = []
  for speed_idx in range(len(CurvatureDLookup.SPEED_ANCHORS)):
    speed_counts = counts[speed_idx]
    speed_valid = CurvatureDLookup.speed_curve_valid(counts, speed_idx) if has_debug_arrays else bool(np.any(fit_valid[speed_idx]))
    valid_bucket_count = int(np.count_nonzero(speed_counts >= CurvatureDLookup.MIN_BUCKET_POINTS)) if has_debug_arrays else int(np.count_nonzero(fit_valid[speed_idx]))
    total_points = int(round(float(speed_counts.sum()))) if has_debug_arrays else int(np.count_nonzero(fit_valid[speed_idx]))
    if total_points == 0 and not speed_valid:
      continue

    best_idx = int(np.argmax(speed_counts)) if total_points > 0 else 0
    best_points = int(round(float(speed_counts[best_idx])))
    best_bias = float(biases[speed_idx, best_idx])
    best_corr = float(corrections[speed_idx, best_idx])
    best_bucket = curvature_bucket_label(best_idx)
    fit_points = int(np.count_nonzero(fit_valid[speed_idx]))
    focus_idx = best_idx
    if int(payload.bucketSpeed) == speed_idx and int(payload.bucketCurvature) >= 0:
      focus_idx = int(payload.bucketCurvature)
    speed_entries.append((speed_idx, focus_idx,
      f"  {speed_label(speed_idx)}: valid={speed_valid} total={total_points} "
      f"validBuckets={valid_bucket_count} fitPoints={fit_points} "
      f"topBucket={best_idx} ({best_bucket}) points={best_points} "
      f"bias={best_bias:.8f} corr={best_corr:.8f}")
    )

  if speed_entries:
    print("  speed anchors:")
    for speed_idx, focus_idx, line in speed_entries:
      print(line)
      if has_debug_arrays:
        print_bucket_details(counts, biases, corrections, fit_valid, speed_idx, focus_idx)


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Dump liveCurvatureParameters from a route, segment, local log file, or URL using the local schema."
  )
  parser.add_argument(
    "route_or_segment_name",
    help="Route id, segment range, local rlog/qlog path, or comma URL accepted by LogReader",
  )
  parser.add_argument(
    "--segment",
    help="Optional segment selector to append to a bare route id, e.g. 0, -1, or 2:6",
  )
  parser.add_argument(
    "--mode",
    choices=[m.value for m in ReadMode],
    default=ReadMode.AUTO.value,
    help="LogReader mode: r=rlog, q=qlog, a=auto, i=auto interactive",
  )
  parser.add_argument("--limit", type=int, default=5, help="How many liveCurvatureParameters events to sample")
  args = parser.parse_args()

  target = args.route_or_segment_name.strip()
  is_local_path = Path(target).exists()
  if args.segment and not is_local_path and "://" not in target:
    selector = args.segment.strip().lstrip("/")
    if "/q" not in target and "/r" not in target and "/a" not in target and "/i" not in target:
      target = f"{target.rstrip('/')}/{selector}"

  lr = LogReader(target, default_mode=ReadMode(args.mode))

  init_data = None
  curvature_msgs = []
  valid_count = 0
  live_valid_count = 0

  for msg in lr:
    which = msg.which()
    if which == "initData" and init_data is None:
      init_data = msg.initData
    elif which == "liveCurvatureParameters":
      curvature_msgs.append(msg)
      valid_count += int(bool(msg.valid))
      live_valid_count += int(bool(msg.liveCurvatureParameters.liveValid))

  print(f"liveCurvatureParameters events: {len(curvature_msgs)}")
  if curvature_msgs:
    print(f"transport valid count: {valid_count}")
    print(f"payload liveValid count: {live_valid_count}")

    first = message_summary(curvature_msgs[0])
    last = message_summary(curvature_msgs[-1])
    print("first event:")
    for key, value in first.items():
      print(f"  {key}: {value}")
    print("last event:")
    for key, value in last.items():
      print(f"  {key}: {value}")

    sample_indices = np.linspace(0, len(curvature_msgs) - 1, min(args.limit, len(curvature_msgs)), dtype=int)
    printed = set()
    for idx in sample_indices:
      idx = int(idx)
      if idx in printed:
        continue
      printed.add(idx)
      print_message_summary(f"sample event #{idx}", curvature_msgs[idx].liveCurvatureParameters)
  else:
    print("No liveCurvatureParameters events found.")

  if init_data is None:
    print("No initData found.")
    return

  cached = decode_cached_param(init_data)
  if cached is None:
    print("initData: no LiveCurvatureParameters param entry found.")
    return

  print(f"initData cache bytes: {cached['byte_len']}")
  if cached["redacted"]:
    print("initData cache: redacted (DONT_LOG)")
  elif cached["decoded"] is not None:
    print_message_summary("decoded initData cache", cached["decoded"])
  else:
    print(f"initData cache decode error: {cached['error']}")


if __name__ == "__main__":
  main()
