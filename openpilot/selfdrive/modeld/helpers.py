import sys
import threading
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not chestnut_present():
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

def load_driving_models(make_model, use_chestnut: bool, timeout: float, params):
  big_model = model = None
  if use_chestnut:
    def load_big():
      nonlocal big_model
      try:
        wait_for_chestnut()
        candidate = make_model(True)
        candidate.warmup()
        big_model = candidate
      except Exception:
        cloudlog.exception("chestnut load failed")

    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(timeout)
    model = None if loader.is_alive() else big_model
    if model is None:
      cloudlog.warning("big model did not load, using small model without cycling onroad")

  # Keep the small model available for an in-process fallback during inference too.
  small_model = make_model(False)
  if use_chestnut:
    params.put_bool("ChestnutActive", model is not None)
  params.put_bool("ChestnutLoading", False)
  return (small_model if model is None else model), small_model


def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  return path.is_file() and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))
