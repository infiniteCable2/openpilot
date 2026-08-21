import importlib.util
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import call, MagicMock, patch

import pytest


def stub_module(name, **attributes):
  module = ModuleType(name)
  for attribute, value in attributes.items():
    setattr(module, attribute, value)
  sys.modules[name] = module


stub_module("openpilot.common.basedir", BASEDIR="/tmp")
stub_module("openpilot.common.params", Params=MagicMock())
stub_module("openpilot.common.hardware", HARDWARE=MagicMock())
stub_module("openpilot.common.swaglog", cloudlog=MagicMock())
stub_module("openpilot.sunnypilot.selfdrive.pandad.rivian_long_flasher", flash_rivian_long=MagicMock())

MODULE_NAME = "pandad_update_under_test"
spec = importlib.util.spec_from_file_location(MODULE_NAME, Path(__file__).parents[1] / "pandad.py")
assert spec is not None and spec.loader is not None
pandad = importlib.util.module_from_spec(spec)
sys.modules[MODULE_NAME] = pandad
spec.loader.exec_module(pandad)
flash_panda = pandad.flash_panda
prepare_internal_panda = pandad.prepare_internal_panda


SERIAL = "00112233445566778899aabb"
EXPECTED_SIGNATURE = b"expected firmware signature"


def configure_panda(panda_cls, spi_protocol_version=0x83, spi_protocol_namespace=b"ICSP"):
  panda_cls.SUPPORTED_DEVICES = (b"\x09",)
  panda_cls.SPI_PROTOCOL_VERSION = 0x83
  panda_cls.SPI_PROTOCOL_NAMESPACE = b"ICSP"

  old_panda = MagicMock()
  old_panda.get_type.return_value = b"\x09"
  old_panda.is_internal.return_value = True
  old_panda.bootstub = False
  old_panda.get_version.return_value = "old"
  old_panda.get_signature.return_value = b"old firmware signature"

  new_panda = MagicMock()
  new_panda.bootstub = False
  new_panda.get_signature.return_value = EXPECTED_SIGNATURE
  new_panda.is_connected_spi.return_value = True
  new_panda.get_spi_protocol_version.return_value = spi_protocol_version
  new_panda.get_spi_protocol_namespace.return_value = spi_protocol_namespace

  panda_cls.side_effect = (old_panda, new_panda)
  return old_panda, new_panda


@patch(f"{MODULE_NAME}.get_expected_signature", return_value=EXPECTED_SIGNATURE)
@patch(f"{MODULE_NAME}.Panda")
def test_flash_uses_migration_transport_and_verifies_private_protocol(panda_cls, _expected_signature):
  pandad.HARDWARE.reset_mock()
  old_panda, new_panda = configure_panda(panda_cls)

  flash_panda(SERIAL)

  assert panda_cls.call_args_list == [call(SERIAL, allow_legacy_spi=True), call(SERIAL, allow_legacy_spi=True)]
  old_panda.flash.assert_called_once_with(reconnect=False)
  old_panda.close.assert_called_once_with()
  pandad.HARDWARE.reset_internal_panda.assert_called_once_with()
  new_panda.get_spi_protocol_version.assert_called_once_with()
  new_panda.get_spi_protocol_namespace.assert_called_once_with()
  new_panda.close.assert_called_once_with()


@patch(f"{MODULE_NAME}.get_expected_signature", return_value=EXPECTED_SIGNATURE)
@patch(f"{MODULE_NAME}.Panda")
def test_flash_rejects_legacy_protocol_after_update(panda_cls, _expected_signature):
  pandad.HARDWARE.reset_mock()
  old_panda, new_panda = configure_panda(panda_cls, spi_protocol_version=2)

  with pytest.raises(AssertionError):
    flash_panda(SERIAL)

  old_panda.flash.assert_called_once_with(reconnect=False)
  old_panda.get_signature.assert_called_once_with()
  new_panda.get_signature.assert_not_called()
  pandad.HARDWARE.reset_internal_panda.assert_called_once_with()


@patch(f"{MODULE_NAME}.get_expected_signature", return_value=EXPECTED_SIGNATURE)
@patch(f"{MODULE_NAME}.Panda")
def test_flash_rejects_wrong_namespace_after_update(panda_cls, _expected_signature):
  pandad.HARDWARE.reset_mock()
  old_panda, new_panda = configure_panda(panda_cls, spi_protocol_namespace=b"UPST")

  with pytest.raises(AssertionError):
    flash_panda(SERIAL)

  old_panda.flash.assert_called_once_with(reconnect=False)
  new_panda.get_spi_protocol_version.assert_called_once_with()
  new_panda.get_spi_protocol_namespace.assert_called_once_with()
  new_panda.get_signature.assert_not_called()
  pandad.HARDWARE.reset_internal_panda.assert_called_once_with()


@patch(f"{MODULE_NAME}.get_expected_signature", return_value=EXPECTED_SIGNATURE)
@patch(f"{MODULE_NAME}.Panda")
def test_external_panda_keeps_transport_reconnect(panda_cls, _expected_signature):
  pandad.HARDWARE.reset_mock()
  panda_cls.SUPPORTED_DEVICES = (b"\x09",)
  panda_cls.SPI_PROTOCOL_VERSION = 0x83
  panda_cls.SPI_PROTOCOL_NAMESPACE = b"ICSP"
  panda = panda_cls.return_value
  panda.get_type.return_value = b"\x09"
  panda.is_internal.return_value = False
  panda.bootstub = False
  panda.get_version.return_value = "old"
  panda.get_signature.side_effect = (b"old firmware signature", EXPECTED_SIGNATURE)
  panda.is_connected_spi.return_value = False

  flash_panda(SERIAL)

  panda_cls.assert_called_once_with(SERIAL, allow_legacy_spi=True)
  panda.flash.assert_called_once_with(reconnect=True)
  pandad.HARDWARE.reset_internal_panda.assert_not_called()
  panda.close.assert_called_once_with()


def test_update_attempts_alternate_reset_and_rom_recovery():
  pandad.HARDWARE.reset_mock()

  prepare_internal_panda(0)
  prepare_internal_panda(1)
  prepare_internal_panda(2)

  assert pandad.HARDWARE.reset_internal_panda.call_count == 2
  pandad.HARDWARE.recover_internal_panda.assert_called_once_with()
