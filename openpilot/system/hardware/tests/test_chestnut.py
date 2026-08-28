import ctypes
from unittest.mock import patch

from openpilot.system.hardware.chestnut import flash


def test_set_pcie_power_off():
  with patch.object(flash, "find_chestnut", return_value=("/sys/test", None, None)), \
       patch.object(flash, "open_device", return_value=42), \
       patch.object(flash.fcntl, "ioctl") as ioctl, \
       patch.object(flash.os, "close") as close:

    assert flash.set_pcie_power(False)
    assert ioctl.call_count == 1
    ctrl = ioctl.call_args.args[2]
    assert (ctrl.request_type, ctrl.request, ctrl.value) == (0x40, 0xF3, 0)
    close.assert_called_once_with(42)


def test_set_pcie_power_on_checks_link():
  def ioctl(_fd, _request, ctrl):
    if ctrl.request_type == 0xC0:
      ctypes.cast(ctrl.data, ctypes.POINTER(ctypes.c_ubyte))[0] = 0x78

  with patch.object(flash, "find_chestnut", return_value=("/sys/test", None, None)), \
       patch.object(flash, "open_device", return_value=42), \
       patch.object(flash.fcntl, "ioctl", side_effect=ioctl) as ioctl_mock, \
       patch.object(flash.os, "close"):

    assert flash.set_pcie_power(True)
    assert ioctl_mock.call_count == 2
    power_ctrl = ioctl_mock.call_args_list[0].args[2]
    assert (power_ctrl.request_type, power_ctrl.request, power_ctrl.value) == (0x40, 0xF3, 1)


def test_set_pcie_power_without_chestnut():
  with patch.object(flash, "find_chestnut", return_value=(None, None, None)):
    assert not flash.set_pcie_power(False)
