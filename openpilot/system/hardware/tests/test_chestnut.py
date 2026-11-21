import ctypes
from unittest.mock import call, patch

from openpilot.system.hardware.chestnut import flash, power


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


def test_offroad_cycle_does_not_power_cycle_gpu():
  state = power.ChestnutOffroadPower()
  with patch.object(power, "set_pcie_power") as switch:
    state.update(0., fresh=True, started=False, present=True, model_running=False)
    state.update(1., fresh=True, started=True, present=True, model_running=True)
    state.update(4., fresh=True, started=True, present=True, model_running=True)
    state.update(5., fresh=True, started=False, present=True, model_running=False)
    state.update(9.9, fresh=True, started=False, present=True, model_running=False)
    switch.assert_not_called()


def test_offroad_power_waits_for_model_and_retries_only_failed_shutdown():
  state = power.ChestnutOffroadPower()
  with patch.object(power, "set_pcie_power", side_effect=[False, True, True]) as switch:
    state.update(0., fresh=True, started=False, present=True, model_running=True)
    state.update(5., fresh=True, started=False, present=True, model_running=True)
    switch.assert_not_called()
    state.update(6., fresh=True, started=False, present=True, model_running=False)
    state.update(10.9, fresh=True, started=False, present=True, model_running=False)
    switch.assert_called_once_with(False)
    state.update(11., fresh=True, started=False, present=True, model_running=False)
    state.update(20., fresh=True, started=False, present=True, model_running=False)
    assert switch.call_args_list == [call(False), call(False)]
    state.update(21., fresh=True, started=True, present=True, model_running=True)
    state.update(22., fresh=True, started=False, present=True, model_running=False)
    state.update(27., fresh=True, started=False, present=True, model_running=False)
    assert switch.call_args_list == [call(False), call(False), call(False)]


def test_offroad_power_requires_current_device_state_and_resets_on_disconnect():
  state = power.ChestnutOffroadPower()
  with patch.object(power, "set_pcie_power", return_value=True) as switch:
    state.update(0., fresh=True, started=False, present=True, model_running=False)
    state.update(5., fresh=False, started=False, present=True, model_running=False)
    state.update(6., fresh=True, started=False, present=True, model_running=False)
    switch.assert_not_called()
    state.update(11., fresh=True, started=False, present=True, model_running=False)
    switch.assert_called_once_with(False)
    state.update(12., fresh=True, started=False, present=False, model_running=False)
    state.update(13., fresh=True, started=False, present=True, model_running=False)
    state.update(18., fresh=True, started=False, present=True, model_running=False)
    assert switch.call_args_list == [call(False), call(False)]
