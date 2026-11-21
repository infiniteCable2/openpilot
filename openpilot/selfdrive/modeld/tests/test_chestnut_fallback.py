from unittest.mock import Mock, call, patch

import pytest

from openpilot.selfdrive.modeld import helpers


@pytest.mark.parametrize("failure", [None, "enumerate", "load", "warmup"])
def test_initial_model_choice_never_requests_an_onroad_cycle(failure):
  params = Mock()
  big, small = Mock(), Mock()
  if failure == "warmup":
    big.warmup.side_effect = RuntimeError("GPU failed during warmup")

  def make_model(is_big):
    if is_big and failure == "load":
      raise RuntimeError("GPU failed during load")
    return big if is_big else small

  enumerate_error = TimeoutError("GPU not found") if failure == "enumerate" else None
  with patch.object(helpers, "wait_for_chestnut", side_effect=enumerate_error):
    active, fallback = helpers.load_driving_models(make_model, True, 5., params)

  assert active is (big if failure is None else small)
  assert fallback is small
  assert params.mock_calls == [call.put_bool("ChestnutActive", failure is None), call.put_bool("ChestnutLoading", False)]


def test_stalled_big_model_selects_small_without_waiting_again():
  params = Mock()
  small = Mock()
  make_model = Mock(return_value=small)
  with patch.object(helpers.threading, "Thread") as thread:
    loader = thread.return_value
    loader.is_alive.return_value = True
    active, fallback = helpers.load_driving_models(make_model, True, 0.1, params)
    loader.join.assert_called_once_with(0.1)
  assert active is fallback is small
  make_model.assert_called_once_with(False)
  assert params.mock_calls == [call.put_bool("ChestnutActive", False), call.put_bool("ChestnutLoading", False)]


def test_small_only_does_not_initialize_chestnut():
  params = Mock()
  small = Mock()
  make_model = Mock(return_value=small)
  with patch.object(helpers, "wait_for_chestnut") as wait:
    active, fallback = helpers.load_driving_models(make_model, False, 5., params)
  assert active is fallback is small
  wait.assert_not_called()
  make_model.assert_called_once_with(False)
  assert params.mock_calls == [call.put_bool("ChestnutLoading", False)]
