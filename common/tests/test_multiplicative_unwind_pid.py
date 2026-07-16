import pytest

from openpilot.common.pid import MultiplicativeUnwindPID


def test_override_unwinds_integrator_multiplicatively():
  pid = MultiplicativeUnwindPID(0.0, 1.0, rate=10, min_cmd=1e-3, ki_red_time=0.2)
  pid.update(1.0)
  initial_i = pid.i

  pid.update(0.0, override=True)
  first_override_i = pid.i
  pid.update(0.0, override=True)

  assert initial_i > first_override_i > pid.i
  assert pid.i == pytest.approx(1e-3)


def test_safety_limit_freezes_integrator_and_preserves_feedforward():
  pid = MultiplicativeUnwindPID(0.0, 1.0, k_f=2.0, rate=10)
  output = pid.update(1.0, feedforward=0.25, freeze_integrator=True)

  assert pid.i == 0.0
  assert pid.f == pytest.approx(0.5)
  assert output == pytest.approx(0.5)
