import pytest

from openpilot.selfdrive.selfdrived.blind_spot import LEFT, RIGHT, warning_direction


@pytest.mark.parametrize(
  "left_blinker,right_blinker,left_blindspot,right_blindspot,expected",
  [
    (True, False, True, False, LEFT),
    (True, False, True, True, LEFT),
    (True, False, False, True, None),
    (False, True, False, True, RIGHT),
    (False, True, True, True, RIGHT),
    (False, True, True, False, None),
    (False, False, True, True, None),
    (True, True, True, True, None),
  ],
)
def test_warning_direction(left_blinker, right_blinker, left_blindspot, right_blindspot, expected):
  assert warning_direction(left_blinker, right_blinker, left_blindspot, right_blindspot) == expected
