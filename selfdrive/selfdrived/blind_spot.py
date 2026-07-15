LEFT = "left"
RIGHT = "right"


def warning_direction(left_blinker: bool, right_blinker: bool,
                      left_blindspot: bool, right_blindspot: bool) -> str | None:
  """Return the side that should trigger a blind-spot warning.

  A warning is only valid when exactly one blinker is active and the blind-spot
  detection is on that same side. This also prevents warnings while hazards are
  active.
  """
  if left_blinker and not right_blinker and left_blindspot:
    return LEFT
  if right_blinker and not left_blinker and right_blindspot:
    return RIGHT
  return None
