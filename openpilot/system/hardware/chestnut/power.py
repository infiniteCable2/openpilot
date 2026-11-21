from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.chestnut.flash import set_pcie_power


OFFROAD_POWER_DELAY = 5.0
POWER_OFF_RETRY_INTERVAL = 5.0


class ChestnutOffroadPower:
  def __init__(self):
    self.next_attempt: float | None = None
    self.powered_off = False

  def update(self, now: float, *, fresh: bool, started: bool, present: bool, model_running: bool) -> None:
    # Only tinygrad powers the GPU on for inference, when it opens the device.
    if started or not present:
      self.next_attempt = None
      self.powered_off = False
      return

    if not fresh:
      self.next_attempt = None
      return

    if self.next_attempt is None:
      self.next_attempt = now + OFFROAD_POWER_DELAY

    # Ignore short offroad cycles and never cut the rails while a model owns them.
    if self.powered_off or model_running or now < self.next_attempt:
      return

    self.powered_off = set_pcie_power(False)
    self.next_attempt = now + POWER_OFF_RETRY_INTERVAL
    cloudlog.event("chestnut power", enabled=False, success=self.powered_off, error=not self.powered_off)
