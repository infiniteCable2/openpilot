import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.volkswagen.values import DBC, VolkswagenFlags

NO_OBJECT = -5
RADAR_ADDR = 0x24F
RADAR_SIGNALS = {
    RADAR_SAME_LANE_01: ('Same_Lane_01_Long_Distance', 'Same_Lane_01_LD_Offset', 'Same_Lane_01_Lat_Distance', 'Same_Lane_01_Rel_Velo'),
    RADAR_SAME_LANE_02: ('Same_Lane_02_Long_Distance', 'Same_Lane_02_LD_Offset', 'Same_Lane_02_Lat_Distance', 'Same_Lane_02_Rel_Velo'),
    RADAR_LEFT_LANE_01: ('Left_Lane_01_Long_Distance', 'Left_Lane_01_LD_Offset', 'Left_Lane_01_Lat_Distance', 'Left_Lane_01_Rel_Velo'),
    RADAR_LEFT_LANE_02: ('Left_Lane_02_Long_Distance', 'Left_Lane_02_LD_Offset', 'Left_Lane_02_Lat_Distance', 'Left_Lane_02_Rel_Velo'),
    RADAR_RIGHT_LANE_01: ('Right_Lane_01_Long_Distance', 'Right_Lane_01_LD_Offset', 'Right_Lane_01_Lat_Distance', 'Right_Lane_01_Rel_Velo'),
    RADAR_RIGHT_LANE_02: ('Right_Lane_02_Long_Distance', 'Right_Lane_02_LD_Offset', 'Right_Lane_02_Lat_Distance', 'Right_Lane_02_Rel_Velo'),
}

# info: distance signals can move without physical distance change ...

def get_radar_can_parser(CP):
  if CP.flags & VolkswagenFlags.MEB:
    messages = [("MEB_Distance_01", 25)]
  else:
    return None

  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 2)


class RadarInterface(RadarInterfaceBase):
  pass
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_ADDR
    self.track_id = 0
    self.previous_offsets = {}

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    msg = self.rcp.vl["MEB_Distance_01"]

    for signal_part, signal_fields in RADAR_SIGNALS.items():
      if signal_part not in self.pts:
        self.pts[signal_part] = car.RadarData.RadarPoint.new_message()
        self.pts[signal_part].trackId = self.track_id
        self.track_id += 1

      long_distance, ld_offset, lat_distance, rel_velo = signal_fields
      current_offset = msg[ld_offset]

      if signal_part not in self.previous_offsets:
        self.previous_offsets[signal_part] = NO_OBJECT
    
      # Check if the current offset is valid and matches the previous offset
      valid = current_offset != NO_OBJECT and current_offset == self.previous_offsets[signal_part]
      self.previous_offsets[signal_part] = current_offset
      
      if valid:
        self.pts[signal_part].measured = True
        self.pts[signal_part].dRel = msg[long_distance] + msg[ld_offset]
        self.pts[signal_part].yRel = msg[lat_distance]
        self.pts[signal_part].vRel = msg[rel_velo] * CV.KPH_TO_MS
        self.pts[signal_part].aRel = float('nan')
        self.pts[signal_part].yvRel = float('nan')
        
      else:
        del self.pts[signal_part]

    ret.points = list(self.pts.values())
    return ret
