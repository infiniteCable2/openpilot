import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.volkswagen.values import DBC, VolkswagenFlags
from collections import defaultdict

RADAR_ADDR = 0x24F
NO_OBJECT  = 0
LANE_TYPES = ['Same_Lane', 'Left_Lane', 'Right_Lane']

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
    self.previous_objects = defaultdict(lambda: NO_OBJECT)

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

    if self.rcp is None or not self.rcp.can_valid:
      ret.errors = ["canError"]
      return ret

    msg = self.rcp.vl["MEB_Distance_01"]

    # tempory collection of active object ids
    active_object_ids = set()

    # iterate over lane types and dynamic signal parts (01, 02)
    for lane_type in LANE_TYPES:
      for idx in range(1, 3):
        signal_part = f'{lane_type}_0{idx}'
        long_distance = f'{signal_part}_Long_Distance'
        object = f'{signal_part}_ObjectID'
        lat_distance = f'{signal_part}_Lat_Distance'
        rel_velo = f'{signal_part}_Rel_Velo'

        current_object = msg[object]

        # add current object id to collection
        if current_object != NO_OBJECT:
          active_object_ids.add(current_object)

        if signal_part not in self.pts:
          self.pts[signal_part] = car.RadarData.RadarPoint.new_message()
          self.pts[signal_part].trackId = self.track_id
          self.track_id += 1

        # check if current object does differ from previous
        if current_object != NO_OBJECT:
          if current_object != self.previous_objects.get(signal_part):
            # new object detected -> create, otherwise just update
            self.pts[signal_part] = car.RadarData.RadarPoint.new_message()
            self.pts[signal_part].trackId = self.track_id
            self.track_id += 1
          
          self.pts[signal_part].measured = True
          self.pts[signal_part].dRel = msg[long_distance]
          self.pts[signal_part].yRel = msg[lat_distance]
          self.pts[signal_part].vRel = msg[rel_velo] * CV.KPH_TO_MS
          self.pts[signal_part].aRel = float('nan')
          self.pts[signal_part].yvRel = float('nan')
          
        else:
          # no object
          self.pts[signal_part].measured = False

        self.previous_objects[signal_part] = current_object

    # remove object ids that do not exist anymore
    tracked_object_ids = {pt.trackId for pt in self.pts.values()}

    # remove irrelevant signal part if object id is not active anymore
    for signal_part, pt in list(self.pts.items()):
      if pt.trackId not in active_object_ids:
        self.pts.pop(signal_part, None)

    ret.points = list(self.pts.values())
    return ret
