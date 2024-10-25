import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.volkswagen.values import DBC, VolkswagenFlags

RADAR_ADDR = 0x24F
NO_OBJECT  = 0
LANE_TYPES = ['Same_Lane', 'Left_Lane', 'Right_Lane']

# info: distance signals can move without physical distance change ...
# this is not raw data

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

    # Temporäre Sammlung der Informationen aller aktiven Objekte (nach ID geordnet)
    active_objects = {}

    # Gehe über alle Signal Parts (6 in diesem Fall)
    for lane_type in LANE_TYPES:
      for idx in range(1, 3):
        signal_part = f'{lane_type}_0{idx}'
        long_distance = f'{signal_part}_Long_Distance'
        object_id = f'{signal_part}_ObjectID'
        lat_distance = f'{signal_part}_Lat_Distance'
        rel_velo = f'{signal_part}_Rel_Velo'

        current_object_id = msg[object_id]

        # Wenn eine Objekt-ID vorhanden ist, sammeln wir alle relevanten Daten
        if current_object_id != NO_OBJECT:
          if current_object_id not in active_objects:
            active_objects[current_object_id] = {
              "long_distance": msg[long_distance],
              "lat_distance": msg[lat_distance],
              "rel_velo": msg[rel_velo] * CV.KPH_TO_MS
            }
          else:
            # Objekt-IDs sollten eineindeutig sein: PoC ist im Widerspruch
            ret.errors = ["canError"]
            return ret
            
    # Aktualisiere die Radarpunkte basierend auf den aktiven Objekt-IDs
    for object_id, data in active_objects.items():
      if object_id not in self.pts:
        self.pts[object_id] = car.RadarData.RadarPoint.new_message()
        self.pts[object_id].trackId = self.track_id
        self.track_id += 1

      self.pts[object_id].measured = True
      self.pts[object_id].dRel = data["long_distance"]
      self.pts[object_id].yRel = data["lat_distance"]
      self.pts[object_id].vRel = data["rel_velo"]
      self.pts[object_id].aRel = float('nan')
      self.pts[object_id].yvRel = float('nan')

    # Entferne Objekte, die in keinem Signal Part mehr vorkommen
    tracked_ids = set(self.pts.keys())
    active_ids = set(active_objects.keys())
    for object_id in tracked_ids - active_ids:
      self.pts.pop(object_id, None)

    ret.points = list(self.pts.values())
    return ret
