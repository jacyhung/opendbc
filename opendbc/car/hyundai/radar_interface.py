import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

MANDO_RADAR_ADDR = 0x500
MANDO_RADAR_COUNT = 32
MRREVO14F_RADAR_ADDR = 0x602
MRREVO14F_RADAR_COUNT = 16
MRR35_RADAR_ADDR = 0x3A5
MRR35_RADAR_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, radar_addr, radar_count):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(radar_addr, radar_addr + radar_count)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.CP_flags = CP.flags
    if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
      self.radar_addr, self.radar_count = MRREVO14F_RADAR_ADDR, MRREVO14F_RADAR_COUNT
    elif self.CP_flags & HyundaiFlags.MRR35_RADAR:
      self.radar_addr, self.radar_count = MRR35_RADAR_ADDR, MRR35_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = MANDO_RADAR_ADDR, MANDO_RADAR_COUNT
    self.updated_messages = set()
    self.trigger_msg = self.radar_addr + self.radar_count - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, self.radar_addr, self.radar_count)

    # Adjacent lane tracking
    self.left_lane_lead = None
    self.right_lane_lead = None

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      if self.use_radar_interface_ext:
        return self.update_ext(ret)

    for addr in range(self.radar_addr, self.radar_addr + self.radar_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:

        if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          for i in ("1", "2"):
            track_key = f"{addr}_{i}"
            dist = msg[f"{i}_DISTANCE"]
            if track_key not in self.pts:
              self.pts[track_key] = structs.RadarData.RadarPoint()
              self.pts[track_key].trackId = self.track_id
              self.track_id += 1
            if dist != 255.75:
              pt = self.pts[track_key]
              pt.measured = True
              pt.dRel = dist
              pt.yRel = msg[f"{i}_LATERAL"]
              pt.vRel = msg[f"{i}_SPEED"]
              pt.aRel = float('nan')
              pt.yvRel = float('nan')
            else:
              del self.pts[track_key]

        elif self.CP_flags & HyundaiFlags.MRR35_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          if addr not in self.pts:
            self.pts[addr] = structs.RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1
          if msg['STATE'] in (3, 4):
            self.pts[addr].measured = True
            self.pts[addr].dRel = msg['LONG_DIST']
            self.pts[addr].yRel = msg['LAT_DIST']
            self.pts[addr].vRel = msg['REL_SPEED']
            self.pts[addr].aRel = msg['REL_ACCEL']
            self.pts[addr].yvRel = float('nan')
          else:
            del self.pts[addr]

        else:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          if addr not in self.pts:
            self.pts[addr] = structs.RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1
          if msg['STATE'] in (3, 4):
            azimuth = math.radians(msg['AZIMUTH'])
            self.pts[addr].measured = True
            self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
            self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
            self.pts[addr].vRel = msg['REL_SPEED']
            self.pts[addr].aRel = msg['REL_ACCEL']
            self.pts[addr].yvRel = float('nan')
          else:
            del self.pts[addr]

    ret.points = list(self.pts.values())
    
    # Extract adjacent lane leads for CCNC display
    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      self._update_adjacent_lane_leads(ret.points)
    
    return ret

  def _update_adjacent_lane_leads(self, radar_points):
    """
    Extract left and right lane lead vehicles from radar points.
    
    Lane detection strategy:
    - Center lane: |yRel| < 1.8m (typical lane width ~3.6m, so ±1.8m from center)
    - Left lane: yRel < -1.8m (further left than center lane boundary)
    - Right lane: yRel > 1.8m (further right than center lane boundary)
    
    Note: Adjust threshold based on your testing. Common values:
    - Narrow lanes (US/Europe): 1.5-1.7m
    - Standard lanes: 1.8m (recommended starting point)
    - Wide lanes (highways): 2.0m
    """
    # Lane boundary threshold - distance from vehicle center to lane edge
    # This assumes ~3.6m lane width (typical for highways)
    LANE_BOUNDARY = 1.8  # meters
    MAX_LATERAL = 5.5  # Maximum lateral distance to consider (filters out barriers, signs, etc.)
    
    # Filter valid tracks by lane with realistic lateral distances
    # Center lane: tracks within ±LANE_BOUNDARY
    center_lane = [pt for pt in radar_points if pt.measured and abs(pt.yRel) <= LANE_BOUNDARY]
    # Left lane: tracks beyond left boundary but not too far (realistic adjacent lane)
    left_lane = [pt for pt in radar_points if pt.measured and -MAX_LATERAL < pt.yRel < -LANE_BOUNDARY]
    # Right lane: tracks beyond right boundary but not too far (realistic adjacent lane)
    right_lane = [pt for pt in radar_points if pt.measured and LANE_BOUNDARY < pt.yRel < MAX_LATERAL]
    
    # Find closest in each lane
    self.left_lane_lead = min(left_lane, key=lambda pt: pt.dRel) if left_lane else None
    self.right_lane_lead = min(right_lane, key=lambda pt: pt.dRel) if right_lane else None
    
    # Debug: Uncomment to see all tracks and their lane assignments
    self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)
  
  def _debug_print_tracks(self, all_tracks, center, left, right):
    """Debug helper: Print all radar tracks with lane assignments."""
    print(f"\n=== Radar Tracks (Total: {len(all_tracks)}) ===")
    print(f"Center lane: {len(center)} | Left lane: {len(left)} | Right lane: {len(right)}")
    
    # Sort by distance for easier reading
    sorted_tracks = sorted(all_tracks, key=lambda pt: pt.dRel)
    
    for pt in sorted_tracks:
      # Determine which lane this track is in (with filtering)
      if abs(pt.yRel) <= 1.8:
        lane = "CENTER"
      elif -5.5 < pt.yRel < -1.8:
        lane = "LEFT  "
      elif 1.8 < pt.yRel < 5.5:
        lane = "RIGHT "
      else:
        lane = "IGNORE"  # Too far lateral - likely barrier/sign
      
      # Mark if this is the selected lead for that lane
      marker = ""
      if self.left_lane_lead and pt.trackId == self.left_lane_lead.trackId:
        marker = " ← LEFT LEAD"
      elif self.right_lane_lead and pt.trackId == self.right_lane_lead.trackId:
        marker = " ← RIGHT LEAD"
      
      print(f"  [{lane}] Track {pt.trackId}: {pt.dRel:5.1f}m ahead, {pt.yRel:+5.2f}m lateral, {pt.vRel:+5.1f}m/s{marker}")
    print("=" * 60)
