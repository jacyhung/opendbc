from collections import deque
import copy
import math

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, Buttons, CarControllerParams
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.hyundai.escc import EsccCarStateBase
from opendbc.sunnypilot.car.hyundai.mads import MadsCarState
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125

# Cancel button can sometimes be ACC pause/resume button, main button can also enable on some cars
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


class CarState(CarStateBase, EsccCarStateBase, MadsCarState, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    EsccCarStateBase.__init__(self)
    MadsCarState.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.cruise_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.lda_button = 0

    self.gear_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                          "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"
    if CP.flags & HyundaiFlags.CANFD:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      self.shifter_values = can_define.dv["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    elif CP.flags & HyundaiFlags.FCEV:
      self.shifter_values = can_define.dv["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    else:
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.accelerator_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                                 "ACCELERATOR_ALT" if CP.flags & HyundaiFlags.HYBRID else \
                                 "ACCELERATOR_BRAKE_ALT"
    self.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else \
                                 "CRUISE_BUTTONS"
    self.is_metric = False
    self.buttons_counter = 0

    self.cruise_info = {}
    self.msg_161, self.msg_162, self.msg_1b5 = {}, {}, {}

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    # Adjacent lane radar tracking
    self.left_lane_lead = None
    self.right_lane_lead = None
    self.left_lane_lead_rear = None
    self.right_lane_lead_rear = None
    
    # Lane quality from vision (used to gate adjacent lane detection)
    self.left_lane_quality = 0
    self.right_lane_quality = 0
    
    # Track IDs and counts for stability (stick with same vehicle)
    self.left_lane_track_id = None
    self.right_lane_track_id = None
    self.left_lane_track_count = 0  # How many frames we've seen this track
    self.right_lane_track_count = 0
    self.left_lane_lost_count = 0  # Frames since track was lost
    self.right_lane_lost_count = 0
    self.MIN_TRACK_COUNT = 5  # Require 0.5 seconds of stability (fast enough to catch cars)
    self.MAX_LOST_COUNT = 10  # Allow track to be lost for 1 second before switching
    
    # Lane thresholds for adjacent lane detection
    self.LANE_BOUNDARY = 1.8  # Minimum lateral distance to be considered adjacent lane
    self.MAX_LATERAL = 4.5  # Maximum lateral distance (tighter to exclude shoulders)
    # REAR signals are for blind spot area: vehicles beside/behind you (negative dRel or very close)
    self.REAR_DISTANCE_MAX = 10.0  # meters - max distance for REAR signals (1-10m range)
    self.MIN_RELATIVE_VELOCITY = -30.0  # m/s - minimum relative velocity to consider (filters stationary objects)

    self.params = CarControllerParams(CP)

  def update_radar_tracks(self, radar_interface):
    """Update adjacent lane tracking data from radar interface."""
    if hasattr(radar_interface, 'left_lane_lead'):
      self.left_lane_lead = radar_interface.left_lane_lead
      self.right_lane_lead = radar_interface.right_lane_lead
  
  def update_adjacent_lanes_from_live_tracks(self, live_tracks, v_ego=0.0):
    """
    Update adjacent lane leads from liveTracks message (published by radard).
    This is called from openpilot's control loop with sm['liveTracks'].
    
    Args:
        live_tracks: RadarData message with radar points
        v_ego: Vehicle's own velocity in m/s (used for stationary object filtering)
    """
    if not live_tracks or not hasattr(live_tracks, 'points'):
      return
    
    # SAFETY: Only work at speeds >= 20 mph (8.94 m/s)
    # At low speeds, too much noise and ambiguity
    MIN_SPEED_FOR_ADJACENT_LANES = 8.94  # 20 mph
    if v_ego < MIN_SPEED_FOR_ADJACENT_LANES:
      self.left_lane_lead = None
      self.right_lane_lead = None
      return
    
    # Filter valid tracks by lane
    left_lane = []
    right_lane = []
    
    for pt in live_tracks.points:
      if not pt.measured:
        continue
      
      # ============ SIMPLE, CLEAR FILTERING ============
      
      # Filter 1: Basic sanity - remove obvious glitches
      if pt.dRel < 0.75 or pt.dRel > 200.0:
        continue
      
      # Filter 2: Must be in adjacent lane area
      if abs(pt.yRel) < self.LANE_BOUNDARY or abs(pt.yRel) > self.MAX_LATERAL:
        continue
      
      # Filter 3: Velocity-based (only when moving or stopped)
      if v_ego > 1.0:  # We're moving
        # Real vehicles will have vRel significantly different from 0
        # Walls/barriers will have vRel ≈ 0
        if abs(pt.vRel) < 1.0:  # Stationary - wall/barrier
          continue
      else:  # We're stopped (v_ego < 1.0)
        # When stopped, EVERYTHING has stable trackIds (walls, signs, parked cars)
        # The ONLY way to identify real passing cars is by velocity
        # Real cars passing = at least 25 km/h = 7 m/s
        if pt.vRel > -7.0:  # Not moving away fast enough (< 25 km/h)
          continue
        # This filters:
        # - Stationary objects (vRel = 0)
        # - Approaching vehicles (vRel > 0) 
        # - Slow moving noise (abs(vRel) < 7.0)
      
      # Left lane: tracks beyond left boundary but not too far
      # ONLY if vision confirms there's a left lane line
      if -self.MAX_LATERAL < pt.yRel < -self.LANE_BOUNDARY:
        if self.left_lane_quality > 0:  # Vision sees left lane line
          left_lane.append(pt)
      # Right lane: tracks beyond right boundary but not too far  
      # ONLY if vision confirms there's a right lane line
      elif self.LANE_BOUNDARY < pt.yRel < self.MAX_LATERAL:
        if self.right_lane_quality > 0:  # Vision sees right lane line
          right_lane.append(pt)
    
    # Track stability with count-based filtering (like radard)
    # Only show tracks that have been stable for MIN_TRACK_COUNT frames
    # This filters out noise/glitches that appear for 1-2 frames
    
    # Left lane: Check if our current track is still valid
    # ONLY search within left_lane to prevent cross-contamination
    if self.left_lane_track_id is not None:
      # Search ONLY in left_lane (not right_lane to prevent wrong-side tracking)
      same_track = next((pt for pt in left_lane if pt.trackId == self.left_lane_track_id), None)
      if same_track:
        # Track found - reset lost count and continue
        self.left_lane_lost_count = 0
        self.left_lane_track_count += 1
        # Only show if we've seen it enough times (filters noise)
        if self.left_lane_track_count >= self.MIN_TRACK_COUNT:
          self.left_lane_lead = same_track
        else:
          self.left_lane_lead = None  # Not confident yet
      else:
        # Track not found - increment lost count and CLEAR lead
        self.left_lane_lost_count += 1
        self.left_lane_lead = None  # Don't show stale data
        if self.left_lane_lost_count > self.MAX_LOST_COUNT:
          # Track truly lost - find new closest in left lane
          candidate = min(left_lane, key=lambda pt: pt.dRel) if left_lane else None
          if candidate:
            self.left_lane_track_id = candidate.trackId
            self.left_lane_track_count = 1
            self.left_lane_lost_count = 0
            self.left_lane_lead = None  # Don't show until MIN_TRACK_COUNT
          else:
            self.left_lane_track_id = None
            self.left_lane_track_count = 0
            self.left_lane_lost_count = 0
            self.left_lane_lead = None
    else:
      # No current track, find closest in left lane and start counting
      candidate = min(left_lane, key=lambda pt: pt.dRel) if left_lane else None
      if candidate:
        self.left_lane_track_id = candidate.trackId
        self.left_lane_track_count = 1
        self.left_lane_lost_count = 0
        self.left_lane_lead = None  # Don't show until MIN_TRACK_COUNT
      else:
        self.left_lane_lead = None
    
    # Right lane: Same logic - ONLY search within right_lane
    if self.right_lane_track_id is not None:
      # Search ONLY in right_lane (not left_lane to prevent wrong-side tracking)
      same_track = next((pt for pt in right_lane if pt.trackId == self.right_lane_track_id), None)
      if same_track:
        # Track found - reset lost count and continue
        self.right_lane_lost_count = 0
        self.right_lane_track_count += 1
        if self.right_lane_track_count >= self.MIN_TRACK_COUNT:
          self.right_lane_lead = same_track
        else:
          self.right_lane_lead = None
      else:
        # Track not found - increment lost count and CLEAR lead
        self.right_lane_lost_count += 1
        self.right_lane_lead = None  # Don't show stale data
        if self.right_lane_lost_count > self.MAX_LOST_COUNT:
          # Track truly lost - find new one
          candidate = min(right_lane, key=lambda pt: pt.dRel) if right_lane else None
          if candidate:
            self.right_lane_track_id = candidate.trackId
            self.right_lane_track_count = 1
            self.right_lane_lost_count = 0
            self.right_lane_lead = None
          else:
            self.right_lane_track_id = None
            self.right_lane_track_count = 0
            self.right_lane_lost_count = 0
            self.right_lane_lead = None
    else:
      # No current track - find closest in right lane
      candidate = min(right_lane, key=lambda pt: pt.dRel) if right_lane else None
      if candidate:
        self.right_lane_track_id = candidate.trackId
        self.right_lane_track_count = 1
        self.right_lane_lost_count = 0
        self.right_lane_lead = None
      else:
        self.right_lane_lead = None
    
    # REAR signals disabled for now
    self.left_lane_lead_rear = None
    self.right_lane_lead_rear = None
    
    # Minimal debug output (only when leads change)
    # Uncomment for debugging:
    # if self.left_lane_lead:
    #   print(f"[CS] LEFT: {self.left_lane_lead.dRel:.1f}m @ {self.left_lane_lead.yRel:.2f}m")
    # if self.right_lane_lead:
    #   print(f"[CS] RIGHT: {self.right_lane_lead.dRel:.1f}m @ {self.right_lane_lead.yRel:.2f}m")

  def recent_button_interaction(self) -> bool:
    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    return any(btn in ENABLE_BUTTONS for btn in self.cruise_buttons) or any(self.main_buttons)

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP.flags & HyundaiFlags.CANFD:
      return self.update_canfd(can_parsers)

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    cp_cruise = cp_cam if self.CP.flags & HyundaiFlags.CAMERA_SCC else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    self.parse_wheel_speeds(ret,
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.standstill = cp.vl["WHL_SPD11"]["WHL_SPD_FL"] <= STANDSTILL_THRESHOLD and cp.vl["WHL_SPD11"]["WHL_SPD_RR"] <= STANDSTILL_THRESHOLD

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # Mimic how dash converts to imperial.
      # Sorento is the only platform where CF_Clu_VehicleSpeed is already imperial when not is_metric
      # TODO: CGW_USM1->CF_Gway_DrLockSoundRValue may describe this
      if not self.is_metric and self.CP.carFingerprint not in (CAR.KIA_SORENTO,):
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = cp.vl["TCS13"]["ACCEnable"] == 0
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False
    elif not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverOverride"] == 2  # 2 includes regen braking by user on HEV/EV
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    ret.espDisabled = cp.vl["TCS11"]["TCS_PAS"] == 1
    ret.espActive = cp.vl["TCS11"]["ABS_ACT"] == 1
    ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV | HyundaiFlags.FCEV):
      if self.CP.flags & HyundaiFlags.FCEV:
        ret.gasPressed = cp.vl["FCEV_ACCELERATOR"]["ACCELERATOR_PEDAL"] > 0
      elif self.CP.flags & HyundaiFlags.HYBRID:
        ret.gasPressed = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] > 0
      else:
        ret.gasPressed = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] > 0
    else:
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.flags & HyundaiFlags.FCEV:
      gear = cp.vl["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      gear = cp.vl["TCU12"]["CUR_GR"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if (not self.CP.openpilotLongitudinalControl or self.CP.flags & HyundaiFlags.CAMERA_SCC) and not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      scc_warning = cp_cruise.vl["SCC12"]["TakeOverReq"] == 1  # sometimes only SCC system shows an FCW
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = (aeb_warning or scc_warning) and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    prev_cruise_buttons = self.cruise_buttons[-1]
    prev_main_buttons = self.main_buttons[-1]
    prev_lda_button = self.lda_button
    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    if self.CP.flags & HyundaiFlags.HAS_LDA_BUTTON:
      self.lda_button = cp.vl["BCM_PO_11"]["LDA_BTN"]

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise}),
                        *create_button_events(self.lda_button, prev_lda_button, {1: ButtonType.lkas})]

    if self.CP.openpilotLongitudinalControl:
      ret.cruiseState.available = self.get_main_cruise(ret)

    CarStateExt.update(self, ret, ret_sp, can_parsers, speed_conv)

    ret.blockPcmEnable = not self.recent_button_interaction()

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    return ret, ret_sp

  def update_canfd(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.is_metric = cp.vl["CRUISE_BUTTONS_ALT"]["DISTANCE_UNIT"] != 1
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    if self.CP.flags & (HyundaiFlags.EV | HyundaiFlags.HYBRID):
      ret.gasPressed = cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL"] > 1e-5
    else:
      ret.gasPressed = bool(cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0

    gear = cp.vl[self.gear_msg_canfd]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # TODO: figure out positions
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHL_SpdFLVal"],
      cp.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"],
      cp.vl["WHEEL_SPEEDS"]["WHL_SpdRLVal"],
      cp.vl["WHEEL_SPEEDS"]["WHL_SpdRRVal"],
    )
    ret.standstill = cp.vl["WHEEL_SPEEDS"]["WHL_SpdFLVal"] <= STANDSTILL_THRESHOLD and cp.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"] <= STANDSTILL_THRESHOLD and \
                     cp.vl["WHEEL_SPEEDS"]["WHL_SpdRLVal"] <= STANDSTILL_THRESHOLD and cp.vl["WHEEL_SPEEDS"]["WHL_SpdRRVal"] <= STANDSTILL_THRESHOLD

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"]
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0

    alt = "_ALT" if self.CP.carFingerprint in (CAR.HYUNDAI_KONA_EV_2ND_GEN, CAR.HYUNDAI_IONIQ_5_N) else ""
    if self.CP.flags & HyundaiFlags.CCNC and not self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING:
      self.msg_161, self.msg_162, self.msg_1b5 = map(copy.copy, (cp_cam.vl["CCNC_0x161"], cp_cam.vl["CCNC_0x162"], cp_cam.vl["FR_CMR_03_50ms"]))
      self.cruise_info = copy.copy((cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp).vl["SCC_CONTROL"])
      alt = "_ALT"
      # Update lane quality from vision for adjacent lane detection gating
      self.left_lane_quality = self.msg_1b5.get("Info_LftLnQualSta", 0)
      self.right_lane_quality = self.msg_1b5.get("Info_RtLnQualSta", 0)
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"][f"LEFT_LAMP{alt}"],
                                                                      cp.vl["BLINKERS"][f"RIGHT_LAMP{alt}"])
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"][f"FL_INDICATOR{alt}"] != 0
      ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"][f"FR_INDICATOR{alt}"] != 0

    # cruise state
    # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
    ret.cruiseState.available = cp.vl["TCS"]["ACCEnable"] == 0
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
    else:
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])

    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1

    prev_cruise_buttons = self.cruise_buttons[-1]
    prev_main_buttons = self.main_buttons[-1]
    prev_lda_button = self.lda_button
    self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    self.lda_button = cp.vl[self.cruise_btns_msg_canfd]["LDA_BTN"]
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING:
      self.lfa_block_msg = copy.copy(cp_cam.vl["CAM_0x362"] if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT
                                          else cp_cam.vl["CAM_0x2a4"])

    MadsCarState.update_mads_canfd(self, ret, can_parsers)

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise}),
                        *create_button_events(self.lda_button, prev_lda_button, {1: ButtonType.lkas})]

    if self.CP.openpilotLongitudinalControl:
      ret.cruiseState.available = self.get_main_cruise(ret)

    CarStateExt.update_canfd_ext(self, ret, ret_sp, can_parsers, speed_factor)

    ret.blockPcmEnable = not self.recent_button_interaction()

    return ret, ret_sp

  def get_can_parsers_canfd(self, CP):
    msgs = []
    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      # TODO: this can be removed once we add dynamic support to vl_all
      msgs += [
        # this message is 50Hz but the ECU frequently stops transmitting for ~0.5s
        ("CRUISE_BUTTONS", 1)
      ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], msgs, CanBus(CP).ECAN),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).CAM),
    }

  def get_can_parsers(self, CP, CP_SP):
    if CP.flags & HyundaiFlags.CANFD:
      return self.get_can_parsers_canfd(CP)

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
