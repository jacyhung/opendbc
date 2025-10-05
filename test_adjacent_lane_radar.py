#!/usr/bin/env python3
"""
Test script for adjacent lane radar tracking.
Replays a recorded route and shows radar track assignments.

Usage:
  python test_adjacent_lane_radar.py <route_name>
  
Example:
  python test_adjacent_lane_radar.py "bc40c72b728178f2/00000006--ee76ae8c42"
"""

import sys
import os
from collections import defaultdict

# Add opendbc to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def create_test_car_params():
    """Create CarParams for HYUNDAI_SONATA_HEV_2024 with MRR35 radar."""
    CP = structs.CarParams.new_message()
    CP.carFingerprint = CAR.HYUNDAI_SONATA_HEV_2024
    CP.flags = HyundaiFlags.CCNC | HyundaiFlags.MRR35_RADAR | HyundaiFlags.CANFD
    CP.radarUnavailable = False
    
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = HyundaiFlagsSP.RADAR_FULL_RADAR.value
    
    return CP, CP_SP


def print_radar_summary(ri, frame_num):
    """Print a summary of radar tracks and lane assignments."""
    if not hasattr(ri, 'pts') or not ri.pts:
        print(f"Frame {frame_num}: No radar tracks")
        return
    
    tracks = list(ri.pts.values())
    valid_tracks = [pt for pt in tracks if pt.measured]
    
    if not valid_tracks:
        print(f"Frame {frame_num}: No valid tracks")
        return
    
    # Categorize by lane
    LANE_BOUNDARY = 1.8
    center = [pt for pt in valid_tracks if abs(pt.yRel) <= LANE_BOUNDARY]
    left = [pt for pt in valid_tracks if pt.yRel < -LANE_BOUNDARY]
    right = [pt for pt in valid_tracks if pt.yRel > LANE_BOUNDARY]
    
    print(f"\n{'='*80}")
    print(f"Frame {frame_num}: {len(valid_tracks)} valid tracks")
    print(f"  Center: {len(center)} | Left: {len(left)} | Right: {len(right)}")
    
    # Show selected leads
    if ri.left_lane_lead:
        print(f"  ← LEFT LEAD:  {ri.left_lane_lead.dRel:5.1f}m @ {ri.left_lane_lead.yRel:+5.2f}m, {ri.left_lane_lead.vRel:+5.1f}m/s")
    else:
        print(f"  ← LEFT LEAD:  None")
    
    if ri.right_lane_lead:
        print(f"  → RIGHT LEAD: {ri.right_lane_lead.dRel:5.1f}m @ {ri.right_lane_lead.yRel:+5.2f}m, {ri.right_lane_lead.vRel:+5.1f}m/s")
    else:
        print(f"  → RIGHT LEAD: None")
    
    # Show all tracks sorted by distance
    print(f"\n  All tracks (sorted by distance):")
    sorted_tracks = sorted(valid_tracks, key=lambda pt: pt.dRel)
    
    for pt in sorted_tracks:
        # Determine lane
        if abs(pt.yRel) <= LANE_BOUNDARY:
            lane = "CENTER"
        elif pt.yRel < -LANE_BOUNDARY:
            lane = "LEFT  "
        else:
            lane = "RIGHT "
        
        # Mark selected leads
        marker = ""
        if ri.left_lane_lead and pt.trackId == ri.left_lane_lead.trackId:
            marker = " ◄ LEFT LEAD"
        elif ri.right_lane_lead and pt.trackId == ri.right_lane_lead.trackId:
            marker = " ► RIGHT LEAD"
        
        print(f"    [{lane}] Track {pt.trackId:3d}: "
              f"dist={pt.dRel:5.1f}m, lat={pt.yRel:+5.2f}m, "
              f"vel={pt.vRel:+5.1f}m/s, accel={pt.aRel:+5.2f}m/s²{marker}")


def test_with_log_reader(route_name):
    """Test radar interface with a recorded route using LogReader."""
    try:
        from tools.lib.logreader import LogReader
    except ImportError:
        print("ERROR: Could not import LogReader. Make sure you're running this from openpilot environment.")
        print("Alternative: Use the manual test mode below.")
        return False
    
    print(f"Loading route: {route_name}")
    lr = LogReader(route_name)
    
    # Create radar interface
    CP, CP_SP = create_test_car_params()
    ri = RadarInterface(CP, CP_SP)
    
    frame = 0
    can_msgs = []
    
    print("\nProcessing CAN messages...")
    print("(Showing every 50th frame to avoid spam)\n")
    
    for msg in lr:
        if msg.which() == 'can':
            # Collect CAN messages
            can_msgs.extend([(m.address, m.busTime, m.dat, m.src) for m in msg.can])
            
            # Update radar every frame
            if len(can_msgs) > 0:
                ret = ri.update([(0, can_msgs)])
                can_msgs = []
                
                # Print summary every 50 frames (1 second at 50Hz)
                if frame % 50 == 0 and ret is not None:
                    print_radar_summary(ri, frame)
                
                frame += 1
    
    print(f"\n{'='*80}")
    print(f"Processed {frame} frames")
    return True


def test_manual_mode():
    """Manual test mode - create synthetic radar data."""
    print("\n" + "="*80)
    print("MANUAL TEST MODE - Creating synthetic radar tracks")
    print("="*80)
    
    CP, CP_SP = create_test_car_params()
    ri = RadarInterface(CP, CP_SP)
    
    # Simulate some radar points
    print("\nSimulating scenario: Highway with cars in all lanes")
    
    # Create synthetic radar points
    from opendbc.car import structs
    
    # Center lane - car 50m ahead
    center_car = structs.RadarData.RadarPoint()
    center_car.trackId = 1
    center_car.measured = True
    center_car.dRel = 50.0
    center_car.yRel = 0.2  # Slightly right of center
    center_car.vRel = -5.0  # Approaching at 5 m/s
    center_car.aRel = 0.0
    
    # Left lane - car 35m ahead
    left_car = structs.RadarData.RadarPoint()
    left_car.trackId = 2
    left_car.measured = True
    left_car.dRel = 35.0
    left_car.yRel = -2.5  # Left lane
    left_car.vRel = -2.0
    left_car.aRel = 0.0
    
    # Right lane - car 60m ahead
    right_car = structs.RadarData.RadarPoint()
    right_car.trackId = 3
    right_car.measured = True
    right_car.dRel = 60.0
    right_car.yRel = 2.8  # Right lane
    right_car.vRel = 1.0  # Moving away
    right_car.aRel = 0.0
    
    # Another left lane car further back
    left_car_2 = structs.RadarData.RadarPoint()
    left_car_2.trackId = 4
    left_car_2.measured = True
    left_car_2.dRel = 80.0
    left_car_2.yRel = -3.2
    left_car_2.vRel = 0.5
    left_car_2.aRel = 0.0
    
    radar_points = [center_car, left_car, right_car, left_car_2]
    
    # Test the lane detection
    ri._update_adjacent_lane_leads(radar_points)
    
    # Print results
    print("\nSynthetic Radar Tracks:")
    for pt in radar_points:
        lane = "CENTER" if abs(pt.yRel) <= 1.8 else ("LEFT" if pt.yRel < 0 else "RIGHT")
        print(f"  Track {pt.trackId}: {pt.dRel}m @ {pt.yRel:+.1f}m ({lane} lane)")
    
    print("\nDetected Adjacent Leads:")
    if ri.left_lane_lead:
        print(f"  LEFT:  Track {ri.left_lane_lead.trackId} at {ri.left_lane_lead.dRel}m")
    else:
        print(f"  LEFT:  None")
    
    if ri.right_lane_lead:
        print(f"  RIGHT: Track {ri.right_lane_lead.trackId} at {ri.right_lane_lead.dRel}m")
    else:
        print(f"  RIGHT: None")
    
    # Verify results
    print("\n" + "="*80)
    print("VERIFICATION:")
    assert ri.left_lane_lead is not None, "Should detect left lane lead"
    assert ri.left_lane_lead.trackId == 2, "Should select closest left lane car (Track 2)"
    assert ri.right_lane_lead is not None, "Should detect right lane lead"
    assert ri.right_lane_lead.trackId == 3, "Should select right lane car (Track 3)"
    print("✓ All checks passed!")
    print("="*80)


def main():
    print("="*80)
    print("Adjacent Lane Radar Tracking Test")
    print("="*80)
    
    if len(sys.argv) > 1:
        route_name = sys.argv[1]
        success = test_with_log_reader(route_name)
        if not success:
            print("\nFalling back to manual test mode...")
            test_manual_mode()
    else:
        print("\nNo route specified. Running manual test mode...")
        print("To test with a recorded route, run:")
        print("  python test_adjacent_lane_radar.py <route_name>")
        print()
        test_manual_mode()


if __name__ == "__main__":
    main()
