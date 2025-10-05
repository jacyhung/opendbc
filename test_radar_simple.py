#!/usr/bin/env python3
"""
Simple test for adjacent lane radar tracking.
Works with opendbc's process_replay.py

Usage:
  # First, get a route segment
  python test_radar_simple.py
"""

import sys
import os

# Add opendbc to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def test_synthetic_data():
    """Test with synthetic radar data to verify logic."""
    print("="*80)
    print("Testing Adjacent Lane Radar Logic with Synthetic Data")
    print("="*80)
    
    # Create CarParams
    CP = structs.CarParams.new_message()
    CP.carFingerprint = CAR.HYUNDAI_SONATA_HEV_2024
    CP.flags = HyundaiFlags.CCNC | HyundaiFlags.MRR35_RADAR | HyundaiFlags.CANFD
    CP.radarUnavailable = False
    
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = HyundaiFlagsSP.RADAR_FULL_RADAR.value
    
    # Create radar interface
    ri = RadarInterface(CP, CP_SP)
    
    print("\n📊 Test Scenario: 3-lane highway with multiple vehicles")
    print("-" * 80)
    
    # Create test tracks
    tracks = []
    
    # Track 1: Center lane, 45m ahead (your lane lead)
    t1 = structs.RadarData.RadarPoint()
    t1.trackId = 1
    t1.measured = True
    t1.dRel = 45.0
    t1.yRel = 0.3  # Slightly right of center
    t1.vRel = -3.0
    t1.aRel = 0.0
    tracks.append(t1)
    
    # Track 2: Left lane, 30m ahead (SHOULD BE LEFT LEAD)
    t2 = structs.RadarData.RadarPoint()
    t2.trackId = 2
    t2.measured = True
    t2.dRel = 30.0
    t2.yRel = -2.5  # Left lane
    t2.vRel = -5.0
    t2.aRel = -0.5
    tracks.append(t2)
    
    # Track 3: Left lane, 70m ahead (further left car)
    t3 = structs.RadarData.RadarPoint()
    t3.trackId = 3
    t3.measured = True
    t3.dRel = 70.0
    t3.yRel = -3.0
    t3.vRel = 0.0
    t3.aRel = 0.0
    tracks.append(t3)
    
    # Track 4: Right lane, 55m ahead (SHOULD BE RIGHT LEAD)
    t4 = structs.RadarData.RadarPoint()
    t4.trackId = 4
    t4.measured = True
    t4.dRel = 55.0
    t4.yRel = 2.8  # Right lane
    t4.vRel = 1.0
    t4.aRel = 0.0
    tracks.append(t4)
    
    # Track 5: Right lane, 90m ahead (further right car)
    t5 = structs.RadarData.RadarPoint()
    t5.trackId = 5
    t5.measured = True
    t5.dRel = 90.0
    t5.yRel = 3.2
    t5.vRel = 2.0
    t5.aRel = 0.0
    tracks.append(t5)
    
    # Track 6: Center lane, 80m ahead (far center car)
    t6 = structs.RadarData.RadarPoint()
    t6.trackId = 6
    t6.measured = True
    t6.dRel = 80.0
    t6.yRel = -0.2
    t6.vRel = -1.0
    t6.aRel = 0.0
    tracks.append(t6)
    
    # Print input data
    print("\n📍 Input Radar Tracks:")
    print(f"{'Track':<8} {'Distance':<12} {'Lateral':<12} {'Velocity':<12} {'Lane':<10}")
    print("-" * 80)
    
    LANE_BOUNDARY = 1.8
    for t in sorted(tracks, key=lambda x: x.dRel):
        if abs(t.yRel) <= LANE_BOUNDARY:
            lane = "CENTER"
        elif t.yRel < -LANE_BOUNDARY:
            lane = "LEFT"
        else:
            lane = "RIGHT"
        
        print(f"#{t.trackId:<7} {t.dRel:>6.1f}m      {t.yRel:>+6.2f}m      {t.vRel:>+6.1f}m/s    {lane}")
    
    # Run the lane detection
    print("\n🔍 Running lane detection algorithm...")
    ri._update_adjacent_lane_leads(tracks)
    
    # Print results
    print("\n✅ Detection Results:")
    print("-" * 80)
    
    if ri.left_lane_lead:
        print(f"LEFT LEAD:   Track #{ri.left_lane_lead.trackId} at {ri.left_lane_lead.dRel:.1f}m "
              f"(lateral: {ri.left_lane_lead.yRel:+.2f}m, vel: {ri.left_lane_lead.vRel:+.1f}m/s)")
    else:
        print("LEFT LEAD:   None detected")
    
    if ri.right_lane_lead:
        print(f"RIGHT LEAD:  Track #{ri.right_lane_lead.trackId} at {ri.right_lane_lead.dRel:.1f}m "
              f"(lateral: {ri.right_lane_lead.yRel:+.2f}m, vel: {ri.right_lane_lead.vRel:+.1f}m/s)")
    else:
        print("RIGHT LEAD:  None detected")
    
    # Verify correctness
    print("\n🧪 Verification:")
    print("-" * 80)
    
    errors = []
    
    # Check left lead
    if ri.left_lane_lead is None:
        errors.append("❌ FAIL: No left lane lead detected (expected Track #2)")
    elif ri.left_lane_lead.trackId != 2:
        errors.append(f"❌ FAIL: Wrong left lead (got Track #{ri.left_lane_lead.trackId}, expected Track #2)")
    else:
        print("✓ Left lane lead correct: Track #2 at 30.0m")
    
    # Check right lead
    if ri.right_lane_lead is None:
        errors.append("❌ FAIL: No right lane lead detected (expected Track #4)")
    elif ri.right_lane_lead.trackId != 4:
        errors.append(f"❌ FAIL: Wrong right lead (got Track #{ri.right_lane_lead.trackId}, expected Track #4)")
    else:
        print("✓ Right lane lead correct: Track #4 at 55.0m")
    
    # Check that closest in each lane was selected
    if ri.left_lane_lead and ri.left_lane_lead.dRel == 30.0:
        print("✓ Closest left lane vehicle selected")
    
    if ri.right_lane_lead and ri.right_lane_lead.dRel == 55.0:
        print("✓ Closest right lane vehicle selected")
    
    print("\n" + "="*80)
    if errors:
        print("❌ TEST FAILED")
        for err in errors:
            print(f"  {err}")
        return False
    else:
        print("✅ ALL TESTS PASSED!")
        print("\nThe adjacent lane detection logic is working correctly.")
        print("You can now test with real data from your recorded route.")
        return True


def print_usage():
    """Print usage instructions."""
    print("\n" + "="*80)
    print("How to Test with Your Recorded Route:")
    print("="*80)
    print("""
1. First, run this synthetic test to verify the logic:
   python test_radar_simple.py

2. To test with your actual route, you need to:
   a) Make sure you have a route recorded with radar data
   b) Use openpilot's tools to replay it
   
3. Enable debug output in radar_interface.py:
   - Open: opendbc/car/hyundai/radar_interface.py
   - Find line 178
   - Uncomment: self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)
   
4. Run your route through openpilot and watch the console output

5. You should see output like:
   === Radar Tracks (Total: 8) ===
   Center lane: 2 | Left lane: 3 | Right lane: 3
   
     [CENTER] Track 945: 45.2m ahead, +0.15m lateral,  -2.3m/s
     [LEFT  ] Track 941: 35.8m ahead, -2.35m lateral,  -5.2m/s ← LEFT LEAD
     [RIGHT ] Track 939: 28.4m ahead, +2.42m lateral,  -3.1m/s ← RIGHT LEAD
     ...

6. Adjust LANE_BOUNDARY in radar_interface.py line 163 if needed based on results
""")
    print("="*80)


if __name__ == "__main__":
    print("\n🚗 Adjacent Lane Radar Tracking - Test Suite\n")
    
    # Run synthetic test
    success = test_synthetic_data()
    
    # Print usage info
    print_usage()
    
    sys.exit(0 if success else 1)
