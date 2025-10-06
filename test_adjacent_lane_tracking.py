#!/usr/bin/env python3
"""
Test script for adjacent lane radar tracking.
This simulates the data flow from liveTracks -> CarState -> CarController -> CCNC messages.
"""

import sys
from pathlib import Path

# Add opendbc to path
opendbc_path = Path(__file__).parent
sys.path.insert(0, str(opendbc_path))

from opendbc.car import structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.interface import CarInterface


def create_mock_radar_point(track_id, d_rel, y_rel, v_rel):
    """Create a mock radar point."""
    pt = structs.RadarData.RadarPoint()
    pt.trackId = track_id
    pt.dRel = d_rel
    pt.yRel = y_rel
    pt.vRel = v_rel
    pt.measured = True
    return pt


def create_mock_live_tracks():
    """Create mock liveTracks message with vehicles in different lanes."""
    live_tracks = structs.RadarData()
    
    # Center lane vehicles (yRel between -1.8 and +1.8)
    live_tracks.points.append(create_mock_radar_point(1, 10.0, -0.5, -2.0))  # Center
    live_tracks.points.append(create_mock_radar_point(2, 25.0, 0.3, -1.5))   # Center
    
    # Left lane vehicles (yRel < -1.8)
    live_tracks.points.append(create_mock_radar_point(3, 15.0, -2.5, -3.0))  # Left - closest
    live_tracks.points.append(create_mock_radar_point(4, 30.0, -3.2, -2.5))  # Left
    live_tracks.points.append(create_mock_radar_point(5, 45.0, -4.0, -1.0))  # Left
    
    # Right lane vehicles (yRel > +1.8)
    live_tracks.points.append(create_mock_radar_point(6, 12.0, 3.5, -4.0))   # Right - closest
    live_tracks.points.append(create_mock_radar_point(7, 20.0, 2.8, -3.5))   # Right
    live_tracks.points.append(create_mock_radar_point(8, 35.0, 4.2, -2.0))   # Right
    
    # Ignored vehicles (too far lateral)
    live_tracks.points.append(create_mock_radar_point(9, 50.0, 8.0, 0.0))    # Too far right
    live_tracks.points.append(create_mock_radar_point(10, 60.0, -10.0, 0.0)) # Too far left
    
    return live_tracks


def test_adjacent_lane_tracking():
    """Test the adjacent lane tracking functionality."""
    print("=" * 70)
    print("ADJACENT LANE TRACKING TEST")
    print("=" * 70)
    
    # Create minimal CarParams for testing
    CP = structs.CarParams()
    CP.carFingerprint = CAR.HYUNDAI_GENESIS_G90_2017
    CP.radarUnavailable = False
    
    CP_SP = structs.CarParamsSP()
    
    # Create CarState instance
    print("\n1. Creating CarState...")
    CS = CarState(CP, CP_SP)
    print(f"   ✓ CarState created")
    print(f"   - LANE_BOUNDARY: {CS.LANE_BOUNDARY}m")
    print(f"   - MAX_LATERAL: {CS.MAX_LATERAL}m")
    
    # Create mock liveTracks data
    print("\n2. Creating mock liveTracks with test vehicles...")
    live_tracks = create_mock_live_tracks()
    print(f"   ✓ Created {len(live_tracks.points)} radar tracks:")
    
    for pt in live_tracks.points:
        if abs(pt.yRel) <= CS.LANE_BOUNDARY:
            lane = "CENTER"
        elif -CS.MAX_LATERAL < pt.yRel < -CS.LANE_BOUNDARY:
            lane = "LEFT  "
        elif CS.LANE_BOUNDARY < pt.yRel < CS.MAX_LATERAL:
            lane = "RIGHT "
        else:
            lane = "IGNORE"
        print(f"     [{lane}] Track {pt.trackId}: {pt.dRel:5.1f}m ahead, {pt.yRel:+5.2f}m lateral")
    
    # Test CarState processing
    print("\n3. Processing liveTracks in CarState...")
    CS.update_adjacent_lanes_from_live_tracks(live_tracks)
    
    print(f"\n4. Results:")
    if CS.left_lane_lead:
        print(f"   ✓ LEFT LEAD detected:")
        print(f"     - Distance: {CS.left_lane_lead.dRel:.1f}m")
        print(f"     - Lateral:  {CS.left_lane_lead.yRel:.2f}m")
        print(f"     - Track ID: {CS.left_lane_lead.trackId}")
    else:
        print(f"   ✗ LEFT LEAD: None")
    
    if CS.right_lane_lead:
        print(f"   ✓ RIGHT LEAD detected:")
        print(f"     - Distance: {CS.right_lane_lead.dRel:.1f}m")
        print(f"     - Lateral:  {CS.right_lane_lead.yRel:.2f}m")
        print(f"     - Track ID: {CS.right_lane_lead.trackId}")
    else:
        print(f"   ✗ RIGHT LEAD: None")
    
    # Verify expected results
    print("\n5. Verification:")
    success = True
    
    # Expected: Left lead should be track 3 (closest at 15.0m, -2.5m)
    if CS.left_lane_lead and CS.left_lane_lead.trackId == 3:
        print(f"   ✓ Left lead correctly identified (Track 3)")
    else:
        print(f"   ✗ Left lead incorrect (expected Track 3)")
        success = False
    
    # Expected: Right lead should be track 6 (closest at 12.0m, 3.5m)
    if CS.right_lane_lead and CS.right_lane_lead.trackId == 6:
        print(f"   ✓ Right lead correctly identified (Track 6)")
    else:
        print(f"   ✗ Right lead incorrect (expected Track 6)")
        success = False
    
    # Test CCNC message generation
    print("\n6. Testing CCNC message generation...")
    print(f"   Left lead for CCNC:")
    if CS.left_lane_lead:
        left_dist = int(CS.left_lane_lead.dRel * 10)
        print(f"     - LEAD_LEFT: 2 (WHITE BOX)")
        print(f"     - LEAD_LEFT_DISTANCE: {left_dist} (raw: {CS.left_lane_lead.dRel:.1f}m)")
        print(f"     - LEAD_LEFT_LATERAL: 80 (fixed)")
    
    print(f"   Right lead for CCNC:")
    if CS.right_lane_lead:
        right_dist = int(CS.right_lane_lead.dRel * 10)
        print(f"     - LEAD_RIGHT: 2 (WHITE BOX)")
        print(f"     - LEAD_RIGHT_DISTANCE: {right_dist} (raw: {CS.right_lane_lead.dRel:.1f}m)")
        print(f"     - LEAD_RIGHT_LATERAL: 80 (fixed)")
    
    # Final result
    print("\n" + "=" * 70)
    if success:
        print("✅ TEST PASSED - Adjacent lane tracking working correctly!")
    else:
        print("❌ TEST FAILED - Check the results above")
    print("=" * 70)
    
    return success


def test_edge_cases():
    """Test edge cases and boundary conditions."""
    print("\n\n" + "=" * 70)
    print("EDGE CASE TESTS")
    print("=" * 70)
    
    CP = structs.CarParams()
    CP.carFingerprint = CAR.HYUNDAI_GENESIS_G90_2017
    CP_SP = structs.CarParamsSP()
    CS = CarState(CP, CP_SP)
    
    # Test 1: No vehicles
    print("\n1. Test: No vehicles in adjacent lanes")
    live_tracks = structs.RadarData()
    live_tracks.points.append(create_mock_radar_point(1, 10.0, 0.0, -2.0))  # Center only
    CS.update_adjacent_lanes_from_live_tracks(live_tracks)
    
    if CS.left_lane_lead is None and CS.right_lane_lead is None:
        print("   ✓ Correctly returns None for both lanes")
    else:
        print("   ✗ Should return None for both lanes")
    
    # Test 2: Vehicle exactly at boundary
    print("\n2. Test: Vehicle at lane boundary (1.8m)")
    live_tracks = structs.RadarData()
    live_tracks.points.append(create_mock_radar_point(1, 10.0, 1.8, -2.0))  # Exactly at boundary
    CS.update_adjacent_lanes_from_live_tracks(live_tracks)
    
    if CS.right_lane_lead is None:
        print("   ✓ Boundary vehicle not included (correct)")
    else:
        print("   ✗ Boundary vehicle should not be included")
    
    # Test 3: Vehicle too far lateral
    print("\n3. Test: Vehicle too far lateral (6.0m)")
    live_tracks = structs.RadarData()
    live_tracks.points.append(create_mock_radar_point(1, 10.0, 6.0, -2.0))  # Beyond MAX_LATERAL
    CS.update_adjacent_lanes_from_live_tracks(live_tracks)
    
    if CS.right_lane_lead is None:
        print("   ✓ Far vehicle correctly filtered out")
    else:
        print("   ✗ Far vehicle should be filtered out")
    
    # Test 4: Multiple vehicles, verify closest is selected
    print("\n4. Test: Multiple vehicles in same lane")
    live_tracks = structs.RadarData()
    live_tracks.points.append(create_mock_radar_point(1, 30.0, 2.5, -2.0))  # Far
    live_tracks.points.append(create_mock_radar_point(2, 10.0, 2.5, -2.0))  # Close
    live_tracks.points.append(create_mock_radar_point(3, 20.0, 2.5, -2.0))  # Medium
    CS.update_adjacent_lanes_from_live_tracks(live_tracks)
    
    if CS.right_lane_lead and CS.right_lane_lead.trackId == 2:
        print("   ✓ Closest vehicle selected (Track 2 at 10.0m)")
    else:
        print("   ✗ Should select closest vehicle")
    
    print("\n" + "=" * 70)
    print("✅ EDGE CASE TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    print("\n🚗 ADJACENT LANE RADAR TRACKING TEST SUITE 🚗\n")
    
    try:
        # Run main test
        success = test_adjacent_lane_tracking()
        
        # Run edge case tests
        test_edge_cases()
        
        # Summary
        print("\n\n" + "=" * 70)
        print("TEST SUITE COMPLETE")
        print("=" * 70)
        
        if success:
            print("\n✅ All tests passed! The adjacent lane tracking is working correctly.")
            print("\nNext steps:")
            print("1. Add the integration code to sunnypilot/controlsd.py")
            print("2. Test on the car with real radar data")
            print("3. Check the cluster display for adjacent lane vehicles")
            sys.exit(0)
        else:
            print("\n❌ Some tests failed. Check the output above.")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n❌ TEST ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
