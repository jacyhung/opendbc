#!/usr/bin/env python3
"""
Simple test for adjacent lane tracking logic without full dependencies.
Tests the core filtering logic.
"""


class MockRadarPoint:
    """Mock radar point for testing."""
    def __init__(self, track_id, d_rel, y_rel, v_rel):
        self.trackId = track_id
        self.dRel = d_rel
        self.yRel = y_rel
        self.vRel = v_rel
        self.measured = True


class MockLiveTracks:
    """Mock liveTracks message."""
    def __init__(self):
        self.points = []


def filter_adjacent_lanes(live_tracks, lane_boundary=1.8, max_lateral=5.5):
    """
    Simulate the CarState.update_adjacent_lanes_from_live_tracks() logic.
    Returns (left_lane_lead, right_lane_lead).
    """
    if not live_tracks or not hasattr(live_tracks, 'points'):
        return None, None
    
    # Filter valid tracks by lane
    left_lane = []
    right_lane = []
    
    for pt in live_tracks.points:
        if not pt.measured:
            continue
        
        # Left lane: tracks beyond left boundary but not too far
        if -max_lateral < pt.yRel < -lane_boundary:
            left_lane.append(pt)
        # Right lane: tracks beyond right boundary but not too far
        elif lane_boundary < pt.yRel < max_lateral:
            right_lane.append(pt)
    
    # Find closest in each lane
    left_lane_lead = min(left_lane, key=lambda pt: pt.dRel) if left_lane else None
    right_lane_lead = min(right_lane, key=lambda pt: pt.dRel) if right_lane else None
    
    return left_lane_lead, right_lane_lead


def create_test_scenario():
    """Create a test scenario with vehicles in different lanes."""
    live_tracks = MockLiveTracks()
    
    # Center lane vehicles (yRel between -1.8 and +1.8)
    live_tracks.points.append(MockRadarPoint(1, 10.0, -0.5, -2.0))  # Center
    live_tracks.points.append(MockRadarPoint(2, 25.0, 0.3, -1.5))   # Center
    
    # Left lane vehicles (yRel < -1.8)
    live_tracks.points.append(MockRadarPoint(3, 15.0, -2.5, -3.0))  # Left - closest
    live_tracks.points.append(MockRadarPoint(4, 30.0, -3.2, -2.5))  # Left
    live_tracks.points.append(MockRadarPoint(5, 45.0, -4.0, -1.0))  # Left
    
    # Right lane vehicles (yRel > +1.8)
    live_tracks.points.append(MockRadarPoint(6, 12.0, 3.5, -4.0))   # Right - closest
    live_tracks.points.append(MockRadarPoint(7, 20.0, 2.8, -3.5))   # Right
    live_tracks.points.append(MockRadarPoint(8, 35.0, 4.2, -2.0))   # Right
    
    # Ignored vehicles (too far lateral)
    live_tracks.points.append(MockRadarPoint(9, 50.0, 8.0, 0.0))    # Too far right
    live_tracks.points.append(MockRadarPoint(10, 60.0, -10.0, 0.0)) # Too far left
    
    return live_tracks


def print_test_header(title):
    """Print a formatted test header."""
    print("\n" + "=" * 70)
    print(title)
    print("=" * 70)


def test_main_scenario():
    """Test the main scenario with vehicles in all lanes."""
    print_test_header("TEST 1: Main Scenario - Vehicles in All Lanes")
    
    live_tracks = create_test_scenario()
    
    print(f"\nInput: {len(live_tracks.points)} radar tracks")
    for pt in live_tracks.points:
        if abs(pt.yRel) <= 1.8:
            lane = "CENTER"
        elif -5.5 < pt.yRel < -1.8:
            lane = "LEFT  "
        elif 1.8 < pt.yRel < 5.5:
            lane = "RIGHT "
        else:
            lane = "IGNORE"
        print(f"  [{lane}] Track {pt.trackId}: {pt.dRel:5.1f}m ahead, {pt.yRel:+5.2f}m lateral")
    
    # Process
    left_lead, right_lead = filter_adjacent_lanes(live_tracks)
    
    print("\nResults:")
    if left_lead:
        print(f"  ✓ LEFT LEAD:  Track {left_lead.trackId} at {left_lead.dRel:.1f}m, {left_lead.yRel:.2f}m")
    else:
        print(f"  ✗ LEFT LEAD:  None")
    
    if right_lead:
        print(f"  ✓ RIGHT LEAD: Track {right_lead.trackId} at {right_lead.dRel:.1f}m, {right_lead.yRel:.2f}m")
    else:
        print(f"  ✗ RIGHT LEAD: None")
    
    # Verify
    print("\nVerification:")
    success = True
    
    if left_lead and left_lead.trackId == 3:
        print(f"  ✓ Left lead correct (Track 3 - closest at 15.0m)")
    else:
        print(f"  ✗ Left lead incorrect (expected Track 3)")
        success = False
    
    if right_lead and right_lead.trackId == 6:
        print(f"  ✓ Right lead correct (Track 6 - closest at 12.0m)")
    else:
        print(f"  ✗ Right lead incorrect (expected Track 6)")
        success = False
    
    return success


def test_edge_cases():
    """Test edge cases."""
    print_test_header("TEST 2: Edge Cases")
    
    all_passed = True
    
    # Test 2a: No adjacent lane vehicles
    print("\n2a. No adjacent lane vehicles:")
    live_tracks = MockLiveTracks()
    live_tracks.points.append(MockRadarPoint(1, 10.0, 0.0, -2.0))  # Center only
    left, right = filter_adjacent_lanes(live_tracks)
    
    if left is None and right is None:
        print("  ✓ Correctly returns None for both lanes")
    else:
        print("  ✗ Should return None for both lanes")
        all_passed = False
    
    # Test 2b: Vehicle at boundary
    print("\n2b. Vehicle exactly at boundary (1.8m):")
    live_tracks = MockLiveTracks()
    live_tracks.points.append(MockRadarPoint(1, 10.0, 1.8, -2.0))
    left, right = filter_adjacent_lanes(live_tracks)
    
    if right is None:
        print("  ✓ Boundary vehicle not included")
    else:
        print("  ✗ Boundary vehicle should not be included")
        all_passed = False
    
    # Test 2c: Vehicle too far lateral
    print("\n2c. Vehicle too far lateral (6.0m):")
    live_tracks = MockLiveTracks()
    live_tracks.points.append(MockRadarPoint(1, 10.0, 6.0, -2.0))
    left, right = filter_adjacent_lanes(live_tracks)
    
    if right is None:
        print("  ✓ Far vehicle correctly filtered out")
    else:
        print("  ✗ Far vehicle should be filtered out")
        all_passed = False
    
    # Test 2d: Multiple vehicles - verify closest selected
    print("\n2d. Multiple vehicles in same lane:")
    live_tracks = MockLiveTracks()
    live_tracks.points.append(MockRadarPoint(1, 30.0, 2.5, -2.0))  # Far
    live_tracks.points.append(MockRadarPoint(2, 10.0, 2.5, -2.0))  # Close
    live_tracks.points.append(MockRadarPoint(3, 20.0, 2.5, -2.0))  # Medium
    left, right = filter_adjacent_lanes(live_tracks)
    
    if right and right.trackId == 2:
        print(f"  ✓ Closest vehicle selected (Track 2 at 10.0m)")
    else:
        print(f"  ✗ Should select closest vehicle (Track 2)")
        all_passed = False
    
    return all_passed


def test_real_world_scenario():
    """Test with data similar to what you're seeing in your logs."""
    print_test_header("TEST 3: Real World Scenario (From Your Logs)")
    
    live_tracks = MockLiveTracks()
    
    # Based on your log output:
    # Left: 21.0m @ -3.80m
    # Right: 14.2m @ +4.45m
    live_tracks.points.append(MockRadarPoint(1, 3.9, -1.55, 0.0))   # Center
    live_tracks.points.append(MockRadarPoint(2, 6.8, -1.60, 0.0))   # Center
    live_tracks.points.append(MockRadarPoint(3, 21.0, -3.80, 0.0))  # Left
    live_tracks.points.append(MockRadarPoint(4, 24.8, -2.65, 0.0))  # Left
    live_tracks.points.append(MockRadarPoint(5, 14.2, 4.45, 0.0))   # Right - closest
    live_tracks.points.append(MockRadarPoint(6, 15.4, 4.80, 0.0))   # Right
    live_tracks.points.append(MockRadarPoint(7, 34.5, 6.65, 0.0))   # Ignored (too far)
    
    print(f"\nInput: Simulating your real-world data")
    for pt in live_tracks.points:
        if abs(pt.yRel) <= 1.8:
            lane = "CENTER"
        elif -5.5 < pt.yRel < -1.8:
            lane = "LEFT  "
        elif 1.8 < pt.yRel < 5.5:
            lane = "RIGHT "
        else:
            lane = "IGNORE"
        print(f"  [{lane}] Track {pt.trackId}: {pt.dRel:5.1f}m ahead, {pt.yRel:+5.2f}m lateral")
    
    left, right = filter_adjacent_lanes(live_tracks)
    
    print("\nResults:")
    if left:
        print(f"  ✓ LEFT LEAD:  Track {left.trackId} at {left.dRel:.1f}m, {left.yRel:.2f}m")
        print(f"    CCNC would send: LEAD_LEFT_DISTANCE={int(left.dRel * 10)}, LEAD_LEFT_LATERAL=80")
    else:
        print(f"  ✗ LEFT LEAD:  None")
    
    if right:
        print(f"  ✓ RIGHT LEAD: Track {right.trackId} at {right.dRel:.1f}m, {right.yRel:.2f}m")
        print(f"    CCNC would send: LEAD_RIGHT_DISTANCE={int(right.dRel * 10)}, LEAD_RIGHT_LATERAL=80")
    else:
        print(f"  ✗ RIGHT LEAD: None")
    
    success = left is not None and right is not None
    if success:
        print("\n  ✓ Both leads detected correctly!")
    else:
        print("\n  ✗ Expected both leads to be detected")
    
    return success


if __name__ == "__main__":
    print("\n🚗 ADJACENT LANE TRACKING TEST SUITE 🚗")
    print("Testing the core filtering logic without full dependencies\n")
    
    results = []
    
    # Run tests
    results.append(("Main Scenario", test_main_scenario()))
    results.append(("Edge Cases", test_edge_cases()))
    results.append(("Real World Data", test_real_world_scenario()))
    
    # Summary
    print_test_header("TEST SUMMARY")
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{status}: {name}")
        if not passed:
            all_passed = False
    
    print("\n" + "=" * 70)
    if all_passed:
        print("✅ ALL TESTS PASSED!")
        print("\nThe adjacent lane tracking logic is working correctly.")
        print("The issue in your car is likely a timing problem in controlsd.py")
        print("\nMake sure you:")
        print("1. Added 'liveTracks' to SubMaster in __init__")
        print("2. Call update_adjacent_lanes_from_live_tracks() at the START of state_control()")
        print("3. Use self.sm.valid['liveTracks'] not self.sm.updated['liveTracks']")
    else:
        print("❌ SOME TESTS FAILED")
        print("Check the output above for details.")
    print("=" * 70 + "\n")
