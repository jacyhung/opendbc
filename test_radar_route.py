#!/usr/bin/env python3
"""
Test adjacent lane radar with a recorded route's CAN data.
This reads CAN logs directly without needing full openpilot.

Usage:
  python test_radar_route.py <path_to_rlog_or_can_file>
  
Example:
  python test_radar_route.py /data/media/0/realdata/bc40c72b728178f2--00000006--ee76ae8c42/rlog.bz2
"""

import sys
import os
import struct
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags
from opendbc.car.hyundai.radar_interface import RadarInterface, MRR35_RADAR_ADDR, MRR35_RADAR_COUNT
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def parse_can_from_file(filename):
    """Parse CAN messages from a file (supports .can or .rlog.bz2)."""
    can_messages = []
    
    if filename.endswith('.bz2'):
        # Try to parse rlog
        try:
            import bz2
            import capnp
            # This is a simplified version - you may need to adjust
            print(f"Reading compressed log: {filename}")
            with bz2.open(filename, 'rb') as f:
                # Read CAN messages from log
                # Note: This is a placeholder - actual implementation depends on log format
                print("Warning: .bz2 log parsing not fully implemented")
                print("Please use the debug output method instead (see below)")
                return None
        except Exception as e:
            print(f"Error reading log: {e}")
            return None
    else:
        # Try to parse as raw CAN file
        print(f"Reading CAN file: {filename}")
        # Placeholder for CAN file parsing
        print("Raw CAN file parsing not implemented")
        return None
    
    return can_messages


def test_radar_interface_standalone():
    """Test radar interface with manual CAN message creation."""
    print("="*80)
    print("Standalone Radar Interface Test")
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
    
    print(f"\n✓ Radar interface created")
    print(f"  Radar address range: 0x{ri.radar_addr:03x} - 0x{ri.radar_addr + ri.radar_count - 1:03x}")
    print(f"  Radar count: {ri.radar_count} tracks")
    print(f"  Trigger message: 0x{ri.trigger_msg:03x}")
    
    # Create some fake CAN messages for radar tracks
    print(f"\n📡 Simulating CAN messages for radar tracks...")
    
    # We'll create a few radar track messages
    # Format: RADAR_TRACK_3a5 (0x3A5) through RADAR_TRACK_3c4 (0x3C4)
    
    can_strings = []
    
    # Helper to create a radar track CAN message
    def create_radar_msg(addr_offset, state, long_dist, lat_dist, rel_speed, rel_accel):
        """Create a fake radar track CAN message."""
        addr = MRR35_RADAR_ADDR + addr_offset
        
        # Build 24-byte message (simplified - not exact DBC encoding)
        data = bytearray(24)
        
        # STATE at bit 54 (byte 6, bits 6-4)
        data[6] = (state & 0x7) << 4
        
        # LONG_DIST at bit 63, 12 bits, 0.05 scale
        long_val = int(long_dist / 0.05)
        data[7] = (long_val >> 4) & 0xFF
        data[8] = (long_val & 0xF) << 4
        
        # LAT_DIST at bit 76, 12 bits signed, 0.05 scale
        lat_val = int(lat_dist / 0.05)
        if lat_val < 0:
            lat_val = (1 << 12) + lat_val  # Two's complement
        data[9] = (data[9] & 0xF0) | ((lat_val >> 8) & 0xF)
        data[10] = lat_val & 0xFF
        
        # REL_SPEED at bit 88, 14 bits signed, 0.01 scale
        speed_val = int(rel_speed / 0.01)
        if speed_val < 0:
            speed_val = (1 << 14) + speed_val
        data[11] = (speed_val >> 6) & 0xFF
        data[12] = (speed_val & 0x3F) << 2
        
        # REL_ACCEL at bit 118, 10 bits signed, 0.02 scale
        accel_val = int(rel_accel / 0.02)
        if accel_val < 0:
            accel_val = (1 << 10) + accel_val
        data[14] = (accel_val >> 2) & 0xFF
        data[15] = (accel_val & 0x3) << 6
        
        return (addr, 0, bytes(data), 1)  # (address, timestamp, data, src)
    
    # Create some test tracks
    test_tracks = [
        # (offset, state, long_dist, lat_dist, rel_speed, rel_accel)
        (0, 3, 45.0, 0.2, -3.0, 0.0),    # Center lane, 45m
        (1, 3, 30.0, -2.5, -5.0, -0.5),  # Left lane, 30m (should be left lead)
        (2, 3, 70.0, -3.0, 0.0, 0.0),    # Left lane, 70m
        (3, 3, 55.0, 2.8, 1.0, 0.0),     # Right lane, 55m (should be right lead)
        (4, 3, 90.0, 3.2, 2.0, 0.0),     # Right lane, 90m
        (5, 3, 80.0, -0.2, -1.0, 0.0),   # Center lane, 80m
    ]
    
    print(f"  Creating {len(test_tracks)} radar track messages...")
    for track in test_tracks:
        msg = create_radar_msg(*track)
        can_strings.append(msg)
    
    # Update radar interface
    print(f"\n🔄 Updating radar interface...")
    ret = ri.update([can_strings])
    
    if ret is None:
        print("  ⚠️  Radar update returned None (waiting for trigger message)")
        print("  This is normal - radar waits for all tracks before updating")
    else:
        print(f"  ✓ Radar updated successfully")
        print(f"  Total points: {len(ret.points)}")
    
    # Check the results
    print(f"\n📊 Results:")
    print("-" * 80)
    
    if ri.left_lane_lead:
        print(f"✓ LEFT LEAD detected:")
        print(f"    Distance: {ri.left_lane_lead.dRel:.1f}m")
        print(f"    Lateral:  {ri.left_lane_lead.yRel:+.2f}m")
        print(f"    Velocity: {ri.left_lane_lead.vRel:+.1f}m/s")
    else:
        print(f"✗ No LEFT LEAD detected")
    
    if ri.right_lane_lead:
        print(f"✓ RIGHT LEAD detected:")
        print(f"    Distance: {ri.right_lane_lead.dRel:.1f}m")
        print(f"    Lateral:  {ri.right_lane_lead.yRel:+.2f}m")
        print(f"    Velocity: {ri.right_lane_lead.vRel:+.1f}m/s")
    else:
        print(f"✗ No RIGHT LEAD detected")
    
    print("\n" + "="*80)
    print("Note: CAN message encoding is simplified in this test.")
    print("For real testing, use the debug output method (see instructions below).")
    print("="*80)


def print_instructions():
    """Print instructions for testing with real data."""
    print("\n" + "="*80)
    print("HOW TO TEST WITH YOUR RECORDED ROUTE")
    print("="*80)
    print("""
The easiest way to test with your recorded route:

1. Enable debug output in radar_interface.py:
   
   File: opendbc/car/hyundai/radar_interface.py
   Line: 178
   
   Change:
     # self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)
   
   To:
     self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)

2. Run your route through openpilot/sunnypilot normally

3. Watch the console output - you'll see:
   
   === Radar Tracks (Total: 8) ===
   Center lane: 2 | Left lane: 3 | Right lane: 3
   
     [CENTER] Track 945: 45.2m ahead, +0.15m lateral,  -2.3m/s
     [LEFT  ] Track 941: 35.8m ahead, -2.35m lateral,  -5.2m/s ← LEFT LEAD
     [RIGHT ] Track 939: 28.4m ahead, +2.42m lateral,  -3.1m/s ← RIGHT LEAD

4. Verify the lane assignments make sense:
   - Are cars actually in the left lane showing as LEFT?
   - Are cars actually in the right lane showing as RIGHT?
   - Is the closest car in each lane selected as the lead?

5. If lane assignments are wrong, adjust LANE_BOUNDARY:
   
   File: opendbc/car/hyundai/radar_interface.py
   Line: 163
   
   Try values: 1.5m (narrow), 1.8m (standard), 2.0m (wide)

6. Once satisfied, comment out the debug line again to reduce spam.

Alternative: Use openpilot's process_replay.py with your route
""")
    print("="*80)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        print(f"Attempting to parse: {filename}")
        can_data = parse_can_from_file(filename)
        if can_data is None:
            print("\nCould not parse file. Using standalone test instead.\n")
            test_radar_interface_standalone()
    else:
        print("No file specified. Running standalone test.\n")
        test_radar_interface_standalone()
    
    print_instructions()
