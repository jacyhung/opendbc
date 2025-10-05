# Testing Adjacent Lane Radar Tracking

This guide explains how to test and verify the adjacent lane radar tracking implementation.

## Quick Start - Synthetic Test

The fastest way to verify the logic works:

```bash
python test_radar_simple.py
```

This will run a synthetic test with fake radar data and verify that:
- ✅ Left lane vehicles are detected correctly
- ✅ Right lane vehicles are detected correctly  
- ✅ The closest vehicle in each lane is selected as the lead

**Expected output:**
```
✅ ALL TESTS PASSED!
The adjacent lane detection logic is working correctly.
```

---

## Testing with Your Recorded Route

### Method 1: Debug Output (Recommended)

This is the easiest way to see what's happening with real data.

**Step 1:** Enable debug output

Edit: `opendbc/car/hyundai/radar_interface.py`

Find line 178 and uncomment it:
```python
# Before:
# self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)

# After:
self._debug_print_tracks(radar_points, center_lane, right_lane)
```

**Step 2:** Run your route through openpilot/sunnypilot

The console will show detailed output like:

```
=== Radar Tracks (Total: 8) ===
Center lane: 2 | Left lane: 3 | Right lane: 3

  [CENTER] Track 945:  45.2m ahead, +0.15m lateral,  -2.3m/s
  [CENTER] Track 937:  78.5m ahead, -0.42m lateral,  +1.1m/s
  [LEFT  ] Track 941:  35.8m ahead, -2.35m lateral,  -5.2m/s ← LEFT LEAD
  [LEFT  ] Track 949:  62.1m ahead, -3.10m lateral,  +0.8m/s
  [LEFT  ] Track 953:  95.3m ahead, -2.88m lateral,  -1.5m/s
  [RIGHT ] Track 939:  28.4m ahead, +2.42m lateral,  -3.1m/s ← RIGHT LEAD
  [RIGHT ] Track 946:  51.7m ahead, +3.25m lateral,  +2.0m/s
  [RIGHT ] Track 958:  88.9m ahead, +2.95m lateral,  -0.5m/s
============================================================
```

**Step 3:** Verify the results

Watch the output while driving and check:
- ✅ Are vehicles in the left lane marked as `[LEFT]`?
- ✅ Are vehicles in the right lane marked as `[RIGHT]`?
- ✅ Is the closest vehicle in each lane selected (marked with `← LEFT LEAD` or `← RIGHT LEAD`)?

**Step 4:** Adjust threshold if needed

If lane assignments are wrong, edit `opendbc/car/hyundai/radar_interface.py` line 163:

```python
# For narrow lanes (city streets)
LANE_BOUNDARY = 1.5  # meters

# For standard lanes (default)
LANE_BOUNDARY = 1.8  # meters

# For wide lanes (highways)
LANE_BOUNDARY = 2.0  # meters
```

**Step 5:** Disable debug output when done

Comment out line 178 again to reduce console spam:
```python
# self._debug_print_tracks(radar_points, center_lane, left_lane, right_lane)
```

---

### Method 2: Test Scripts

If you want to test without running full openpilot:

```bash
# Synthetic test (no route needed)
python test_radar_simple.py

# Standalone test (simulates CAN messages)
python test_radar_route.py

# Full test with route (requires openpilot tools)
python test_adjacent_lane_radar.py "bc40c72b728178f2/00000006--ee76ae8c42"
```

---

## Understanding the Output

### Lane Assignments

The radar assigns tracks to lanes based on lateral position (`yRel`):

| Lateral Position | Lane Assignment | Typical Range |
|-----------------|----------------|---------------|
| `-1.8m to +1.8m` | **CENTER** (your lane) | -0.5m to +0.5m |
| `< -1.8m` | **LEFT** lane | -2.0m to -3.5m |
| `> +1.8m` | **RIGHT** lane | +2.0m to +3.5m |

### Track Information

Each track shows:
- **Track ID**: Unique identifier (radar assigns these)
- **Distance**: Longitudinal distance ahead (meters)
- **Lateral**: Side-to-side position (negative = left, positive = right)
- **Velocity**: Relative speed (negative = approaching, positive = moving away)

### Lead Selection

For each adjacent lane:
- The **closest** vehicle (smallest `dRel`) is selected as the lead
- This is the vehicle shown on your instrument cluster

---

## Troubleshooting

### No tracks detected
- ✅ Check that `RADAR_FULL_RADAR` flag is enabled in sunnypilot settings
- ✅ Verify radar is working (check if any tracks appear at all)
- ✅ Make sure you're driving with other vehicles around

### Wrong lane assignments
- ✅ Adjust `LANE_BOUNDARY` threshold (line 163 in radar_interface.py)
- ✅ Check if your car's radar has different lateral accuracy
- ✅ Verify lane width on the roads you're testing

### No adjacent leads shown on cluster
- ✅ Verify `LEAD_LEFT_LATERAL` and `LEAD_RIGHT_LATERAL` are set to 80 (hardcoded)
- ✅ Check that openpilot is engaged (adjacent lanes only show when enabled)
- ✅ Verify CAN messages are being sent (use CAN sniffer on 0x162)

### Cluster shows wrong distance
- ✅ Check `LEAD_LEFT_DISTANCE` and `LEAD_RIGHT_DISTANCE` scaling (should be `* 10`)
- ✅ Verify radar tracks have correct `dRel` values in debug output

---

## Common Lane Width Reference

| Road Type | Lane Width | Recommended Threshold |
|-----------|-----------|----------------------|
| US Interstate/Highway | 3.6-3.7m | 1.8m |
| US City Streets | 3.0-3.3m | 1.5m |
| European Motorway | 3.5m | 1.75m |
| Narrow Roads | 2.7-3.0m | 1.4m |

---

## Files Modified

1. **opendbc/car/hyundai/radar_interface.py**
   - Added `_update_adjacent_lane_leads()` method
   - Added `_debug_print_tracks()` helper
   - Stores `left_lane_lead` and `right_lane_lead`

2. **opendbc/car/hyundai/carstate.py**
   - Added `left_lane_lead` and `right_lane_lead` variables
   - Added `update_radar_tracks()` method

3. **opendbc/car/hyundai/hyundaicanfd.py**
   - Updated `create_ccnc()` to accept adjacent lane parameters
   - Populates `LEAD_LEFT_*` and `LEAD_RIGHT_*` signals

4. **opendbc/car/hyundai/carcontroller.py**
   - Passes adjacent lane data to `create_ccnc()`

---

## Integration with openpilot/sunnypilot

To complete the integration, add this to your openpilot/sunnypilot code where radar is updated:

```python
# In controlsd.py or equivalent
if self.rk is not None:
    ret.radarState = self.rk.update(can_strings)
    
    # Update CarState with adjacent lane data
    self.CS.update_radar_tracks(self.rk)
```

---

## Questions?

If you encounter issues:
1. Run `python test_radar_simple.py` to verify basic logic
2. Enable debug output to see what the radar is actually detecting
3. Check the lateral positions of vehicles to tune the threshold
4. Verify CAN messages are being sent correctly

Good luck! 🚗 🚗 🚗
