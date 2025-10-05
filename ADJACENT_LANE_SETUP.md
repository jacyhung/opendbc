# Adjacent Lane Radar Tracking Setup

## ✅ Implementation Complete in opendbc

All the code for adjacent lane tracking is in opendbc. You need **ONE line** in openpilot/sunnypilot to connect it.

## 🔌 Integration (Two Changes Required)

In your **openpilot/sunnypilot** `selfdrive/controls/controlsd.py`:

### Change 1: Add 'liveTracks' to SubMaster (in `__init__`)

Around line 44, add `'liveTracks'` to the list:
```python
self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                               'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                               'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay', 'liveTracks'] + self.sm_services_ext,
                              poll='selfdriveState')
```

### Change 2: Update adjacent lanes in `state_control()` method

Find the `state_control()` method (around line 85) and add this at the **very beginning**:

```python
def state_control(self):
    # Update adjacent lane tracking from radar data BEFORE processing
    if self.sm.valid['liveTracks']:
        self.CI.CS.update_adjacent_lanes_from_live_tracks(self.sm['liveTracks'])
    
    CS = self.sm['carState']
    # ... rest of the method
```

**Important:** 
- Use `self.sm.valid['liveTracks']` not `self.sm.updated['liveTracks']`
- Add it at the START of `state_control()`, not in `update()`!

## 🎯 How It Works

```
RadarInterface (detects vehicles)
    ↓
CI.set_radar_interface() (links to controller)
    ↓
CarController (gets data directly)
    ↓
create_ccnc() (sends to cluster)
    ↓
Cluster Display 🚗 🚗 🚗
```

## 🧪 Testing

1. **Enable RADAR_FULL_RADAR** in your sunnypilot settings
2. **Add the one line** above to your openpilot code
3. **Drive** and watch the debug output every 2 seconds
4. **Check cluster** for adjacent lane vehicle icons

## 📊 Debug Output

You'll see output like this every 2 seconds:

```
======================================================================
[14:01:23] RADAR SUMMARY - Total: 12 tracks
======================================================================
Lane Distribution: CENTER=4 | LEFT=3 | RIGHT=5
✓ LEFT LEAD:    4.5m ahead, -2.25m lateral
✓ RIGHT LEAD:  18.7m ahead, +3.20m lateral

Closest vehicles per lane:
  LEFT lane (yRel: -5.5 to -1.8m):
    1.   4.5m ahead, -2.25m lateral,  -5.2m/s
    2.  45.8m ahead, -3.10m lateral,  +1.1m/s
  ...
```

## 🔧 Adjusting Thresholds

If vehicles aren't being detected correctly, adjust in `radar_interface.py` line 163:

```python
LANE_BOUNDARY = 1.8  # Adjust between 1.5-2.0 based on your roads
MAX_LATERAL = 5.5    # Maximum lateral distance to consider
```

## 📝 Files Modified in opendbc

1. `opendbc/car/hyundai/radar_interface.py` - Detects adjacent lane vehicles
2. `opendbc/car/hyundai/carstate.py` - Stores radar data (not used in this approach)
3. `opendbc/car/hyundai/hyundaicanfd.py` - Sends data to cluster via CCNC
4. `opendbc/car/hyundai/carcontroller.py` - Gets data from radar interface
5. `opendbc/car/hyundai/interface.py` - Links radar interface to controller

## ✨ That's It!

Just add that one line in openpilot and you're done! The radar is already detecting vehicles (as shown in your debug output with the truck at -2.25m lateral).
