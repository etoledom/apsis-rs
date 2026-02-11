# Drone Sim

Simple simulator of telemetry from a drone flying an automated mission.
The telemetry is transmited on 10Hz via websockets.

### The telemetry model is:
```JSON
{
  "vehicle_id": "drone-001",              // string, fixed for v1
  "timestamp_ns": 1700000000000000,       // i64 unix ns, monotonic
  "mission_time_s": 45.23,                // f64, time since mission start
  "mode": "Climb",                        // enum: Idle, Arming, Takeoff, Climb, Cruise, Descent, Landing, Landed
  "altitude_m": 125.4,                    // f64, 0-500m ASL
  "ground_speed_mps": 8.2,                // f64, 0-25 m/s
  "heading_deg": 270.1,                   // f64, 0-360 magnetic heading
  "lat_deg": 40.4168,                     // f64, e.g. Madrid area: 40.3–40.6
  "lon_deg": -3.7038,                     // f64, Madrid area: -3.8 to -3.6
  "battery_soc_pct": 78.5,                // f64, 0-100%
  "link_rssi": -65,                       // i32, -120 to 0 dBm
  "fault_flags": 0                        // u32, bitfield (1=low batt, 2=link loss, 4=sensor fail, etc.)
}
```

### Automated mission profile:

```
0–5s:     Idle (on ground, armed, alt=0)
5–15s:    Takeoff + Climb (to 200m, speed=5 m/s)
15–90s:   Cruise (hold 200m, speed=12 m/s, small heading drift)
90–105s:  Descent (to 0m, speed=8 m/s)
105–110s: Landing (touchdown, disarm)
110–150s: Idle (on ground, armed, alt=0)

```

### Dynamics model:

```
Idle (0–5s, 110–150s):
  target_alt=0m, ground_speed=0, lat/lon=fixed (home), heading=unchanged, battery=-0.1%/s

Takeoff+Climb (5–15s):
  target_alt=200m, vert_speed=+5 m/s, ground_speed→5 m/s, lat/lon=drift slowly forward, mode=Climb

Cruise (15–90s):
  target_alt=200m, ground_speed=12 m/s, heading=270° ±10° drift, lat/lon=move ~1km total, mode=Cruise

Descent (90–105s):
  target_alt=0m, vert_speed=-4 m/s, ground_speed=8 m/s, lat/lon→home, mode=Descent

Landing (105–110s):
  ground_speed→0, alt→0, lat/lon=home, mode=Landing→Landed
```

## Realism integration with added noise and faults

### Noise (always active):

- altitude: ±2m gaussian
- ground_speed: ±0.5 m/s
- heading: ±2°
- lat/lon: ±0.00005°
- battery: ±0.5%
- link_rssi: ±5 dBm

### Faults (~1–2% total probability per tick):

- GPS glitch (2%): lat/lon jump ±0.01° for 1–3 ticks
- Link fade (1%): rssi→-110 dBm, skip 1 packet, recover next
- Battery glitch (0.5%): soc spike/drop ±20% for 1 tick
- Sensor dropout (0.5%): altitude=-999 for 1 tick

### Telemetry connection

Via WebSocket URL:  ws://{vectra}/ws/telemetry?vehicle_id=drone-001

Payload: the JSON schema we defined (sent every 100ms = 10Hz)

Connection:
- On connect: send `{"type": "connect", "vehicle_id": "drone-001"}`
- Normal: send telemetry JSON every tick
- On disconnect/reconnect: resume from current mission_time
- Fallback (later): HTTP POST /telemetry every 1s batch

### Mission parameters:

Home position: lat=40.4168°, lon=-3.7038° (Madrid Retiro Park-ish)

Mission params:
- Climb target: 200m
- Cruise speed: 12 m/s (~43 km/h)
- Cruise distance: ~1km total (straight line or loose circle)
- Battery start: 100%, drain: 0.15%/s base + 0.05%/s per m/s climb
- Fault probs: as defined
- Tick rate: 10 Hz (100ms)

## Sim CLI params:

$drone-sim [options]

Options:
--ws-url <URL>           ws://localhost:8080/ws/telemetry (default)
--vehicle-id <ID>        drone-001 (default)
--mission-duration <s>   150s (default)
--tick-rate <hz>         10 (default)
--seed <u64>             42 (for reproducible faults)
--fault-prob <0.0-1.0>   0.02 total (default)
--dry-run                Print to stdout, no WS
