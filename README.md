# Autonomous Self-Recharging UAV — PX4 SITL starter

## Simulated features
- PX4 SITL flight through MAVSDK-Python
- Local-NED waypoint navigation
- Software battery drain and low-SOC trigger
- Return to a virtual overhead-line harvesting zone and land
- Virtual receiver-coil alignment search
- Voltage/current/temperature safety checks
- Simulated charging to target SOC and mission continuation

## Important limitations and safety
The transmission line is represented by a virtual waypoint. The power, temperature, alignment and battery models are illustrative software models, not validated electromagnetic or battery-chemistry simulations. No actual line-current, induced-voltage, EMI, HV insulation, or physical charger is modeled. Do not use this script on a real UAV or near live overhead power lines.

## Requirements
- PX4 Autopilot + Gazebo (SITL)
- Python 3.10+
- MAVSDK-Python

Install dependency:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Install and start PX4/Gazebo using the official guide for your PX4 version:
https://docs.px4.io/main/en/sim_gazebo_gz/
Ensure MAVSDK telemetry is available on UDP port 14540, then run:
```bash
python main.py
```
If your setup uses another UDP endpoint, edit `system_address` in `main.py`.

## First test
Set `low_battery_percent = 60.0` in `Config` to trigger charging early; after verifying the sequence, change it to 30.0.

## Tunable values
- `mission_waypoints`: local NED mission points (metres)
- `line_harvesting_point`: virtual harvesting point
- `starting_soc_percent`, `low_battery_percent`, `resume_battery_percent`
- `flight_drain_percent_per_second`, `charge_rate_percent_per_second`
- voltage/current/temperature limits

NED uses positive down, so altitude 5 m is `down = -5`.

## State machine
MISSION -> LOW SOC -> RETURN TO VIRTUAL HARVESTING ZONE -> LAND -> ALIGNMENT SEARCH
-> SAFETY CHECK -> SIMULATED CHARGE -> TARGET SOC -> TAKEOFF -> RESUME MISSION

## Known next steps
1. SOC is maintained by Python, not PX4 battery telemetry.
2. Alignment is a virtual search; the UAV does not physically move its coil.
3. Charging is simulated; no charger hardware is controlled.
4. A more realistic next version can add a custom Gazebo world and a separate low-voltage power-transfer model.
