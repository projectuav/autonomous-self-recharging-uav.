import asyncio, math, time
from dataclasses import dataclass
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

@dataclass
class Config:
    system_address: str = "udpin://0.0.0.0:14540"
    mission_waypoints = ((0.,0.,-5.,0.),(12.,0.,-5.,0.),(12.,10.,-5.,90.),(0.,10.,-5.,180.))
    line_harvesting_point = (0.,0.,-5.,0.)
    starting_soc_percent: float = 65.0
    low_battery_percent: float = 30.0
    resume_battery_percent: float = 80.0
    flight_drain_percent_per_second: float = 3.0
    charge_rate_percent_per_second: float = 0.35
    charge_voltage_min_v: float = 10.0
    charge_voltage_max_v: float = 16.8
    charge_current_max_a: float = 3.0
    charge_temperature_max_c: float = 45.0
    alignment_steps: int = 8
    alignment_step_m: float = 0.02
CFG=Config()

class EnergyModel:
    def __init__(self,soc): self.soc=float(soc); self.last=time.monotonic()
    def drain_tick(self):
        now=time.monotonic(); dt=now-self.last; self.last=now
        self.soc=max(0.,self.soc-CFG.flight_drain_percent_per_second*dt)
    def charge_tick(self,mult=1.):
        now=time.monotonic(); dt=now-self.last; self.last=now
        self.soc=min(100.,self.soc+CFG.charge_rate_percent_per_second*mult*dt)

def received_power(offset):
    # Illustrative curve only; not an electromagnetic field solver.
    v=12.; factor=math.exp(-((offset/.08)**2)); i=2.5*factor
    p=v*i; temp=28.+8.*(i/2.5)
    safe=(CFG.charge_voltage_min_v<=v<=CFG.charge_voltage_max_v and
          i<=CFG.charge_current_max_a and temp<CFG.charge_temperature_max_c)
    return v,i,p,temp,safe

async def wait_connected(drone):
    print("Connecting to PX4 SITL...")
    async for s in drone.core.connection_state():
        if s.is_connected: print("Connected."); return

async def wait_ready(drone):
    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok:
            print("Local/home position ready."); return

async def stop_offboard(drone):
    try: await drone.offboard.stop()
    except OffboardError: pass

async def move_to(drone,n,e,d,yaw,timeout=45):
    print(f"Navigate N={n:.1f}, E={e:.1f}, D={d:.1f} m")
    await drone.offboard.set_position_ned(PositionNedYaw(n,e,d,yaw))
    try: await drone.offboard.start()
    except OffboardError as ex: raise RuntimeError(f"Offboard failed: {ex}")
    start=time.monotonic()
    async for pv in drone.telemetry.position_velocity_ned():
        p=pv.position
        err=math.sqrt((p.north_m-n)**2+(p.east_m-e)**2+(p.down_m-d)**2)
        if err<0.8: break
        if time.monotonic()-start>timeout:
            print("Waypoint timeout; retaining best-effort setpoint."); break
        await asyncio.sleep(.2)

async def takeoff(drone,alt=5.):
    print("Arming/takeoff...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(alt)
    await drone.action.takeoff()
    await asyncio.sleep(5)

async def land(drone):
    await stop_offboard(drone)
    print("Landing at virtual harvesting zone...")
    await drone.action.land()
    async for flying in drone.telemetry.in_air():
        if not flying: print("Landed."); return
        await asyncio.sleep(.5)

async def optimize_alignment():
    print("Virtual coil-alignment search:")
    best=None
    for k in range(CFG.alignment_steps+1):
        offset=(k-CFG.alignment_steps//2)*CFG.alignment_step_m
        v,i,p,t,safe=received_power(offset)
        print(f" offset={offset:+.2f}m V={v:.1f}V I={i:.2f}A P={p:.1f}W T={t:.1f}C {'SAFE' if safe else 'UNSAFE'}")
        if safe and (best is None or p>best["p"]):
            best={"offset":offset,"v":v,"i":i,"p":p,"t":t}
    if best is None: raise RuntimeError("No safe alignment; charging aborted.")
    print(f"Selected offset {best['offset']:+.2f}m, modeled power {best['p']:.1f}W")
    return best

async def charge(energy):
    result=await optimize_alignment()
    print("Simulating energy harvesting and Li-Po charging (software model only)...")
    energy.last=time.monotonic()
    while energy.soc<CFG.resume_battery_percent:
        v,i,p,t,safe=received_power(result["offset"])
        if not safe: print("Safety limit exceeded; stop charging."); return
        energy.charge_tick(min(1.,p/30.))
        print(f" SOC={energy.soc:5.1f}% P_rx={p:4.1f}W V={v:.1f}V I={i:.2f}A T={t:.1f}C")
        await asyncio.sleep(1.)
    print(f"Target reached: {energy.soc:.1f}%")

async def main():
    drone=System(); energy=EnergyModel(CFG.starting_soc_percent)
    await drone.connect(system_address=CFG.system_address)
    await wait_connected(drone); await wait_ready(drone)
    await takeoff(drone)
    for idx,(n,e,d,yaw) in enumerate(CFG.mission_waypoints):
        if energy.soc<=CFG.low_battery_percent:
            print(f"LOW BATTERY: {energy.soc:.1f}%")
            ln,le,ld,ly=CFG.line_harvesting_point
            await move_to(drone,ln,le,ld,ly); await land(drone)
            await charge(energy); await takeoff(drone)
            print("Resuming mission.")
        await move_to(drone,n,e,d,yaw)
        energy.last=time.monotonic()
        await asyncio.sleep(4.)
        energy.drain_tick()
        print(f"Waypoint {idx+1}/{len(CFG.mission_waypoints)}; modeled SOC={energy.soc:.1f}%")
    await stop_offboard(drone); await drone.action.land()
    print("Mission complete.")
if __name__=="__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("Stopped.")
