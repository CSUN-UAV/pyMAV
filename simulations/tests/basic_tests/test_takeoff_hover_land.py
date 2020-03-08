import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))
from Simulation import Simulation

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math


# ORIGINAL TAKEOFF
def test():
    simulation = Simulation()
    simulation.Drone.connect_vehicle_to_test()
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    print("ready to arm")
    simulation.Drone.utils.takeoff.original_drone_takeoff_no_gps(target_altitude=1, vehicle=simulation.Drone.vehicle)
    print("ready to hover")
    simulation.Drone.utils.hover.original_hover_no_gps(duration=3, vehicle=simulation.Drone.vehicle, target_altitude=1)
    print("ready to land")
    simulation.Drone.utils.landing.land(simulation.Drone.vehicle)
    print("Completed...")


# run missionplanner or something similar
test()