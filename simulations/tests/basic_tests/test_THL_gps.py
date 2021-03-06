import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))
from Simulation import Simulation

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math

def test():
    simulation = Simulation()
    simulation.Drone.connect_vehicle_to_test()

    simulation.Drone.utils.takeoff.drone_takeoff_with_gps(target_altitude=3, vehicle=simulation.Drone.vehicle)

    simulation.Drone.utils.hover.original_hover_no_gps(duration=5, vehicle=simulation.Drone.vehicle, target_altitude=3)

    simulation.Drone.utils.landing.land(vehicle=simulation.Drone.vehicle)
test()