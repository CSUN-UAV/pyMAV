# from pathlib import Path
# two_up = Path(__file__).resolve().parents[1]

import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

from Simulation import Simulation

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math

def test():
    simulation = Simulation()
    if simulation.Drone.vehicle:
        simulation.Drone.vehicle.close()
    simulation.Drone.connect_vehicle_to_test()

    drone = simulation.Drone.vehicle

    simulation.Drone.utils.takeoff.drone_takeoff_with_gps(target_altitude=0.5, vehicle=drone)

    #hover
    print("hovering..")
    simulation.Drone.utils.hover.hover(duration=5, vehicle=drone)

    print("going to target altitude 10")
    simulation.Drone.utils.change_altitude.gainAltitude(target_altitude=10, vehicle=drone)

    print("dropping to altitude 5")
    simulation.Drone.utils.change_altitude.loseAltitude(target_altitude=5, vehicle=drone)

    print("hovering...")
    simulation.Drone.utils.hover.hover(duration=5, vehicle=drone)

    print("landing...")
    simulation.Drone.utils.landing.land(vehicle=drone)

    print("completed")
    simulation.Drone.vehicle.close()

    # simulation.Drone.utils.hover.original_hover_no_gps(duration=5, vehicle=simulation.Drone.vehicle, target_altitude=3)

    # simulation.Drone.utils.goto.go_to_loc(vehicle=simulation.Drone.vehicle, North=5, East=0)

    # simulation.Drone.utils.goto.return_home(vehicle=simulation.Drone.vehicle)

    # simulation.Drone.utils.landing.land(vehicle=simulation.Drone.vehicle)
    # simulation.Drone.vehicle.close()
test()