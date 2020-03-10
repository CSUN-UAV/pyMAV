import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))

from Drone import Drone
def main():
    drone = Drone()

    target_altitude =  0.3

    drone.connect_vehicle_to_serial0()

    drone.utils.takeoff.original_drone_takeoff_no_gps(target_altitude=target_altitude, vehicle=drone.vehicle)

    drone.utils.hover.hover(duration=5, vehicle=drone.vehicle)

    drone.utils.change_altitude.gainAltitude(vehicle=drone.vehicle)

    drone.utils.hover.hover(duration=5, vehicle=drone.vehicle)

    drone.utils.landing.landing(vehicle=drone.vehicle)

    print("completed")

if __name__ == "__main__":
    main()
    