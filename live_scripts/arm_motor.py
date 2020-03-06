import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))

from Drone import Drone
def main():
    drone = Drone()

    drone.connect_vehicle_to_serial0()

    drone.utils.takeoff.original_drone_takeoff_no_gps()

    # drone.utils.goto.go_to_loc(vehicle=drone.vehicle, North=3, East=0)

    # drone.utils.goto.return_home(vehicle=drone.vehicle)

    # drone.utils.landing.land(vehicle=drone.vehicle)

    print("completed")

if __name__ == "__main__":
    main()
    