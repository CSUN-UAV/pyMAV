from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import Mavlink
import time
class Landing:
    def __init__(self):
        pass

    def land(self, vehicle=None):
        # vehicle.mode = VehicleMode("LAND")
        print("Landing...")
        while vehicle.location.global_relative_frame.alt>=0.15:
            print("Altitude: %f" %vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt<=0.5:
                print("Switching to Landing mode...")
                vehicle.mode = VehicleMode('LAND')
            Mavlink.send_velocity(vehicle=vehicle, velocity_x=0, velocity_y=0, velocity_z=0.25)
            time.sleep(0.2)
        print("Landed")
