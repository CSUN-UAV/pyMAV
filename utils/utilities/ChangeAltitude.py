from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import Mavlink
import time

class ChangeAltitude:
    def __init__(self):
        pass

    def gainAltitude(self, vehicle=None, target_altitude=0.5, speedZ=-0.25):
        print("changing altitude to %f" %target_altitude)
        while True:
            curr = vehicle.location.global_relative_frame.alt
            print("Curr: %f, desired: %f", (curr, target_altitude))
            if curr >= target_altitude * 0.8:
                print("Reached target altitude")
                break
            Mavlink.send_velocity(vehicle=vehicle, velocity_x=0, velocity_y=0, velocity_z=speedZ)
            time.sleep(0.2)
    
    def loseAltitude(self, vehicle=None, target_altitude=0.5, speedZ=0.25):
        while True:
            curr = vehicle.location.global_relative_frame.alt
            print("Curr: %f, desired: %f", (curr, target_altitude))
            if curr - target_altitude <= 0.7:
                print("Reached target altitude")
                break
            Mavlink.send_velocity(vehicle=vehicle, velocity_x=0, velocity_y=0, velocity_z=speedZ)
            time.sleep(0.2)

        # self.attitude.set_attitude(vehicle=vehicle, target_altitude=target_altitude)
        # while True:
        #     print(" Altitude: ", vehicle.location.global_relative_frame.alt, "Want: ", target_altitude)      
        #     if vehicle.location.global_relative_frame.alt>=target_altitude*0.95: #Trigger just below target alt.
        #         print("Reached target altitude")
        #         break
        #     time.sleep(0.2)