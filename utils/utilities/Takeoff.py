from Attitude import Attitude
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math

class Takeoff:
    def __init__(self):
        self.attitude = Attitude()
        pass

    def drone_takeoff_with_gps(self, target_altitude=0, vehicle=None ):
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
            
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        while not vehicle.armed:      
            print(" Waiting to arm")
            time.sleep(0.4)

        print("Taking off!")
        vehicle.simple_takeoff(target_altitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt, "Want: ", target_altitude)      
            if vehicle.location.global_relative_frame.alt>=target_altitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(0.3)
    
    def original_drone_takeoff_no_gps(self, target_altitude, vehicle=None):
	    ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.65
        SMOOTH_TAKEOFF_THRUST = 0.55

        # simulation needed?
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        
        print "Arming motors, NO GPS"

        # SIMULATION NEEDS GUIDED MODE
        # vehicle.mode    = VehicleMode("GUIDED")
        vehicle.mode    = VehicleMode("GUIDED_NOGPS")
        vehicle.armed   = True

        while not vehicle.armed:
            print " Waiting for vehicle to arm..."
            vehicle.armed = True
            time.sleep(0.5)

        print "Taking off!"

        thrust = DEFAULT_TAKEOFF_THRUST

        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %(current_altitude, target_altitude))
            if current_altitude >= target_altitude*0.95: # Trigger just below target alt.
                thrust = 0.5
                print("Reached target altitude")
                break
            elif current_altitude >= target_altitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.attitude.set_attitude(thrust = thrust, vehicle=vehicle, target_altitude=target_altitude)
            time.sleep(0.1)

    def original_drone_takeoff_gps(self):
        pass
