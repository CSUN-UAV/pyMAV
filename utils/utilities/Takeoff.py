class Takeoff:
    def __init__(self):
        pass

    def simple_drone_takeoff(self):
        pass
    
    def original_drone_takeoff_no_gps(self, target_altitude):
	    ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.65
        SMOOTH_TAKEOFF_THRUST = 0.55

        print "Arming motors, no gps"

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
            set_attitude(thrust = thrust)
            time.sleep(0.1)

    def original_drone_takeoff_gps(self):
        pass
