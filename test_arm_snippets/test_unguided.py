def arm_and_takeoff_nogps(aTargetAltitude):
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.5
    # print("Basic pre-arm checks")
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

while not vehicle.armed:
    print(" Waiting for arming...")
    vehicle.armed = True
    time.sleep(1)
print("Taking off!")
global thrust
thrust = DEFAULT_TAKEOFF_THRUST
while True:
    #current_altitude = vehicle.location.global_relative_frame.alt
    #if sonar_alt<4.00:
    current_altitude = sonar_alt
    print(" Altitude: %f  Desired: %f" % (current_altitude, aTargetAltitude))
    if current_altitude >= aTargetAltitude * 0.95:
        print("Reached target altitude Borhan ")
        thrust = SMOOTH_TAKEOFF_THRUST
        break
    elif current_altitude >= aTargetAltitude * 0.6:
        thrust = SMOOTH_TAKEOFF_THRUST
    set_attitude(thrust=thrust)
    time.sleep(0.2)