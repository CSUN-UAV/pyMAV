from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math
#from flight_assist import send_velocity
connection_string = "/dev/serial0"
print("Connecting to...% s" % connection_string)
vehicle = connect(connection_string, baud=57600,wait_ready=False)
a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)

import threading

def arm_and_takeoff_no_gps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.65
    SMOOTH_TAKEOFF_THRUST = 0.55

    print ("Arming motors")

    # Copter should arm in GUIDED no gps mode
    vehicle.mode    = VehicleMode("GUIDED_NOGPS")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for vehicle to arm...")
        vehicle.armed = True
        time.sleep(1)

    print ("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            thrust = 0.5
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.4)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
#Check that vehicle has reached takeoff altitude
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print ("Reached target altitude")
            break
        vehicle.simple_goto(a_location)
    time.sleep(1)
    thrust = 0.7
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= aTargetAltitude*0.95:
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = 0.6
        set_attitude(thrust = thrust)
        time.sleep(1)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):

    #Duration is seconds to do this for

   msg = vehicle.message_factory.set_attitude_target_encode(
       0,
       0,                                         #target system
       0,                                         #target component
       0b00000000,                                #type mask: bit 1 is LSB
       to_quaternion(roll_angle, pitch_angle),    #q
       0,                                         #body roll rate in radian
       0,                                         #body pitch rate in radian
       math.radians(yaw_rate),                    #body yaw rate in radian
       thrust)                                    #thrust

       vehicle.send_mavlink(msg)

     if duration != 0:
       # Divide the duration into the frational and integer parts
       modf = math.modf(duration)
        
    #   # Sleep for the fractional part
       time.sleep(modf[0])
        
    #   # Send command to vehicle on 1 Hz cycle
       for x in range(0,int(modf[1])):
           time.sleep(1)
           vehicle.send_mavlink(msg)

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0.5, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def altitude_holder(target_altitude):
    ACCEPTABLE_ALTITUDE_ERROR = 0.15
    global current_thrust
    
    print("Altitdude holer started")
    while(vehicle.mode != "LAND"):
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f Target Altitude: %f " % (current_altitude, target_altitude))
        print(" Attitude: %s" % vehicle.attitude)
        #print " Velocity: %s" % vehicle.velocity
        #print " Groundspeed: %s" % vehicle.groundspeed    # settable

        if(current_altitude < target_altitude - ACCEPTABLE_ALTITUDE_ERROR):
            current_thrust += 0.01
            current_thrust = 0.65 if current_thrust > 0.65 else current_thrust
            print("THRUST UP")
        elif(current_altitude > target_altitude + ACCEPTABLE_ALTITUDE_ERROR):
            current_thrust -= 0.01
            current_thrust = 0.35 if current_thrust < 0.35 else current_thrust
            print("THRUST DOWN")
        else:
            current_thrust = 0.5
            print("THRUST HOLD")

        #set_thrust(current_thrust)
            time.sleep(0.1)

TARGET_ALTIDUDE = 0.5

arm_and_takeoff_no_gps(TARGET_ALTIDUDE)

t = threading.Thread(target=altitude_holder, args=(TARGET_ALTIDUDE,))
t.daemon = True
t.start()

print("Hold position for 3 seconds")
set_attitude(duration = 3)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

########WILL POST LANDING FUNCTION HERE
def set_velcoity_body(vehicle,vx,vy,vz):
    
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0,0,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b000111111000111,
    0,0,0,
    vx,vy,vz,
    0,0,0,
    0,0)
    vehicle.send_mavlink(msg)
    vehcile.flush()
        






def land():
    while not vehicle.location.global_relative_frame.alt==0:
        if(vehicle.location.global_relative_frame.alt <= .25):
            print("OUR ALTITUDE IS NOW LESS THAN 25m")
            vehicle.mode = VehicleMode("LAND")
            set_velocity_body(vehicle, 0, 0, 0, 1)
        elif(vehicle.location.global_relative_frame.alt > 30):
            print("OUR ALTITUDE IS NOW MORE THAN 25m")
            vehicle.mode = VehicleMode('LAND')
            set_velocity_body(vehicle, 0, 0, 0, 1)
        else:
            set_velocity_body(vehicle, 0, 0, 0, 1)





land()
##########


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


print("Completed")


# Copter should arm in GUIDED mode
# vehicle.mode    = VehicleMode("GUIDED_NOGPS")
# vehicle.armed   = True

# # Confirm vehicle armed before attempting to take off
# while not vehicle.armed:
#   print " Waiting for vehicle to arm..."
#   vehicle.armed = True
#   time.sleep(1)

# print("Hold position for 3 seconds")
# set_attitude(duration = 3)

