from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import sched, time, math

connection_string = "/dev/serial0"
print("Connecting to...% s" % connection_string)
vehicle = connect(connection_string, baud=57600,wait_ready=False)
a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
from flight_assist import send_velocity

import threading

def arm_and_takeoff_gps(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	##### CONSTANTS #####
	DEFAULT_TAKEOFF_THRUST = 0.65
	SMOOTH_TAKEOFF_THRUST = 0.55

	print "Arming motors"

	# Copter should arm in GUIDED no gps mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True

	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		print " Waiting for vehicle to arm..."
		vehicle.armed = True
		time.sleep(1)

	print "Taking off!"

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
		print " Attitude: %s" % vehicle.attitude
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


def condition_yaw(heading, relative=False):
	"""
	Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

	This method sets an absolute heading by default, but you can set the `relative` parameter
	to `True` to set yaw relative to the current yaw heading.

	By default the yaw of the vehicle will follow the direction of travel. After setting 
	the yaw using this function there is no way to return to the default yaw "follow direction 
	of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

	For more information see: 
	http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	"""
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		1,          # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# send command to vehicle
	vehicle.send_mavlink(msg)




"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned LocationGlobal has the same `alt` value
	as `original_location`.

	The function is useful when you want to move the vehicle around specifying locations relative to 
	the current vehicle position.

	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius = 6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	if type(original_location) is LocationGlobal:
		targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
	else:
		raise Exception("Invalid Location object passed")
		
	return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.

	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
	"""
	Returns the bearing between the two LocationGlobal objects passed as parameters.

	This method is an approximation, and may not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""	
	off_x = aLocation2.lon - aLocation1.lon
	off_y = aLocation2.lat - aLocation1.lat
	bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
	if bearing < 0:
		bearing += 360.00
	return bearing;



"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
	MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use Vehicle.simple_goto (default) or 
	goto_position_target_global_int to travel to a specific position in metres 
	North and East from the current location. 
	This method reports distance to the destination.
"""

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
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# send command to vehicle
	vehicle.send_mavlink(msg)



def goto_position_target_local_ned(north, east, down):
	"""	
	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
	location in the North, East, Down frame.

	It is important to remember that in this frame, positive altitudes are entered as negative 
	"Down" values. So if down is "10", this will be 10 metres below the home altitude.

	Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
	ignored. For more information see: 
	http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

	See the above link for information on the type_mask (0=enable, 1=ignore). 
	At time of writing, acceleration and yaw bits are ignored.

	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111111000, # type_mask (only positions enabled)
		north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
		0, 0, 0, # x, y, z velocity in m/s  (not used)
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	# send command to vehicle
	vehicle.send_mavlink(msg)



def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
	"""
	Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

	The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
	the target position. This allows it to be called with different position-setting commands. 
	By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

	The method reports the distance to target every two seconds.
	"""
	
	currentLocation = vehicle.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_metres(currentLocation, targetLocation)
	gotoFunction(targetLocation)
	
	#print "DEBUG: targetLocation: %s" % targetLocation
	#print "DEBUG: targetLocation: %s" % targetDistance

	while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
		#print "DEBUG: mode: %s" % vehicle.mode.name
		remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
		print("Distance to target: ", remainingDistance)
		if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
			print("Reached target")
			break;
		time.sleep(0.5)

TARGET_ALTIDUDE = 0.5

arm_and_takeoff_gps(TARGET_ALTIDUDE)

print("Set groundspeed to 5m/s.")
vehicle.groundspeed=5

goto(5, 0)
goto(-5, 0)

t = threading.Thread(target=altitude_holder, args=(TARGET_ALTIDUDE,))
t.daemon = True
t.start()

print("Hold position for 3 seconds")
set_attitude(duration = 3)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

########WILL POST LANDING FUNCTION HERE

def land(vehicle):
	while not vehicle.location.global_relative_frame.alt==0:
		if(vehicle.location.global_relative_frame.alt <= .25):
			print("OUR ALTITUDE IS NOW LESS THAN 25m")
			vehicle.mode = VehicleMode('LAND')
			send_velocity(vehicle, 0, 0, -0.25, 1)
		elif(vehicle.location.global_relative_frame.alt > 30):
			print("OUR ALTITUDE IS NOW MORE THAN 25m")
			vehicle.mode = VehicleMode('LAND')
			send_velocity(vehicle, 0, 0, -0.25, 1)
		else:
			send_velocity(vehicle, 0, 0, -0.25, 1)
