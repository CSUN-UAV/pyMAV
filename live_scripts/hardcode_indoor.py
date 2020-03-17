from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative	
from pymavlink import mavutil # Needed for command message definitions	
import sched, time, math

# connection_string = "/dev/serial0"
connection_string = "127.0.0.1:14551"
vehicle = connect(connection_string,wait_ready=True)

def arm_and_takeoff_no_gps(aTargetAltitude):
	DEFAULT_TAKEOFF_THRUST = 0.65	
	SMOOTH_TAKEOFF_THRUST = 0.55

	# simulation needed?
	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)

	print "Arming motors"	

	# Copter should be GUIDED if simulating/GPS available
	vehicle.mode = VehicleMode("GUIDED")
	# vehicle.mode    = VehicleMode("GUIDED_NOGPS")	
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
		time.sleep(0.2)

def send_velocity(vehicle=None, velocity_x=0, velocity_y=0, velocity_z=0, duration=0):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0,0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0, # x y z position
		velocity_x, velocity_y, velocity_z,
		0, 0, 0, # acceleration
		0, 0) #yaw, yaw_rate
	vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
				yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = True,
				duration = 0, thrust=0.5):
	send_attitude_target(roll_angle, pitch_angle,
						yaw_angle, yaw_rate, use_yaw_rate, thrust)
	start = time.time()
	while time.time() - start < duration:
		send_attitude_target(roll_angle, pitch_angle,
							yaw_angle, yaw_rate, use_yaw_rate,
							thrust)
		time.sleep(0.1)
	# Reset attitude, or it will persist for 1s more due to the timeout
	send_attitude_target(0, 0,
						0, 0, True, thrust=thrust)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
						yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
						thrust = 0.5):

    global current_thrust

    if not use_yaw_rate and yaw_angle is None:
        yaw_angle = vehicle.attitude.yaw

    if yaw_angle is None:
        yaw_angle = 0.0

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

def hover(duration = 1):
	set_attitude(duration=duration)

def gain_attitude(target_altitude=0.5, speedZ=-0.25):
	print("changing altitude to %f" %target_altitude)
	while True:
		curr = vehicle.location.global_relative_frame.alt
		print("Curr: %f, desired: %f", (curr, target_altitude))
		if curr >= target_altitude * 0.8:
			print("Reached target altitude")
			break
		send_velocity(vehicle=vehicle, velocity_x=0, velocity_y=0, velocity_z=speedZ)
		time.sleep(0.2)

def lose_altitude(target_altitude=0.5, speedZ=0.25):
	while True:
		curr = vehicle.location.global_relative_frame.alt
		print("Curr: %f, desired: %f", (curr, target_altitude))
		if curr - target_altitude <= 0.7:
			print("Reached target altitude")
			break
		send_velocity(vehicle=vehicle, velocity_x=0, velocity_y=0, velocity_z=speedZ)
		time.sleep(0.2)

def land():
	# vehicle.mode = VehicleMode("LAND")
	print("Landing...")
	while vehicle.location.global_relative_frame.alt>=0.15:
		print("Altitude: %f" %vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt<=0.5:
			print("Switching to Landing mode...")
			vehicle.mode = VehicleMode('LAND')
		send_velocity(velocity_x=0, velocity_y=0, velocity_z=0.25)
		time.sleep(0.2)
	print("Landed")

target_altitude = 10
arm_and_takeoff_no_gps(target_altitude)

print("Holding for 3 seconds")

set_attitude(duration=3)

print("gaining altitude 10 more meters")

gain_attitude(target_altitude=20)

print("Holding for 3 seconds")

set_attitude(duration=3)

print("Losing altitude")

lose_altitude(target_altitude=10)

print("Holding for 3 seocnds")

print("Going to land")

land()

print("Completed")