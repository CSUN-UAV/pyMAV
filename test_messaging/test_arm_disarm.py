
#from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#import time
#import sys
#import pymavlink.mavutil as mavutil
#from pvmavlink import mavutil
from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)
print("ping")
print(vehicle.mode)
print(vehicle.gps_0)
def arm_and_takeoff(takeoff_alt):
	print("prearm")
	while not vehicle.is_armable:
		print("waiting for vehicle to initialize")
		time.sleep(1)
	print("Arming motors")
	#vehicle.mode = VehicleMode("STABILIZE")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	vehicle.simple_takeoff(takeoff_alt)
	while True:
		print("altitude ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=takeoff_alt*.95:
			print("Reached Target Altitude..")
			break
		time.sleep(2)
	#while not vehicle.armed:
	#	print("waiting to arm..")
	#	time.sleep(1)

	print("taking off")
arm_and_takeoff(10)

def send_body_velocity(v_x, v_y, v_z, duration=0):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_NED,0b0000111111000111, 0, 0, 0, v_x, v_y, v_z,0,0,0,0,0)
	for x in range(0, duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)

v_x = 0
v_y = 5
v_z = 0
duration = 10

send_body_velocity(v_x, v_y, v_z, duration)

vehicle.mode = VehicleMode("LAND")

#master = mavutil.mavlink_connection(
#        '/dev/ttyUSB0',
#        57600)
###--------------
#mav=mavutil.mavlink_connection('tcp:127.0.0.1:5760')
#mav.wait_heartbeat()#mav.mav.command_long_send(mav.target_system,mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)

#time.sleep(3)
#mav.mav.command_long_send(mav.target_system,mav.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)




####

#behicle = connect('/dev/ttyUSB0', baud=57600)

#vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=False)

#print("pre arm check")
#print(vehicle.mode)

#vehicle.mode = VehicleMode("STABILIZE")
#vehicle.armed = True

#while not vehicle.is_armable:
#	print("waiting...")
#	time.sleep(1)

#time.sleep(5)
#print("disarming")
#vehicle.armed = False

#behicle.mode = VehicleMode("STABILIZE")

#while not behicle.is_armable:
#	print("waiting to arm")
#	time.sleep(1)
#behicle.mode= VehicleMode("GUIDED")
#behicle.armed = True

#behicle.flush()

#print("Is armed:% s"%behicle.armed)

#while not behicle.armed:
#	print('waiting')
#	behicle.armed = True
#	time.sleep(2)
#while not behicle.armed: time.sleep(1)

#print("taking off")

#master.mav.heartbeat_send(
#	6,
#	8,
#	192,
#	8,
#	4,
#	3)

#master.mav.command_long_send(
#	1,
#	1,
#	400,
#	0,
#	1,
#	0, 0, 0, 0, 0, 0)

#time.sleep(2)

#master.mav.command_long_send(
#	1,
#	1,
#	400,
#	0,
#	0,
#	0, 0, 0, 0, 0, 0)

#disarm
#master.mav.command_long_send(
#	master.target_system,
#	master.target_component,
#	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#	0,
#	0, 0, 0, 0, 0, 0, 0)

