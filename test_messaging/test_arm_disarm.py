from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

import time
import sys
import pymavlink.mavutil as mavutil
#master = mavutil.mavlink_connection(
#        '/dev/ttyUSB0',
#        57600)

behicle = connect('/dev/ttyUSB0', baud=57600)

behicle.mode = VehicleMode("STABILIZE")

#while not behicle.is_armable:
#	print("waiting to arm")
#	time.sleep(1)
#behicle.mode= VehicleMode("GUIDED")
behicle.armed = True

behicle.flush()

print("Is armed:% s"%behicle.armed)

while not behicle.armed:
	print('waiting')
	behicle.armed = True
	time.sleep(2)
#while not behicle.armed: time.sleep(1)

print("taking off")

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

