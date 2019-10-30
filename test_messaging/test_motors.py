import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

master.wait_heartbeat()

master.mav.command_long_send(
	master.target_system,
	master.target_component,
	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
	0,
	1, 0, 0, 0, 0, 0, 0)


master.mav.manual_control_send(
	master.target_system,
	500,
	-500,
	250,
	500,
	0)

buttons = 1 + 1 << 3 + 1 << 7

master.mav.manual_control_send(
	master.target_system,
	0,
	0,
	0,
	0,
	buttons)

