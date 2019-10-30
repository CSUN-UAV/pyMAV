
from pymavlink import mavutil

master = mavutil.mavlink_connection(
	'/dev/ttyUSB0',
	57600)

master.reboot_autopilot()
