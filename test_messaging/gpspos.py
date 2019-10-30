
import time


from pymavlink import mavutil

master = mavutil.mavlink_connection(
        '/dev/ttyUSB0',
        57600)

#master.reboot_autopilot()



master.wait_heartbeat()


try:
        alt = the_connection_messages['GPS_RAW_INT'].alt
        timestamp = the_connection.time_since('GPS_RAW_INT')
except:
        print("NO GPS")

print("arming...")
master.mav.command_long_send(
	master.target_system,
	master.target_component,
	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
	0,
	1, 0, 0, 0, 0, 0, 0)
print("armed!")

while True:
	print("ted")
	time.sleep(2)
	master.mav.gps_input_send(
	0,
	0,
	8|16|32,
	0,
	0,
	3,
	0,
	0,
	100,
	1,
	1,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	7
)

