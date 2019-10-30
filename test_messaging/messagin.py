from pymavlink.dialects.v20 import common as mavlink2

from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', 57600)

# Wait for the first heartbeat 

the_connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
	msg = the_connection_recv_match(blocking=True)
	print(msg)

try:
	alt = the_connection_messages['GPS_RAW_INT'].alt
	timestamp = the_connection.time_since('GPS_RAW_INT')
except:
	print("NO GPS")


