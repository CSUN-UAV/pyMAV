# from pymavlink.dialects.v20 import common as mavlink2

from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))