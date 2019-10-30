from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

master.wait_heartbeat()

mode = 'STABILIZE'

if mode not in master.mode_mapping():
	print("Unknown:{}".format(mode))
	exit(1)

mode_id = master.mode_mapping()[mode]

print(mode_id)

master.mav.set_mode_send(
	master.target_system,
	mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	mode_id)

ack = False

while not ack:
	ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
	ack_msg = ack_msg.to_dict()
	
	print(ack_msg)


