from pymavlink import mavutil
import time
master = mavutil.mavlink_connection(
        '/dev/ttyUSB0',
        57600)

#master.reboot_autopilot()


master.wait_heartbeat()

master.mav.param_request_list_send(
	master.target_system,
	master.target_component
	)

while True:
	print('wtf')
	time.sleep(0.01)
	try:
		message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
		print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	except Exception as e:
		print(e)
		exit(0)

