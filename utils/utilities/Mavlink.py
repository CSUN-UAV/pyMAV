from pymavlink import mavutil 

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