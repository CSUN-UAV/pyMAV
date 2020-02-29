import math
import time
class Attitude:
    def __init__(self):
        pass

    def set_attitude(self,roll_angle = 0.0, pitch_angle = 0.0,
                    yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                    thrust = 0.5, duration = 0, vehicle=None, target_altitude=0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                            yaw_angle, yaw_rate, False,
                            thrust, vehicle=vehicle)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                yaw_angle, yaw_rate, False,
                                thrust, vehicle=vehicle)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                            0, 0, True,
                            thrust, vehicle=vehicle)

        vehicle.simple_takeoff(target_altitude) # Take off to target altitude
        # Check that vehicle has reached takeoff altitude
        while True:
            print " Altitude: ", vehicle.location.global_relative_frame.alt 
            #Break and return from function just below target altitude.        
            if vehicle.location.global_relative_frame.alt>=target_altitude*0.95: 
                print "Reached target altitude"
                break
        time.sleep(1)
        thrust = 0.7
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            if current_altitude >= target_altitude*0.95:
                break
            elif current_altitude >= target_altitude*0.6:
                thrust = 0.6
            set_attitude(thrust = thrust)
            time.sleep(.1)

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                            yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                            thrust = 0.5, vehicle=None):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        vehicle.send_mavlink(msg)

    def to_quaternion(self,roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    # def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):

    #     # Duration is seconds to do this for

    # 	msg = vehicle.message_factory.set_attitude_target_encode(
    #     	0,
    #     	0,                                         #target system
    #     	0,                                         #target component
    #     	0b00000000,                                #type mask: bit 1 is LSB
    #     	to_quaternion(roll_angle, pitch_angle),    #q
    #     	0,                                         #body roll rate in radian
    #     	0,                                         #body pitch rate in radian
    #     	math.radians(yaw_rate),                    #body yaw rate in radian
    #     	thrust)                                    #thrust

    #     vehicle.send_mavlink(msg)

    #     if duration != 0:
    #     	# Divide the duration into the frational and integer parts
    #     	modf = math.modf(duration)
            
    #     	# Sleep for the fractional part
    #     	time.sleep(modf[0])
            
    #     	# Send command to vehicle on 1 Hz cycle
    #     	for x in range(0,int(modf[1])):
    #     		time.sleep(1)
    #     		vehicle.send_mavlink(msg)