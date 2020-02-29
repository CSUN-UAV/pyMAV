from Utils import Utils
from dronekit import connect

class Drone:
    def __init__(self):
        self.utils = Utils()
        self.vehicle = None

    def connect_vehicle_to_test(self):
        print("Connecting to vehicle on: %s" % (self.utils.connect.connect_udp(),))
        self.vehicle = connect(self.utils.connect.connect_udp(), wait_ready=True)

    def connect_vehicle_to_serial0(self):
        print("Connecting to vehicle on: %s" % (self.utils.connect.connect_serial(),))
        self.vehicle = connect(self.utils.connect.connect_serial(), wait_ready=True)
