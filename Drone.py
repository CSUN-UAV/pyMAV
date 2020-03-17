from dronekit import connect, Command
from Utils import Utils
import os
import time

class Drone:
    def __init__(self):
        self.utils = Utils()
        self.vehicle = None
        self.pyclean()
        self.cmd = None
    # def cleanv2(self):
    #     self.cmd.clear()
    #     self.vehicle.flush()
    #     self.vehicle.reboot()


    def pyclean(self):
        os.chdir("..")
        os.system("./clean_pyc.sh")
        print("Cleaned python cache.")

    def connect_vehicle_to_test(self):
        print("Connecting to vehicle on: %s" % (self.utils.connect.connect_udp(),))
        self.vehicle = connect(self.utils.connect.connect_udp(), wait_ready=True)
        print 'CLEAR MISSION '
        self.vehicle.commands.download()
        self.vehicle.commands.clear()
        self.vehicle.flush()

    def connect_vehicle_to_serial0(self):
        print("Connecting to vehicle on: %s" % (self.utils.connect.connect_serial(),))
        self.vehicle = connect(self.utils.connect.connect_serial(), wait_ready=False, baud=57600, heartbeat_timeout=60)

