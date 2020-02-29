from utilities import Attitude, Takeoff, Connect, Hover, Landing
import sys

# from Attitude import Attitude
# from Connect import Connect
class Utils:
    def __init__(self, drone=None):
        self.attitude = Attitude.Attitude()
        self.takeoff = Takeoff.Takeoff()
        self.connect = Connect.Connect()
        self.hover = Hover.Hover()
        self.landing = Landing.Landing()
    def hello(self):
        return "hello"