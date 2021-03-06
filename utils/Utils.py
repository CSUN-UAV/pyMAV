from utils.utilities import Attitude, Takeoff, Connect, Hover, Landing, GoTo, ChangeAltitude
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
        self.goto = GoTo.GoTo()
        self.change_altitude = ChangeAltitude.ChangeAltitude()
    def hello(self):
        return "hello"