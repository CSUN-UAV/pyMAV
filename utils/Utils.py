from utilities import Attitude, Takeoff

class Utils:
    def __init__(self):
        self.attitude = Attitude.Attitude()
        self.takeoff = Takeoff.Takeoff()
        pass
    def hello(self):
        return "hello"