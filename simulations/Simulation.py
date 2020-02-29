import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))

import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Drone import Drone
from Utils import Utils

class Simulation:
    def __init__(self, vehicle=None):
        self.Drone = Drone()