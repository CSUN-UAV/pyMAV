import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from Drone import Drone
from Utils import Utils
import os

class Simulation:
    def __init__(self, vehicle=None):
        self.Drone = Drone()
        self.pyclean()
    
    def pyclean(self):
        os.chdir("../../")
        os.system("./clean_pyc.sh")
        print("Cleaned python cache.")
