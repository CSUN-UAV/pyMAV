import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), "../utils"))

from Utils import Utils

class Simulation:
    def __init__(self):
        utils = Utils()
        print(utils.hello())

simulation = Simulation()
