from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

class Landing:
    def __init__(self):
        pass

    def land(self, vehicle):
        vehicle.mode = VehicleMode("LAND")
        print("Landing...")