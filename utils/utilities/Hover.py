from Attitude import Attitude

class Hover:
    def __init__(self):
        self.attitude = Attitude()
        pass

    def original_hover_no_gps(self, duration=0,vehicle=None, target_altitude=0):
        self.attitude.set_attitude(duration=duration, vehicle=vehicle, target_altitude=target_altitude)
        print("hovering for %d", duration)

    def hover(self, duration=0,vehicle=None, target_altitude=0):
        self.attitude.set_attitude(duration=duration, vehicle=vehicle, target_altitude=target_altitude)
        print("hovering for %d", duration)
