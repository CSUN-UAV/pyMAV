import math
import time
from dronekit import VehicleMode, LocationGlobal, LocationGlobalRelative

class GoTo:
    def __init__(self):
        self.original_location = None
        pass

    def go_to_loc(self, vehicle=None, North=0, East=0):
        currentLocation = vehicle.location.global_relative_frame
        print(currentLocation, "curr loc")
        if not self.original_location:
            self.original_location = currentLocation
        targetLocation = self.get_location_metres(currentLocation, North, East)
        print(targetLocation, "targetloc")
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        vehicle.simple_goto(targetLocation)
        while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            remainingDistance=self.get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
                print("Reached target, returning...")
                break;
            time.sleep(0.25)
    
    def return_home(self, vehicle=None):
        current_location = vehicle.location.global_relative_frame
        vehicle.simple_goto(self.original_location)
        targetDistance = self.get_distance_metres(current_location, self.original_location)
        while vehicle.mode.name=="GUIDED":
            remaining_dist = self.get_distance_metres(vehicle.location.global_relative_frame, self.original_location)
            print("Distance to home: ", remaining_dist)
            if remaining_dist <= targetDistance * 0.1:
                print("returned home.")
                break
            time.sleep(0.25)
                


    def get_location_metres(self, original_location, dNorth, dEast):
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
            
        return targetlocation;

    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5