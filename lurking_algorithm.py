"""
    Lurking Algorithm 

    Decides on landing zones for Ras based on the results of analyzing
    the heatmap, sensor map, and weighted average.

    Can provide information both for long-term charging station locations
    and real-time assistance. 
"""

# Given the heatmap and slam map, decides where Ras's charging station should
# be docked. 

# Step 4: Get path and MSE about that path 
# Step 6: Where do we place the charger?
#         - Stay away from the path 
#         - Are we in a wall? If so, move until inside of wall facing person
#         - Are we in a tight space? If so, pick a close sensor as origin
#               and try again
#         - Are we annoying the user here? Input data later 

# language imports
import sys
import math

# library imports
import matplotlib.pyplot as plt
import numpy as np

from slam_map import SlamMap
from heatmap import Heatmap
from weighted_average import WeightedAverage

class ChargerPlacement():
    """
        Decides on a long-term "ideal" landing spot for Ras. 
    """
    def __init__(self, smarthome_data_filepath, slam_map_filepath):
        self.slam_map = SlamMap(slam_map_filepath)

        # a tuple (x,y) showing the average location of our occupant 
        self.weighted_average_point = WeightedAverage(smarthome_data_filepath).get_weighted_average_point() 

    def pick_charger_location(self):
        if self.check_if_we_are_inside_wall(self.weighted_average_point):
            print "oh no, we are in a wall"

        return "here"

    def check_if_we_are_inside_wall(self, point):
        return True

class LurkingAlgorithm():
    """
        Decides where Ras should be in realtime as the human moves around.
    """
    def __init__(self):
        print "nothing yet" 

def main():
    # get command line arguments
    smarthome_data_filepath = sys.argv[1]
    slam_map_filepath = sys.argv[2]

    charger_placement = ChargerPlacement(smarthome_data_filepath, slam_map_filepath)

    print charger_placement.pick_charger_location() 
    
if __name__ == "__main__":
    main()