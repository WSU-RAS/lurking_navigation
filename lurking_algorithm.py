"""
    Lurking Algorithm 

    Decides on landing zones for Ras based on the results of analyzing
    the heatmap, sensor map, and the weighted average. 

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

import math
# language imports
import sys

# library imports
import matplotlib.pyplot as plt
import numpy as np

from heatmap import Heatmap
from slam_map import SlamMap
from weighted_average import WeightedAverage


class ChargerPlacement():
    """
        Decides on a long-term "ideal" landing spot for Ras. 
    """
    def __init__(self, smarthome_data_filepath, slam_map_filepath):
        self.slam_map = SlamMap(slam_map_filepath)
        self.heatmap = Heatmap(smarthome_data_filepath)

        # set offset of heatmap to match slam map
        self.heatmap.set_offset(self.slam_map.origin[0], self.slam_map.origin[1])

        # a tuple (x,y) showing the average location of our occupant 
        self.weighted_average_point = WeightedAverage(self.heatmap).get_weighted_average_point() 

    def pick_charger_location(self):
        # (1) Get the long-term weighted average and use it to find the starting point
        selected_point = self.weighted_average_point

        # (2) Check if this point is in a tight space, if True, then find new point
        #       by branching out in 1/10 meter increments until not in a tight 
        #       space anymore. Use edge detection for walls later 

        # (3) Get distance from path until we are (certain distance) from a path 

        # (4) Are we inside a wall? Move until we are not in a wall 
        if self.check_if_inside_wall((30,50)):
            selected_point = self.move_out_of_wall(selected_point)
            print "ugh, a wall"

        return selected_point

    # Returns distance between the walls closest to the selected point 
    def get_room_width(self, point):
        return 42 

    # Returns distance from the selected point to the nearest travel line 
    def get_distance_from_path(self, point):
        return 42 

    # Returns true if selected point is inside a wall to a certain margin of error
    def check_if_inside_wall(self, point):
        stuff = self.slam_map.map[point[0]][point[1]]

        self.slam_map.display_as_heatmap() 

        print "here we got a", stuff

        return True

    # Moves until we are not inside a wall 
    def move_out_of_wall(self, point):
        return 42 

    # Displays the maps and the suggested charger placement 
    def display_charger_placement(self):
        print "can't wait to see this"

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
    charger_placement.display_charger_placement()

    print charger_placement.pick_charger_location() 
    
if __name__ == "__main__":
    main()
