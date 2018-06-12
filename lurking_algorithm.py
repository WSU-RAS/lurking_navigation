"""
    Lurking Algorithm 

    Decides on landing zones for Ras based on the results of analyzing
    the heatmap, sensor map, and weighted average.

    Can provide information both for long-term charging station locations
    and real-time assistance. 
"""

# Given the heatmap and slam map, decides where Ras's charging station should
# be docked. 

# Step 1: Input files: slam map, sensor map
# Step 2: Process slam map, create heatmap from sensor map
# Step 3: Get weighted average using heatmap
# Step 4: Get path and MSE about that path 
# Step 5: Set "starting position" of algorithm to result of weighted_average
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

class ChargerPlacement():
    """
        Decides on a long-term "ideal" landing spot for Ras. 
    """
    def __init__(self, smarthome_data_filepath, slam_map_filepath):
        self.heatmap = Heatmap(smarthome_data_filepath)
        self.slam_map = SlamMap(slam_map_filepath)

    def pick_charger_location(self):


        return "what about here?"

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