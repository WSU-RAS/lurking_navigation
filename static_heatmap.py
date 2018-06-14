"""
    Static Heatmap
"""
import sys

from heatmap import Heatmap 

class StaticHeatmap(Heatmap):
    def __init__(self, smarthome_data_filepath):
        Heatmap.__init__(self, smarthome_data_filepath)

if __name__ == "__main__":
    # Take in the file we will construct a heatmap from 
    smarthome_data_filepath = sys.argv[1]

    static_heatmap = StaticHeatmap(smarthome_data_filepath) 