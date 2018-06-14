"""
    Dynamic Heatmap
"""
import sys

from heatmap import Heatmap 

class DynamicHeatmap(Heatmap):
    def __init__(self, smarthome_data_filepath):
        Heatmap.__init__(self, smarthome_data_filepath)

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    print "ooh, such empty"
    # dynamic_heatmap = DynamicHeatmap(smarthome_data_filepath) 