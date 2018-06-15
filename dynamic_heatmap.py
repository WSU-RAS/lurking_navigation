"""
    Dynamic Heatmap
"""
import sys

from heatmap import Heatmap 

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath):
        Heatmap.__init__(self, sensor_list_filepath)

    def _import_sensor_information(self):
        print "wow what a great method"

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    print "ooh, such empty"
    # dynamic_heatmap = DynamicHeatmap(sensor_list_filepath) 