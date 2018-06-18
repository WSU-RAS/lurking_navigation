import sys
import math

from heatmap import Heatmap 

TIME_TICK_LENGTH = 8           # in seconds
HEATMAP_DECAY_STRENGTH = 0.8   # how quickly the heatmap decays

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath):
        Heatmap.__init__(self, sensor_list_filepath)

        """
            dictionary of sensors 
            update heatmap every time tick 
            display every time tick 
        """
        self.heatmap = [
                    [0.0 for j in range(int(math.ceil(self.map_height / self.map_resolution)))] 
                    for i in range(int(math.ceil(self.map_width / self.map_resolution)))
                  ]

    def _init_ras_location(self):
        print "nothing yet"
        return (42, 42)
    
    def _import_sensor_information(self):
        print "wow what a great method"

    def get_heatmap_array(self):
        """ get heatmap array

        get the heatmap as a 2d array indicating the hotness at each area.
        Note that the heatmap will be either normalized or not based on whether _normalize_heatmap has been run.

        """
        self.heatmap = self._decay_heatmap(self.heatmap)

        return self.heatmap

    def _decay_heatmap(self, heatmap):
        """
            Decay every point in the heatmap by a certain preset percentage. 
        """
        
        for row_index, row in enumerate(heatmap):
            for col_index, point in enumerate(row): 
                heatmap[row_index][col_index] += 3
        
        return heatmap

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    sensor_list_filepath = sys.argv[1]

    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath) 

    dynamic_heatmap.get_heatmap_array()