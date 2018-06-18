import sys
from heatmap import Heatmap 

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, smarthome_data_source):
        Heatmap.__init__(self, sensor_list_filepath)

        """
            dictionary of sensors 
            update heatmap every time tick 
            display every time tick 
        """

        print smarthome_data_source

    def _import_sensor_information(self):
        print "wow what a great method"

    def 

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    sensor_list_filepath = sys.argv[1]
    smarthome_data_source = sys.argv[2]

    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath, smarthome_data_source) 