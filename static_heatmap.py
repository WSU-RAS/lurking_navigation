"""
    Static Heatmap
"""
import sys

from config import Config
from heatmap import Heatmap, DataLine 
from weighted_average import WeightedAverage

class StaticHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, smarthome_data_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)
        self._load_from_file(smarthome_data_filepath)

    def _load_from_file(self, smarthome_data_filepath):
        """ load heatmap from filepath

        get all the data out from a file and load into our heatmap
        then normalize our heatmap.

        """
        # build the heatmap
        smarthome_data_file = open(smarthome_data_filepath, 'r')

        next_point = self._get_next_point(smarthome_data_file, self.sensor_map, self.ignored_sensors)

        while next_point is not None:
            # add to the sensor map
            if next_point in self.point_map:
                self.point_map[next_point] += 1
            else: 
                self.point_map[next_point] = 1

            # get the next point
            next_point = self._get_next_point(smarthome_data_file, self.sensor_map, self.ignored_sensors)

        # normalize the heatmap
        self._normalize_heatmap()

    def _get_next_point(self, smarthome_data_file, sensor_map, ignored_sensors):
        while True:
            next_line = smarthome_data_file.readline()

            # check if the file is empty
            if len(next_line) <= 0:
                return None

            data = DataLine(next_line)

            if data.sensor_type == "Control4-Motion" and data.message == "ON" and data.sensor_name not in ignored_sensors:
                # found a motion sensor trigger on the 1st floor
                point = sensor_map[data.sensor_name]
                return point

    def _normalize_heatmap(self):
        """ normalize the heatmap

        divide each value of the point_map by the sum of all values.

        """
        # sum up all the values
        sum = 0
        for key, value in self.point_map.iteritems():
            sum += value

        # divide each entry by the sum
        for key, value in self.point_map.iteritems():
            new_value = value / float(sum)
            self.point_map[key] = new_value

    def get_normalized_heatmap(self):
        return self.get_heatmap_array()

    def get_point_map(self):
        return self.point_map

if __name__ == "__main__":
    # Take in the file we will construct a heatmap from 
    sensor_list_filepath = sys.argv[1]
    smarthome_data_filepath = sys.argv[2]
    config_filepath = sys.argv[3]

    config = Config(config_filepath)

    static_heatmap = StaticHeatmap(sensor_list_filepath, smarthome_data_filepath, config) 
    static_heatmap.display_heatmap() 

