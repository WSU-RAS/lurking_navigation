"""
    Static Heatmap
"""
import sys
from heatmap import Heatmap 

class StaticHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, smarthome_data_filepath):
        Heatmap.__init__(self, sensor_list_filepath, smarthome_data_filepath)

    def _load_from_file(self, filepath):
        """ load heatmap from filepath

        get all the data out from a file and load into our heatmap
        then normalize our heatmap.

        """
        

        print self.sensor_map
        print self.ignored_sensors

        # build the heatmap
        next_point = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)
        while next_point is not None:
            # add to the sensor map
            if next_point in self.point_map:
                self.point_map[next_point] += 1
            else:
                self.point_map[next_point] = 1

            # get the next point
            next_point = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)

        # normalize the heatmap
        self._normalize_heatmap()

    def _get_next_point(self, file, sensor_map, ignored_sensors):
        """ get the next point out of 
        """
        while True:
            next_line = file.readline()

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

if __name__ == "__main__":
    # Take in the file we will construct a heatmap from 
    sensor_list_filepath = sys.argv[1]
    smarthome_data_filepath = sys.argv[2]

    static_heatmap = StaticHeatmap(sensor_list_filepath, smarthome_data_filepath) 