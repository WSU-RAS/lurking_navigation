"""
    Accepts data collected from a smarthome and creates a normalized heatmap
    reflecting how often each sensor is triggered. 
"""

# language imports
import sys
import math

# library imports
import matplotlib.pyplot as plt
import numpy as np

def get_sensor_map(filepath):
    """ get sensor map

    Given a file containing a list of sensors, builds two separate lists:
         - sensor_map: list of sensors in the smarthome and x-, y- coordinates about
         the origin
         - ignored_sensors: found using the IGNORED flag. Sensors that aren't tracked.

    sensor_map dictionary:
    key: string, sensor name
    value: tuple, (x, y)

    """
    sensor_map = {}
    ignored_sensors = {}

    data_file = open(filepath, "r")



    # sensor_map["D002"] = (0.1125, 3.625)
    

    return sensor_map, ignored_sensors

class Heatmap():
    """ Heatmap class for creating a grid of the smarthome, where each spot 
    contains the likelihood that a human will be there.

    Loads smarthome data from a file and uses that to build a heatmap of the area representing
    human activity

    No public attributes

    """
    def __init__(self, smarthome_data_filepath):
        # map of points, key: (x, y) value: heat value
        self.point_map = {}

        # map construction variables
        self.map_width = 9.0 # in meters
        self.map_height = 6.0 # in meters
        self.map_resolution = 0.125
        self.x_offset = 0 # x offset in map points
        self.y_offset = 0 # y offset in map points

        # self.load_from_file(smarthome_data_filepath)

        self.sensor_map = {}
        self.ignored_sensors = {}

    def load_from_file(self, filepath):
        """ load heatmap from filepath

        get all the data out from a file and load into our heatmap
        then normalize our heatmap.

        """
        # load the sensor map and a list of ignore sensors 
        self.sensor_map, self.ignored_sensors = get_sensor_map(filepath)

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

    def set_offset(self, offset_x, offset_y):
        """ Set the origin offset of the heatmap

        Adjusts all the points in the point_map to reflect the new offset.
        Adjusts the map width and height to fit the new offset.
        Note that this function handles adjusting the offset multiple times

        inputs:
            offset_x - x offset from 0,0 (which should be the top left corner of the map) 
                       in map points (NOT meters)
            offset_y - y offset from 0,0 in map points

        returns:
            None

        potential issues:
            adjustment of map dimensions and points may not be reliable after multiple offsets
                due to floating point precision
        
        """
        # setup variables
        old_x_offset = self.x_offset
        old_y_offset = self.y_offset
        new_x_offset = offset_x
        new_y_offset = offset_y

        self.x_offset = offset_x
        self.y_offset = offset_y

        # adjust pointmap
        new_point_map = {}
        for point, value in self.point_map.iteritems():
            new_x = point[0] + (new_x_offset - old_x_offset) * self.map_resolution
            new_y = point[1] + (new_y_offset - old_y_offset) * self.map_resolution
            new_point = (new_x, new_y)

            new_point_map[new_point] = value

        self.point_map = new_point_map

        # adjust map dimensions
        self.map_width += (new_x_offset - old_x_offset) * self.map_resolution
        self.map_height += (new_y_offset - old_y_offset) * self.map_resolution

    def get_heatmap_array(self):
        """ get heatmap array

        get the heatmap as a 2d array indicating the hotness at each area.
        Default resolution will be 1/8th meter squares
        Default size will be 6m x 9m

        Note that the heatmap will be either normalized or not based on whether _normalize_heatmap has been run.

        """
        heatmap = [
                    [0.0 for j in range(int(math.ceil(self.map_height / self.map_resolution)))] 
                    for i in range(int(math.ceil(self.map_width / self.map_resolution)))
                  ]

        # load things into the heatmap
        for point, value in self.point_map.iteritems():
            x_index = int(point[0] / self.map_resolution)
            y_index = int(point[1] / self.map_resolution)

            heatmap[x_index][y_index] += value

        return heatmap

    def display_heatmap(self):
        heatmap_array = self.get_heatmap_array()
        np_heatmap = np.array(heatmap_array)
        axis = plt.gca()

        # display heatmap
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')

        # display sensor text
        for sensor_name, point in self.sensor_map.iteritems():
            x = int(point[0] / self.map_resolution) + 1
            y = int(point[1] / self.map_resolution) + 1
            plt.text(x, y, sensor_name, bbox=dict(facecolor='white', alpha=0.8))

        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()

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

class DataLine():
    """ Data line

    Processess and stores a line of the datafile.
    Basically just a container class.

    """
    def __init__(self, line):
        split = line.split()

        self.date = split[0]
        self.time = split[1]
        self.sensor_name = split[2]
        self.message = split[3]
        self.sensor_type = split[4]

if __name__ == "__main__":
    print "not much, wbu"

    # build the heatmap and mse map
    # heatmap = Heatmap(smarthome_data_filepath)

    # display the heatmap
    # heatmap.display_heatmap()
