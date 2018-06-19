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
    ignored_sensors = []

    data_file = open(filepath, "r")

    with data_file as file_pointer:
        line = "empty"

        while line:
            line = file_pointer.readline()
            line = line.replace('\r\n', '')
            sensor_information = line.split()

            # Only add the sensor to the map if we know where it is on the map 
            if (len(sensor_information) > 1):
                sensor_map[sensor_information[0]] = (float(sensor_information[1]), float(sensor_information[2]))
            elif (len(sensor_information) == 1): 
                # Add to the list of ignored sensors since it doesn't have coordinates 
                ignored_sensors.append(sensor_information[0])
    
    data_file.close() 
    return sensor_map, ignored_sensors

class Heatmap():
    """ Heatmap class for creating a grid of the smarthome, where each spot 
    contains the likelihood that a human will be there.

    Loads smarthome data from a file and uses that to build a heatmap of the area representing human activity

    No public attributes

    """
    def __init__(self, sensor_list_filepath, config):
        """ initialize the heatmap sensor list and config

        input:
            sensor_list_filepath - filepath to list of sensors
            config - config object containing configuration for the site we want to use
        """
        # map of points, key: (x, y) value: heat value
        self.point_map = {}

        # map construction variables
        self.map_width = config.map_width
        self.map_height = config.map_height
        self.map_resolution = config.map_resolution
        self.x_offset = 0
        self.y_offset = 0

        self.sensor_map, self.ignored_sensors = get_sensor_map(sensor_list_filepath)

        offset_point = config.slam_origin
        self.set_offset(offset_point[0], offset_point[1])

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
        # set up variables
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

        # adjust sensor map
        new_sensor_map = {}
        for sensor, point in self.sensor_map.iteritems():
            new_x = point[0] + (new_x_offset - old_x_offset) * self.map_resolution
            new_y = point[1] + (new_y_offset - old_y_offset) * self.map_resolution
            new_point = (new_x, new_y)

            new_sensor_map[sensor] = new_point

        self.sensor_map = new_sensor_map

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
        """
        for sensor_name, point in self.sensor_map.iteritems():
            x = int(point[0] / self.map_resolution) + 1
            y = int(point[1] / self.map_resolution) + 1
            plt.text(x, y, sensor_name, bbox=dict(facecolor='white', alpha=0.8))
        """

        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()

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
