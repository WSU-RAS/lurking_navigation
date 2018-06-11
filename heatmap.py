# constructs a heatmap of where the human is over time given smarthome data 
#   and uses that heatmap to create an MSE map that

# language imports
import sys
import string

# library imports
import matplotlib.pyplot as plt
import numpy as np

def get_sensor_map():
    """ get sensor map

    builds and returns a hardcoded dictionary mapping sensor names to their locations in the kyoto smarthome

    key: string, sensor name
    value: tuple, (x, y)

    """
    sensor_map = {}
    sensor_map["D002"] = (0.1125,3.625)
    sensor_map["T102"] = (0.1125,3.625)
    sensor_map["M004"] = (0.6875,0.875)
    sensor_map["M005"] = (0.6875,2.0625)
    sensor_map["M011"] = (0.6875,3.3125)
    sensor_map["M012"] = (0.6875,4.525)
    sensor_map["M003"] = (1.9375,0.875)
    sensor_map["M006"] = (1.9375,2.0625)
    sensor_map["M010"] = (1.9375,3.3125)
    sensor_map["M013"] = (1.9375,4.525)
    sensor_map["T001"] = (1.9375,2.0625)
    sensor_map["M002"] = (3.125,0.875)
    sensor_map["M007"] = (3.125,2.0625)
    sensor_map["M009"] = (3.125,3.3125)
    sensor_map["M014"] = (3.12,4.525)
    sensor_map["D013"] = (3.5,0.875)
    sensor_map["M008"] = (3.5,2.75)
    sensor_map["M001"] = (4.0,1.5625)
    sensor_map["M015"] = (4.0,4.1875)
    sensor_map["M023"] = (5.15,1.5625)
    sensor_map["M053"] = (5.15,4.1875)
    sensor_map["M016"] = (5.15,4.1875)
    sensor_map["M022"] = (5.375,1.5625)
    sensor_map["M017"] = (5.65,4.125)
    sensor_map["T002"] = (5.65,4.125)
    sensor_map["M021"] = (6.5375,1.5625)
    sensor_map["M019"] = (6.5375,2.8125)
    sensor_map["M018"] = (6.5375,4.125)
    sensor_map["M026"] = (6.625,0.6)
    sensor_map["M020"] = (7.275,2.8125)
    sensor_map["M051"] = (7.8125,4.0625)
    sensor_map["M025"] = (7.6875,0.6)
    sensor_map["M024"] = (7.5625,1.5)
    sensor_map["T101"] = (8.40,0.9375)
    sensor_map["D001"] = (8.50,0.9375)

    return sensor_map

def get_ignored_sensors():
    """ get ignored sensors

    returns a list of all the sensors on the 2nd floor

    """
    # Values in the following list are on the 2nd floor and will be ignored
    ignored_sensors = ["M041","M032","M039","M027","M045","M037","M028","M030","M057","M029","M031","M036","M033","M034","M038","M040","M055","M054","M056","M042","M043","M035","M044","M050","M049","M046","M048","M047","M052","M058","M059","M060","MA201","MA202","MA203","MA204","MA205","MA206","MA207"]

    return ignored_sensors

class Heatmap():
    """ Heatmap class for creating a grid of the smarthome, where each spot 
    contains the likelihood that a human will be there.

    Loads smarthome data from a file and uses that to build a heatmap of the area representing
    human activity

    No public attributes

    """
    def __init__(self):
        # map of points, key: (x, y) value: heat value
        self.point_map = {}

        # constants
        self.map_width = 9.0 # in meters
        self.map_height = 6.0 # in meters
        self.map_resolution = 0.125

    def load_from_file(self, filepath):
        """ load heatmap from filepath

        get all the data out from a file and load into our heatmap
        then normalize our heatmap.

        """
        # open the file
        data_file = open(filepath, "r")

        # load the sensor map
        sensor_map = get_sensor_map()
        ignored_sensors = get_ignored_sensors()

        # build the heatmap
        next_point = self._get_next_point(data_file, sensor_map, ignored_sensors)
        while next_point is not None:
            # add to the sensor map
            if next_point in self.point_map:
                self.point_map[next_point] += 1
            else:
                self.point_map[next_point] = 1

            # get the next point
            next_point = self._get_next_point(data_file, sensor_map, ignored_sensors)

        # normalize the heatmap
        self._normalize_heatmap()

    def get_heatmap_array(self):
        """ get heatmap array

        get the heatmap as a 2d array indicating the hotness at each area.
        Default resolution will be 1/8th meter squares
        Default size will be 6m x 9m

        Note that the heatmap will be either normalized or not based on whether _normalize_heatmap has been run.

        """
        heatmap = [
                    [0.0 for j in range(int(self.map_height / self.map_resolution))] 
                    for i in range(int(self.map_width / self.map_resolution))
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
        plt.imshow(np_heatmap, cmap='hot', interpolation='nearest')
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
    # get command line arguments
    data_filepath = sys.argv[1]

    # build the heatmap and mse map
    heatmap = Heatmap()

    heatmap.load_from_file(data_filepath)
    heatmap.display_heatmap()
