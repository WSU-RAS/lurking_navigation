# language imports
import sys

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
    sensor_map["D002"] = (3.625,0.1125)
    sensor_map["T102"] = (3.625,0.1125)
    sensor_map["M004"] = (0.875,0.6875)
    sensor_map["M005"] = (2.0625,0.6875)
    sensor_map["M011"] = (3.3125,0.6875)
    sensor_map["M012"] = (4.525,0.6875)
    sensor_map["M003"] = (0.875,1.9375)
    sensor_map["M006"] = (2.0625,1.9375)
    sensor_map["M010"] = (3.3125,1.9375)
    sensor_map["M013"] = (4.525,1.9375)
    sensor_map["T001"] = (2.0625,1.9375)
    sensor_map["M002"] = (0.875,3.125)
    sensor_map["M007"] = (2.0625,3.125)
    sensor_map["M009"] = (3.3125,3.125)
    sensor_map["M014"] = (4.525,3.125)
    sensor_map["D013"] = (0.875,3.5)
    sensor_map["M008"] = (2.75,3.5)
    sensor_map["M001"] = (1.5625,4.0)
    sensor_map["M015"] = (4.1875,4.0)
    sensor_map["M023"] = (1.5625,5.15)
    sensor_map["M053"] = (4.1875,5.15)
    sensor_map["M016"] = (4.1875,5.15)
    sensor_map["M022"] = (1.5625,5.375)
    sensor_map["M017"] = (4.125,5.65)
    sensor_map["T002"] = (4.125,5.65)
    sensor_map["M021"] = (1.5625,6.5375)
    sensor_map["M019"] = (2.8125,6.5375)
    sensor_map["M018"] = (4.125,6.5375)
    sensor_map["M026"] = (0.6,6.625)
    sensor_map["M020"] = (2.8125,7.275)
    sensor_map["M051"] = (4.0625,7.8125)
    sensor_map["M025"] = (0.6,7.6875)
    sensor_map["M024"] = (1.5,7.5625)
    sensor_map["T101"] = (0.9375,8.40)
    sensor_map["D001"] = (0.9375,8.50)

    sensor_map["M041"] = (-1,-1)
    sensor_map["M032"] = (-1,-1)
    sensor_map["M039"] = (-1,-1)
    sensor_map["M027"] = (-1,-1)
    sensor_map["M045"] = (-1,-1)
    sensor_map["M037"] = (-1,-1)
    sensor_map["M028"] = (-1,-1) 
    sensor_map["M030"] = (-1,-1)
    sensor_map["M057"] = (-1,-1)
    sensor_map["M029"] = (-1,-1)
    sensor_map["M031"] = (-1,-1)
    sensor_map["M036"] = (-1,-1)
    sensor_map["M033"] = (-1,-1)
    sensor_map["M034"] = (-1,-1)
    sensor_map["M038"] = (-1,-1)
    sensor_map["M040"] = (-1,-1)
    sensor_map["M055"] = (-1,-1) 
    sensor_map["M054"] = (-1,-1)
    sensor_map["M056"] = (-1,-1)
    sensor_map["M042"] = (-1,-1)
    sensor_map["M043"] = (-1,-1)
    sensor_map["M035"] = (-1,-1)
    sensor_map["M044"] = (-1,-1)
    sensor_map["M050"] = (-1,-1)
    sensor_map["M049"] = (-1,-1)
    sensor_map["M046"] = (-1,-1)
    sensor_map["M048"] = (-1,-1)
    sensor_map["M047"] = (-1,-1)
    sensor_map["M052"] = (-1,-1)
    sensor_map["M058"] = (-1,-1)
    sensor_map["M059"] = (-1,-1)
    sensor_map["M060"] = (-1,-1)
    sensor_map["MA201"] = (-1,-1)
    sensor_map["MA202"] = (-1,-1)
    sensor_map["MA203"] = (-1,-1)
    sensor_map["MA204"] = (-1,-1)
    sensor_map["MA205"] = (-1,-1)
    sensor_map["MA206"] = (-1,-1)
    sensor_map["MA207"] = (-1,-1)
    return sensor_map

class Heatmap():
    """ Heatmap class for storing human activity hotness

    Loads smarthome data from a file and uses that to build a heatmap of the area representing
    human activity

    No public attributes

    """
    def __init__(self):
        # map of points, key: (x, y) value: heat value
        self.point_map = {}

        # constants
        self.map_width = 6.0 # in meters
        self.map_height = 9.0 # in meters
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

        # build the heatmap
        next_point = self._get_next_point(data_file, sensor_map)
        while next_point is not None:
            # add to the sensor map
            if next_point in self.point_map:
                self.point_map[next_point] += 1
            else:
                self.point_map[next_point] = 1

            # get the next point
            next_point = self._get_next_point(data_file, sensor_map)

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
        for point, value in self.point_map:
            x_index = int(point[0] / self.map_resolution)
            y_index = int(point[1] / self.map_resolution)

            heatmap[x_index][y_index] += value

        return heatmap

    def display_heatmap(self):
        heatmap_array = self.get_heatmap_array()
        np_heatmap = np.array(heatmap_array)
        plt.imshow(np_heatmap, cmap='hot', interpolation='nearest')
        plt.show()

    def _get_next_point(self, file, sensor_map):
        """ get the next point out of 
        """
        while True:
            next_line = file.readline()

            # check if the file is empty
            if next_line == False:
                return None

            data = DataLine(next_line)

            if data.sensor_type == "Control4-Motion" and data.message == "ON":
                # found a motion sensor trigger
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
    Probably not even worth existing.

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

    # build the heatmap
    heatmap = Heatmap()
    heatmap.load_from_file(data_filepath)

    # display the heatmap
    heatmap.display_heatmap()
