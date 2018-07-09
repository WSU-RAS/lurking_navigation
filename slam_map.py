"""
    Accepts slam map data and transforms the map to match the heatmap 
    created in the Heatmap class. 
"""

import numpy as np
import scipy
from scipy import ndimage
import matplotlib.pyplot as plt
import sys

from config import Config

class SlamMap():
    """ slam map class

    class containing the slam map in the same format as our heatmap.
    Goal: get the slam map to overlap with our heatmap

    """
    def __init__(self, data_filepath, config):
        # variable declarations
        self.map = None # note that this is a numpy array not a 2d array
        self.config = config

        # get raw slam data
        raw_data = RawSlamData(data_filepath)
        self.resolution = raw_data.resolution

        # transform slam map so that it has the same origin as our heatmap
        self.origin = self._get_new_origin()

        # fit into 2d array
        self.map = self._process_raw_data(raw_data)


    def _process_raw_data(self, raw_data):
        """ process raw data map

        Reshape the raw map into a 2d array then rotate around the origin.
        numpy makes this way too easy.

        """

        # load the map as a 2d array
        np_map = np.array(raw_data.data)
        np_map = np.reshape(np_map, (raw_data.width, raw_data.height), order='F')

        # clip uncertain values
        full = np.full_like(np_map, self.config.slam_uncertainty_cutoff)
        np_map = np.maximum(np_map, full)

        # scale to between 0 and 1
        np_map = np.subtract(np_map, full)
        amax = np.amax(np_map)
        np_map = np.true_divide(np_map, amax)

        # downsample to obtain the same resolution as the heatmap
        heatmap_resolution = self.config.map_resolution
        slam_resolution = self.resolution
        resolution_multiplier = slam_resolution / heatmap_resolution
        np_map = scipy.misc.imresize(np_map, resolution_multiplier)

        self.origin = tuple(i / heatmap_resolution for i in self.origin)

        self.original_slam_map = np_map

        # rotate the map around the origin
        # do 90 increment rotaion
        degree_rotation = self.config.slam_rotation
        full_rotations = int(degree_rotation / 90)

        if degree_rotation < 0:
            # negative partial
            partial_rotation = -(-degree_rotation % 90)
        else:
            # positive partial
            partial_rotation = degree_rotation % 90

        np_map = np.rot90(np_map, full_rotations)
        np_map = ndimage.rotate(np_map, partial_rotation, reshape=False)

        return np_map

    def _get_new_origin(self):
        """

        get the origin of the top left corner of the apartment in array values

        """
        origin = self.config.slam_origin
        return origin

    def display_as_heatmap(self):
        plt.imshow(np.transpose(self.map), cmap='hot', interpolation='nearest')

        # flip y axis
        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])

        plt.show()


class RawSlamData():
    """ slam map raw data class

    container class for loading and holding slam map raw data

    """
    def __init__(self, filepath):
        # variables
        self.resolution = 0.0
        self.width = 0
        self.height = 0
        self.origin = (0.0, 0.0)
        self.data = []

        # load data
        self._load_data(filepath)

    def _load_data(self, filepath):
        data_file = open(filepath, "r")
        line = data_file.readline()

        # drop the header
        while "resolution" not in line:
            line = data_file.readline()

        # load up resolution
        split = line.split()
        self.resolution = float(split[1])
        line = data_file.readline()

        # load up the width
        assert "width" in line
        split = line.split()
        self.width = int(split[1])
        line = data_file.readline()

        # load up the height
        assert "height" in line
        split = line.split()
        self.height = int(split[1])
        line = data_file.readline()

        # load up the origin
        line = data_file.readline()
        line = data_file.readline()
        assert "x" in line
        split = line.split()
        x = float(split[1])

        line = data_file.readline()
        assert "y" in line
        split = line.split()
        y = float(split[1])

        self.origin = (x, y)

        # load up the data
        while "data" not in line:
            line = data_file.readline()

        # get the array between [ and ]
        data = get_between(line, "[", "]")
        data = data.split(", ")
        data = [int(num) for num in data]
        self.data = data


def get_between(s, first, last):
    """ get string between strings

    get the first substring that exists between the first instances of both first and last
    (the first instance of last after the first instance of first obviously)
    code from cji on stack overflow

    parameters:
        s - string to operate on
        first - substring to start finding between
        last - substring to end finding between

    returns:
        substring in s that is between the leftmost instance of substring first
        and the leftmost instance of substring last AFTER substring first.

    """
    try:
        start = s.index(first) + len(first)
        end = s.index(last, start)
        return s[start:end]
    except ValueError:
        return ""

def main():
    # get command line arguments
    data_filepath = sys.argv[1]
    config_filepath = sys.argv[2]

    config = Config(config_filepath)

    slam_map = SlamMap(data_filepath, config)
    slam_map.display_as_heatmap()

if __name__ == "__main__":
    main()
