"""
python base_placement_algorithm.py ~/smarthome_data/casas_slam_map.txt ~/smarthome_data/tokyo_sensors.txt ~/smarthome_data/tokyo.20170301-20170302_20180613.203344.txt 
"""

import sys

import numpy as np
import matplotlib.pyplot as plt

from config import Config
from slam_map import SlamMap
from reachability_map import ReachabilityMap
from heatmap import Heatmap
from static_heatmap import StaticHeatmap
from path_map import PathMap
from weighted_average import WeightedAverage
from wall_map import WallMap
from cramped_map import CrampedMap

# utility
from path_map import get_point_distance
from overlay import get_custom_colormap_green

class BasePlacer:
    """ Finds where the robot should be placed based on our metrics

    Loads up the heatmap, slam map, path map, and weighted average.
    Then uses each of these metrics to find the best point by finding
    a weighted value for every point on the map.

    """
    def __init__(self, slam_data_filepath, sensor_list_filepath, smarthome_data_filepath, config):
        # variables
        self.map = None
        self.slam_weight = 1.0
        self.wa_weight = 0.1
        self.path_weight = 1500.0
        self.wall_weight = 0.5
        self.cramped_weight = 100

        # load slam map
        self.slam_map = slam_map = SlamMap(slam_data_filepath, config)

        # load reachability map
        self.reachability_map = reachability_map = ReachabilityMap(slam_map)

        # load heatmap
        self.static_heatmap = static_heatmap = StaticHeatmap(sensor_list_filepath,
                                                             smarthome_data_filepath,
                                                             config)
        static_heatmap.set_offset(slam_map.origin[0], slam_map.origin[1])

        # load path map
        self.path_map_array = PathMap(self.static_heatmap).get_as_array()

        # load weighted average
        self.weighted_average = WeightedAverage(static_heatmap)
        self.average_point = self.weighted_average.get_weighted_average_point()

        self.wall_map = WallMap()
        self.wall_map_array = self.wall_map.getMap()

        self.cramped_map = CrampedMap(reachability_map.map)
        self.cramped_map_array = self.cramped_map.map

        self._build_map()

    def _build_map(self):

        # find the value of each point
        placement_map = np.zeros_like(self.static_heatmap.get_heatmap_array())

        for i in range(placement_map.shape[0]):
            for j in range(placement_map.shape[1]):
                # find the value of a given location
                placement_map[i, j] = self.find_value(i, j)

        # rescale to between zero and 1
        # note that all values in the map are less than zero
        # amin = np.amin(placement_map)
        # placement_map = np.subtract(placement_map, amin)
        # amax = np.amax(placement_map)
        # placement_map = np.true_divide(placement_map, amax)

        # apply reachability mask
        placement_map = self.apply_reachability_mask(placement_map)

        self.map = placement_map

    def apply_reachability_mask(self, placement_map):
        reachability_map = self.reachability_map.map

        masked_map = np.zeros_like(reachability_map, dtype="float")

        for i in range(reachability_map.shape[0]):
            for j in range(reachability_map.shape[1]):
                # skip values outside the placement map
                if i >= placement_map.shape[0] or j >= placement_map.shape[1]:
                    continue

                if reachability_map[i, j] > 0:
                    masked_map[i, j] = placement_map[i, j]
        
        return masked_map

    def find_value(self, i, j):
        """
        
        find the placement value at the given array location

        """

        # get weighted average value
        p1 = (i, j)
        p2 = self.average_point
        wa_value = -(get_point_distance(p1, p2)**2) * self.wa_weight

        # get path map value
        path_value = -self.path_map_array[i, j] * self.path_weight

        # get wall map value
        wall_value = self.wall_map_array[i, j] * self.wall_weight

        # get cramped map value
        cramped_value = -self.cramped_map_array[i, j] * self.cramped_weight

        return wa_value + path_value + wall_value + cramped_value

    def get_best_point(self):
        """ get the location of the best point

        returns:
            tuple (x, y) - indecies of the best scoring map location
        """
        max_value = 0.0 # minimum value of the map is 0.0
        best_point = (-1, -1)

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i, j] > max_value:
                    max_value = self.map[i, j]
                    best_point = (i, j)

        return best_point

    def display_as_heatmap(self):
        best_point = self.get_best_point()

        # display
        plt.imshow(np.transpose(self.map), cmap='hot', interpolation='nearest')
        plt.plot(best_point[0], best_point[1], 'go')

        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()

    def display_top(self, top_percent=0.1):
        top_map = np.copy(self.map)

        minimum = np.amin(top_map)
        maximum = np.amax(top_map)

        value_range = maximum - minimum
        cutoff = maximum - value_range * top_percent

        # clip any values below the desired percentage
        full = np.full_like(top_map, cutoff)
        top_map = np.maximum(top_map, full)
        top_map = np.subtract(top_map, full)

        # scale to between 0 and 1
        amax = np.amax(top_map)
        top_map = np.true_divide(top_map, amax)

        plt.imshow(np.transpose(top_map), cmap='hot', interpolation='nearest')
        plt.imshow(np.transpose(self.slam_map.map), cmap=get_custom_colormap_green(), interpolation='nearest')

        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()


def main():
    slam_data_filepath = sys.argv[1]
    sensor_list_filepath = sys.argv[2]
    smarthome_data_filepath = sys.argv[3]
    config_filepath = sys.argv[4]

    config = Config(config_filepath)

    base_placer = BasePlacer(slam_data_filepath, 
                             sensor_list_filepath, 
                             smarthome_data_filepath, 
                             config)
    #base_placer.display_top(0.1)
    base_placer.display_as_heatmap()

if __name__ == "__main__":
    main()

