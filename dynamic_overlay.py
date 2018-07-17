"""
    Displays all the maps at once. To be called from within LurkingAI. 
"""
import sys 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from config import Config
from slam_map import SlamMap
from dynamic_heatmap import DynamicHeatmap
from path_map import PathMap
from weighted_average import WeightedAverage
from reachability_map import ReachabilityMap
from wall_map import WallMap

class DynamicOverlay():
    def __init__(self, dynamic_heatmap, slam_data_filepath, weighted_average, config):
        #smarthome_data_filepath = sys.argv[2]
        #sensor_map_filepath = sys.argv[3]
        #config_filepath = sys.argv[4]

        # Clears the screen from the last time we called this 
        plt.gcf().clear()

        # get slam map
        slam_map = SlamMap(slam_data_filepath, config)

        heatmap = dynamic_heatmap.get_heatmap_array()

        # get heatmap
        #heatmap = StaticHeatmap(sensor_map_filepath, smarthome_data_filepath, config)
        #heatmap.set_offset(slam_map.origin[0], slam_map.origin[1])

        # get path map
        path_map = PathMap(dynamic_heatmap)

        # get the reachability map
        reachability_map = ReachabilityMap(slam_map)

        # get the wall map
        wall_map = WallMap()
 
        ### plot all the things to plot
        # plot the path map
        path_array = path_map.get_as_array()
        #HIDDEN plt.imshow(np.transpose(path_array), cmap='hot', interpolation='nearest')

        # plot the heatmap
        plt.imshow(np.transpose(dynamic_heatmap.get_heatmap()), cmap=get_custom_colormap_blue(), interpolation='nearest')
        # np_heatmap = np.array(dynamic_heatmap.get_heatmap())
        # axis = plt.gca()

        # plt.imshow(np.transpose(np_heatmap),
        #           cmap='hot', interpolation='nearest')

        # plot the slam map
        #HIDDEN plt.imshow(np.transpose(slam_map.map), cmap=get_custom_colormap_green(), interpolation='nearest')

        # plot the weighted average
        # average_point = weighted_average.get_weighted_average_point()
        plt.plot(weighted_average[0], weighted_average[1], 'go')

        # plot the wall map
        #HIDDEN plt.imshow(np.transpose(wall_map.getMap()), cmap=get_custom_colormap_green())

        # plot the reachability map
        #HIDDEN plt.imshow(np.transpose(reachability_map.map), cmap=get_custom_colormap_blue(), interpolation='nearest')

        # flip y axis
        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])

        plt.show()

    def display_all(self):
        print "hello it's a b u g"

def get_custom_colormap_green():
    cdict = \
            {
                'red': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'green': ((0.0, 0.0, 0.0),
                        (1.0, 1.0, 1.0)),
                'blue': ((0.0, 0.0, 0.0),
                        (1.0, 0.5, 0.5)),
                'alpha': ((0.0, 0.0, 0.0),
                        (0.1, 1.0, 1.0),
                        (1.0, 1.0, 1.0))
            }
    return LinearSegmentedColormap('alpha_red', cdict)

def get_custom_colormap_blue():
    cdict = \
            {
                'red': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'green': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'blue': ((0.0, 0.0, 0.0),
                        (1.0, 1.0, 1.0)),
                'alpha': ((0.0, 0.0, 0.0),
                        (0.1, 1.0, 1.0),
                        (1.0, 1.0, 1.0))
            }
    return LinearSegmentedColormap('alpha_red', cdict)

