"""
    Double checks that the slam map and heatmap are overlaid properly.
"""
import sys 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from config import Config
from slam_map import SlamMap
from static_heatmap import StaticHeatmap
from path_map import PathMap
from weighted_average import WeightedAverage
from reachability_map import ReachabilityMap

def main():
    # get command line arguments
    slam_data_filepath = sys.argv[1]
    smarthome_data_filepath = sys.argv[2]
    sensor_map_filepath = sys.argv[3]
    config_filepath = sys.argv[4]

    # load the config
    config = Config(config_filepath)

    ### get all the things to plot
    # get slam map
    slam_map = SlamMap(slam_data_filepath, config)

    # get heatmap
    heatmap = StaticHeatmap(sensor_map_filepath, smarthome_data_filepath, config)
    heatmap.set_offset(slam_map.origin[0], slam_map.origin[1])

    # get path map
    path_map = PathMap(heatmap)

    # get the weighted average
    weighted_average = WeightedAverage(heatmap)

    # get the reachability map
    reachability_map = ReachabilityMap(slam_map)

    ### plot all the things to plot
    # plot the path map
    path_array = path_map.get_as_array()
    #plt.imshow(np.transpose(path_array), cmap='hot', interpolation='nearest')

    # plot the heatmap
    heatmap = heatmap.get_heatmap_array()
    plt.imshow(np.transpose(heatmap), cmap=get_custom_colormap_blue(), interpolation='nearest')

    # plot the slam map
    plt.imshow(np.transpose(slam_map.map), cmap=get_custom_colormap_green(), interpolation='nearest')

    # plot the weighted average
    average_point = weighted_average.get_weighted_average_point()
    #plt.plot(average_point[0], average_point[1], 'go')

    # plot the reachability map
    #plt.imshow(np.transpose(reachability_map.map), cmap=get_custom_colormap_blue(), interpolation='nearest')

    # flip y axis
    axis = plt.gca()
    axis.set_ylim(axis.get_ylim()[::-1])

    plt.show()


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

if __name__ == "__main__":
    main()
