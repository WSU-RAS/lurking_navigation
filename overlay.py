"""

Quick program for double checking the slam map and heatmap overlay properly

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from slam_map import SlamMap
from heatmap import Heatmap

def main():
    slam_data_filepath = "data/kyoto_slam_map.txt"
    heatmap_data_filepath = "data/sensor_data.txt"

    slam_map = SlamMap(slam_data_filepath)
    heatmap = Heatmap()
    heatmap.load_from_file(heatmap_data_filepath)
    heatmap.set_offset(slam_map.origin[0], slam_map.origin[1])

    # plot the slam map
    heatmap = heatmap.get_heatmap_array()
    plt.imshow(np.transpose(heatmap), cmap='hot', interpolation='nearest')
    plt.imshow(slam_map.map, cmap=get_custom_colormap(), interpolation='nearest')

    # flip y axis
    axis = plt.gca()
    axis.set_ylim(axis.get_ylim()[::-1])

    plt.show()


def get_custom_colormap():
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

if __name__ == "__main__":
    main()
