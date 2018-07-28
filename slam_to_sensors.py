

import sys
import numpy as np

from util.transform import transform_point_from_slam

from config import Config
from slam_map import SlamMap

class SlamConverter():
    def __init__(self, slam_map_filepath, config_filepath):
        self.config = Config(config_filepath)
        self.slam_map = SlamMap(slam_map_filepath, self.config)

    def convert_point(self, point):
        return transform_point_from_slam(point, self.slam_map)


if __name__ == "__main__":
    # do imports
    import matplotlib.pyplot as plt

    # set up data
    slam_data_filepath = sys.argv[1]
    config_filepath = sys.argv[2]

    config = Config(config_filepath)
    slam_map = SlamMap(slam_data_filepath, config)
    fake_point = (-2.0, -1.0)
    fake_point_map_scale = tuple(i / config.map_resolution for i in fake_point)

    # transform
    transformed_point = transform_point_from_slam(fake_point, slam_map)

    # display old 
    plt.imshow(np.transpose(slam_map.original_slam_map), cmap='hot', interpolation='nearest')

    plt.figure()
