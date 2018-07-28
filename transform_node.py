

import matplotlib.pyplot as plt

import rospy
import numpy as np

from slam_to_sensors import SlamConverter
from slam_map import SlamMap
from config import Config

class TransformNode():
    def __init__(self, config_filepath, slam_map_filepath):

        self.config = Config(config_filepath)
        self.slam_map = SlamMap(slam_map_filepath, self.config)
        self.converter = SlamConverter(slam_map_filepath, config_filepath)

        self.listener()

    def listener(self):
        rospy.init_node('slam_converter', anonymous=True)

        rospy.Subscriber("...???", SOMETHING_PUB, self.transform_and_display)


    def transform_and_display(self, data):
        point = data.whatever

        transformed_point = self.converter.convert_point(point)

        # display everything
        plt.imshow(np.transpose(self.slam_map.map), cmap='hot', interpolation='nearest')
        plt.plot(transformed_point[0], transformed_point[1], 'go')

        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()


