#!/usr/bin/env python
import math
import random
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy

from config import Config
from heatmap import Heatmap
from ras_msgs.msg import SensorPub

TIME_TICK = 1                  # in seconds
HEATMAP_DECAY_STRENGTH = 0.8   # how quickly the heatmap decays

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)
        self.heatmap = self.get_heatmap_array()

    @staticmethod
    def update(triggered_sensor):
        # get sensor coordinates of sensor that was triggered


        print "not much, wbu"

    def _get_sensor_location(self, triggered_sensor):
        print "wowza mickey"

    def _decay_heatmap(self, heatmap):
        """
            Decay every point in the heatmap by a certain preset percentage.
            This method is called every TIME_TICK. 
        """
        for row_index, row in enumerate(heatmap):
            for col_index in enumerate(row): 
                heatmap[row_index][col_index] *= HEATMAP_DECAY_STRENGTH # currently 0.8
        
        return heatmap

    def increment_heatmap_array(self, heatmap):
        rand = random.randint(1,50)
        heatmap[rand][rand] += 10
        return heatmap

    """
        Displays heatmap once - called every time tick from lurking_ai 
    """
    def display_heatmap(self):

        # heatmap_array = self.get_heatmap_array()
        self.heatmap = self.increment_heatmap_array(self.heatmap)

        np_heatmap = np.array(self.heatmap)
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')

        axis = plt.gca()
        
        if (axis.get_ylim()[0] > axis.get_ylim()[1]):
            axis.set_ylim(axis.get_ylim()[::-1])

        # print axis.get_ylim()[0] 

        # makes sure our origin is correctly set without flipping the y-axis
        # axis.set_ylim(axis.get_ylim()[0])

        # plt.show
        fig = plt.gcf()
        fig.show() 
        fig.canvas.draw() 