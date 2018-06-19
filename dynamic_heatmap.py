#!/usr/bin/env python
import rospy
from ras_msgs.msg import SensorPub
from config import Config
from heatmap import Heatmap 

import sys
import math
import time 
import random 

import numpy as np 
import matplotlib.pyplot as plt

TIME_TICK_LENGTH = 1           # in seconds
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
            for col_index, point in enumerate(row): 
                heatmap[row_index][col_index] *= HEATMAP_DECAY_STRENGTH # currently 0.8
        
        return heatmap

    def increment_heatmap_array(self, heatmap):
        rand = random.randint(1,50)
        heatmap[rand][rand] += 10
        return heatmap

    def display_heatmap(self):
        program_status = True 

        # Initialize the heatmap plot 
        # heatmap_array = self.get_heatmap_array()
        np_heatmap = np.array(self.heatmap)
        axis = plt.gca()
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')
        axis.set_ylim(axis.get_ylim()[::-1])
        #plt.show()

        fig = plt.gcf()
        fig.show() 
        fig.canvas.draw() 

        while True:
            # self._decay_heatmap(np_heatmap)
            
            np_heatmap = np.array(self.heatmap)
            axis = plt.gca()
            plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')

            fig = plt.gcf()
            fig.show() 
            fig.canvas.draw() 

            # get input using method that calls listener node object 
            # decay heatmap 
            # update existing plot
            # display plot 

            time.sleep(TIME_TICK_LENGTH)

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    sensor_list_filepath = sys.argv[1]
    config_filepath = sys.argv[2]
    config = Config(config_filepath)

    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath, config) 
    dynamic_heatmap.display_heatmap()

