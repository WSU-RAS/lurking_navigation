#!/usr/bin/env python
import rospy
from ras_msgs.msg import SensorPub
from config import Config
from heatmap import Heatmap 

import sys
import math
import time 

import numpy as np 
import matplotlib.pyplot as plt

TIME_TICK_LENGTH = 8           # in seconds
HEATMAP_DECAY_STRENGTH = 0.8   # how quickly the heatmap decays

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)

        """
            dictionary of sensors 
            update heatmap every time tick 
            display every time tick 
        """
        self.heatmap = [
                    [0.0 for j in range(int(math.ceil(self.map_height / self.map_resolution)))] 
                    for i in range(int(math.ceil(self.map_width / self.map_resolution)))
                  ]

    def _init_ras_location(self):
        print "nothing yet"
        return (42, 42)

    def get_heatmap_array(self):
        """ get heatmap array

        get the heatmap as a 2d array indicating the hotness at each area.
        Note that the heatmap will be either normalized or not based on whether _normalize_heatmap has been run.

        """
        self.heatmap = self._decay_heatmap(self.heatmap)

        return self.heatmap

    def _update_heatmap_array(self, triggered_sensor):
        # get sensor coordinates of sensor that was triggered
        # 

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

    def display_heatmap(self):
        program_status = True 

        # Initialize the heatmap plot 
        heatmap_array = self.get_heatmap_array()
        np_heatmap = np.array(heatmap_array)
        axis = plt.gca()
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')
        axis.set_ylim(axis.get_ylim()[::-1])

        while True:
            print "hmm"
            plt.show()

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

    #dynamic_heatmap.listen_to_sensors() # listens for sensors to be triggered
    #dynamic_heatmap.get_heatmap_array()
