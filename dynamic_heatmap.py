#!/usr/bin/env python
import rospy
from ras_msgs.msg import SensorPub

import sys
import math

from config import Config
from heatmap import Heatmap 

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
    
    def _import_sensor_information(self):
        print "wow what a great method"

    def get_heatmap_array(self):
        """ get heatmap array

        get the heatmap as a 2d array indicating the hotness at each area.
        Note that the heatmap will be either normalized or not based on whether _normalize_heatmap has been run.

        """
        self.heatmap = self._decay_heatmap(self.heatmap)

        return self.heatmap

    def _decay_heatmap(self, heatmap):
        """
            Decay every point in the heatmap by a certain preset percentage. 
        """
        
        for row_index, row in enumerate(heatmap):
            for col_index, point in enumerate(row): 
                heatmap[row_index][col_index] += 3
        
        return heatmap

    def callback(self, data):
        rospy.loginfo("%s was tripped" % (data.name))

    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True) # Might need to be true depending on how many are published
        rospy.Subscriber("sensor_tripped", SensorPub, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == "__main__":
    # Acquire input source for realtime heatmap generation 
    sensor_list_filepath = sys.argv[1]
    config_filepath = sys.argv[2]
    config = Config(config_filepath)

    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath, config) 
    dynamic_heatmap.listener() # listens for sensors to be triggered
    dynamic_heatmap.get_heatmap_array()
