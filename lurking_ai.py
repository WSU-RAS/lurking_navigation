#!/usr/bin/env python
"""
    LurkingAI listens to sensor data and uses it to create a DynamicHeatmap 
    object, acquire a weighted average, and create a path map. 
    This class also imports the reachability map. 
    Those four pieces of information are used to suggest an "optimal" location
    for Ras to be at every TIME_TICK. 

    Requirements:
    - roscore must be running
    - The publisher must be running in another terminal
"""
import sys
import rospy

from dynamic_heatmap import DynamicHeatmap
from reachability_map import ReachabilityMap
from path_map import PathMap
from weighted_average import WeightedAverage
from ras_msgs.msg import SensorPub
from config import Config

TIME_TICK = 1           # in seconds
HEATMAP_DECAY_STRENGTH = 0.8   # how quickly the heatmap decays

class LurkingAI():
    """
        Subscribes to sensor data and publishes a landing location for Ras. 
    """
    def __init__(self, dynamic_heatmap, reachability_map):
        self.listener()

    def callback(self, data):
        # Tell the heatmap handler that a sensor was triggered
        dynamic_heatmap.update(data.name) 
        dynamic_heatmap.display_heatmap() 


        # create new pathmap based on heatmap 

        self.get_landing_zone() 
        rospy.loginfo("%s was tripped" % (data.name))

    """
        Creates a listener node that acquires sensor data continuously. 
    """
    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True) 
        rospy.Subscriber("sensor_tripped", SensorPub, self.callback)
        rospy.spin() 

    """
        Using a heatmap and weighted average, returns an "optimal"
        landing zone for Ras at this current TIME_TICK. 
    """
    def get_landing_zone(self):
        print (42,42)

if __name__ == "__main__":
    # Acquire filepaths 
    sensor_list_filepath = sys.argv[1]
    config_filepath = sys.argv[2]
    config = Config(config_filepath)

    # Create a dynamic heatmap object 
    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath, config) 

    # Get the reachability map 
    reachability_map = {} # empty

    # Create a LurkingAI object that sends information to the heatmap 
    lurking_ai = LurkingAI(dynamic_heatmap, reachability_map)

