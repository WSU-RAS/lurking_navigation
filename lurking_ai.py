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
from base_placement_algorithm import BasePlacer
from path_map import PathMap
from weighted_average import WeightedAverage
from ras_msgs.msg import SensorPub
from config import Config

TIME_TICK  = 60  # in seconds
UPDATE_RAS = 5   # in seconds

class LurkingAI():
    """
        Subscribes to sensor data and publishes a landing location for Ras. 
    """
    def __init__(self, dynamic_heatmap, reachability_map):
        self.listener()

    """
        Creates a listener node that acquires sensor data continuously. 
    """
    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True) 
        rospy.Timer(rospy.Duration(TIME_TICK), self.timer_callback)
        rospy.Timer(rospy.Duration(1), self.get_landing_zone)
        rospy.Subscriber("sensor_tripped", SensorPub, self.update_heatmap)
        rospy.spin() 

    """
        Inform the heatmap handler that sensor was triggered
    """
    def update_heatmap(self, data):
        dynamic_heatmap.update_heatmap(data.name) 
        dynamic_heatmap.display_heatmap() 

    """
        Every TIME_TICK, decay the heatmap by the HEATMAP_DECAY_STRENGTH. 
    """
    def timer_callback(self, data):
        dynamic_heatmap.decay_heatmap()

    """
        Using the heatmap, pathmap, and weighted average, returns an "optimal"
        landing zone for Ras at this current moment. 
    """
    def get_landing_zone(self, data):
        landing_zone = (10,15)

        # Use the dynamic_heatmap to grab the weighted average 
        weighted_average = WeightedAverage(dynamic_heatmap)
        average_point = weighted_average.get_weighted_average_point()
        print average_point

        landing_zone = average_point

        # Create a pathmap based on the heatmap 

        # Combine these somehow to determine landing zone 

        dynamic_heatmap.mark_spot_on_map(landing_zone)
        print landing_zone

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

