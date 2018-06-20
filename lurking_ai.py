#!/usr/bin/env python
import rospy

from dynamic_heatmap import DynamicHeatmap
from ras_msgs.msg import SensorPub

class ListenerNode():
    """
        Subscribes to sensor data and publishes a location for Ras to go to. 
    """
    def __init__(self):
        print "nothing"

        self.listener()

    def callback(self, data):
        DynamicHeatmap.update(data.name)
        rospy.loginfo("%s was tripped" % (data.name))

    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True) 
        rospy.Subscriber("sensor_tripped", SensorPub, self.callback)
        rospy.spin()

if __name__ == "__main__":
    
    listener = ListenerNode()
