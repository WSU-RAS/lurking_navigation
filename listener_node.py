#!/usr/bin/env python
import rospy
from ras_msgs.msg import SensorPub
from dynamic_heatmap import DynamicHeatmap
 
class ListenerNode():
    """
        Subscribes to sensor data and publishes a location for Ras to go to. 
    """
    def __init__(self):
        print "nothing"

        self.dynamic_heatmap = DynamicHeatmap()
        self.listener()

    def callback(self, data):
        DynamicHeatmap.update(data.name)
        rospy.loginfo("%s was tripped" % (data.name))

    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True) #Might need to be true depending on how many are published
        rospy.Subscriber("sensor_tripped", SensorPub, self.callback)
        #spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == "__main__":

    listener = ListenerNode()
