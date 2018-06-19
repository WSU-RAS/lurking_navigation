#!/usr/bin/env python
import rospy
from ras_msgs.msg import SensorPub
 
def callback(data):
    rospy.loginfo("%s was tripped" % (data.name))

def listener():
    rospy.init_node('sensor_listener', anonymous=True) #Might need to be true depending on how many are published
    rospy.Subscriber("sensor_tripped", SensorPub, callback)
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()