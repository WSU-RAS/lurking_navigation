from ras_msgs.msg import SensorPub
import rospy
#Will need to have roscore or roslaunch running first
def process_data(sen_name, sen_msg, sen_type):

    if sen_msg == "ON" and sen_type == "Control4-Motion":
        #print("Found {} and it was {} and is a {} at time {}".format(sen_name, sen_msg, sen_type, sen_time))
        pub_it(sen_name)



#http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
def pub_it(sen_name):
    pub = rospy.Publisher('sensor_tripped', SensorPub, queue_size=50)
    rospy.init_node("sensor_watcher", anonymous=True) #Maybe true since this is going to be publishing so many things
    #r = rospy.Rate(10) -- 10hz, not sure what this is for, involved in sleep
    msg = SensorPub()
    msg.name = sen_name

    if not rospy.is_shutdown():
        rospy.loginfo(sen_name + " was found.")
        pub.publish(msg)
        #r.sleep() #not sure if needed

