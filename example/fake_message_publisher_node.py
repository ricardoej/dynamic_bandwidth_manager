#!/usr/bin/env python

import rospy
from sys import getsizeof
from dynamic_bandwidth_manager.msg import FakeMessage
from dynamic_bandwidth_manager.srv import SubscribeTopicDBMTest

def start():
    pub = rospy.Publisher('topic_name', FakeMessage, queue_size=10)
    rospy.init_node('fake_message_publisher_node', anonymous=True)
    rate = rospy.Rate(rospy.get_param('~max_rate', 60))
    while not rospy.is_shutdown():
        message = FakeMessage()
        message_size = rospy.get_param('~message_size_in_kb', 1)
        message.data = (10,) * message_size * 1024
        rospy.loginfo("Publishing message with approximate size of %d Kb on %s" % (message_size, rospy.get_time()))
        pub.publish(message)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass