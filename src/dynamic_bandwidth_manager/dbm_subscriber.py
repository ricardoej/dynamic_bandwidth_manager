import rospy
import sys

class DBMSubscriber(rospy.Subscriber):
	def __init__(self, name, data_class, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
		self.managed_topic_name = name + "/optimized"
		rospy.Subscriber.__init__(self, self.managed_topic_name, data_class, callback, callback_args, queue_size, buff_size, tcp_nodelay)