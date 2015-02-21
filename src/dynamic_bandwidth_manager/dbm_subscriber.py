import rospy

class DBMSubscriber(rospy.Subscriber):
	def __init__(self, name, data_class, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
		rospy.Subscriber.__init__(self, name, data_class, callback, callback_args, queue_size, buff_size, tcp_nodelay)