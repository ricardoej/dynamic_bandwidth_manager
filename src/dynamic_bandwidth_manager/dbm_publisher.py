import rospy
from .dbm_param import DBMParam

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
		rospy.Publisher.__init__(self, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)
		rospy.Subscriber(name, rospy.AnyMsg, self.callback)

	def callback(self, data):
		if not hasattr(self, 'last_message_size'):
			self.last_message_size = 0

		current_message_size = len(data._buff)

		if current_message_size != self.last_message_size:
			DBMParam.set_message_size_in_bytes(self.name, current_message_size)

		self.last_message_size = current_message_size