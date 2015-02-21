import rospy
import sys
from .dbm_param import DBMParam

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
		rospy.Publisher.__init__(self, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)

		def decorate_method(method):
			def wrapper(message, connection_override=None):
				if not hasattr(self, 'last_message_size'):
					self.last_message_size = 0

				current_message_size = sys.getsizeof(message)

				if current_message_size != self.last_message_size:
					DBMParam.set_message_size_in_bytes(name, current_message_size)

				self.last_message_size = current_message_size

				return method(message, connection_override)

			return wrapper

		new_function = decorate_method(self.impl.publish)
		setattr(self.impl, 'publish', new_function)