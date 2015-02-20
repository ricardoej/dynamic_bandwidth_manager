import rospy
import pickle
from std_msgs.msg import String

class DBMSubscriber(rospy.Subscriber):
	def __init__(self, name, data_class, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
		self.primary_data_class = data_class
		rospy.Subscriber.__init__(self, name, String, callback, callback_args, queue_size, buff_size, tcp_nodelay)

		def decorate_method(method):
			def wrapper(msg, cb, cb_args):
				if not hasattr(self, 'last_message_data'):
					self.last_message_data = None

				if msg.data == '':
					msg = self.primary_data_class()
					msg.data = self.last_message_data
				else:
					msg.data = pickle.loads(msg.data)
					self.last_message_data = msg.data

				return method(msg, cb, cb_args)

			return wrapper

		new_function = decorate_method(self.impl._invoke_callback)
		setattr(self.impl, '_invoke_callback', new_function)