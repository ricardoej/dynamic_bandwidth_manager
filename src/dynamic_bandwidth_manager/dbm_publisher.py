import rospy
import pickle
import sys
from std_msgs.msg import String
from .dbm_param import DBMParam

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
		class mySubscribeListener(rospy.SubscribeListener):
			def __init__(self, primary_subscriber_listener):
				self.primary_subscriber_listener = primary_subscriber_listener
				self.has_new_subscriber = False

			def peer_subscribe(self, topic_name, topic_publish, peer_publish):
				if self.primary_subscriber_listener is not None:
					self.primary_subscriber_listener.peer_subscribe(topic_name, topic_publish, peer_publish)
				self.has_new_subscriber = True

		    	def peer_unsubscribe(self, topic_name, num_peers):
		        	if self.primary_subscriber_listener is not None:
					self.primary_subscriber_listener.peer_unsubscribe(topic_name, num_peers)

		self.equality_message_optimization_is_enabled = DBMParam.get_equality_message_optimization_is_enabled(name)
		self.primary_data_class = data_class
		self.primary_subscriber_listener = subscriber_listener
		self.new_subscriber_listener = mySubscribeListener(self.primary_subscriber_listener)

		rospy.Publisher.__init__(self, name, String, self.new_subscriber_listener, tcp_nodelay, latch, headers, queue_size)

		def decorate_method(method):
			def wrapper(message, connection_override=None):
				if not hasattr(self, 'last_message'):
					self.last_message = None

				if not hasattr(self, 'last_message_size'):
					self.last_message_size = 0

				if message.data != self.last_message or self.new_subscriber_listener.has_new_subscriber or not self.equality_message_optimization_is_enabled:
					current_message = String(pickle.dumps(message.data))
					self.new_subscriber_listener.has_new_subscriber = False
				else:
					current_message = String()

				current_message_size = sys.getsizeof(current_message)

				if current_message_size != self.last_message_size:
					DBMParam.set_message_size_in_bytes(name, current_message_size)

				self.last_message = message.data
				self.last_message_size = current_message_size

				return method(current_message, connection_override)

			return wrapper

		new_function = decorate_method(self.impl.publish)
		setattr(self.impl, 'publish', new_function)