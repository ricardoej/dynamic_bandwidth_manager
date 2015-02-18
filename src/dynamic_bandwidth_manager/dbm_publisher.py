import rospy
import sys
from .dbm_param import DBMParam

class DBMPublisher(rospy.Publisher):
	def publish(self, *args, **kwds):
		if not hasattr(self, 'last_message_size'):
			self.last_message_size = 0
		current_message_size = sys.getsizeof(args[0])
		if current_message_size != self.last_message_size:
			DBMParam.set_message_size_in_bytes(self.name, current_message_size)
		super(DBMPublisher, self).publish(*args, **kwds)
		self.last_message_size = current_message_size