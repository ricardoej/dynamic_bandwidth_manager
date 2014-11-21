#!/usr/bin/env
import rospy
import sys
from .dbm_param import DBMParam

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
		rospy.Publisher.__init__(self, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)
		self.dbm_param = DBMParam(self.name)

	def publish(self, *args, **kwds):
		if not hasattr(self, 'last_message_size'):
			self.last_message_size = 0
		current_message_size = sys.getsizeof(args)
		if current_message_size != self.last_message_size:
			self.dbm_param.set_message_size_in_bytes(current_message_size)
		super(DBMPublisher, self).publish(*args, **kwds)
		self.last_message_size = sys.getsizeof(args)