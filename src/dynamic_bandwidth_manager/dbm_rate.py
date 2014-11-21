#!/usr/bin/env
import rospy
from .dbm_param import DBMParam

class DBMRate(rospy.Rate):
	def __init__(self, topic_name, min_frequency, max_frequency, default_frequency):
		self.topic_name = topic_name
		self.default_frequency = default_frequency
		self.dbm_param = DBMParam(topic_name)
		self.dbm_param.create_params(min_frequency, max_frequency, default_frequency)
		rospy.Rate.__init__(self, default_frequency)

	def sleep(self):
		current_frequency = self.dbm_param.get_current_frequency()
		if current_frequency == 0:
			current_frequency = self.default_frequency
		self.sleep_dur = rospy.rostime.Duration(0, int(1e9/current_frequency))
		super(DBMRate, self).sleep()