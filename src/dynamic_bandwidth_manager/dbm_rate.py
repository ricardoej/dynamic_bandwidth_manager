#!/usr/bin/env
import rospy
from .dbm_param import DBMParam

class DBMRate(rospy.Rate):
	def __init__(self, topic_name, min_frequency, max_frequency, default_frequency):
		dbmParam = DBMParam(topic_name)
		dbmParam.create_params(min_frequency, max_frequency, default_frequency)
		rospy.Rate.__init__(self, default_frequency)
