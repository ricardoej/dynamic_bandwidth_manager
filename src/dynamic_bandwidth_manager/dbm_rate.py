#!/usr/bin/env
import rospy

class DBMRate(rospy.Rate):
	def __init__(self, hz):
		rospy.Rate.__init__(self, hz)
