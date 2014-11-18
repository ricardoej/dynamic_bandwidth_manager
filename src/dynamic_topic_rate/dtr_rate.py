#!/usr/bin/env
import rospy

class DTRRate(rospy.Rate):
	def __init__(self, hz):
		rospy.Rate.__init__(self, hz)
