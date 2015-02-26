import rospy
from .dbm_param import DBMParam

class DBMRate(rospy.Rate):
	def __init__(self, publisher, min_frequency, max_frequency, default_frequency):
		self.topic_name = publisher.name
		self.default_frequency = default_frequency
		DBMParam.create_params(self.topic_name, min_frequency, max_frequency, default_frequency)
		rospy.Rate.__init__(self, default_frequency)

	def sleep(self):
		current_frequency = DBMParam.get_current_frequency(self.topic_name)
		if current_frequency == 0:
			current_frequency = self.default_frequency
		self.sleep_dur = rospy.rostime.Duration(0, int(1e9/current_frequency))
		super(DBMRate, self).sleep()