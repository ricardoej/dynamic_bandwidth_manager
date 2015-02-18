import rospy

class DBMParam:
	@staticmethod
	def create_params(topic_name, min_frequency, max_frequency, default_frequency):
		DBMParam.__add_managed_topic(topic_name)
		DBMParam.__add_frequency(topic_name, default_frequency)
		DBMParam.__add_min_frequency(topic_name, min_frequency)
		DBMParam.__add_max_frequency(topic_name, max_frequency)
		DBMParam.__add_priority(topic_name)
		DBMParam.__add_message_size(topic_name)
		DBMParam.__add_max_bandwidth()
		DBMParam.__add_max_bandwidth_utilization()
		DBMParam.__add_optimization_rate()
		DBMParam.__add_manage_local_subscribers()

	@staticmethod
	def __add_managed_topic(topic_name):
		param_name = "/dbm/topics"
		managedTopics = rospy.get_param(param_name, {})
		if topic_name not in managedTopics:
			managedTopics[topic_name] = 0
			rospy.set_param(param_name, managedTopics)

	@staticmethod
	def __add_frequency(topic_name, default_frequency):
		param_name = topic_name + "/dbm/frequency/current_value"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, default_frequency)

	@staticmethod
	def __add_min_frequency(topic_name, min_frequency):
		param_name = topic_name + "/dbm/frequency/min"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, min_frequency)

	@staticmethod
	def __add_max_frequency(topic_name, max_frequency):
		param_name = topic_name + "/dbm/frequency/max"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, max_frequency)

	@staticmethod
	def __add_priority(topic_name):
		param_name = topic_name + "/dbm/priority"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 1.0)

	@staticmethod
	def __add_message_size(topic_name):
		param_name = topic_name + "/dbm/message_size_in_bytes"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 0.0)

	@staticmethod
	def __add_max_bandwidth():
		param_name = "/dbm/max_bandwidth_in_mbit"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 10.0)

	@staticmethod
	def __add_max_bandwidth_utilization():
		param_name = "/dbm/max_bandwidth_utilization"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 1)

	@staticmethod
	def __add_optimization_rate():
		param_name = "/dbm/optimization_rate"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 1)

	@staticmethod
	def __add_manage_local_subscribers():
		param_name = "/dbm/manage_local_subscribers"
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, False)

	@staticmethod
	def get_current_frequency(topic_name):
		param_name = topic_name + "/dbm/frequency/current_value"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def set_current_frequency(topic_name, frequency):
		param_name = topic_name + "/dbm/frequency/current_value"
		rospy.set_param(param_name, frequency)

	@staticmethod
	def get_min_frequency(topic_name):
		param_name = topic_name + "/dbm/frequency/min"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def get_max_frequency(topic_name):
		param_name = topic_name + "/dbm/frequency/max"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def get_priority(topic_name):
		param_name = topic_name + "/dbm/priority"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def get_message_size_in_bytes(topic_name):
		param_name = topic_name + "/dbm/message_size_in_bytes"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def set_message_size_in_bytes(topic_name, size):
		param_name = topic_name + "/dbm/message_size_in_bytes"
		rospy.set_param(param_name, size)

	@staticmethod
	def get_max_bandwidth_in_mbits():
		param_name = "/dbm/max_bandwidth_in_mbit"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def get_max_bandwidth_utilization():
		param_name = "dbm/max_bandwidth_utilization"
		if rospy.has_param(param_name):
			return rospy.get_param(param_name)
		else:
			return 0

	@staticmethod
	def get_managed_topics():
		param_name = "dbm/topics"
		if rospy.has_param(param_name):
			return [topic for topic, isManaged in rospy.get_param(param_name).iteritems() if isManaged == 1]
		else:
			return []