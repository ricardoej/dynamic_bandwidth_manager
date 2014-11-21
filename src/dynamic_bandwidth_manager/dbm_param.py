import rospy

class DBMParam:
	def __init__(self, topic_name):
		self.topic_name = topic_name

	def create_params(self, min_frequency, max_frequency, default_frequency):
		self.__add_managed_topic();
		self.__add_frequency(default_frequency);
		self.__add_min_frequency(min_frequency);
		self.__add_max_frequency(max_frequency);
		self.__add_priority();
		self.__add_message_size();
		self.__add_max_bandwidth();
		self.__add_max_bandwidth_utilization();

	def __add_managed_topic(self):
		param_name = "/dbm/topics";
		managedTopics = rospy.get_param(param_name, {});
		if self.topic_name not in managedTopics:
			managedTopics[self.topic_name] = 0;
			rospy.set_param(param_name, managedTopics);

	def __add_frequency(self, default_frequency):
		param_name = self.topic_name + "/dbm/frequency/current_value";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, default_frequency);

	def __add_min_frequency(self, min_frequency):
		param_name = self.topic_name + "/dbm/frequency/min";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, min_frequency);

	def __add_max_frequency(self, max_frequency):
		param_name = self.topic_name + "/dbm/frequency/max";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, max_frequency);

	def __add_priority(self):
		param_name = self.topic_name + "/dbm/priority";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 1.0);

	def __add_message_size(self):
		param_name = self.topic_name + "/dbm/message_size_in_bytes";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 0.0);

	def __add_max_bandwidth(self):
		param_name = "/dbm/max_bandwidth_in_mbit";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 10.0);

	def __add_max_bandwidth_utilization(self):
		param_name = "/dbm/max_bandwidth_utilization";
		if not rospy.has_param(param_name):
			rospy.set_param(param_name, 1);

	def get_current_frequency(self):
		param_name = self.topic_name + "/dbm/frequency/current_value";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def set_current_frequency(self, frequency):
		param_name = self.topic_name + "/dbm/frequency/current_value";
		rospy.set_param(param_name, frequency);

	def get_min_frequency(self):
		param_name = self.topic_name + "/dbm/frequency/min";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def get_max_frequency(self):
		param_name = self.topic_name + "/dbm/frequency/max";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def get_priority(self):
		param_name = self.topic_name + "/dbm/priority";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def get_message_size_in_bytes(self):
		param_name = self.topic_name + "/dbm/message_size_in_bytes";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def set_message_size_in_bytes(self, size):
		param_name = self.topic_name + "/dbm/message_size_in_bytes";
		rospy.set_param(param_name, size);

	def get_max_bandwidth_in_mbits(self):
		param_name = "/dbm/max_bandwidth_in_mbit";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def get_max_bandwidth_utilization(self):
		param_name = "/dbm/max_bandwidth_utilization";
		if rospy.has_param(param_name):
			return rospy.get_param(param_name);
		else:
			return 0

	def get_managed_topics(self):
		param_name = "/dbm/topics";
		if rospy.has_param(param_name):
			return [topic for topic, isManaged in rospy.get_param(param_name).iteritems() if isManaged == 1];
		else:
			return []