import rospy

class DBMParam:
	def __init__(self, topic_name):
		self.topic_name = topic_name

	def create_params(self, min_frequency, max_frequency, default_frequency):
		self.__addManagedTopic();
		self.__addFrequency(default_frequency);
		self.__addMinFrequency(min_frequency);
		self.__addMaxFrequency(max_frequency);
		self.__addPriority();
		self.__addMessageSize();
		self.__addMaxBandwidth();
		self.__addMaxBandwidthUtilization();

	def __addManagedTopic(self):
		paramName = "/dbm/topics";
		managedTopics = rospy.get_param(paramName, {});
		if self.topic_name not in managedTopics:
			managedTopics[self.topic_name] = 0;
			rospy.set_param(paramName, managedTopics);

	def __addFrequency(self, default_frequency):
		paramName = self.topic_name + "/dbm/frequency/current_value";
		rospy.set_param(paramName, default_frequency);

	def __addMinFrequency(self, min_frequency):
		paramName = self.topic_name + "/dbm/frequency/min";
		rospy.set_param(paramName, min_frequency);

	def __addMaxFrequency(self, max_frequency):
		paramName = self.topic_name + "/dbm/frequency/max";
		rospy.set_param(paramName, max_frequency);

	def __addPriority(self):
		paramName = self.topic_name + "/dbm/priority";
		rospy.set_param(paramName, 1.0);

	def __addMessageSize(self):
		paramName = self.topic_name + "/dbm/message_size_in_bytes";
		rospy.set_param(paramName, 0.0);

	def __addMaxBandwidth(self):
		paramName = "/dbm/max_bandwidth_in_mbit";
		rospy.set_param(paramName, 10.0);

	def __addMaxBandwidthUtilization(self):
		paramName = "/dbm/max_bandwidth_utilization";
		rospy.set_param(paramName, 1);