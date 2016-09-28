import rospy
import rosgraph
import dynamic_bandwidth_manager
from urlparse import urlparse
from .dbm_param import DBMParam

class DBMOptimizer:
	def __init__(self, get_new_frequencies_method):
		self.__get_new_frequencies_method = get_new_frequencies_method
		self.__optimization_rate = rospy.get_param('dbm/optimization_rate', 1)
		self.__loop_rate = rospy.Rate(self.__optimization_rate)

	def start(self):
		while not rospy.is_shutdown():
			managed_topics = rospy.get_param('dbm/topics', {})
			self.__verify_if_topics_should_be_managed(managed_topics)
			managed_topics = [topic for topic, isManaged in managed_topics.iteritems() if isManaged == 1]
			#priorities = self.__normalize_priorities(managed_topics)
			priorities = dict(zip(managed_topics, (dynamic_bandwidth_manager.DBMParam.get_priority(topic) for topic in managed_topics)))
			frequencies = self.__get_new_frequencies_method(managed_topics, priorities)
			for key in frequencies:
				DBMParam.set_current_frequency(key, frequencies[key])
			self.__loop_rate.sleep()

	def __verify_if_topics_should_be_managed(self, topics):
		for topic in topics.keys():
			if self.__there_are_external_subscribers(topic):
				topics[topic] = 1
			else:
				topics[topic] = 0
		rospy.set_param("/dbm/topics", topics)
		return topics

	def __there_are_external_subscribers(self, topic):
		pubs = self.__get_publishers(topic)
		subs = self.__get_subscribers(topic)
		if len(subs) == 0: # There are no subscribers
			return False
		elif rospy.get_param('dbm/manage_local_subscribers', False):
			return True
		elif self.__publishers_and_subscribers_are_in_the_same_host(pubs, subs): # There are no external node listening the topic
			return False
		else:
			return True

	def __get_subscribers(self, topic):
		topic = rosgraph.names.script_resolve_name('rostopic', topic)
		master = rosgraph.Master('/rostopic')
		state = master.getSystemState()
		subs = [x[1] for x in state[1] if x[0] == topic]
		if len(subs) > 0:
			return subs[0]
		else:
			return []

	def __get_publishers(self, topic):
		topic = rosgraph.names.script_resolve_name('rostopic', topic)
		master = rosgraph.Master('/rostopic')
		state = master.getSystemState()
		pubs = [x[1] for x in state[0] if x[0] == topic]
		if len(pubs) > 0:
			return pubs[0]
		else:
			return []

	def __get_node_location(self, node_name):
		master = rosgraph.Master('/rosnode')
		node_uri = urlparse(master.lookupNode(node_name))
		return node_uri.hostname

	def __publishers_and_subscribers_are_in_the_same_host(self, pubs, subs):
		for publisher in pubs:
			for subscriber in subs:
				if self.__get_node_location(publisher) != self.__get_node_location(subscriber):
					return False
		return True

	def __normalize_priorities(self, managed_topics):
		priorities = {}
		prioritiessum = sum(dynamic_bandwidth_manager.DBMParam.get_priority(topic) * dynamic_bandwidth_manager.DBMParam.get_message_size_in_bytes(topic) for topic in managed_topics)
		for topic in managed_topics:
			if not prioritiessum == 0:
				priorities[topic] = (dynamic_bandwidth_manager.DBMParam.get_priority(topic) * dynamic_bandwidth_manager.DBMParam.get_message_size_in_bytes(topic)) / prioritiessum
			else:
				priorities[topic] = dynamic_bandwidth_manager.DBMParam.get_priority(topic)
		return priorities
