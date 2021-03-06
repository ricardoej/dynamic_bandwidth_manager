import rospy
from .dbm_param import DBMParam
from .dbm_rate import DBMRate
from threading import Thread
import socket
import rosgraph
import roslib.message
import dynamic_bandwidth_manager
import sys
from operator import itemgetter
from std_msgs.msg import String

class ROSTopicException(Exception):
    pass

# code adapted from rqt_plot
def msgevalgen(pattern):
    evals = []  # list of (field_name, slice_object) pairs
    fields = [f for f in pattern.split('/') if f]
    for f in fields:
        if '[' in f:
            field_name, rest = f.split('[', 1)
            if not rest.endswith(']'):
                print("missing closing ']' in slice spec '%s'" % f)
                return None
            rest = rest[:-1]  # slice content, removing closing bracket
            try:
                array_index_or_slice_object = _get_array_index_or_slice_object(rest)
            except AssertionError as e:
                print("field '%s' has invalid slice argument '%s': %s"
                      % (field_name, rest, str(e)))
                return None
            evals.append((field_name, array_index_or_slice_object))
        else:
            evals.append((f, None))

    def msgeval(msg, evals):
        for i, (field_name, slice_object) in enumerate(evals):
            try: # access field first
                msg = getattr(msg, field_name)
            except AttributeError:
                print("no field named %s in %s" % (field_name, pattern))
                return None

            if slice_object is not None: # access slice
                try:
                    msg = msg.__getitem__(slice_object)
                except IndexError as e:
                    print("%s: %s" % (str(e), pattern))
                    return None

                # if a list is returned here (i.e. not only a single element accessed),
                # we need to recursively call msg_eval() with the rest of evals
                # in order to handle nested slices
                if isinstance(msg, list):
                    rest = evals[i + 1:]
                    return [msgeval(m, rest) for m in msg]
        return msg

    return (lambda msg: msgeval(msg, evals)) if evals else None

def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def _get_topic_type(topic):
    try:
        val = _master_get_topic_types(rosgraph.Master('/rostopic'))
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)

        # try to ignore messages which don't have the field specified as part of the topic name
        while matches:
            t, t_type = matches[0]
            msg_class = roslib.message.get_message_class(t_type)
            if not msg_class:
                # if any class is not fetchable skip ignoring any message types
                break
            msg = msg_class()
            nested_attributes = topic[len(t) + 1:].rstrip('/')
            nested_attributes = nested_attributes.split('[')[0]
            if nested_attributes == '':
                break
            try:
                _get_nested_attribute(msg, nested_attributes)
            except AttributeError:
                # ignore this type since it does not have the requested field
                matches.pop(0)
                continue
            matches = [(t, t_type)]
            break

    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None
    
def get_topic_type(topic, blocking=False):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def get_topic_class(topic, blocking=False):
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % topic_type)
    return msg_class, real_topic, msg_eval

def _sleep(duration):
    rospy.rostime.wallsleep(duration)

class RateThread(Thread):
	def __init__(self, rate, pub, get_message_method, print_message = False):
		Thread.__init__(self)

		self.rate = rate
		self.pub = pub
		self.get_message_method = get_message_method
		self.print_message = print_message

	def run(self):
		while not rospy.is_shutdown():
		        message = self.get_message_method()

		        if self.print_message:
		        	rospy.loginfo(message)

		        if message:
		        	self.pub.publish(message)
		        self.rate.sleep()

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, min_frequency, max_frequency, default_frequency = None,
		subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=10):
		super(DBMPublisher, self).__init__(name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)

		self.name = name
		self.data_class = data_class
		self.min_frequency = min_frequency
		self.max_frequency = max_frequency
		self.default_frequency = default_frequency
		self.subscriber_listener = subscriber_listener
		self.tcp_nodelay = tcp_nodelay
		self.latch = latch
		self.headers = headers
		self.queue_size = queue_size

		msg_class, real_topic, _ = get_topic_class(name, blocking=True) #pause hz until topic is published
		rospy.Subscriber(real_topic, msg_class, self.callback)

	def start(self, get_message_method, print_message = False, full_rate = True):
		self.managed_topic_name = self.name + "/optimized"

		full_rate_pub = rospy.Publisher(self.name, self.data_class,
			self.subscriber_listener, self.tcp_nodelay, self.latch, self.headers, self.queue_size)
		managed_rate_pub = rospy.Publisher(self.managed_topic_name, self.data_class,
			self.subscriber_listener, self.tcp_nodelay, self.latch, self.headers, self.queue_size)

		managed_rate = DBMRate(self.name, self.min_frequency, self.max_frequency, self.default_frequency)
		managed_rate_thread = RateThread(managed_rate, managed_rate_pub, get_message_method, print_message)
		managed_rate_thread.start()

		if full_rate:
			full_rate = rospy.Rate(self.max_frequency)
			full_rate_thread = RateThread(full_rate, full_rate_pub, get_message_method)
			full_rate_thread.start()

	def callback(self, data):
		if not hasattr(self, 'last_message_size'):
			self.last_message_size = 0

		current_message_size = len(data.data)

		if current_message_size != self.last_message_size:
			DBMParam.set_message_size_in_bytes(self.name, current_message_size)

		self.last_message_size = current_message_size