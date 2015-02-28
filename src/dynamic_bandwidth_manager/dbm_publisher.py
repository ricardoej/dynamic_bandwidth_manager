import rospy
from .dbm_param import DBMParam
from .dbm_rate import DBMRate
from threading import Thread

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

		        self.pub.publish(message)
		        self.rate.sleep()

class DBMPublisher(rospy.Publisher):
	def __init__(self, name, data_class, min_frequency, max_frequency, default_frequency = None,
		subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
		super(DBMPublisher, self).__init__(name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)

		self.min_frequency = min_frequency
		self.max_frequency = max_frequency
		self.default_frequency = default_frequency
		self.subscriber_listener = subscriber_listener
		self.tcp_nodelay = tcp_nodelay
		self.latch = latch
		self.headers = headers
		self.queue_size = queue_size

		rospy.Subscriber(name, rospy.AnyMsg, self.callback)

	def start(self, get_message_method, print_message = False):
		self.managed_topic_name = self.name + "/optimized"

		full_rate_pub = rospy.Publisher(self.name, self.data_class,
			self.subscriber_listener, self.tcp_nodelay, self.latch, self.headers, self.queue_size)
		managed_rate_pub = rospy.Publisher(self.managed_topic_name, self.data_class,
			self.subscriber_listener, self.tcp_nodelay, self.latch, self.headers, self.queue_size)

		full_rate = rospy.Rate(self.max_frequency)
		managed_rate = DBMRate(managed_rate_pub, self.min_frequency, self.max_frequency, self.default_frequency)

		full_rate_thread = RateThread(full_rate, full_rate_pub, get_message_method)
		managed_rate_thread = RateThread(managed_rate, managed_rate_pub, get_message_method, print_message)

		managed_rate_thread.start()
		full_rate_thread.start()

	def callback(self, data):
		if not hasattr(self, 'last_message_size'):
			self.last_message_size = 0

		current_message_size = len(data._buff)

		if current_message_size != self.last_message_size:
			DBMParam.set_message_size_in_bytes(self.name, current_message_size)

		self.last_message_size = current_message_size