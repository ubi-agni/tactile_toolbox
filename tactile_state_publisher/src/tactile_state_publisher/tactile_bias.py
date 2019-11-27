#!/usr/bin/env python
# license removed for brevity
import rospy
from tactile_msgs.msg import TactileState
from std_srvs.srv import EmptyResponse, Empty
from copy import deepcopy
import numpy as np

MAX_VAL = 65536

class tactile_bias(object):
	def __init__(self, count = 200):
		self.initialized = False
		self.initial_average_count = count
		self.average_counter = count
		self.average_vec = []
		self.bias = []
		self.pub = rospy.Publisher('/tactile_states_biased', TactileState, queue_size=10)
		self.sub = rospy.Subscriber("/tactile_states", TactileState, self.callback)
		self.service = rospy.Service('tactile_bias/bias', Empty, self.handle_bias)
		rospy.spin()

	def handle_bias(self, req):
		self.average_counter = self.initial_average_count
		self.initialized = False
		return EmptyResponse()
		
	def compute_averages(self, vecs):
		avgs = []
		for vec in vecs:
			avgs.append(np.mean(np.array(vec)))
		return avgs
		
	def callback(self, data):
		if not self.initialized:
			if len(self.average_vec) == 0:
				self.average_vec = [0]*len(data.sensors[0].values)
				for i,val in enumerate(data.sensors[0].values):
					self.average_vec[i] = []
			if self.average_counter > 0:
				for i,val in enumerate(data.sensors[0].values):
					self.average_vec[i].append(val)
				self.average_counter -= 1
			if self.average_counter <= 0:
				self.bias = self.compute_averages(self.average_vec)
				print ("avg:", self.bias)
				self.initialized = True
		else:
			newvals = []
			for i,val in enumerate(data.sensors[0].values):
				newvals.append(self.bias[i]-val) # values are decreasing when pressure augments, so bias - val is correct
			newvals_pos = np.maximum(np.zeros(len(newvals)), newvals)
			msg = deepcopy(data)
			msg.sensors[0].values = newvals_pos
			self.pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('tactile_bias', anonymous=False)
	tactile_bias()
