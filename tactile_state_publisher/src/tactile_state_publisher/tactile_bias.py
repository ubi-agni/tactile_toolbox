#!/usr/bin/env python
# license removed for brevity
import rospy
from tactile_msgs.msg import TactileState
from std_srvs.srv import EmptyResponse, Empty
import numpy as np

SAMPLE_SIZE = 200


class tactile_bias(object):
    def __init__(self, count=SAMPLE_SIZE):
        self.initialized = False
        self.initial_average_count = count
        self.average_counter = count
        self.average_vec = []
        self.bias = None
        self.pub = rospy.Publisher('/tactile_states_biased', TactileState, queue_size=10)
        self.sub = rospy.Subscriber("/tactile_states", TactileState, self.callback)
        self.service = rospy.Service('tactile_bias/bias', Empty, self.handle_bias)
        rospy.spin()

    def reset_bias(self):
        self.average_counter = self.initial_average_count
        self.average_vec = []
        self.initialized = False

    def handle_bias(self, req):
        self.reset_bias()
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
                for i, val in enumerate(data.sensors[0].values):
                    self.average_vec[i] = []
            # accumulate values
            if self.average_counter > 0:
                for i, val in enumerate(data.sensors[0].values):
                    self.average_vec[i].append(val)
                self.average_counter -= 1
            # enough data to compute the average
            if self.average_counter <= 0:
                self.bias = np.asarray(self.compute_averages(self.average_vec))
                rospy.loginfo("computed bias:" + str(self.bias))
                self.initialized = True
        else:
            if len(data.sensors[0].values) != len(self.bias):
                # reset the bias to new length
                self.reset_bias()
                return
            newvals = np.asarray(data.sensors[0].values) - self.bias
            data.sensors[0].values = newvals
            self.pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('tactile_bias', anonymous=False)
    tactile_bias()
