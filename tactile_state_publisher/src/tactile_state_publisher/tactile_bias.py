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
        self.average_vec = None
        self.bias = None
        self.pub = rospy.Publisher('/tactile_states_biased', TactileState, queue_size=10)
        self.sub = rospy.Subscriber("/tactile_states", TactileState, self.callback)
        self.service = rospy.Service('tactile_bias/reset', Empty, self.reset_bias)
        rospy.spin()

    def reset_bias(self, req=None):
        self.average_counter = self.initial_average_count
        self.average_vec = None
        self.initialized = False
        return EmptyResponse()

    def callback(self, data):
        if not self.initialized:
            if self.average_vec is None:
                self.average_vec = np.empty((0, len(data.sensors[0].values)), float)
            # accumulate values
            if self.average_counter > 0:
                self.average_vec = np.vstack((self.average_vec, np.asarray(data.sensors[0].values)))
                self.average_counter -= 1
            else:  # enough data to compute the average
                self.bias = self.average_vec.mean(0)
                rospy.loginfo("Acquired " + str(self.initial_average_count) + " samples")
                rospy.loginfo("  computed bias:" + str(self.bias))
                rospy.loginfo("  standard deviation:" + str(np.std(self.average_vec,  axis=0)))
                self.initialized = True
        else:
            if len(data.sensors[0].values) != len(self.bias):
                # reset the bias to new length
                self.reset_bias()
                return
            newvals = np.asarray(data.sensors[0].values) - self.bias
            data.sensors[0].values = newvals
            self.pub.publish(data)
