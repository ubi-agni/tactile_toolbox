#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy
from rospy.numpy_msg import numpy_msg
from tactile_msgs.msg import TactileState
from std_srvs.srv import EmptyResponse, Empty
from argparse import ArgumentParser

SAMPLE_SIZE = 200


class Bias(object):
    def __init__(self, values, count):
        self.values = numpy.zeros(values.shape)
        self.squared = numpy.zeros(values.shape)
        self.count = count


class tactile_bias(object):
    def __init__(self, count=SAMPLE_SIZE):
        self.initial_average_count = count
        self.reset_bias()
        self.pub = rospy.Publisher('/tactile_states_biased', numpy_msg(TactileState), queue_size=10)
        self.sub = rospy.Subscriber('/tactile_states', numpy_msg(TactileState), self.callback)
        self.service = rospy.Service('tactile_bias/reset', Empty, self.reset_bias)
        rospy.spin()

    def reset_bias(self, req=None):
        self.biases = dict()  # mapping sensor name to Bias objects
        return EmptyResponse()

    def callback(self, data):
        for sensor in data.sensors:
            try:
                bias = self.biases[sensor.name]
            except KeyError:
                bias = self.biases[sensor.name] = Bias(sensor.values, self.initial_average_count)
            self.compute(sensor.values, bias)

    def compute(self, values, bias):
        if bias.count == 0:
            values = values - bias.values
        else:
            bias.values += values
            bias.squared += values * values
            bias.count -= 1  # decrement seen-samples-count
            if bias.count == 0:
                N = float(self.initial_average_count)
                bias.values /= N
                rospy.loginfo("Acquired {} samples".format(self.initial_average_count))
                rospy.loginfo("  computed bias: " + str(bias.values))
                rospy.loginfo("  std deviation: " + str(numpy.sqrt(numpy.maximum(bias.squared / (N-1)
                                                                                 - (N/(N-1)) * bias.values * bias.values, 0))))


if __name__ == '__main__':
    parser = ArgumentParser(description='Applies a bias on each value of a TactileState \
                                        message incoming on /tactile_states and outgoing on /tactile_states_biased.\n\
                                        \n Bias is computed at start or on request through /tactile_bias/reset service')
    parser.add_argument("--samples", type=int, nargs='?', default=SAMPLE_SIZE,
                        help="sample size for average computation of the bias, default is " + str(SAMPLE_SIZE))
    args, unknown = parser.parse_known_args()
    rospy.init_node('tactile_bias', anonymous=False)
    tactile_bias(args.samples)
