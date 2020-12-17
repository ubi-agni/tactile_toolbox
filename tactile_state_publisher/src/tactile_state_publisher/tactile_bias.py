#!/usr/bin/env python
# license removed for brevity
import rospy
from rospy.numpy_msg import numpy_msg
from tactile_msgs.msg import TactileState
from std_srvs.srv import EmptyResponse, Empty
import numpy

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
                bias.values /= self.initial_average_count
                rospy.loginfo("Acquired " + str(self.initial_average_count) + " samples")
                rospy.loginfo("  computed bias: " + str(bias.values))
                rospy.loginfo("  std deviation: " + str(numpy.sqrt(bias.squared / (self.initial_average_count-1)
                                                                   - bias.values * bias.values)))
