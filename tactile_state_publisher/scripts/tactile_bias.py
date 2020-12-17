#!/usr/bin/env python
# license removed for brevity

import rospy
from tactile_state_publisher.tactile_bias import tactile_bias, SAMPLE_SIZE
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Applies a bias on each value of a TactileState \
                                     message incoming on /tactile_states and outgoing on /tactile_states_biased.\n\
                                     \n Bias is computed at start or on request through /tactile_bias/bias service')
    parser.add_argument("samples", type=int, nargs='?', default=SAMPLE_SIZE,
                        help="sample size for average computation of the bias, default is " + str(SAMPLE_SIZE))
    args = parser.parse_args()
    rospy.init_node('tactile_bias', anonymous=False)
    tactile_bias(args.samples)
