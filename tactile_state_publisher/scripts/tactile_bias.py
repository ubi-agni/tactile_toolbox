#!/usr/bin/env python
# license removed for brevity

import rospy
from tactile_state_publisher.tactile_bias_module import tactile_bias

if __name__ == '__main__':
    rospy.init_node('tactile_bias', anonymous=False)
    tactile_bias()
