#!/usr/bin/env python

# https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

from __future__ import print_function
import numpy as np
import sys
import tf2_ros
import rospy
from iiwa_picknmove_control.srv import *


def add_float_arrays_client(x, y):
    # Wait until the service becomes available
    rospy.wait_for_service('add_float_arrays')

    try:
        # Create a handl for calling the service
        add_float_arrays = rospy.ServiceProxy('add_float_arrays', AddTwoFloatArrays)

        # Call the service
        resp1 = add_float_arrays(x, y)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 7:
        x = np.array(sys.argv[1:4]).astype(float)
        y = np.array(sys.argv[4:7]).astype(float)
    else:
        print(usage())
        sys.exit(1)

    print('x type: {}'.format(type(x)))
    print("Requesting %s+%s" % (x, y))
    print("%s + %s = %s"%(x, y, add_float_arrays_client(x, y)))