#!/usr/bin/env python

# https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

from __future__ import print_function

import sys
import rospy
from iiwa_picknmove_control.srv import *


def add_two_ints_client(x, y):
    # Wait until the service becomes available
    rospy.wait_for_service('add_two_ints')

    try:
        # Create a handl for calling the service
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)

        # Call the service
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    for i in range(10):
        x += i
        y += i
        print("Requesting %s+%s" % (x, y))
        print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))